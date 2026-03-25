#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>

//中心线点结构
struct CenterlinePoint {
    double x;           // x坐标 (米)
    double y;           // y坐标 (米)
    double timestamp;   // 时间戳
    double confidence;  // 置信度
};

// 中心线帧数据
struct CenterlineFrame {
    std::vector<CenterlinePoint> points;
    double timestamp;
    int frame_id;
    double curvature;    // 平均曲率
    double length;       // 路径长度
};

class TemporalCenterlineFitter {
private:
    // 历史帧数据队列
    std::deque<CenterlineFrame> history_frames_;
    int max_history_size_;          // 最大历史帧数
    double resolution_;              // 地图分辨率
    double vehicle_x_;              // 车辆x坐标
    double vehicle_y_;              // 车辆y坐标
    bool debug_mode_;               // 调试模式
    
public:
    TemporalCenterlineFitter(int max_history = 10, double resolution = 0.05, bool debug = false)
        : max_history_size_(max_history), resolution_(resolution), debug_mode_(debug) {
        vehicle_x_ = 0;
        vehicle_y_ = 0;
    }
    
    void setVehiclePosition(double x, double y) {
        vehicle_x_ = x;
        vehicle_y_ = y;
    }
    
    /**
     * 处理单帧图像，提取中心线并融合历史信息
     */
    CenterlineFrame processFrame(const cv::Mat& occupancy_grid, int frame_id, double timestamp) {
        // 1. 提取当前帧中心线
        auto current_points = extractCenterlineSingle(occupancy_grid);
        
        // 2. 创建当前帧数据
        CenterlineFrame current_frame;
        current_frame.frame_id = frame_id;
        current_frame.timestamp = timestamp;
        current_frame.points = current_points;
        current_frame.length = calculatePathLength(current_points);
        current_frame.curvature = calculateAverageCurvature(current_points);
        
        // 3. 与时序历史拟合
        CenterlineFrame fused_frame = temporalFusion(current_frame);
        
        // 4. 添加到历史队列
        addToHistory(fused_frame);
        
        // 5. 预测下一帧中心线（可选）	
        CenterlineFrame predicted_frame = predictNextFrame();
        
        if (debug_mode_ && !fused_frame.points.empty()) {
            visualizeFusion(current_frame, fused_frame, predicted_frame);
        }
        
        return fused_frame;
    }
    
private:
    /**
     * 单帧中心线提取（非避障版本）
     */
    std::vector<CenterlinePoint> extractCenterlineSingle(const cv::Mat& occupancy_grid) {
        std::vector<CenterlinePoint> centerline;
        
        if (occupancy_grid.empty()) {
            return centerline;
        }
        
        int height = occupancy_grid.rows;
        int width = occupancy_grid.cols;
        
        // 车辆像素坐标
        int vehicle_px = std::max(0, std::min(width - 1, (int)(vehicle_x_ / resolution_)));
        int vehicle_py = std::max(0, std::min(height - 1, (int)(vehicle_y_ / resolution_)));
        
        // 扫描参数
        int scan_start_y = std::max(0, vehicle_py - 300);
        int scan_end_y = vehicle_py;
        int scan_step = 5;
        
        //边界跟踪
        int last_left_x = vehicle_px;
        int last_right_x = vehicle_px;
        int window_width = 100;
        
        for (int y = scan_end_y; y >= scan_start_y; y -= scan_step) {
            if (y < 0 || y >= height) continue;
            
            // 搜索左边界
            int search_left_start = std::max(0, last_left_x - window_width);
            int search_left_end = std::max(0, std::min(width - 1, last_left_x));
            
            int left_x = -1;
            for (int x = search_left_end; x >= search_left_start; x--) {
                if (x < 0 || x >= width) continue;
                if (occupancy_grid.at<uchar>(y, x) == 0) {
                    left_x = x + 1;
                    break;
                }
            }
            
            if (left_x == -1) left_x = search_left_start;
            left_x = std::max(0, std::min(width - 1, left_x));
            
            // 搜索右边界
            int search_right_start = std::max(0, std::min(width - 1, last_right_x));
            int search_right_end = std::min(width - 1, last_right_x + window_width);
            
            int right_x = -1;
            for (int x = search_right_start; x <= search_right_end; x++) {
                if (x < 0 || x >= width) continue;
                if (occupancy_grid.at<uchar>(y, x) == 0) {
                    right_x = x - 1;
                    break;
                }
            }
            
            if (right_x == -1) right_x = search_right_end;
            right_x = std::max(0, std::min(width - 1, right_x));
            
            // 计算中心点
            if (right_x > left_x) {
                CenterlinePoint point;
                point.x = (left_x + right_x) / 2.0 * resolution_;
                point.y = y * resolution_;
                point.confidence = 1.0;
                point.timestamp = 0;
                
                centerline.push_back(point);
                
                last_left_x = left_x;
                last_right_x = right_x;
            }
        }
        
        // 	按y坐标排序（从远到近）
        std::sort(centerline.begin(), centerline.end(),
                  [](const CenterlinePoint& a, const CenterlinePoint& b) {
                      return a.y > b.y;
                  });
        
        return centerline;
    }
    
    /**
     * 时序融合：将当前帧与历史帧进行加权平均
     */
    CenterlineFrame temporalFusion(const CenterlineFrame& current_frame) {
        CenterlineFrame fused_frame = current_frame;
        
        if (history_frames_.empty() || current_frame.points.empty()) {
            return fused_frame;
        }
        
        //对齐历史帧和当前帧的路径点
        std::vector<std::vector<CenterlinePoint>> aligned_points;
        aligned_points.push_back(current_frame.points);
        
        // 对齐历史帧
        for (const auto& history_frame : history_frames_) {
            if (!history_frame.points.empty()) {
                auto aligned = alignPathPoints(history_frame.points, current_frame.points);
                aligned_points.push_back(aligned);
            }
        }
        
        // 加权融合（越新的帧权重越高）
        std::vector<CenterlinePoint> fused_points;
        size_t max_points = current_frame.points.size();
        
        for (size_t i = 0; i < max_points; ++i) {
            double sum_x = 0, sum_y = 0;
            double weight_sum = 0;
            
            // 当前帧权重最高
            double current_weight = 0.5;
            sum_x += current_frame.points[i].x * current_weight;
            sum_y += current_frame.points[i].y * current_weight;
            weight_sum += current_weight;
            
            // 历史帧权重递减
            double weight = 0.3;
            for (size_t j = 1; j < aligned_points.size() && j <= (size_t)max_history_size_; ++j) {
                if (i < aligned_points[j].size()) {
                    double frame_weight = weight * (1.0 - j * 0.1);
                    if (frame_weight > 0) {
                        sum_x += aligned_points[j][i].x * frame_weight;
                        sum_y += aligned_points[j][i].y * frame_weight;
                        weight_sum += frame_weight;
                    }
                }
            }
            
            if (weight_sum > 0) {
                CenterlinePoint fused_point;
                fused_point.x = sum_x / weight_sum;
                fused_point.y = sum_y / weight_sum;
                fused_point.confidence = std::min(1.0, weight_sum / (max_history_size_ + 1));
                fused_point.timestamp = current_frame.timestamp;
                fused_points.push_back(fused_point);
            } else {
                fused_points.push_back(current_frame.points[i]);
            }
        }
        
        fused_frame.points = fused_points;
        fused_frame.curvature = calculateAverageCurvature(fused_points);
        fused_frame.length = calculatePathLength(fused_points);
        
        return fused_frame;
    }
    
    /**
     * 对齐两个路径点集
     */
    std::vector<CenterlinePoint> alignPathPoints(
        const std::vector<CenterlinePoint>& source,
        const std::vector<CenterlinePoint>& target) {
        
        std::vector<CenterlinePoint> aligned;
        
        for (const auto& target_point : target) {
            // 在source中找到y坐标最接近的点
            double min_dist = std::numeric_limits<double>::max();
            int best_idx = -1;
            
            for (size_t i = 0; i < source.size(); ++i) {
                double dist = std::abs(source[i].y - target_point.y);
                if (dist < min_dist && dist < 0.5) {
                    min_dist = dist;
                    best_idx = i;
                }
            }
            
            if (best_idx >= 0) {
                aligned.push_back(source[best_idx]);
            } else {
                // 如果没有找到匹配，使用最近点
                if (!source.empty()) {
                    double min_y_dist = std::numeric_limits<double>::max();
                    int nearest_idx = 0;
                    for (size_t i = 0; i < source.size(); ++i) {
                        double dist = std::abs(source[i].y - target_point.y);
                        if (dist < min_y_dist) {
                            min_y_dist = dist;
                            nearest_idx = i;
                        }
                    }
                    aligned.push_back(source[nearest_idx]);
                } else {
                    aligned.push_back(target_point);
                }
            }
        }
        
        return aligned;
    }
    
    /**
     *  预测下一帧中心线
     */
    CenterlineFrame predictNextFrame() {
        CenterlineFrame predicted;
        
        if (history_frames_.size() < 2) {
            return predicted;
        }
        
        // 使用简单的线性预测
        std::vector<CenterlinePoint> predicted_points;
        
        const auto& latest_frame = history_frames_.back();
        const auto& prev_frame = history_frames_[history_frames_.size() - 2];
        
        if (latest_frame.points.empty() || prev_frame.points.empty()) {
            return predicted;
        }
        
        size_t num_points = std::min(latest_frame.points.size(), prev_frame.points.size());
        
        for (size_t i = 0; i < num_points; ++i) {
            // 计算位移变化
            double dx = latest_frame.points[i].x - prev_frame.points[i].x;
            
            CenterlinePoint point;
            point.x = latest_frame.points[i].x + dx;
            point.y = latest_frame.points[i].y;
            point.confidence = 0.7;
            point.timestamp = latest_frame.timestamp + 0.1;
            
            predicted_points.push_back(point);
        }
        
        predicted.points = predicted_points;
        predicted.frame_id = latest_frame.frame_id + 1;
        predicted.timestamp = latest_frame.timestamp + 0.1;
        predicted.length = calculatePathLength(predicted_points);
        predicted.curvature = calculateAverageCurvature(predicted_points);
        
        return predicted;
    }
    
    /**
     * 添加帧到历史队列
     */
    void addToHistory(const CenterlineFrame& frame) {
        history_frames_.push_back(frame);
        
        while ((int)history_frames_.size() > max_history_size_) {
            history_frames_.pop_front();
        }
    }
    
    /**
     * 计算路径长度
     */
    double calculatePathLength(const std::vector<CenterlinePoint>& points) {
        if (points.size() < 2) return 0;
        
        double length = 0;
        for (size_t i = 1; i < points.size(); ++i) {
            double dx = points[i].x - points[i-1].x;
            double dy = points[i].y - points[i-1].y;
            length += std::sqrt(dx*dx + dy*dy);
        }
        return length;
    }
    
    /**
     * 计算平均曲率
     */
    double calculateAverageCurvature(const std::vector<CenterlinePoint>& points) {
        if (points.size() < 3) return 0;
        
        double total_curvature = 0;
        int count = 0;
        
        for (size_t i = 1; i < points.size() - 1; ++i) {
            double dx1 = points[i].x - points[i-1].x;
            double dy1 = points[i].y - points[i-1].y;
            double dx2 = points[i+1].x - points[i].x;
            double dy2 = points[i+1].y - points[i].y;
            
            double angle1 = std::atan2(dy1, dx1);
            double angle2 = std::atan2(dy2, dx2);
            double angle_diff = std::abs(angle2 - angle1);
            
            total_curvature += angle_diff;
            count++;
        }
        
        return count > 0 ? total_curvature / count : 0;
    }
    
    /**
     * 可视化融合结果
     */
    void visualizeFusion(const CenterlineFrame& current, 
                         const CenterlineFrame& fused,
                         const CenterlineFrame& predicted) {
        cv::Mat vis = cv::Mat::zeros(600, 800, CV_8UC3);
        
        // 绘制当前帧中心线（绿色）
        for (size_t i = 1; i < current.points.size(); ++i) {
            cv::Point p1(current.points[i-1].x / resolution_, 
                        current.points[i-1].y / resolution_);
            cv::Point p2(current.points[i].x / resolution_, 
                        current.points[i].y / resolution_);
            if (p1.x >= 0 && p1.x < 800 && p1.y >= 0 && p1.y < 600 &&
                p2.x >= 0 && p2.x < 800 && p2.y >= 0 && p2.y < 600) {
                cv::line(vis, p1, p2, cv::Scalar(0, 255, 0), 2);
            }
        }
        
        // 绘制融合后中心线（蓝色）
        for (size_t i = 1; i < fused.points.size(); ++i) {
            cv::Point p1(fused.points[i-1].x / resolution_, 
                        fused.points[i-1].y / resolution_);
            cv::Point p2(fused.points[i].x / resolution_, 
                        fused.points[i].y / resolution_);
            if (p1.x >= 0 && p1.x < 800 && p1.y >= 0 && p1.y < 600 &&
                p2.x >= 0 && p2.x < 800 && p2.y >= 0 && p2.y < 600) {
                cv::line(vis, p1, p2, cv::Scalar(255, 0, 0), 3);
            }
        }
        
        // 绘制预测中心线（红色）
        if (!predicted.points.empty()) {
            for (size_t i = 1; i < predicted.points.size(); ++i) {
                cv::Point p1(predicted.points[i-1].x / resolution_, 
                            predicted.points[i-1].y / resolution_);
                cv::Point p2(predicted.points[i].x / resolution_, 
                            predicted.points[i].y / resolution_);
                if (p1.x >= 0 && p1.x < 800 && p1.y >= 0 && p1.y < 600 &&
                    p2.x >= 0 && p2.x < 800 && p2.y >= 0 && p2.y < 600) {
                    cv::line(vis, p1, p2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                }
            }
        }
        
        // 添加图例
        cv::putText(vis, "Current (Green)", cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        cv::putText(vis, "Fused (Blue)", cv::Point(10, 50), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
        cv::putText(vis, "Predicted (Red)", cv::Point(10, 70), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        
        cv::imshow("Temporal Fusion", vis);
        cv::waitKey(1);
    }
    
public:
    /**
     * 保存历史数据到文件
     */
    void saveHistoryToFile(const std::string& filename) {
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "Cannot open file: " << filename << std::endl;
            return;
        }
        
        for (const auto& frame : history_frames_) {
            file << "Frame " << frame.frame_id << ", Timestamp: " << frame.timestamp << std::endl;
            file << "Length: " << frame.length << "m, Curvature: " << frame.curvature << std::endl;
            
            for (const auto& point : frame.points) {
                file << point.x << "," << point.y << "," << point.confidence << std::endl;
            }
            file << "---" << std::endl;
        }
        
        file.close();
        std::cout << "History saved to: " << filename << std::endl;
    }
    
    /**
     * 获取最新融合的中心线
     */
    CenterlineFrame getLatestCenterline() {
        if (history_frames_.empty()) {
            return CenterlineFrame();
        }
        return history_frames_.back();
    }
    
    /**
     * 清除历史数据
     */
    void clearHistory() {
        history_frames_.clear();
    }
};

// 生成测试图像序列（修复版）
std::vector<cv::Mat> generateTestSequence(int num_frames, int width, int height) {
    std::vector<cv::Mat> sequence;
    
    // 使用C++11随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise_dist(0, 15);
    
    for (int frame = 0; frame < num_frames; ++frame) {
        cv::Mat road = cv::Mat::zeros(height, width, CV_8UC1);
        
        // 模拟随时间变化的弯道（动态场景）
        double phase = frame * 0.1;  // 随时间变化的相位
        
        for (int y = 0; y < height; ++y) {
            double t = y / (double)height;
            // 道路形状随时间变化
            double offset = 60 * sin(t * CV_PI + phase);
            
            int left_x = std::max(0, width/3 + (int)offset);
            int right_x = std::min(width-1, width*2/3 + (int)offset);
            
             // 确保边界有效
            left_x = std::max(0, std::min(width-1, left_x));
            right_x = std::max(0, std::min(width-1, right_x));
            
            for (int x = left_x; x <= right_x; ++x) {
                if (x >= 0 && x < width) {
                    road.at<uchar>(y, x) = 255;
                }
            }
            
            // 添加边界线
            if (left_x >= 0 && left_x < width)
                road.at<uchar>(y, left_x) = 0;
            if (right_x >= 0 && right_x < width)
                road.at<uchar>(y, right_x) = 0;
        }
        
        // 添加轻微噪声（手动添加，避免randn问题）
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (road.at<uchar>(y, x) == 255) {
                    int noise = (int)noise_dist(gen);
                    int val = 255 + noise;
                    val = std::max(0, std::min(255, val));
                    road.at<uchar>(y, x) = val > 200 ? 255 : 0;
                }
            }
        }
        
        sequence.push_back(road);
    }
    
    return sequence;
}

