#include "centerline_extract.h"

// ==================== CenterlinePoint 实现 ====================

CenterlinePoint::CenterlinePoint() 
    : curvature(0), heading(0), confidence(1.0), 
      road_width(0), timestamp(0) {}

// ==================== 平滑滤波器 SmoothingFilter 实现 ====================
// 移动平均平滑
std::vector<CenterlinePoint> SmoothingFilter::movingAverage(const std::vector<CenterlinePoint>& points, int window) {
    if (points.size() < (size_t)window) return points;
    
    std::vector<CenterlinePoint> smoothed;
    int half = window / 2;
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (i < (size_t)half || i >= points.size() - half) {
            smoothed.push_back(points[i]);
            continue;
        }
        
        cv::Point2d sum_pos(0, 0);
        double sum_width = 0;
	    int count = 0;
        
        for (int j = -half; j <= half; ++j) {
            sum_pos += points[i + j].position;
            sum_width += points[i + j].road_width;
	        count++;
        }
        
        CenterlinePoint sp = points[i];
        sp.position = sum_pos / count;
        sp.road_width = sum_width / count;
        smoothed.push_back(sp);
    }
    
    return smoothed;
}

// 高斯平滑
std::vector<CenterlinePoint> SmoothingFilter::gaussianSmooth(const std::vector<CenterlinePoint>& points, double sigma) {
    if (points.size() < 5) return points;
    
    int kernel_size = std::max(3, (int)(sigma * 3) * 2 + 1);
    std::vector<double> kernel(kernel_size);
    double sum = 0;
    
    for (int i = 0; i < kernel_size; ++i) {
        int x = i - kernel_size / 2;
        kernel[i] = std::exp(-(x * x) / (2 * sigma * sigma));
        sum += kernel[i];
    }
    
    for (int i = 0; i < kernel_size; ++i) {
        kernel[i] /= sum;
    }
    
    std::vector<CenterlinePoint> smoothed;
    int half = kernel_size / 2;
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (i < (size_t)half || i >= points.size() - half) {
            smoothed.push_back(points[i]);
            continue;
        }
        
        cv::Point2d sum_pos(0, 0);
        double sum_width = 0;
        
        for (int j = -half; j <= half; ++j) {
            double weight = kernel[j + half];
            sum_pos += points[i + j].position * weight;
            sum_width += points[i + j].road_width * weight;
        }
        
        CenterlinePoint sp = points[i];
        sp.position = sum_pos;
        sp.road_width = sum_width;
        smoothed.push_back(sp);
    }
    
    return smoothed;
}

// ==================== 中心线提取器 CenterlineExtractor 实现 ====================

CenterlineExtractor::CenterlineExtractor(double resolution, bool debug)
    : resolution_(resolution), debug_(debug) {}

void CenterlineExtractor::setVehiclePosition(const cv::Point2d& pos) {
    vehicle_pos_ = pos;
}

std::vector<CenterlinePoint> CenterlineExtractor::extract(const cv::Mat& occupancy_grid) {
    std::vector<CenterlinePoint> centerline;
    
    if (occupancy_grid.empty()) return centerline;
    
    int height = occupancy_grid.rows;
    int width = occupancy_grid.cols;
    
    // 车辆像素坐标
    int vehicle_px = (int)(vehicle_pos_.x / resolution_);
    int vehicle_py = (int)(vehicle_pos_.y / resolution_);
    
    vehicle_px = std::max(0, std::min(width - 1, vehicle_px));
    vehicle_py = std::max(0, std::min(height - 1, vehicle_py));
    
    // 扫描参数 - 从车辆位置向上扫描到图像顶部
    int scan_step = 10;
    int scan_start = 100;
    int scan_end = vehicle_py;
    
    for (int y = scan_end; y >= scan_start; y -= scan_step) {
        if (y < 0 || y >= height) continue;
        
        // 搜索左边界：从车辆位置向左找第一个黑色像素（边界）
        int left = -1;
        for (int x = vehicle_px; x >= 0; x--) {
            if (x >= width) continue;
            uchar pixel = occupancy_grid.at<uchar>(y, x);
            if (pixel == 0) {
                left = x;
                break;
            }
        }
        
        // 搜索右边界：从车辆位置向右找第一个黑色像素（边界）
        int right = -1;
        for (int x = vehicle_px; x < width; x++) {
            if (x < 0) continue;
            uchar pixel = occupancy_grid.at<uchar>(y, x);
            if (pixel == 0) {
                right = x;
                break;
            }
        }
        
        // 如果找到左右边界，计算中心点
        if (left >= 0 && right >= 0 && right > left) {
            double center_x = (left + right) / 2.0;
            
            CenterlinePoint point;
            point.position.x = center_x * resolution_;
            point.position.y = y * resolution_;
            point.road_width = (right - left) * resolution_;
            point.confidence = 0.95;
            centerline.push_back(point);
        }
    }
    
    if (centerline.empty()) {
        if (debug_) std::cout << "  No boundaries found, generating default path" << std::endl;
        // 生成默认直行路径
        for (int y = vehicle_py; y >= 0; y -= 10) {
            CenterlinePoint point;
            point.position.x = vehicle_pos_.x;
            point.position.y = y * resolution_;
            point.road_width = 8.0;
            point.confidence = 0.6;
            centerline.push_back(point);
        }
    }
    
    // 平滑处理
    if (centerline.size() > 50) {
        centerline = SmoothingFilter::movingAverage(centerline, 50);
        centerline = SmoothingFilter::gaussianSmooth(centerline, 1.0);
    }
    
    // 计算曲率和航向
    for (size_t i = 1; i < centerline.size() - 1; ++i) {
        double dx1 = centerline[i].position.x - centerline[i-1].position.x;
        double dy1 = centerline[i].position.y - centerline[i-1].position.y;
        double dx2 = centerline[i+1].position.x - centerline[i].position.x;
        double dy2 = centerline[i+1].position.y - centerline[i].position.y;
        
        centerline[i].heading = std::atan2(dy1, dx1);
        
        double angle1 = std::atan2(dy1, dx1);
        double angle2 = std::atan2(dy2, dx2);
        double dtheta = angle2 - angle1;
        if (dtheta > CV_PI) dtheta -= 2 * CV_PI;
        if (dtheta < -CV_PI) dtheta += 2 * CV_PI;
        double ds = (std::hypot(dx1, dy1) + std::hypot(dx2, dy2)) / 2.0;
        if (ds > 1e-6) centerline[i].curvature = dtheta / ds;
    }
    
    return centerline;
}

// ==================== 时序处理器 TemporalProcessor 实现 ====================

TemporalProcessor::TemporalProcessor(int max_history) : max_history_(max_history) {}

double TemporalProcessor::calculatePathLength(const std::vector<CenterlinePoint>& points) {
    if (points.size() < 2) return 0;
    double length = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        length += cv::norm(points[i].position - points[i-1].position);
    }
    return length;
}

FrameData TemporalProcessor::process(const FrameData& current) {
    FrameData fused = current;
    
    if (history_.empty()) {
        history_.push_back(current);
        return fused;
    }
    
    // 时序融合
    std::vector<CenterlinePoint> fused_points;
    size_t num_points = current.points.size();
    
    for (size_t i = 0; i < num_points; ++i) {
        cv::Point2d sum_pos(0, 0);
        double sum_width = 0;
        double sum_confidence = 0;
        double total_weight = 0;
        
        // 当前帧权重最高
        double curr_weight = 0.5;
        sum_pos += current.points[i].position * curr_weight;
        sum_width += current.points[i].road_width * curr_weight;
        sum_confidence += current.points[i].confidence * curr_weight;
        total_weight += curr_weight;
        
        // 历史帧权重指数衰减
        int hist_count = 0;
        for (auto it = history_.rbegin(); it != history_.rend() && hist_count < max_history_; ++it, ++hist_count) {
            if (i < it->points.size()) {
                double hist_weight = 0.3 * std::pow(0.85, hist_count);
                sum_pos += it->points[i].position * hist_weight;
                sum_width += it->points[i].road_width * hist_weight;
                sum_confidence += it->points[i].confidence * hist_weight;
                total_weight += hist_weight;
            }
        }
        
        CenterlinePoint fused_point = current.points[i];
        if (total_weight > 0) {
            fused_point.position = sum_pos / total_weight;
            fused_point.road_width = sum_width / total_weight;
            fused_point.confidence = sum_confidence / total_weight;
        }
        fused_points.push_back(fused_point);
    }
    
    // 时序平滑
    if (fused_points.size() > 5) {
        fused_points = SmoothingFilter::gaussianSmooth(fused_points, 0.8);
    }
    
    fused.points = fused_points;
    fused.path_length = calculatePathLength(fused_points);
    
    history_.push_back(fused);
    while ((int)history_.size() > max_history_) {
        history_.pop_front();
    }
    
    return fused;
}

void TemporalProcessor::clear() {
    history_.clear();
}

// ==================== StartPointAligner 起点对齐器实现 ====================

StartPointAligner::StartPointAligner() {}

void StartPointAligner::setVehiclePosition(const cv::Point2d& pos) {
    vehicle_pos_ = pos;
}

// 对齐起点到车辆位置
std::vector<CenterlinePoint> StartPointAligner::align(const std::vector<CenterlinePoint>& points) {
    if (points.empty()) return points;
    
    std::vector<CenterlinePoint> aligned = points;
    cv::Point2d offset = vehicle_pos_ - aligned[0].position;
    
    for (auto& p : aligned) {
        p.position += offset;
    }
    
    return aligned;
}

// 确保起点精确对齐
std::vector<CenterlinePoint> StartPointAligner::ensureExact(const std::vector<CenterlinePoint>& points) {
    if (points.empty()) return points;
    
    std::vector<CenterlinePoint> aligned = points;
    aligned[0].position = vehicle_pos_;
    aligned[0].confidence = 1.0;
    
    // 平滑前几个点，避免突变
    if (aligned.size() > 2) {
        int smooth_count = std::min(3, (int)aligned.size());
        for (int i = 1; i < smooth_count; ++i) {
            double t = (double)i / smooth_count;
            aligned[i].position = vehicle_pos_ * (1 - t) + points[i].position * t;
        }
    }
    
    return aligned;
}

// ==================== 中心校正器  ====================

CenterCorrector::CenterCorrector() {}

void CenterCorrector::setVehiclePosition(const cv::Point2d& pos) {
    vehicle_pos_ = pos;
}

double CenterCorrector::calculateOffset(const std::vector<CenterlinePoint>& points) {
    if (points.empty()) return 0;
    
    double total = 0;
    int num = std::min((int)points.size(), 30);
    for (int i = 0; i < num; ++i) {
        total += points[i].position.x - vehicle_pos_.x;
    }
    return total / num;
}

std::vector<CenterlinePoint> CenterCorrector::correct(const std::vector<CenterlinePoint>& points) {
    if (points.empty()) return points;
    
    double offset = calculateOffset(points);
    
    // 历史平滑
    offset_history_.push_back(offset);
    if (offset_history_.size() > 10) offset_history_.pop_front();
    
    double smooth_offset = 0;
    for (double o : offset_history_) smooth_offset += o;
    smooth_offset /= offset_history_.size();
    
    // 校正
    std::vector<CenterlinePoint> corrected = points;
    for (auto& p : corrected) {
        p.position.x -= smooth_offset * 0.6;
    }
    
    return corrected;
}

void CenterCorrector::reset() {
    offset_history_.clear();
}

// ==================== Visualizer 可视化器实现 ====================

Visualizer::Visualizer(double resolution, bool save_images)
    : resolution_(resolution), save_images_(save_images) {}

double Visualizer::calculateLength(const std::vector<CenterlinePoint>& points) {
    if (points.size() < 2) return 0;
    double length = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        length += cv::norm(points[i].position - points[i-1].position);
    }
    return length;
}

void Visualizer::show(const cv::Mat& image, const std::vector<CenterlinePoint>& points,
                      const cv::Point2d& vehicle_pos, int frame_id, double offset) {
    
    if (image.empty()) return;
    
    cv::Mat display;
    cv::cvtColor(image, display, cv::COLOR_GRAY2BGR);
    
    // 绘制中心线
    if (points.size() >= 2) {
        for (size_t i = 1; i < points.size(); ++i) {
            cv::Point p1(points[i-1].position.x / resolution_, 
                        points[i-1].position.y / resolution_);
            cv::Point p2(points[i].position.x / resolution_, 
                        points[i].position.y / resolution_);
            
            if (p1.x >= 0 && p1.x < display.cols && p1.y >= 0 && p1.y < display.rows &&
                p2.x >= 0 && p2.x < display.cols && p2.y >= 0 && p2.y < display.rows) {
                cv::line(display, p1, p2, cv::Scalar(0, 255, 0), 3);
            }
        }
    }
    
    // 绘制点（可选）
    if (frame_id % 5 == 0 && points.size() > 0) {
        for (const auto& p : points) {
            cv::Point pt(p.position.x / resolution_, p.position.y / resolution_);
            if (pt.x >= 0 && pt.x < display.cols && pt.y >= 0 && pt.y < display.rows) {
                cv::circle(display, pt, 2, cv::Scalar(0, 255, 255), -1);
            }
        }
    }
    
    // 绘制起点
    if (!points.empty()) {
        cv::Point start(points[0].position.x / resolution_, 
                       points[0].position.y / resolution_);
        if (start.x >= 0 && start.x < display.cols && start.y >= 0 && start.y < display.rows) {
            cv::circle(display, start, 10, cv::Scalar(255, 0, 255), -1);
        }
    }
    
    // 绘制车辆位置
    cv::Point vehicle_px(vehicle_pos.x / resolution_, vehicle_pos.y / resolution_);
    if (vehicle_px.x >= 0 && vehicle_px.x < display.cols && 
        vehicle_px.y >= 0 && vehicle_px.y < display.rows) {
        cv::circle(display, vehicle_px, 12, cv::Scalar(0, 255, 255), -1);
    }
    
    // 绘制理想中心线（白色虚线）
    cv::Point ideal_start(vehicle_px.x, vehicle_px.y);
    cv::Point ideal_end(vehicle_px.x, 0);
    cv::line(display, ideal_start, ideal_end, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    
    // 文字信息
    std::string text = "Frame: " + std::to_string(frame_id) + 
                       " | Points: " + std::to_string(points.size()) +
                       " | Length: " + std::to_string(calculateLength(points)).substr(0, 5) + "m" +
                       " | Offset: " + std::to_string(offset).substr(0, 5) + "m";
    cv::putText(display, text, cv::Point(10, 40), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1.5);
    
    cv::imshow("Centerline Extraction", display);
    
    if (save_images_ && frame_id % 5 == 0) {
        std::string filename = "frame_" + std::to_string(frame_id) + ".png";
        cv::imwrite(filename, display);
    }
    
    cv::waitKey(30);
}

// ==================== 测试数据生成 ====================

std::vector<cv::Mat> generateTestSequence(int num_frames, int width, int height) {
    std::vector<cv::Mat> sequence;
    
    for (int frame = 0; frame < num_frames; ++frame) {
        cv::Mat road = cv::Mat::zeros(height, width, CV_8UC1);
        double phase = frame * 0.12;
        
        // 道路参数
        int left_base = width / 3;      // 640
        int right_base = width * 2 / 3; // 1280
        
        for (int y = 0; y < height; ++y) {
            double t = y / (double)height;
            
            // 左边界（带曲线，随时间变化）
            int left = left_base;
            left += 50 * sin(t * CV_PI * 1.5 + phase);
            left += 25 * sin(t * CV_PI * 3);
            
            // 右边界
            int right = right_base;
            right += 45 * cos(t * CV_PI * 1.2 + phase);
            right += 20 * sin(t * CV_PI * 2.5);
            
            left = std::max(30, std::min(width - 100, left));
            right = std::max(left + 80, std::min(width - 30, right));
            
            // 填充道路
            for (int x = left; x <= right; ++x) {
                if (x >= 0 && x < width) {
                    road.at<uchar>(y, x) = 255;
                }
            }
            
            // 绘制明显的边界线
            if (left > 0 && left < width) {
                road.at<uchar>(y, left) = 0;
                if (left + 1 < width) road.at<uchar>(y, left + 1) = 0;
            }
            
            if (right > 0 && right < width) {
                road.at<uchar>(y, right) = 0;
                if (right - 1 >= 0) road.at<uchar>(y, right - 1) = 0;
            }
        }
        
        sequence.push_back(road);
    }
    
    return sequence;
}