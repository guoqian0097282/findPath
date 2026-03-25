#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

class PracticalCenterlineExtractor {
private:
    cv::Mat occupancy_grid_;
    double resolution_;
    double vehicle_x_;
    double vehicle_y_;
    
public:
    PracticalCenterlineExtractor(double resolution = 0.05) 
        : resolution_(resolution), vehicle_x_(0), vehicle_y_(0) {}
    
    void setOccupancyGrid(const cv::Mat& grid) {
        occupancy_grid_ = grid.clone();
    }
    
    void setVehiclePosition(double x, double y) {
        vehicle_x_ = x;
        vehicle_y_ = y;
    }
    
    /**
     * 实用的中心线提取方法
     * 不追求完美避障，而是生成可用且安全的路径
     */
    std::vector<cv::Point2d> extractCenterLine() {
        std::vector<cv::Point2d> centerline;
        
        if (occupancy_grid_.empty()) {
            return centerline;
        }
        
        int height = occupancy_grid_.rows;
        int width = occupancy_grid_.cols;
        int vehicle_px = vehicle_x_ / resolution_;
        int vehicle_py = vehicle_y_ / resolution_;
        
        std::cout << "Extracting centerline from y=" << vehicle_py << " to 0" << std::endl;
        
        // 记录上一行的中心位置，用于平滑
        double last_center_x = vehicle_px;
        
        // 从车辆位置向上扫描
        for (int y = vehicle_py; y >= 0; y -= 5) {
            // 找到该行的左右边界
            double left_bound = findLeftBoundary(y, vehicle_px);
            double right_bound = findRightBoundary(y, vehicle_px);
            
            if (left_bound >= right_bound - 10) {
                // 道路太窄，使用上一行的中心
                centerline.push_back(cv::Point2d(last_center_x * resolution_, y * resolution_));
                continue;
            }
            
            // 计算理想中心
            double ideal_center = (left_bound + right_bound) / 2.0;
            
            // 考虑上一行的中心，进行平滑
            double smooth_center = ideal_center * 0.7 + last_center_x * 0.3;
            
            // 检查该点是否在障碍物上
            if (isPointInObstacle(smooth_center, y)) {
                // 如果中心点有障碍物，向两侧搜索安全点
                smooth_center = findSafePoint(left_bound, right_bound, y, last_center_x);
            }
            
            // 确保点在道路边界内
            smooth_center = std::max(left_bound + 5, std::min(right_bound - 5, smooth_center));
            
            // 添加到路径
            centerline.push_back(cv::Point2d(smooth_center * resolution_, y * resolution_));
            last_center_x = smooth_center;
        }
        
        // 平滑路径
        centerline = smoothPath(centerline);
        
        // 移除明显的异常点
        centerline = removeOutliers(centerline);
        
        std::cout << "Generated " << centerline.size() << " path points" << std::endl;
        
        return centerline;
    }
    
private:
    /**
     * 找到左边界
     */
    double findLeftBoundary(int y, int vehicle_px) {
        int width = occupancy_grid_.cols;
        
        for (int x = vehicle_px; x >= 0; x--) {
            if (x >= width) continue;
            if (occupancy_grid_.at<uchar>(y, x) == 0) {
                return x + 1;
            }
        }
        return 0;
    }
    
    /**
     * 找到右边界
     */
    double findRightBoundary(int y, int vehicle_px) {
        int width = occupancy_grid_.cols;
        
        for (int x = vehicle_px; x < width; x++) {
            if (x < 0) continue;
            if (occupancy_grid_.at<uchar>(y, x) == 0) {
                return x - 1;
            }
        }
        return width - 1;
    }
    
    /**
     * 检查点是否在障碍物上
     */
    bool isPointInObstacle(double x, int y) {
        int px = (int)x;
        int py = y;
        
        if (px < 0 || px >= occupancy_grid_.cols || py < 0 || py >= occupancy_grid_.rows) {
            return true;
        }
        
        // 检查该点及其周围
        for (int dx = -3; dx <= 3; dx++) {
            for (int dy = -3; dy <= 3; dy++) {
                int nx = px + dx;
                int ny = py + dy;
                if (nx >= 0 && nx < occupancy_grid_.cols && ny >= 0 && ny < occupancy_grid_.rows) {
                    if (occupancy_grid_.at<uchar>(ny, nx) == 0) {
                        return true;
                    }
                }
            }
        }
        
        return false;
    }
    
    /**
     * 找到安全点
     */
    double findSafePoint(double left_bound, double right_bound, int y, double last_center) {
        // 搜索左右两侧，找到最安全的点
        double best_x = (left_bound + right_bound) / 2;
        double best_score = -1;
        
        // 搜索范围：整个道路宽度
        for (double x = left_bound + 5; x <= right_bound - 5; x += 2) {
            if (!isPointInObstacle(x, y)) {
                // 评分：靠近上一行的中心 + 靠近道路中心
                double score = 100 - std::abs(x - last_center) * 2;
                score += 50 - std::abs(x - (left_bound + right_bound) / 2);
                
                if (score > best_score) {
                    best_score = score;
                    best_x = x;
                }
            }
        }
        
        // 如果还是没找到安全点，使用边界附近
        if (best_score < 0) {
            // 检查左侧
            for (double x = left_bound + 5; x <= right_bound - 5; x += 2) {
                if (!isPointInObstacle(x, y)) {
                    return x;
                }
            }
            // 如果所有点都不安全，返回中心
            return (left_bound + right_bound) / 2;
        }
        
        return best_x;
    }
    
    /**
     * 平滑路径
     */
    std::vector<cv::Point2d> smoothPath(const std::vector<cv::Point2d>& path) {
        if (path.size() < 5) return path;
        
        std::vector<cv::Point2d> smoothed;
        int window = 5;
        
        for (size_t i = 0; i < path.size(); ++i) {
            if (i < window/2 || i >= path.size() - window/2) {
                smoothed.push_back(path[i]);
                continue;
            }
            
            cv::Point2d sum(0, 0);
            for (int j = -window/2; j <= window/2; ++j) {
                sum += path[i + j];
            }
            smoothed.push_back(sum / window);
        }
        
        return smoothed;
    }
    
    /**
     * 移除异常点
     */
    std::vector<cv::Point2d> removeOutliers(const std::vector<cv::Point2d>& path) {
        if (path.size() < 3) return path;
        
        std::vector<cv::Point2d> filtered;
        filtered.push_back(path[0]);
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            double dx1 = path[i].x - path[i-1].x;
            double dx2 = path[i+1].x - path[i].x;
            
            // 如果横向变化太大，跳过这个点
            if (std::abs(dx1 - dx2) < 0.5) {
                filtered.push_back(path[i]);
            }
        }
        
        filtered.push_back(path.back());
        return filtered;
    }
    
public:
    cv::Mat drawResult(const std::vector<cv::Point2d>& centerline) {
        cv::Mat result;
        cv::cvtColor(occupancy_grid_, result, cv::COLOR_GRAY2BGR);
        
        // 绘制中心线
        if (!centerline.empty()) {
            std::vector<cv::Point> pixel_points;
            for (const auto& point : centerline) {
                pixel_points.push_back(cv::Point(point.x / resolution_, point.y / resolution_));
            }
            
            for (size_t i = 1; i < pixel_points.size(); ++i) {
                cv::line(result, pixel_points[i-1], pixel_points[i], cv::Scalar(0, 255, 0), 3);
            }
            
            for (const auto& point : pixel_points) {
                cv::circle(result, point, 2, cv::Scalar(0, 255, 0), -1);
            }
        }
        
        // 绘制车辆位置
        cv::circle(result, cv::Point(vehicle_x_ / resolution_, vehicle_y_ / resolution_), 
                   12, cv::Scalar(0, 255, 255), -1);
        
        // 添加统计信息
        std::string info = "Path points: " + std::to_string(centerline.size());
        cv::putText(result, info, cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1.5);
        
        return result;
    }
};

// 创建简化的测试道路（确保边界清晰）
cv::Mat createClearRoad(int width, int height) {
    cv::Mat road = cv::Mat::zeros(height, width, CV_8UC1);
    
    int left_boundary = width/3;
    int right_boundary = width*2/3;
    
    // 填充道路
    for (int y = 0; y < height; ++y) {
        for (int x = left_boundary; x <= right_boundary; ++x) {
            road.at<uchar>(y, x) = 255;
        }
        // 绘制边界线（确保边界清晰）
        if (left_boundary > 0) road.at<uchar>(y, left_boundary) = 0;
        if (right_boundary < width) road.at<uchar>(y, right_boundary) = 0;
    }
    
    // 添加障碍物（但不完全阻塞道路）
    // 障碍物1：正前方
    cv::rectangle(road, cv::Point(width/2 - 40, height/2 - 40),
                  cv::Point(width/2 + 40, height/2 + 40), cv::Scalar(0), -1);
    
    // 障碍物2：左侧边缘
    cv::circle(road, cv::Point(left_boundary + 30, height/3), 30, cv::Scalar(0), -1);
    
    // 障碍物3：右侧边缘
    cv::circle(road, cv::Point(right_boundary - 30, height*2/3), 30, cv::Scalar(0), -1);
    
    return road;
}

