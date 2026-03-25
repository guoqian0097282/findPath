#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>

class RobustCenterlineExtractor {
private:
    cv::Mat occupancy_grid_;
    cv::Mat cost_map_;
    double resolution_;
    double vehicle_x_;
    double vehicle_y_;
    bool debug_mode_;
    
public:
    RobustCenterlineExtractor(double resolution = 0.05, bool debug = false) 
        : resolution_(resolution), vehicle_x_(0), vehicle_y_(0), debug_mode_(debug) {}
    
    void setOccupancyGrid(const cv::Mat& grid) {
        occupancy_grid_ = grid.clone();
        buildCostMap();
    }
    
    void setVehiclePosition(double x, double y) {
        vehicle_x_ = x;
        vehicle_y_ = y;
    }
    
    /**
     * 稳健的中心线提取
     */
    std::vector<cv::Point2d> extractCenterLine() {
        if (occupancy_grid_.empty()) {
            std::cerr << "Error: Occupancy grid is empty!" << std::endl;
            return {};
        }
        
        // 1. 提取道路边界
        auto boundaries = extractBoundariesRobust();
        
        if (boundaries.left.empty() || boundaries.right.empty()) {
            std::cerr << "Warning: No boundaries found" << std::endl;
            return generateFallbackPath();
        }
        
        // 2. 生成中心路径
        auto center_path = generateCenterPath(boundaries);
        
        // 3. 平滑路径
        auto smoothed_path = smoothPath(center_path);
        
        // 4. 避障优化
        auto final_path = avoidObstacles(smoothed_path);
        
        return final_path;
    }
    
private:
    /**
     * 构建代价地图
     */
    void buildCostMap() {
        cost_map_ = cv::Mat::zeros(occupancy_grid_.size(), CV_32F);
        
        cv::Mat dist_transform;
        cv::distanceTransform(occupancy_grid_, dist_transform, cv::DIST_L2, 5);
        
        cv::normalize(dist_transform, cost_map_, 0, 1, cv::NORM_MINMAX);
        cost_map_ = 1.0 - cost_map_;
        
        cv::GaussianBlur(cost_map_, cost_map_, cv::Size(5, 5), 1);
    }
    
    /**
     * 稳健的边界提取
     */
    struct Boundaries {
        std::vector<cv::Point2d> left;
        std::vector<cv::Point2d> right;
        std::vector<double> widths;
    };
    
    Boundaries extractBoundariesRobust() {
        Boundaries boundaries;
        
        int height = occupancy_grid_.rows;
        int width = occupancy_grid_.cols;
        
        // 确保车辆位置在有效范围内
        int vehicle_px = std::max(0, std::min(width - 1, (int)(vehicle_x_ / resolution_)));
        int vehicle_py = std::max(0, std::min(height - 1, (int)(vehicle_y_ / resolution_)));
        
        std::cout << "Vehicle pixel: (" << vehicle_px << ", " << vehicle_py << ")" << std::endl;
        
        // 扫描范围
        int scan_start_y = std::max(0, vehicle_py - 300);
        int scan_end_y = vehicle_py;
        int scan_step = 5;
        
        // 使用滑动窗口跟踪边界
        int last_left_x = vehicle_px;
        int last_right_x = vehicle_px;
        int window_width = 100;
        
        for (int y = scan_end_y; y >= scan_start_y; y -= scan_step) {
            // 动态搜索范围
            int search_left_start = std::max(0, last_left_x - window_width);
            int search_left_end = last_left_x;
            int search_right_start = last_right_x;
            int search_right_end = std::min(width - 1, last_right_x + window_width);
            
            // 找左边界
            int left_x = -1;
            for (int x = search_left_end; x >= search_left_start; x--) {
                if (x < 0 || x >= width) continue;
                if (occupancy_grid_.at<uchar>(y, x) == 0) {
                    left_x = x + 1;
                    break;
                }
            }
            
            // 如果没找到，使用默认值
            if (left_x == -1) {
                left_x = std::max(0, search_left_start);
            }
            
            // 找右边界
            int right_x = -1;
            for (int x = search_right_start; x <= search_right_end; x++) {
                if (x < 0 || x >= width) continue;
                if (occupancy_grid_.at<uchar>(y, x) == 0) {
                    right_x = x - 1;
                    break;
                }
            }
            
            // 如果没找到，使用默认值
            if (right_x == -1) {
                right_x = std::min(width - 1, search_right_end);
            }
            
            // 确保左右边界有效
            if (right_x > left_x && left_x >= 0 && right_x < width) {
                boundaries.left.push_back(cv::Point2d(left_x, y));
                boundaries.right.push_back(cv::Point2d(right_x, y));
                boundaries.widths.push_back(right_x - left_x);
                
                // 更新跟踪位置
                last_left_x = left_x;
                last_right_x = right_x;
            }
        }
        
        std::cout << "Found " << boundaries.left.size() << " boundary points" << std::endl;
        
        // 边界平滑
        if (boundaries.left.size() > 5) {
            boundaries.left = smoothBoundary(boundaries.left);
            boundaries.right = smoothBoundary(boundaries.right);
        }
        
        return boundaries;
    }
    
    /**
     * 平滑边界
     */
    std::vector<cv::Point2d> smoothBoundary(const std::vector<cv::Point2d>& points) {
        if (points.size() < 5) return points;
        
        std::vector<cv::Point2d> smoothed;
        int window = std::min(5, (int)points.size());
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (i < window/2 || i >= points.size() - window/2) {
                smoothed.push_back(points[i]);
                continue;
            }
            
            cv::Point2d sum(0, 0);
            int count = 0;
            for (int j = -window/2; j <= window/2; ++j) {
                sum += points[i + j];
                count++;
            }
            smoothed.push_back(sum / count);
        }
        
        return smoothed;
    }
    
    /**
     *生成中心路径
     */
    std::vector<cv::Point2d> generateCenterPath(const Boundaries& boundaries) {
        std::vector<cv::Point2d> center_path;
        
        size_t num_points = std::min(boundaries.left.size(), boundaries.right.size());
        
        for (size_t i = 0; i < num_points; ++i) {
            cv::Point2d center(
                (boundaries.left[i].x + boundaries.right[i].x) / 2.0,
                boundaries.left[i].y
            );
            center_path.push_back(center);
        }
        
        // 转换到世界坐标
        for (auto& p : center_path) {
            p.x *= resolution_;
            p.y *= resolution_;
        }
        
        return center_path;
    }
    
    /**
     *生成回退路径（当找不到边界时）
     */
    std::vector<cv::Point2d> generateFallbackPath() {
        std::vector<cv::Point2d> path;
        
        int height = occupancy_grid_.rows;
        int width = occupancy_grid_.cols;
        int vehicle_px = vehicle_x_ / resolution_;
        int vehicle_py = vehicle_y_ / resolution_;
        
        // 简单直线路径
        for (int y = vehicle_py; y >= std::max(0, vehicle_py - 300); y -= 10) {
            cv::Point2d point(vehicle_px * resolution_, y * resolution_);
            path.push_back(point);
        }
        
        return path;
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
            int count = 0;
            for (int j = -window/2; j <= window/2; ++j) {
                sum += path[i + j];
                count++;
            }
            smoothed.push_back(sum / count);
        }
        
        return smoothed;
    }
    
    /**
     *  避障优化
     */
    std::vector<cv::Point2d> avoidObstacles(const std::vector<cv::Point2d>& path) {
        if (path.empty()) return path;
        
        std::vector<cv::Point2d> optimized = path;
        
        //简单的避障：检查路径点是否在障碍物上

        for (size_t i = 0; i < optimized.size(); ++i) {
            int px = optimized[i].x / resolution_;
            int py = optimized[i].y / resolution_;
            
            if (px >= 0 && px < cost_map_.cols && py >= 0 && py < cost_map_.rows) {
                if (cost_map_.at<float>(py, px) > 0.8) {
                    // 路径点靠近障碍物，向左右搜索安全点
                    int vehicle_px = vehicle_x_ / resolution_;
                    int safe_x = vehicle_px;
                    
                    // 搜索安全位置
                    for (int offset = 1; offset < 50; ++offset) {
                        if (vehicle_px + offset < cost_map_.cols &&
                            cost_map_.at<float>(py, vehicle_px + offset) < 0.5) {
                            safe_x = vehicle_px + offset;
                            break;
                        }
                        if (vehicle_px - offset >= 0 &&
                            cost_map_.at<float>(py, vehicle_px - offset) < 0.5) {
                            safe_x = vehicle_px - offset;
                            break;
                        }
                    }
                    
                    optimized[i].x = safe_x * resolution_;
                }
            }
        }
        
        return optimized;
    }
    
public:
    /**
     * 绘制结果
     */
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
        std::string info = "Centerline points: " + std::to_string(centerline.size());
        cv::putText(result, info, cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1.5);
        
        return result;
    }
};

// 创建各种测试场景
cv::Mat createCurvedRoad(int width, int height) {
    cv::Mat road = cv::Mat::zeros(height, width, CV_8UC1);
    
    // 弯道（正弦曲线）
    for (int y = 0; y < height; ++y) {
        double t = y / (double)height;
        double offset = 80 * sin(t * CV_PI);
        
        int left_x = std::max(0, width/3 + (int)offset);
        int right_x = std::min(width-1, width*2/3 + (int)offset);
        
        for (int x = left_x; x <= right_x; ++x) {
            if (x >= 0 && x < width) {
                road.at<uchar>(y, x) = 255;
            }
        }
        
        // 绘制边界线
        if (left_x >= 0 && left_x < width)
            road.at<uchar>(y, left_x) = 0;
        if (right_x >= 0 && right_x < width)
            road.at<uchar>(y, right_x) = 0;
    }
    
    return road;
}

cv::Mat createIntersection(int width, int height) {
    cv::Mat road = cv::Mat::zeros(height, width, CV_8UC1);
    
    // 垂直道路
    for (int y = 0; y < height; ++y) {
        for (int x = width/3; x <= width*2/3; ++x) {
            road.at<uchar>(y, x) = 255;
        }
    }
    
    // 水平道路
    for (int y = height/3; y <= height*2/3; ++y) {
        for (int x = 0; x < width; ++x) {
            road.at<uchar>(y, x) = 255;
        }
    }
    
    // 绘制边界线
    for (int y = 0; y < height; ++y) {
        if (y < height/3 || y > height*2/3) {
            if (width/3 > 0) road.at<uchar>(y, width/3) = 0;
            if (width*2/3 < width) road.at<uchar>(y, width*2/3) = 0;
        }
    }
    
    for (int x = 0; x < width; ++x) {
        if (x < width/3 || x > width*2/3) {
            if (height/3 > 0) road.at<uchar>(height/3, x) = 0;
            if (height*2/3 < height) road.at<uchar>(height*2/3, x) = 0;
        }
    }
    
    return road;
}

cv::Mat createObstacleRoad(int width, int height) {
    cv::Mat road = cv::Mat::zeros(height, width, CV_8UC1);
    
    //道路
    for (int y = 0; y < height; ++y) {
        for (int x = width/3; x <= width*2/3; ++x) {
            road.at<uchar>(y, x) = 255;
        }
        road.at<uchar>(y, width/3) = 0;
        road.at<uchar>(y, width*2/3) = 0;
    }
    
    // 添加障碍物
    cv::rectangle(road, cv::Point(width/2 - 40, height/2 - 40),
                  cv::Point(width/2 + 40, height/2 + 40), cv::Scalar(0), -1);
    cv::rectangle(road, cv::Point(width/3 + 30, height/4),
                  cv::Point(width/3 + 80, height/3), cv::Scalar(0), -1);
    cv::circle(road, cv::Point(width*2/3 - 50, height*3/4), 35, cv::Scalar(0), -1);
    
    return road;
}

cv::Mat createVariableWidthRoad(int width, int height) {
    cv::Mat road = cv::Mat::zeros(height, width, CV_8UC1);
    
    for (int y = 0; y < height; ++y) {
        double t = y / (double)height;
        int left_x = width/3 - (int)(30 * sin(t * CV_PI));
        int right_x = width*2/3 + (int)(30 * sin(t * CV_PI));
        
        left_x = std::max(0, left_x);
        right_x = std::min(width-1, right_x);
        
        for (int x = left_x; x <= right_x; ++x) {
            road.at<uchar>(y, x) = 255;
        }
        
        road.at<uchar>(y, left_x) = 0;
        road.at<uchar>(y, right_x) = 0;
    }
    
    return road;
}

