#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

class FixedCenterlineExtractor {
private:
    cv::Mat occupancy_grid_;
    double resolution_;
    double vehicle_x_;
    double vehicle_y_;
    
public:
    FixedCenterlineExtractor(double resolution = 0.05) 
        : resolution_(resolution), vehicle_x_(0), vehicle_y_(0) {}
    
    void setOccupancyGrid(const cv::Mat& grid) {
        occupancy_grid_ = grid.clone();
    }
    
    void setVehiclePosition(double x, double y) {
        vehicle_x_ = x;
        vehicle_y_ = y;
    }
    
    /**
     * 修正版中心线提取
     */
    std::vector<cv::Point2d> extractCenterLine() {
        std::vector<cv::Point2d> centerline;
        
        if (occupancy_grid_.empty()) {
            std::cerr << "Error: Occupancy grid is empty!" << std::endl;
            return centerline;
        }
        
        // 直接扫描找到左右边界
        auto [left_boundary, right_boundary] = findBoundariesByScanning();
        
        if (left_boundary.empty() || right_boundary.empty()) {
            std::cerr << "Failed to find boundaries by scanning" << std::endl;
            return centerline;
        }
        
        // 生成中心线
        centerline = generateCenterline(left_boundary, right_boundary);
        
        return centerline;
    }
    
private:
    /**
     * 通过扫描找到左右边界
     */
    std::pair<std::vector<cv::Point>, std::vector<cv::Point>> findBoundariesByScanning() {
        std::vector<cv::Point> left_boundary;
        std::vector<cv::Point> right_boundary;
        
        int height = occupancy_grid_.rows;
        int width = occupancy_grid_.cols;
        
        // 车辆位置（像素坐标）
        int vehicle_px = (int)(vehicle_x_ / resolution_);
        int vehicle_py = (int)(vehicle_y_ / resolution_);
        
        // 确保车辆位置在图像范围内
        vehicle_px = std::max(0, std::min(width - 1, vehicle_px));
        vehicle_py = std::max(0, std::min(height - 1, vehicle_py));
        
        std::cout << "Vehicle position (pixel): (" << vehicle_px << ", " << vehicle_py << ")" << std::endl;
        
        // 从车辆位置向上扫描
        int scan_start_y = std::max(0, vehicle_py - 300);  // 向上扫描300像素
        int scan_end_y = vehicle_py;
        
        std::cout << "Scanning y from " << scan_start_y << " to " << scan_end_y << std::endl;
        
        for (int y = scan_end_y; y >= scan_start_y; y -= 5) {  // 每5像素扫描一行
            int left_x = -1;
            int right_x = -1;
            
            // 向左找边界
            for (int x = vehicle_px; x >= 0; x--) {
                if (x >= width) continue;
                if (occupancy_grid_.at<uchar>(y, x) == 0) {
                    left_x = x + 1;
                    break;
                }
            }
            
            // 如果没找到左边界，使用默认值
            if (left_x == -1) {
                left_x = 0;
            }
            
            // 向右找边界
            for (int x = vehicle_px; x < width; x++) {
                if (occupancy_grid_.at<uchar>(y, x) == 0) {
                    right_x = x - 1;
                    break;
                }
            }
            
            // 如果没找到右边界，使用默认值
            if (right_x == -1) {
                right_x = width - 1;
            }
            
            // 确保左右边界有效
            if (left_x < right_x && left_x >= 0 && right_x < width) {
                left_boundary.push_back(cv::Point(left_x, y));
                right_boundary.push_back(cv::Point(right_x, y));
            }
        }
        
        std::cout << "Found " << left_boundary.size() << " left boundary points" << std::endl;
        std::cout << "Found " << right_boundary.size() << " right boundary points" << std::endl;
        
        return {left_boundary, right_boundary};
    }
    
    /**
     * 生成中心线
     */
    std::vector<cv::Point2d> generateCenterline(const std::vector<cv::Point>& left, 
                                                 const std::vector<cv::Point>& right) {
        std::vector<cv::Point2d> centerline;
        
        if (left.empty() || right.empty()) {
            return centerline;
        }
        
        // 确保点数相同
        size_t num_points = std::min(left.size(), right.size());
        
        for (size_t i = 0; i < num_points; i++) {
            // 计算中点（像素坐标）
            double center_x = (left[i].x + right[i].x) / 2.0;
            double center_y = left[i].y;
            
            // 转换到世界坐标
            cv::Point2d world_point(center_x * resolution_, center_y * resolution_);
            centerline.push_back(world_point);
        }
        
        // 按y坐标排序（从上到下，即从远到近）
        std::sort(centerline.begin(), centerline.end(),
                  [](const cv::Point2d& a, const cv::Point2d& b) {
                      return a.y > b.y;  // 从远到近排序
                  });
        
        std::cout << "Generated " << centerline.size() << " centerline points" << std::endl;
        
        return centerline;
    }
    
public:
    /**
     * 绘制结果
     */
    cv::Mat drawResult(const std::vector<cv::Point2d>& centerline) {
        cv::Mat result;
        cv::cvtColor(occupancy_grid_, result, cv::COLOR_GRAY2BGR);
        
        // 绘制左右边界（可视化用）
        auto [left, right] = findBoundariesByScanning();
        
        // 绘制左边界（蓝色）
        for (size_t i = 1; i < left.size(); ++i) {
            cv::line(result, left[i-1], left[i], cv::Scalar(255, 0, 0), 1);
        }
        
        // 绘制右边界（红色）
        for (size_t i = 1; i < right.size(); ++i) {
            cv::line(result, right[i-1], right[i], cv::Scalar(0, 0, 255), 1);
        }
        
        // 绘制中心线（绿色）
        if (!centerline.empty()) {
            std::vector<cv::Point> pixel_points;
            for (const auto& point : centerline) {
                pixel_points.push_back(cv::Point(point.x / resolution_, point.y / resolution_));
            }
            
            for (size_t i = 1; i < pixel_points.size(); ++i) {
                cv::line(result, pixel_points[i-1], pixel_points[i], cv::Scalar(0, 255, 0), 3);
            }
            
            // 绘制中心点
            for (const auto& point : pixel_points) {
                cv::circle(result, point, 2, cv::Scalar(0, 255, 0), -1);
            }
        }
        
        // 绘制车辆位置
        cv::circle(result, cv::Point(vehicle_x_ / resolution_, vehicle_y_ / resolution_), 
                   12, cv::Scalar(0, 255, 255), -1);
        
        // 添加说明
        std::string info = "Centerline points: " + std::to_string(centerline.size());
        cv::putText(result, info, cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1.5);
        
        return result;
    }
};

