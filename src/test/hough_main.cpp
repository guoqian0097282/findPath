#include <hough_filter.h>
#include <iostream>
#include <chrono>

int main() {
    std::cout << "=== Fixed Centerline Extractor ===" << std::endl;
    
    // 创建测试道路
    int width = 800;
    int height = 600;
    cv::Mat road = cv::Mat::zeros(height, width, CV_8UC1);
    
    // 定义道路边界（像素坐标）
    int left_boundary = 240;
    int right_boundary = 560;
    
    // 填充道路区域
    for (int y = 0; y < height; ++y) {
        for (int x = left_boundary; x <= right_boundary; ++x) {
            road.at<uchar>(y, x) = 255;
        }
    }
    
    // 绘制明显的边界线
    for (int y = 0; y < height; ++y) {
        road.at<uchar>(y, left_boundary) = 0;
        road.at<uchar>(y, right_boundary) = 0;
    }
    
    // 添加车道线
    for (int y = 0; y < height; y += 30) {
        int center = (left_boundary + right_boundary) / 2;
        cv::line(road, cv::Point(center - 20, y), 
                 cv::Point(center + 20, y), cv::Scalar(200), 2);
    }
    
    std::cout << "Road created: " << width << "x" << height << std::endl;
    std::cout << "Left boundary: " << left_boundary << ", Right boundary: " << right_boundary << std::endl;
    std::cout << "Road width: " << (right_boundary - left_boundary) << " pixels" << std::endl;
    
    // 创建提取器
    FixedCenterlineExtractor extractor(0.05);
    extractor.setOccupancyGrid(road);
    
    // 设置车辆位置（世界坐标）
    double vehicle_x = (left_boundary + right_boundary) / 2.0 * 0.05;  // 道路中心
    double vehicle_y = (height - 50) * 0.05;  // 距离底部50像素
    
    std::cout << "\nVehicle position (world): (" << vehicle_x << ", " << vehicle_y << ")" << std::endl;
    
    extractor.setVehiclePosition(vehicle_x, vehicle_y);
    
    // 提取中心线
    auto centerline = extractor.extractCenterLine();
    
    // 绘制结果
    cv::Mat result = extractor.drawResult(centerline);
    
    // 添加说明文字
    cv::putText(result, "Green: Center Line", cv::Point(20, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1.5);
    cv::putText(result, "Blue: Left Boundary", cv::Point(20, 90), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 1.5);
    cv::putText(result, "Red: Right Boundary", cv::Point(20, 120), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1.5);
    cv::putText(result, "Yellow: Vehicle", cv::Point(20, 150), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1.5);
    
    // 显示结果
    cv::imshow("Centerline Extraction - Fixed Version", result);
    cv::imwrite("centerline_fixed.png", result);
    
    // 输出结果
    if (!centerline.empty()) {
        std::cout << "\n✓ SUCCESS! Centerline extracted successfully!" << std::endl;
        std::cout << "Number of points: " << centerline.size() << std::endl;
        
        // 验证中心线位置
        double expected_center_x = (left_boundary + right_boundary) / 2.0 * 0.05;
        double actual_center_x = centerline[centerline.size()/2].x;
        
        std::cout << "\nVerification:" << std::endl;
        std::cout << "  Expected center X: " << expected_center_x << " m" << std::endl;
        std::cout << "  Actual center X:   " << actual_center_x << " m" << std::endl;
        
        if (std::abs(actual_center_x - expected_center_x) < 0.01) {
            std::cout << "  ✓ Centerline position is accurate!" << std::endl;
        }
        
        // 显示前5个点
        std::cout << "\nFirst 5 centerline points (from farthest to nearest):" << std::endl;
        for (size_t i = 0; i < std::min(centerline.size(), size_t(5)); ++i) {
            std::cout << "  Point " << i << ": (" << centerline[i].x 
                      << ", " << centerline[i].y << ")" << std::endl;
        }
        
        // 显示最后5个点
        std::cout << "\nLast 5 centerline points:" << std::endl;
        for (size_t i = centerline.size() - 5; i < centerline.size(); ++i) {
            std::cout << "  Point " << i << ": (" << centerline[i].x 
                      << ", " << centerline[i].y << ")" << std::endl;
        }
        
        // 计算中心线的长度
        double total_length = 0;
        for (size_t i = 1; i < centerline.size(); ++i) {
            double dx = centerline[i].x - centerline[i-1].x;
            double dy = centerline[i].y - centerline[i-1].y;
            total_length += std::sqrt(dx*dx + dy*dy);
        }
        std::cout << "\nCenterline length: " << total_length << " m" << std::endl;
        
    } else {
        std::cout << "\n✗ FAILED: No centerline extracted!" << std::endl;
        std::cout << "Please check the road map and vehicle position." << std::endl;
    }
    
    std::cout << "\nResult saved to: centerline_fixed.png" << std::endl;
    std::cout << "Press any key to exit..." << std::endl;
    cv::waitKey(0);
    
    return 0;
}