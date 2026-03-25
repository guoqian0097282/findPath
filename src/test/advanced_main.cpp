#include <advance_filter.h>
#include <iostream>
#include <chrono>

int main() {
    std::cout << "=== Robust Centerline Extractor for Complex Scenarios ===" << std::endl;
    
    int width = 800;
    int height = 600;
    
    // 创建测试场景
    std::vector<std::pair<std::string, cv::Mat>> scenarios = {
        {"Curved Road", createCurvedRoad(width, height)},
        {"Intersection", createIntersection(width, height)},
        {"Road with Obstacles", createObstacleRoad(width, height)},
        {"Variable Width Road", createVariableWidthRoad(width, height)}
    };
    
    for (const auto& [name, test_road] : scenarios) {
        std::cout << "\n========================================" << std::endl;
        std::cout << "Processing: " << name << std::endl;
        std::cout << "========================================" << std::endl;
        
        // 创建提取器
        RobustCenterlineExtractor extractor(0.05, false);
        extractor.setOccupancyGrid(test_road);
        
        // 车辆位置在图像底部中央
        double vehicle_x = width / 2.0 * 0.05;
        double vehicle_y = (height - 50) * 0.05;
        extractor.setVehiclePosition(vehicle_x, vehicle_y);
        
        // 提取中心线
        auto centerline = extractor.extractCenterLine();
        
        // 绘制结果
        cv::Mat result = extractor.drawResult(centerline);
        
        // 添加标题
        cv::putText(result, name, cv::Point(20, 60), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        
        //显示统计信息
        if (!centerline.empty()) {
            std::cout << " SUCCESS!" << std::endl;
            std::cout << " Points: " << centerline.size() << std::endl;
            
            // 计算路径长度
            double length = 0;
            for (size_t i = 1; i < centerline.size(); ++i) {
                length += cv::norm(centerline[i] - centerline[i-1]);
            }
            std::cout << "  Path length: " << length << " meters" << std::endl;
            
            // 显示路径点范围
            double min_x = centerline[0].x, max_x = centerline[0].x;
            double min_y = centerline[0].y, max_y = centerline[0].y;
            for (const auto& p : centerline) {
                min_x = std::min(min_x, p.x);
                max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y);
                max_y = std::max(max_y, p.y);
            }
            std::cout << "  X range: [" << min_x << ", " << max_x << "] m" << std::endl;
            std::cout << "  Y range: [" << min_y << ", " << max_y << "] m" << std::endl;
        } else {
            std::cout << " FAILED: No centerline extracted" << std::endl;
        }
        
        // 显示图像
        cv::imshow(name, result);
        cv::waitKey(1500);  // 显示1.5秒
        
        // 保存结果
        std::string filename = name + "_result.png";
        cv::imwrite(filename, result);
        std::cout << "  Saved to: " << filename << std::endl;
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "All tests completed!" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Press any key to exit..." << std::endl;
    cv::waitKey(0);
    
    return 0;
}