#include <avoidObstacle_filter.h>
#include <iostream>
#include <chrono>

int main() {
    std::cout << "=== Practical Centerline Extractor ===" << std::endl;
    
    int width = 800;
    int height = 600;
    
    // 创建测试道路
    cv::Mat road = createClearRoad(width, height);
    
    std::cout << "\nProcessing road with obstacles..." << std::endl;
    std::cout << "========================================" << std::endl;
    
    PracticalCenterlineExtractor extractor(0.05);
    extractor.setOccupancyGrid(road);
    
    double vehicle_x = width / 2.0 * 0.05;
    double vehicle_y = (height - 50) * 0.05;
    extractor.setVehiclePosition(vehicle_x, vehicle_y);
    
    auto centerline = extractor.extractCenterLine();
    
    cv::Mat result = extractor.drawResult(centerline);
    
    cv::putText(result, "Practical Centerline Extraction", cv::Point(20, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(result, "Green: Center Line", cv::Point(20, 90), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1.5);
    cv::putText(result, "Yellow: Vehicle", cv::Point(20, 115), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1.5);
    
    if (!centerline.empty()) {
        std::cout << "\n✓ SUCCESS!" << std::endl;
        std::cout << "  Path points: " << centerline.size() << std::endl;
        
        double length = 0;
        for (size_t i = 1; i < centerline.size(); ++i) {
            length += cv::norm(centerline[i] - centerline[i-1]);
        }
        std::cout << "  Path length: " << length << " meters" << std::endl;
        
        // 验证安全性（简单检查）
        int unsafe_count = 0;
        for (const auto& p : centerline) {
            int px = p.x / 0.05;
            int py = p.y / 0.05;
            if (px >= 0 && px < width && py >= 0 && py < height) {
                // 检查3x3区域
                bool safe = true;
                for (int dx = -2; dx <= 2; dx++) {
                    for (int dy = -2; dy <= 2; dy++) {
                        int nx = px + dx;
                        int ny = py + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            if (road.at<uchar>(ny, nx) == 0) {
                                safe = false;
                                break;
                            }
                        }
                    }
                    if (!safe) break;
                }
                if (!safe) unsafe_count++;
            }
        }
        
        if (unsafe_count == 0) {
            std::cout << "  ✓ Path appears safe (no direct obstacle contact)" << std::endl;
        } else {
            std::cout << "  ⚠ " << unsafe_count << " points near obstacles" << std::endl;
        }
        
        // 显示路径范围
        double min_x = centerline[0].x, max_x = centerline[0].x;
        for (const auto& p : centerline) {
            min_x = std::min(min_x, p.x);
            max_x = std::max(max_x, p.x);
        }
        std::cout << "  X range: [" << min_x << ", " << max_x << "] m" << std::endl;
        
    } else {
        std::cout << "\n✗ FAILED: No path found" << std::endl;
    }
    
    cv::imshow("Practical Centerline", result);
    cv::imwrite("practical_centerline_result.png", result);
    
    std::cout << "\nResult saved to: practical_centerline_result.png" << std::endl;
    std::cout << "Press any key to exit..." << std::endl;
    cv::waitKey(0);
    
    return 0;
}