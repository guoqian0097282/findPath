#include <many_houghFilter.h>
#include <iostream>
#include <chrono>

int main() {
    std::cout << "=== Temporal Centerline Fitter - Multi-Frame Processing ===" << std::endl;
    
    int width = 800;
    int height = 600;
    int num_frames = 60;  // 处理30帧
    
    // 生成测试图像序列
    std::cout << "Generating test sequence with " << num_frames << " frames..." << std::endl;
    auto sequence = generateTestSequence(num_frames, width, height);
    std::cout << "Sequence generated successfully!" << std::endl;
    
    // 创建时序拟合器
    TemporalCenterlineFitter fitter(10, 0.05, true);   // 保留10帧历史，启用调试
    
    // 车辆位置
    double vehicle_x = width / 2.0 * 0.05;
    double vehicle_y = (height - 50) * 0.05;
    fitter.setVehiclePosition(vehicle_x, vehicle_y);
    
    // 处理每一帧
    std::cout << "\nProcessing frames..." << std::endl;
    std::cout << "========================================" << std::endl;
    
    for (int frame_id = 0; frame_id < num_frames; ++frame_id) {
        double timestamp = frame_id * 0.1;  // 100ms间隔
        
        std::cout << "\nFrame " << frame_id << " (t=" << timestamp << "s):" << std::endl;
        
        // 处理当前帧
        CenterlineFrame result = fitter.processFrame(sequence[frame_id], frame_id, timestamp);
        
        // 输出统计信息
        if (!result.points.empty()) {
            std::cout << "  Points: " << result.points.size() << std::endl;
            std::cout << "  Length: " << result.length << " m" << std::endl;
            std::cout << "  Curvature: " << result.curvature << " rad" << std::endl;
            
            // 显示第一个和最后一个点
            std::cout << "  First point: (" << result.points[0].x 
                      << ", " << result.points[0].y << ")" << std::endl;
            std::cout << "  Last point: (" << result.points.back().x 
                      << ", " << result.points.back().y << ")" << std::endl;
        } else {
            std::cout << "  No points extracted!" << std::endl;
        }
        
        // 每5帧保存一次结果
        if (frame_id % 5 == 0 || frame_id == num_frames - 1) {
            cv::Mat display;
            cv::cvtColor(sequence[frame_id], display, cv::COLOR_GRAY2BGR);
            
            // 绘制融合后的中心线
            auto latest = fitter.getLatestCenterline();
            if (!latest.points.empty()) {
                for (size_t i = 1; i < latest.points.size(); ++i) {
                    cv::Point p1(latest.points[i-1].x / 0.05, 
                                latest.points[i-1].y / 0.05);
                    cv::Point p2(latest.points[i].x / 0.05, 
                                latest.points[i].y / 0.05);
                    if (p1.x >= 0 && p1.x < width && p1.y >= 0 && p1.y < height &&
                        p2.x >= 0 && p2.x < width && p2.y >= 0 && p2.y < height) {
                        cv::line(display, p1, p2, cv::Scalar(0, 255, 0), 3);
                    }
                }
            }
            
            // 绘制车辆位置
            cv::circle(display, cv::Point(width/2, height-50), 10, 
                      cv::Scalar(0, 255, 255), -1);
            
            // 添加文字
            std::string text = "Frame " + std::to_string(frame_id);
            cv::putText(display, text, cv::Point(20, 50), 
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            
            cv::imshow("Current Frame", display);
            cv::waitKey(1000);  // 每帧显示100ms
        }
    }
    
    // 保存历史数据
    fitter.saveHistoryToFile("centerline_history.txt");
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "Processing completed!" << std::endl;
    std::cout << "History saved to centerline_history.txt" << std::endl;
    std::cout << "Press any key to exit..." << std::endl;
    cv::waitKey(0);
    
    return 0;
}