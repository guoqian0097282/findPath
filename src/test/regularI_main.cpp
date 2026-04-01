#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>
#include <chrono>
#include <regular_filter.h>

// ==================== 主函数 ====================

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << "Complete Centerline Extraction System" << std::endl;
    std::cout << "Multi-frame | Temporal Fitting | Start Alignment" << std::endl;
    std::cout << "1920x1080 | 0.1m/pixel | Full Vertical Coverage" << std::endl;
    std::cout << "============================================================\n" << std::endl;
    
    // 参数配置
    int width = 1920;
    int height = 1080;
    int num_frames = 1;
    double resolution = 0.01;
    bool debug = true;
    int max_history = 15;
    
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Image size: " << width << "x" << height << " pixels" << std::endl;
    std::cout << "  Resolution: " << resolution << " m/pixel" << std::endl;
    std::cout << "  Physical range: " << width * resolution << "m x " << height * resolution << "m" << std::endl;
    std::cout << "  Frames: " << num_frames << std::endl;
    std::cout << "  History size: " << max_history << std::endl;
    
    // 生成测试序列
    std::cout << "\nGenerating test sequence..." << std::endl;
    // auto sequence = generateTestSequence(num_frames, width, height);
    std::vector<cv::Mat> sequence;
    for(int i=1;i<5;i++)
    {
        cv::Mat image = cv::imread("/home/gq/guoqian/findPath/test_image/"+std::to_string(i)+".jpg");
        if (image.channels() == 3)
        {
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        }
        threshold(image, image, 120, 255, 0);
        for (int i = 960 + 300; i < 1920; i++)
        {
            for (int j = 0; j < 1080; j++)
            {
                image.at<uchar>(j, i) = 0;
            }
        }
        for (int i = 0; i < 960 - 300; i++)
        {
            for (int j = 0; j < 1080; j++)
            {
                image.at<uchar>(j, i) = 0;
            }
        }

        sequence.push_back(image);
    }
    std::cout << "  Generated " << sequence.size() << " frames" << std::endl;
    
    // 车辆位置（图像底部中央）
    cv::Point2d vehicle_pos(width / 2.0 * resolution, (height - 80) * resolution);
    std::cout << "  Vehicle position: (" << vehicle_pos.x << ", " << vehicle_pos.y << ") m" << std::endl;
    std::cout << "  Expected path length: ~" << (height - 80) * resolution << " m\n" << std::endl;
    
    // 创建处理器
    CenterlineExtractor extractor(resolution, debug);
    TemporalProcessor temporal(max_history);
    StartPointAligner aligner;
    CenterCorrector corrector;
    Visualizer visualizer(resolution, false);
    
    extractor.setVehiclePosition(vehicle_pos);
    aligner.setVehiclePosition(vehicle_pos);
    corrector.setVehiclePosition(vehicle_pos);
    
    // 存储结果
    std::vector<FrameData> results;
    std::vector<double> processing_times;
    std::vector<double> lateral_offsets;
    
    std::cout << "Processing frames..." << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    auto total_start = std::chrono::high_resolution_clock::now();
    
    for (int frame_id = 0; frame_id < num_frames; ++frame_id) {
        double timestamp = frame_id * 0.1;
        
        auto frame_start = std::chrono::high_resolution_clock::now();
        
        // 1. 提取中心线
        auto points = extractor.extract(sequence[frame_id]);
        
        // 2. 校正偏左/偏右
        points = corrector.correct(points);
        
        // 3. 对齐起点
        points = aligner.align(points);
        points = aligner.ensureExact(points);
        
        // 4. 最终平滑
        if (points.size() > 5) {
            points = SmoothingFilter::gaussianSmooth(points, 0.8);
        }
        
        // 5. 创建帧数据
        FrameData frame;
        frame.frame_id = frame_id;
        frame.timestamp = timestamp;
        frame.points = points;
        frame.image = sequence[frame_id].clone();
        
        // 6. 计算统计信息
        frame.path_length = 0;
        frame.avg_curvature = 0;
        frame.avg_width = 0;
        
        for (size_t i = 1; i < points.size(); ++i) {
            frame.path_length += cv::norm(points[i].position - points[i-1].position);
        }
        
        for (const auto& p : points) {
            frame.avg_curvature += std::abs(p.curvature);
            frame.avg_width += p.road_width;
        }
        if (!points.empty()) {
            frame.avg_curvature /= points.size();
            frame.avg_width /= points.size();
        }
        
        frame.lateral_offset = points.empty() ? 0 : points[0].position.x - vehicle_pos.x;
        
        // 7. 时序融合
        FrameData fused = temporal.process(frame);
        
        auto frame_end = std::chrono::high_resolution_clock::now();
        double frame_time = std::chrono::duration<double, std::milli>(frame_end - frame_start).count();
        processing_times.push_back(frame_time);
        
        results.push_back(fused);
        lateral_offsets.push_back(fused.lateral_offset);
        
        // 输出统计
        if (frame_id % 5 == 0 || frame_id == num_frames - 1) {
            std::cout << "  Frame " << frame_id << ": " 
                      << fused.points.size() << " points, "
                      << "length=" << std::to_string(fused.path_length).substr(0, 6) << "m, "
                      << "offset=" << std::to_string(fused.lateral_offset).substr(0, 6) << "m, "
                      << "width=" << std::to_string(fused.avg_width).substr(0, 5) << "m, "
                      << "time=" << (int)frame_time << "ms" << std::endl;
        }
        
        // 可视化
        if (frame_id % 3 == 0) {
            visualizer.show(sequence[frame_id], fused.points, vehicle_pos, frame_id, fused.lateral_offset);
        }
    }
    
    auto total_end = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double, std::milli>(total_end - total_start).count();
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Processing completed!" << std::endl;
    std::cout << "Total time: " << total_time / 1000.0 << " s" << std::endl;
    std::cout << "Average per frame: " << total_time / num_frames << " ms" << std::endl;
    std::cout << "FPS: " << 1000.0 / (total_time / num_frames) << std::endl;
    
    // ==================== 统计结果 ====================
    
    std::cout << "\n========== Statistics ==========" << std::endl;
    
    double avg_points = 0, avg_length = 0, avg_offset = 0, avg_width = 0;
    int valid_frames = 0;
    
    for (const auto& r : results) {
        if (!r.points.empty()) {
            avg_points += r.points.size();
            avg_length += r.path_length;
            avg_offset += r.lateral_offset;
            avg_width += r.avg_width;
            valid_frames++;
        }
    }
    
    if (valid_frames > 0) {
        avg_points /= valid_frames;
        avg_length /= valid_frames;
        avg_offset /= valid_frames;
        avg_width /= valid_frames;
    }
    
    std::cout << "Valid frames: " << valid_frames << "/" << num_frames << std::endl;
    std::cout << "Average points per frame: " << avg_points << std::endl;
    std::cout << "Average path length: " << avg_length << " m" << std::endl;
    std::cout << "Average road width: " << avg_width << " m" << std::endl;
    std::cout << "Average lateral offset: " << avg_offset << " m" << std::endl;
    
    if (std::abs(avg_offset) < 0.1) {
        std::cout << "✓ Centerline is well centered (error < 10cm)" << std::endl;
    } else if (std::abs(avg_offset) < 0.2) {
        std::cout << "○ Centerline is acceptable (error < 20cm)" << std::endl;
    } else {
        std::cout << "⚠ Centerline bias: " << avg_offset << " m" << std::endl;
    }
    
    // 纵向覆盖验证
    double expected_length = (height - 80) * resolution;
    double coverage = (avg_length / expected_length) * 100;
    std::cout << "\nVertical Coverage:" << std::endl;
    std::cout << "  Expected: " << expected_length << " m" << std::endl;
    std::cout << "  Actual: " << avg_length << " m" << std::endl;
    std::cout << "  Coverage: " << coverage << "%" << std::endl;
    
    if (coverage > 80) {
        std::cout << "  ✓ Excellent vertical coverage" << std::endl;
    } else if (coverage > 60) {
        std::cout << "  ○ Good vertical coverage" << std::endl;
    } else {
        std::cout << "  ⚠ Limited vertical coverage" << std::endl;
    }
    
    std::cout << "================================" << std::endl;
    
    // ==================== 保存结果 ====================
    
    std::cout << "\nSaving results..." << std::endl;
    
    // 保存CSV
    std::ofstream csv("centerline_results_complete.csv");
    if (csv.is_open()) {
        csv << "Frame,Timestamp,PointIndex,X,Y,Curvature,Heading,Confidence,RoadWidth,LateralOffset\n";
        for (const auto& frame : results) {
            for (size_t i = 0; i < frame.points.size(); ++i) {
                const auto& p = frame.points[i];
                csv << frame.frame_id << ","
                    << frame.timestamp << ","
                    << i << ","
                    << p.position.x << ","
                    << p.position.y << ","
                    << p.curvature << ","
                    << p.heading << ","
                    << p.confidence << ","
                    << p.road_width << ","
                    << frame.lateral_offset << "\n";
            }
        }
        csv.close();
        std::cout << "  Saved to: centerline_results_complete.csv" << std::endl;
    }
    
    // 保存统计报告
    std::ofstream report("analysis_report.txt");
    if (report.is_open()) {
        report << "Centerline Extraction Analysis Report\n";
        report << "====================================\n\n";
        report << "Configuration:\n";
        report << "  Image size: " << width << "x" << height << " pixels\n";
        report << "  Resolution: " << resolution << " m/pixel\n";
        report << "  Frames processed: " << num_frames << "\n";
        report << "  Valid frames: " << valid_frames << "\n";
        report << "  History size: " << max_history << "\n\n";
        
        report << "Performance:\n";
        report << "  Total time: " << total_time / 1000.0 << " s\n";
        report << "  Average per frame: " << total_time / num_frames << " ms\n";
        report << "  FPS: " << 1000.0 / (total_time / num_frames) << "\n\n";
        
        report << "Quality Metrics:\n";
        report << "  Average points per frame: " << avg_points << "\n";
        report << "  Average path length: " << avg_length << " m\n";
        report << "  Average road width: " << avg_width << " m\n";
        report << "  Average lateral offset: " << avg_offset << " m\n";
        report << "  Vertical coverage: " << coverage << "%\n";
        
        report.close();
        std::cout << "  Saved to: analysis_report.txt" << std::endl;
    }
    
    // 创建最终可视化
    std::cout << "\nCreating final visualization..." << std::endl;
    
    cv::Mat final_viz = cv::Mat::zeros(height, width * 2, CV_8UC3);
    
    if (results.size() >= 2) {
        // 第一帧
        cv::Mat roi1 = final_viz(cv::Rect(0, 0, width, height));
        cv::cvtColor(sequence[0], roi1, cv::COLOR_GRAY2BGR);
        
        const auto& r1 = results[0];
        if (!r1.points.empty()) {
            for (size_t j = 1; j < r1.points.size(); ++j) {
                cv::Point p1(r1.points[j-1].position.x / resolution, 
                            r1.points[j-1].position.y / resolution);
                cv::Point p2(r1.points[j].position.x / resolution, 
                            r1.points[j].position.y / resolution);
                if (p1.x >= 0 && p1.x < width && p1.y >= 0 && p1.y < height &&
                    p2.x >= 0 && p2.x < width && p2.y >= 0 && p2.y < height) {
                    cv::line(roi1, p1, p2, cv::Scalar(0, 255, 0), 2);
                }
            }
        }
        cv::circle(roi1, cv::Point(vehicle_pos.x / resolution, vehicle_pos.y / resolution), 
                   10, cv::Scalar(0, 255, 255), -1);
        cv::putText(roi1, "Frame 0 | Length: " + std::to_string(r1.path_length).substr(0, 5) + "m", 
                    cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        
        // 最后一帧
        cv::Mat roi2 = final_viz(cv::Rect(width, 0, width, height));
        cv::cvtColor(sequence[num_frames-1], roi2, cv::COLOR_GRAY2BGR);
        
        const auto& r_last = results.back();
        if (!r_last.points.empty()) {
            for (size_t j = 1; j < r_last.points.size(); ++j) {
                cv::Point p1(r_last.points[j-1].position.x / resolution, 
                            r_last.points[j-1].position.y / resolution);
                cv::Point p2(r_last.points[j].position.x / resolution, 
                            r_last.points[j].position.y / resolution);
                if (p1.x >= 0 && p1.x < width && p1.y >= 0 && p1.y < height &&
                    p2.x >= 0 && p2.x < width && p2.y >= 0 && p2.y < height) {
                    cv::line(roi2, p1, p2, cv::Scalar(0, 255, 0), 2);
                }
            }
        }
        cv::circle(roi2, cv::Point(vehicle_pos.x / resolution, vehicle_pos.y / resolution), 
                   10, cv::Scalar(0, 255, 255), -1);
        cv::putText(roi2, "Frame " + std::to_string(num_frames-1) + " | Length: " + 
                    std::to_string(r_last.path_length).substr(0, 5) + "m", 
                    cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    }
    
    cv::imwrite("final_results_complete.png", final_viz);
    std::cout << "  Saved to: final_results_complete.png" << std::endl;
    
    // 绘制偏移量演化图
    std::cout << "\nCreating offset evolution plot..." << std::endl;
    cv::Mat plot = cv::Mat::zeros(400, 800, CV_8UC3);
    
    if (!lateral_offsets.empty()) {
        double max_offset = 0;
        for (double off : lateral_offsets) {
            max_offset = std::max(max_offset, std::abs(off));
        }
        max_offset = std::max(max_offset, 0.3);
        
        for (size_t i = 0; i < lateral_offsets.size(); ++i) {
            int x = i * 780 / lateral_offsets.size() + 10;
            int y = 200 - (lateral_offsets[i] / max_offset) * 180;
            y = std::max(20, std::min(380, y));
            
            cv::circle(plot, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
            
            if (i > 0) {
                int prev_x = (i-1) * 780 / lateral_offsets.size() + 10;
                int prev_y = 200 - (lateral_offsets[i-1] / max_offset) * 180;
                prev_y = std::max(20, std::min(380, prev_y));
                cv::line(plot, cv::Point(prev_x, prev_y), cv::Point(x, y), cv::Scalar(0, 255, 0), 1);
            }
        }
    }
    
    cv::line(plot, cv::Point(10, 200), cv::Point(790, 200), cv::Scalar(0, 0, 255), 1);
    cv::putText(plot, "Lateral Offset Evolution", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    cv::putText(plot, "Frame", cv::Point(380, 380), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    cv::putText(plot, "Offset (m)", cv::Point(10, 200), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    
    cv::imwrite("offset_evolution.png", plot);
    std::cout << "  Saved to: offset_evolution.png" << std::endl;
    
    std::cout << "\n============================================================" << std::endl;
    std::cout << "All tasks completed successfully!" << std::endl;
    std::cout << "\nGenerated files:" << std::endl;
    std::cout << "  - centerline_results_complete.csv (centerline data)" << std::endl;
    std::cout << "  - analysis_report.txt (performance report)" << std::endl;
    std::cout << "  - final_results_complete.png (visualization)" << std::endl;
    std::cout << "  - offset_evolution.png (offset chart)" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "\nPress any key to exit..." << std::endl;
    
    cv::waitKey(0);
    
    return 0;
}