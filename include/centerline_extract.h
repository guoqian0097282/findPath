#ifndef CENTERLINE_EXTRACTOR_H
#define CENTERLINE_EXTRACTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <chrono>

// ==================== 数据结构 ====================

struct CenterlinePoint {
    cv::Point2d position;      // 位置 (x, y) 单位：米
    double curvature;          // 曲率 (rad/m)
    double heading;            // 航向角 (rad)
    double confidence;         // 置信度 (0-1)
    double road_width;         // 道路宽度 (米)
    double timestamp;          // 时间戳
    
    CenterlinePoint();
};

struct FrameData {
    int frame_id;
    double timestamp;
    std::vector<CenterlinePoint> points;
    double path_length;
    double avg_curvature;
    double avg_width;
    double lateral_offset;
    cv::Mat image;
};

// ==================== 平滑滤波器 ====================

class SmoothingFilter {
public:
    static std::vector<CenterlinePoint> movingAverage(const std::vector<CenterlinePoint>& points, int window = 5);
    static std::vector<CenterlinePoint> gaussianSmooth(const std::vector<CenterlinePoint>& points, double sigma = 1.0);
};

// ==================== 中心线提取器 ====================

class CenterlineExtractor {
private:
    double resolution_;
    cv::Point2d vehicle_pos_;
    bool debug_;
    
public:
    CenterlineExtractor(double resolution = 0.1, bool debug = false);
    void setVehiclePosition(const cv::Point2d& pos);
    std::vector<CenterlinePoint> extract(const cv::Mat& occupancy_grid);
};

// ==================== 时序处理器 ====================

class TemporalProcessor {
private:
    std::deque<FrameData> history_;
    int max_history_;
    double calculatePathLength(const std::vector<CenterlinePoint>& points);
    
public:
    TemporalProcessor(int max_history = 15);
    FrameData process(const FrameData& current);
    void clear();
};

// ==================== 起点对齐器 ====================

class StartPointAligner {
private:
    cv::Point2d vehicle_pos_;
    
public:
    StartPointAligner();
    void setVehiclePosition(const cv::Point2d& pos);
    std::vector<CenterlinePoint> align(const std::vector<CenterlinePoint>& points);
    std::vector<CenterlinePoint> ensureExact(const std::vector<CenterlinePoint>& points);
};

// ==================== 中心校正器 ====================

class CenterCorrector {
private:
    cv::Point2d vehicle_pos_;
    std::deque<double> offset_history_;
    
public:
    CenterCorrector();
    void setVehiclePosition(const cv::Point2d& pos);
    double calculateOffset(const std::vector<CenterlinePoint>& points);
    std::vector<CenterlinePoint> correct(const std::vector<CenterlinePoint>& points);
    void reset();
};

// ==================== 可视化器 ====================

class Visualizer {
private:
    double resolution_;
    bool save_images_;
    double calculateLength(const std::vector<CenterlinePoint>& points);
    
public:
    Visualizer(double resolution = 0.1, bool save_images = false);
    void show(const cv::Mat& image, const std::vector<CenterlinePoint>& points,
              const cv::Point2d& vehicle_pos, int frame_id, double offset);
};

// ==================== 测试数据生成 ====================

std::vector<cv::Mat> generateTestSequence(int num_frames, int width, int height);

#endif // CENTERLINE_EXTRACTOR_H