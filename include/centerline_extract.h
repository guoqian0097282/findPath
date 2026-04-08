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
#include <memory>
#include <mutex>

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
    
    FrameData() : frame_id(0), timestamp(0), path_length(0), 
                  avg_curvature(0), avg_width(0), lateral_offset(0) {}
};

// ==================== 平滑滤波器（单例） ====================

class SmoothingFilter {
private:
    static std::unique_ptr<SmoothingFilter> instance_;
    static std::mutex mutex_;
    
    SmoothingFilter() = default;
    
public:
    ~SmoothingFilter() = default;  // 改为 public
    
    static SmoothingFilter* getInstance();
    
    std::vector<CenterlinePoint> movingAverage(const std::vector<CenterlinePoint>& points, int window = 5);
    std::vector<CenterlinePoint> gaussianSmooth(const std::vector<CenterlinePoint>& points, double sigma = 1.0);
    
    // 删除拷贝构造和赋值操作
    SmoothingFilter(const SmoothingFilter&) = delete;
    SmoothingFilter& operator=(const SmoothingFilter&) = delete;
};

// ==================== 中心线提取器（单例） ====================

class CenterlineExtractor {
private:
    static std::unique_ptr<CenterlineExtractor> instance_;
    static std::mutex mutex_;
    
    double resolution_;
    cv::Point2d vehicle_pos_;
    bool debug_;
    bool initialized_;
    
    CenterlineExtractor();
    
public:
    ~CenterlineExtractor() = default;
    
    static CenterlineExtractor* getInstance();
    
    bool initialize(double resolution = 0.1, bool debug = false);
    void setVehiclePosition(const cv::Point2d& pos);
    std::vector<CenterlinePoint> extract(const cv::Mat& occupancy_grid);
    bool isInitialized() const { return initialized_; }
    
    CenterlineExtractor(const CenterlineExtractor&) = delete;
    CenterlineExtractor& operator=(const CenterlineExtractor&) = delete;
};

// ==================== 时序处理器（单例） ====================

class TemporalProcessor {
private:
    static std::unique_ptr<TemporalProcessor> instance_;
    static std::mutex mutex_;
    
    std::deque<FrameData> history_;
    int max_history_;
    bool initialized_;
    
    TemporalProcessor();
    
    double calculatePathLength(const std::vector<CenterlinePoint>& points);
    
public:
    ~TemporalProcessor() = default;
    
    static TemporalProcessor* getInstance();
    
    bool initialize(int max_history = 15);
    FrameData process(const FrameData& current);
    void clear();
    bool isInitialized() const { return initialized_; }
    
    TemporalProcessor(const TemporalProcessor&) = delete;
    TemporalProcessor& operator=(const TemporalProcessor&) = delete;
};

// ==================== 起点对齐器（单例） ====================

class StartPointAligner {
private:
    static std::unique_ptr<StartPointAligner> instance_;
    static std::mutex mutex_;
    
    cv::Point2d vehicle_pos_;
    bool initialized_;
    
    StartPointAligner();
    
public:
    ~StartPointAligner() = default;
    
    static StartPointAligner* getInstance();
    
    bool initialize();
    void setVehiclePosition(const cv::Point2d& pos);
    std::vector<CenterlinePoint> align(const std::vector<CenterlinePoint>& points);
    std::vector<CenterlinePoint> ensureExact(const std::vector<CenterlinePoint>& points);
    bool isInitialized() const { return initialized_; }
    
    StartPointAligner(const StartPointAligner&) = delete;
    StartPointAligner& operator=(const StartPointAligner&) = delete;
};

// ==================== 中心校正器（单例） ====================

class CenterCorrector {
private:
    static std::unique_ptr<CenterCorrector> instance_;
    static std::mutex mutex_;
    
    cv::Point2d vehicle_pos_;
    std::deque<double> offset_history_;
    bool initialized_;
    
    CenterCorrector();
    
public:
    ~CenterCorrector() = default;
    
    static CenterCorrector* getInstance();
    
    bool initialize();
    void setVehiclePosition(const cv::Point2d& pos);
    double calculateOffset(const std::vector<CenterlinePoint>& points);
    std::vector<CenterlinePoint> correct(const std::vector<CenterlinePoint>& points);
    void reset();
    bool isInitialized() const { return initialized_; }
    
    CenterCorrector(const CenterCorrector&) = delete;
    CenterCorrector& operator=(const CenterCorrector&) = delete;
};

// ==================== 可视化器（单例） ====================

class Visualizer {
private:
    static std::unique_ptr<Visualizer> instance_;
    static std::mutex mutex_;
    
    double resolution_;
    bool save_images_;
    bool initialized_;
    
    Visualizer();
    
    double calculateLength(const std::vector<CenterlinePoint>& points);
    
public:
    ~Visualizer() = default;
    
    static Visualizer* getInstance();
    
    bool initialize(double resolution = 0.1, bool save_images = false);
    void show(const cv::Mat& image, const std::vector<CenterlinePoint>& points,
              const cv::Point2d& vehicle_pos, int frame_id, double offset);
    bool isInitialized() const { return initialized_; }
    
    Visualizer(const Visualizer&) = delete;
    Visualizer& operator=(const Visualizer&) = delete;
};

// ==================== 全局配置管理器（单例） ====================

class ConfigManager {
private:
    static std::unique_ptr<ConfigManager> instance_;
    static std::mutex mutex_;
    
    struct Config {
        double resolution = 0.1;
        bool debug = false;
        int max_history = 15;
        bool save_images = false;
        int scan_step = 10;
        double gaussian_sigma = 1.0;
        int moving_window = 20;
    } config_;
    
    bool initialized_;
    
    ConfigManager() = default;
    
public:
    ~ConfigManager() = default;
    
    static ConfigManager* getInstance();
    
    bool initialize(double resolution = 0.1, bool debug = false, 
                    int max_history = 15, bool save_images = false);
    
    double getResolution() const { return config_.resolution; }
    bool getDebug() const { return config_.debug; }
    int getMaxHistory() const { return config_.max_history; }
    bool getSaveImages() const { return config_.save_images; }
    int getScanStep() const { return config_.scan_step; }
    double getGaussianSigma() const { return config_.gaussian_sigma; }
    int getMovingWindow() const { return config_.moving_window; }
    
    void setResolution(double res) { config_.resolution = res; }
    void setDebug(bool debug) { config_.debug = debug; }
    void setMaxHistory(int history) { config_.max_history = history; }
    void setSaveImages(bool save) { config_.save_images = save; }
    void setScanStep(int step) { config_.scan_step = step; }
    void setGaussianSigma(double sigma) { config_.gaussian_sigma = sigma; }
    void setMovingWindow(int window) { config_.moving_window = window; }
    
    bool isInitialized() const { return initialized_; }
    
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
};

// ==================== 统一入口管理器（单例） ====================

class CenterlineManager {
private:
    static std::unique_ptr<CenterlineManager> instance_;
    static std::mutex mutex_;
    
    bool initialized_;
    cv::Point2d vehicle_pos_;
    
    CenterlineManager();
    
public:
    ~CenterlineManager();
    
    static CenterlineManager* getInstance();
    
    // 一次性初始化所有组件
    bool initialize(double resolution = 0.1, bool debug = false, 
                    int max_history = 15, bool save_images = false);
    
    // 设置车辆位置（同时设置所有组件）
    void setVehiclePosition(const cv::Point2d& pos);
    
    // 处理单帧图像
    FrameData processFrame(const cv::Mat& occupancy_grid, int frame_id, double timestamp = 0);
    
    // 获取当前车辆位置
    cv::Point2d getVehiclePosition() const { return vehicle_pos_; }
    
    // 重置所有组件
    void reset();
    
    // 检查是否已初始化
    bool isInitialized() const { return initialized_; }
    
    CenterlineManager(const CenterlineManager&) = delete;
    CenterlineManager& operator=(const CenterlineManager&) = delete;
};

// ==================== 测试数据生成 ====================

std::vector<cv::Mat> generateTestSequence(int num_frames, int width, int height);

#endif // CENTERLINE_EXTRACTOR_H