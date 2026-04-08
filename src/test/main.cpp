#include "DataType.hpp"
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
// #include <regular_filter.h>
#include <centerline_extract.h>

#include <glob.h>
#include "Eigen/Dense"
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include <tuple>
#include <unistd.h>
#include <sys/stat.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include "Eigen/Dense"
#include "path_optimizer_2/path_optimizer.hpp"
#include "tools/eigen2cv.hpp"
#include "data_struct/data_struct.hpp"
#include "tools/tools.hpp"
#include "data_struct/reference_path.hpp"
#include "tools/spline.h"
#include "config/planning_flags.hpp"
#include "tools/Map.hpp"
#include <tinyspline_ros/tinysplinecpp.h>
#include "bezier.hpp"
// ##########################################################################################################
#include "gridded_path_time_graph.h"
#include "pnc_point.h"
#include "perception_obstacle.h"
#include "planning_gflags.h"
#include <string>
#include <utility>
#include "speed_profile_generator.h"
#include "st_graph_data.h"
#include "piecewise_jerk_speed_problem.h"
#include "piecewise_jerk_problem.h"
#include "linear_interpolation.h"
#include "path_data.h"
#include "DP_speed_planner.h"
#include "st_obstacles_processor.h"

#ifdef __cplusplus
extern "C"
{
#endif
    using namespace cv;
    using namespace std;
    using namespace Eigen;

    PathOptimizationNS::State start_state, end_state, car_state, init_state, car_state_local, init_stateObs;
    std::vector<PathOptimizationNS::State> reference_path_plot;
    std::vector<PathOptimizationNS::State> LocalPath;
    std::vector<PathOptimizationNS::State> LocalPathRefer;
    static int localIndex = 0;
    PathOptimizationNS::ReferencePath reference_path_opt;
    bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;
    std::vector<cv::Point2f> referencePoint;
    double resolution;
    int width, height;

    typedef struct _lqrPose
    {
        int id;
        double x;
        double y;
        double z;
        double theta;
        double kappas;
        double v;
        double a;
    } T_lqrPose;

    typedef struct
    {
        float fPointX; //mm
        float fPointY; //mm
        float fAngle; //0.1^
        float fkappa;
        float fv;
        float fa;
    }Road_Det_Track_PointType;

    #define ROAD_DET_TRACK_POINTNUM 50  //10m
    typedef struct _IpcSignal_Road_Det_TrackType
    {
        uint32_t u32TimeStamp;
        Road_Det_Track_PointType CarPoint;
        Road_Det_Track_PointType sDetTrackPoint[ROAD_DET_TRACK_POINTNUM];
        int8_t s8RoadDirection;
        int8_t s8isRoadWide;
        uint16_t u16RoadWidth;
        int8_t s8CruiseSceneMark;
        uint8_t u8rev;
        uint16_t u16rev;
    }IpcSignal_Road_Det_TrackType;

    #define LH_PATHPLANNING_ALG_VERSION ("LH-PAHTPLAN-V1.51") // pathplanning算法版本号

    static cv::Mat g_sFreeSpaceGray;      // 输入freespace, 只申请一次空间，避免频繁申请释放空间产生内存碎片
    static string g_sglobalPathName = ""; // global 路径名称
    static vector<Vec6d> globalPathPoints;
    static E_PathPlanAlgErrorType e_PathPlanAlgErrorType;
    static bool isFirst;
    static bool updateFlag;
    static double max_speed; // m/s pathPoints[endIndex].v
    // ##########################################################################################################
    // speed set
    static int imageId;
    static StGraphData st_graph_data_;
    static TrajectoryPoint init_point_;
    static DpStSpeedOptimizerConfig default_dp_config;
    static SpeedLimit speed_limit;
    static double max_acceleration = 2.0;
    static double max_deceleration = -4.0;
    static double ref_v_weight = 10.0;
    static double acc_weight = 1.0;
    static double jerk_weight = 3.0;
    static double kappa_penalty_weight = 4.0;
    static double ref_s_weight = 10.0;
    static double longitudinal_jerk_lower_bound = -4;
    static double longitudinal_jerk_upper_bound = 2.0;
    static double total_time = 20;
    static double delta_t = 0.1;
    static std::deque<cv::Mat> globalSequence;
    static int max_size = 400;
    static double centerBeginPixel = 80;
    static int CopyMatImg(T_FreeSpaceImg tFreeSpaceImg, cv::Mat &matDst);
    static void ColorTransformation(cv::Mat cvImgIn, cv::Mat &cvImgOut);
    static void geneatePathPoint(std::vector<PathOptimizationNS::State> result_path, std::vector<PathPoint> *points);
    static int generateSpeed(std::vector<T_Obs> Obs, std::vector<PathPoint> points, double cur_speed, double max_speed, SpeedData *speed_data,vector<T_lqrPose> *resultPose);
    static int LocalPathPlanning(T_FreeSpaceImg tFreeSpaceImg, const cv::Mat& image, std::vector<T_Obs> Obs, const std::vector<CenterlinePoint>& centerPoints,T_CarState tCarState, bool needPath, vector<T_lqrPose> *vectorLqrPose);
    // ##########################################################################################################

    // B样条基函数计算
    double bsplineBasis(int i, int k, const std::vector<double> &knots, double t)
    {
        if (k == 1)
        {
            return (t >= knots[i] && t < knots[i + 1]) ? 1.0 : 0.0;
        }

        double denom1 = knots[i + k - 1] - knots[i];
        double denom2 = knots[i + k] - knots[i + 1];

        double term1 = (denom1 != 0.0) ? (t - knots[i]) / denom1 * bsplineBasis(i, k - 1, knots, t) : 0.0;
        double term2 = (denom2 != 0.0) ? (knots[i + k] - t) / denom2 * bsplineBasis(i + 1, k - 1, knots, t) : 0.0;

        return term1 + term2;
    }

    // 构建B样条设计矩阵
    Eigen::MatrixXd buildDesignMatrix(const std::vector<double> &params,
                                      const std::vector<double> &knots,
                                      int degree)
    {
        int n = params.size();
        int m = knots.size() - degree - 1;
        Eigen::MatrixXd A(n, m);

        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < m; ++j)
            {
                A(i, j) = bsplineBasis(j, degree + 1, knots, params[i]);
            }
        }

        return A;
    }

    // 构建平滑惩罚矩阵
    Eigen::MatrixXd buildSmoothnessMatrix(const std::vector<double> &knots,
                                          int degree, int numControlPoints)
    {
        int m = numControlPoints;
        Eigen::MatrixXd D(m - 2, m);
        D.setZero();

        for (int i = 0; i < m - 2; ++i)
        {
            double h1 = knots[i + degree + 1] - knots[i + 1];
            double h2 = knots[i + degree + 2] - knots[i + 2];

            if (h1 != 0.0)
                D(i, i) = 1.0 / h1;
            if (h1 != 0.0 && h2 != 0.0)
                D(i, i + 1) = -(1.0 / h1 + 1.0 / h2);
            if (h2 != 0.0)
                D(i, i + 2) = 1.0 / h2;
        }

        return D;
    }

    // 基于B样条的曲率平滑
    std::vector<double> bsplineCurvatureSmoothing(
        const std::vector<double> &curvature, // 输入曲率值
        const std::vector<double> &params,    // 参数化值 (通常为[0,1]均匀分布)
        int degree,                           // B样条次数 (通常为3)
        double lambda,                        // 平滑参数
        int numControlPoints                  // 控制点数量 (通常少于数据点)
    )
    {
        // 1. 创建节点向量
        std::vector<double> knots(numControlPoints + degree + 1);

        // 均匀节点 (开放均匀B样条)
        for (int i = 0; i <= degree; ++i)
            knots[i] = params.front();
        for (int i = 1; i < numControlPoints - degree; ++i)
        {
            double t = static_cast<double>(i) / (numControlPoints - degree);
            knots[degree + i] = params.front() + t * (params.back() - params.front());
        }
        for (int i = numControlPoints; i < knots.size(); ++i)
            knots[i] = params.back();

        // 2. 构建设计矩阵A
        Eigen::MatrixXd A = buildDesignMatrix(params, knots, degree);

        // 3. 构建平滑矩阵D
        Eigen::MatrixXd D = buildSmoothnessMatrix(knots, degree, numControlPoints);

        // 4. 构建右侧向量b
        Eigen::VectorXd b(curvature.size());
        for (int i = 0; i < curvature.size(); ++i)
            b(i) = curvature[i];

        // 5. 解线性系统 (AᵀA + λDᵀD)c = Aᵀb
        Eigen::MatrixXd lhs = A.transpose() * A + lambda * D.transpose() * D;
        Eigen::VectorXd rhs = A.transpose() * b;
        Eigen::VectorXd controlPoints = lhs.ldlt().solve(rhs);

        // 6. 计算平滑后的曲率值
        std::vector<double> smoothed(curvature.size());
        for (int i = 0; i < params.size(); ++i)
        {
            double val = 0.0;
            for (int j = 0; j < numControlPoints; ++j)
            {
                val += controlPoints(j) * bsplineBasis(j, degree + 1, knots, params[i]);
            }
            smoothed[i] = val;
        }

        return smoothed;
    }

    // 三点画圆法
    vector<double> calculate_curvature_circle(vector<Point2f> result_path, int interval = 2)
    {
        vector<double> curvature_list;
        for (int i = 0; i < result_path.size(); i++)
        {
            // 初始化头尾的曲率均为0
            if (i < interval)
            {
                curvature_list.push_back(0);
            }
            else if (i > result_path.size() - interval - 1)
            {
                curvature_list.push_back(0);
            }
            // 获取间隔为interval的3个点（二阶差分需要3个点）
            else
            {
                int start = i - interval;
                int end = i + interval + 1;
                // 计算圆心
                double x1 = result_path[start].x;
                double x2 = result_path[i].x;
                double x3 = result_path[end].x;
                double y1 = result_path[start].y;
                double y2 = result_path[i].y;
                double y3 = result_path[end].y;

                double denominator = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

                double x_center = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / denominator;
                double y_center = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / denominator;

                // 计算半径
                double r = sqrt((x1 - x_center) * (x1 - x_center) + (y1 - y_center) * (y1 - y_center));

                // 计算曲率
                double curvature = 1 / r;

                // 判断曲率的正负
                double vector10 = x2 - x1;
                double vector11 = y2 - y1;
                double vector20 = x3 - x2;
                double vector21 = y3 - y2;
                double cross_product = vector10 * vector21 - vector11 * vector20;

                if (cross_product > 0)
                {
                    curvature *= 1; // 曲率为负
                }
                else
                {
                    curvature *= -1; // 曲率为正
                }

                curvature_list.push_back(curvature);
            }
        }
        return curvature_list;
    }
    // 差分法
    vector<double> calculate_curvature_difference(const vector<Point2f> &result_path, int interval = 9)
    {
        if (result_path.size() < 50)
        {
            interval = 5;
        }
        if(result_path.size() < 15)
        {
            vector<double> smoothPath;
            for(int i=0;i<result_path.size();i++)
            {
                smoothPath.push_back(0);
            }
            return smoothPath;
        }
        vector<double> curvature_list(result_path.size(), 0.0);

        for (size_t i = 0; i < result_path.size(); ++i)
        {
            // 初始化头尾的曲率均为0
            if (i < 2 * interval || i >= result_path.size() - 2 * interval)
            {
                curvature_list[i] = 0.0;
            }
            else
            {
                // 获取间隔为interval的5个点（二阶差分需要5个点）
                size_t start = i - 2 * interval;
                size_t end = i + 2 * interval + 1;

                vector<double> x, y;
                for (size_t j = start; j < end; j += interval)
                {
                    x.push_back(result_path[j].x);
                    y.push_back(result_path[j].y);
                }

                // 差分法计算曲率
                // 一阶差分
                vector<double> dx(x.size()), dy(y.size());
                for (size_t j = 1; j < x.size() - 1; ++j)
                {
                    dx[j] = (x[j + 1] - x[j - 1]) / 2.0;
                    dy[j] = (y[j + 1] - y[j - 1]) / 2.0;
                }
                dx[0] = x[1] - x[0];
                dx.back() = x.back() - x[x.size() - 2];
                dy[0] = y[1] - y[0];
                dy.back() = y.back() - y[y.size() - 2];

                // 二阶差分
                vector<double> d2x(dx.size()), d2y(dy.size());
                for (size_t j = 1; j < dx.size() - 1; ++j)
                {
                    d2x[j] = (dx[j + 1] - dx[j - 1]) / 2.0;
                    d2y[j] = (dy[j + 1] - dy[j - 1]) / 2.0;
                }
                d2x[0] = dx[1] - dx[0];
                d2x.back() = dx.back() - dx[dx.size() - 2];
                d2y[0] = dy[1] - dy[0];
                d2y.back() = dy.back() - dy[dy.size() - 2];

                // 计算曲率, 并处理分母为0的情况
                double denominator = pow(dx[2] * dx[2] + dy[2] * dy[2], 1.5);
                double curvature = (denominator == 0) ? 0.0 : (dx[2] * d2y[2] - d2x[2] * dy[2]) / denominator;

                // 只有中间点的曲率是准确计算的，所以只取中间点的曲率
                curvature_list[i] = curvature;
            }
        }

        // 将头尾数据的曲率设置为最近可计算点的曲率
        if (localIndex > 0)
        {
            double a = curvature_list[2 * interval];
            double b = LocalPathRefer[localIndex].k;
            std::cout << " kappas smooth: " << localIndex << "," << abs(a - b) << "," << a << "," << b << std::endl;
            if (abs(a - b) > 0.001 && (a > 0 && b > 0 && b < a) || (a < 0 && b < 0 && b > a))
            {
                fill(curvature_list.begin(), curvature_list.begin() + 2 * interval, LocalPathRefer[localIndex].k);
            }
            else
            {
                fill(curvature_list.begin(), curvature_list.begin() + 2 * interval, curvature_list[2 * interval]);
            }
        }
        else
        {
            fill(curvature_list.begin(), curvature_list.begin() + 2 * interval, curvature_list[2 * interval]);
        }

        fill(curvature_list.end() - 2 * interval, curvature_list.end(), curvature_list[result_path.size() - 2 * interval - 1]);

        // 应用B样条平滑
        int degree = 3;           // 三次B样条
        double lambda = 0.5;      // 平滑参数
        int numControlPoints = 5; // curvature_list.size() / 7; // 控制点数量
        double param;
        std::vector<double> params(curvature_list.size());
        // 创建参数化值 (0到1均匀分布)
        params[0] = 0.0;
        for (int i = 1; i < curvature_list.size(); ++i)
        {
            double dist = abs(curvature_list[i - 1] - curvature_list[i]);
            params[i] = params[i - 1] + dist;
        }
        for (int i = 0; i < curvature_list.size(); ++i)
        {
            params[i] /= params.back();
        }
        std::vector<double> smoothed = bsplineCurvatureSmoothing(
            curvature_list, params, degree, lambda, numControlPoints);
        for (int i = 0; i < smoothed.size(); ++i)
        {
            if (smoothed[i] == 0)
            {
                if (smoothed.size() > 14 && smoothed[smoothed.size() - 13] != 0)
                {
                    smoothed[i] = smoothed[smoothed.size() - 13];
                }
                else
                {
                    smoothed[i] = curvature_list[i];
                }
            }
        }
        return smoothed;
    }

    //规则化角度
    double regularize_angle(double radian_angle)
    {
        if (radian_angle >= 4)
        {
            radian_angle = -(radian_angle - 3.14 - 1.57);
        }
        else if (radian_angle <= -4)
        {
            radian_angle = -(radian_angle + 3.14 + 1.45);
        }

        if (radian_angle >= 3.14)
        {
            radian_angle = radian_angle - M_PI;
        }
        else if (radian_angle <= -3.14)
        {
            radian_angle = radian_angle + M_PI;
        }

        if (radian_angle <= -M_PI / 2)
        {
            radian_angle = radian_angle + M_PI / 2.0;
        }
        else if (radian_angle >= M_PI / 2)
        {
            radian_angle = radian_angle - M_PI / 2.0;
        }

        if (radian_angle <= -0.785)
        {
            radian_angle = radian_angle + M_PI / 2.0;
        }
        else if (radian_angle >= 0.785)
        {
            radian_angle = radian_angle - M_PI / 2.0;
        }

        return radian_angle;
    }
    //按照起点终点差值固定移动相同的距离，生成参考点
    void referenceCb()
    {
        reference_path_plot.clear();
        referencePoint.clear();
        double disx = (end_state.x - start_state.x) / 6;
        double disy = (end_state.y - start_state.y) / 6;
        // printf(disx<<","<<disy);
        cv::Point2f point0(start_state.x, start_state.y);
        cv::Point2f point1(start_state.x + disx, start_state.y + disy);
        cv::Point2f point2(start_state.x + disx * 2, start_state.y + disy * 2);
        cv::Point2f point3(start_state.x + disx * 3, start_state.y + disy * 3);
        cv::Point2f point4(start_state.x + disx * 4, start_state.y + disy * 4);
        cv::Point2f point5(start_state.x + disx * 5, start_state.y + disy * 5);
        cv::Point2f point6(start_state.x + disx * 6, start_state.y + disy * 6);
        referencePoint.push_back(point0);
        referencePoint.push_back(point1);
        referencePoint.push_back(point2);
        referencePoint.push_back(point3);
        referencePoint.push_back(point4);
        referencePoint.push_back(point5);
        referencePoint.push_back(point6);
        for (int i = 0; i < referencePoint.size(); i++)
        {
            PathOptimizationNS::State reference_point;
            reference_point.x = referencePoint[i].x;
            reference_point.y = referencePoint[i].y;
            reference_path_plot.emplace_back(reference_point);
            reference_rcv = reference_path_plot.size() >= 6;
            std::cout << "referenceCb received a reference point" << referencePoint[i] << std::endl;
        }
    }
    //选取参考线上固定距离的点，作为路径规划的参考点
    void referenceTurnPoint(int endIndex, int index, vector<Vec6d> pathPoints, double thX, double thY)
    {
        int num = (endIndex - index) / 10;
        reference_path_plot.clear();
        referencePoint.clear();
        std::vector<double> kappas;
        double refX, refY;
        for (size_t i = 0; i < endIndex - index + 1; i = i + num)
        {
            double xs = pathPoints[index + i][1] - height/2*resolution + centerBeginPixel*resolution/10;  //参考线生成的时候，是以height-80的位置作为车辆的起始点
            double ys = -pathPoints[index + i][0] + width/2*resolution; // 移动坐标系，以图像中心点为原点
            cv::Point2f point0(xs, ys);
            referencePoint.push_back(point0);
            kappas.push_back(pathPoints[index + i][3]);
            // double y = -xs / resolution + 74;
            // double x = -ys / resolution + 96;
            // cv::Point p(x, y);
            // std::cout << "reference_path_plot111111: " << x << "," << y << std::endl;
        }
        for (int i = 0; i < referencePoint.size(); i++)
        {
            PathOptimizationNS::State reference_point;
            reference_point.x = referencePoint[i].x;
            reference_point.y = referencePoint[i].y;
            reference_point.k = kappas[i];
            reference_path_plot.emplace_back(reference_point);
            reference_rcv = reference_path_plot.size() >= 6;
            std::cout << "received a reference point" <<referencePoint[i]<< std::endl;
        }
    }

    static int CopyMatImg(T_FreeSpaceImg tFreeSpaceImg, cv::Mat &matDst)
    {
        int iRet = PathPlan_OK;
        int iIndex = 0, jIndex = 0;
        int iRows = tFreeSpaceImg.uiHeight;
        int iCols = tFreeSpaceImg.uiWidth;
        unsigned char *pucDstImgRowsPtr = NULL;
        unsigned char *pucSrcImgRowsPtr = NULL;

        for (iIndex = 0; iIndex < iRows; iIndex++)
        {
            pucSrcImgRowsPtr = tFreeSpaceImg.pucImg + iIndex * iCols;
            pucDstImgRowsPtr = matDst.ptr(iIndex);
            for (jIndex = 0; jIndex < iCols; jIndex++)
            {
                // matDst.at<unsigned char>(iIndex, jIndex) = tOrbSlamImg.pucImg[iIndex*iCols+jIndex];
                //*(pucDstImgRowsPtr+jIndex) = *(pucSrcImgRowsPtr + jIndex);
                *pucDstImgRowsPtr++ = *pucSrcImgRowsPtr++;
            }
        }

        return iRet;
    }

    //速度规划
    bool Process(DiscretizedPath &discretizedPath_, TrajectoryPoint &init_point, SpeedData *const speed_data, std::vector<STBoundary> boundariesG, double max_speed, double total_length)
    {

        double FLAGS_planning_upper_speed_limit = 3.0;
        if (speed_data == nullptr)
        {
            return false;
        }
        SpeedData reference_speed_data = *speed_data;

        if (discretizedPath_.Length() == 0)
        {
            std::cout << "Empty path data" << std::endl;
            return false;
        }
        std::array<double, 3> init_s = {0.0, init_point.v, init_point.a};

        int num_of_knots = static_cast<int>((reference_speed_data.TotalTime()) / delta_t) + 1;

        // 创建QP速度规划 对象
        PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t, init_s);

        piecewise_jerk_problem.set_weight_ddx(acc_weight);
        piecewise_jerk_problem.set_weight_dddx(jerk_weight);

        piecewise_jerk_problem.set_x_bounds(0.0, total_length);
        piecewise_jerk_problem.set_dx_bounds(0.0, std::fmax(FLAGS_planning_upper_speed_limit, init_point.v));
        piecewise_jerk_problem.set_ddx_bounds(max_deceleration, max_acceleration);
        piecewise_jerk_problem.set_dddx_bound(longitudinal_jerk_lower_bound, longitudinal_jerk_upper_bound);
        piecewise_jerk_problem.set_dx_ref(ref_v_weight, max_speed);

        // Update STBoundary
        // 设置 s 的上下界
        std::vector<std::pair<double, double>> s_bounds;
        for (int i = 0; i < num_of_knots; ++i)
        {
            double curr_t = i * delta_t;
            double s_lower_bound = 0.0;
            double s_upper_bound = total_length * 100;
            for (const STBoundary boundary : boundariesG)
            {
                double s_lower = 0.0;
                double s_upper = 0.0;
                if (!boundary.GetUnblockSRange(curr_t, &s_upper, &s_lower))
                {
                    continue;
                }
                // 不同场景下的 上下界选取
                switch (boundary.boundary_type())
                {
                case STBoundary::BoundaryType::STOP:  // 停车
                case STBoundary::BoundaryType::YIELD: // 终止换道
                    s_upper_bound = std::fmin(s_upper_bound, s_upper);
                    break;
                case STBoundary::BoundaryType::FOLLOW:                       // 跟车
                    s_upper_bound = std::fmin(s_upper_bound, s_upper - 8.0); // 借了点膨胀区域“8.0”
                    break;
                case STBoundary::BoundaryType::OVERTAKE: // 超车
                    s_lower_bound = std::fmax(s_lower_bound, s_lower);
                    break;
                default:
                    break;
                }
            }
            if (s_lower_bound > s_upper_bound)
            {
                std::cout << "s_lower_bound larger than s_upper_bound on STGraph" << std::endl;
                speed_data->clear();
                return false;
            }
            s_bounds.emplace_back(s_lower_bound, s_upper_bound);
        }
        piecewise_jerk_problem.set_x_bounds(std::move(s_bounds));

        // Update SpeedBoundary and ref_s
        std::vector<double> x_ref;
        std::vector<double> penalty_dx;
        std::vector<std::pair<double, double>> s_dot_bounds;

        for (int i = 0; i < num_of_knots; ++i)
        {
            double curr_t = i * delta_t;
            // get path_s
            SpeedPoint sp;
            reference_speed_data.EvaluateByTime(curr_t, &sp);
            const double path_s = sp.s;
            // std::cout<<path_s<<std::endl;
            x_ref.emplace_back(path_s);
            // get curvature
            PathPoint path_point = discretizedPath_.Evaluate(path_s);
            penalty_dx.push_back(std::fabs(path_point.kappa) * kappa_penalty_weight);
            // get v_upper_bound
            const double v_lower_bound = 0.0;
            double v_upper_bound = FLAGS_planning_upper_speed_limit;
            v_upper_bound = speed_limit.GetSpeedLimitByS(path_s);
            s_dot_bounds.emplace_back(v_lower_bound, std::fmax(v_upper_bound, 0.0));
        }
        piecewise_jerk_problem.set_x_ref(ref_s_weight, std::move(x_ref));
        piecewise_jerk_problem.set_penalty_dx(penalty_dx);
        piecewise_jerk_problem.set_dx_bounds(std::move(s_dot_bounds));

        // Solve the problem
        if (!piecewise_jerk_problem.Optimize())
        {
            std::cout << "Piecewise jerk speed optimizer failed!" << std::endl;
            return false;
        }

        // Extract output
        const std::vector<double> &s = piecewise_jerk_problem.opt_x();
        const std::vector<double> &ds = piecewise_jerk_problem.opt_dx();
        const std::vector<double> &dds = piecewise_jerk_problem.opt_ddx();
        // for (int i = 0; i < num_of_knots; ++i)
        // {
        //     std::cout << "For t[" << i * delta_t << "], s = " << s[i] << ", v = " << ds[i]
        //               << ", a = " << dds[i];
        // }
        speed_data->clear();
        speed_data->AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
        for (int i = 1; i < num_of_knots; ++i)
        {
            // Avoid the very last points when already stopped
            if (ds[i] <= 0.0)
            {
                break;
            }
            speed_data->AppendSpeedPoint(s[i], delta_t * i, ds[i], dds[i],
                                         (dds[i] - dds[i - 1]) / delta_t);
            // std::cout << "For s[" << i << "], s = " << s[i] << ", v = " << ds[i]
            //           << ", a = " << dds[i] << ",t = " << delta_t * i << std::endl;
        }
        // SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
        return true;
    }

    // 线性插值函数
    std::vector<double> linearInterpolation(const std::vector<double> &input, int newSize)
    {
        if (input.empty())
            return {};
        if (newSize <= input.size())
            return input; // 如果新大小不大于原大小，直接返回
        std::vector<double> output(newSize);
        if (input.size() < 2)
        {
            for (int i = 0; i < newSize; ++i)
            {
                output[i] = 0;
            }
            return output;
        }
        double scale = static_cast<double>(input.size() - 1) / (newSize - 1);

        for (int i = 0; i < newSize; ++i)
        {
            double pos = i * scale;
            int left = static_cast<int>(pos);
            double alpha = pos - left;

            if (left + 1 < input.size())
            {
                // 线性插值公式: output = input[left] * (1 - alpha) + input[left+1] * alpha
                output[i] = input[left] * (1.0 - alpha) + input[left + 1] * alpha;
            }
            else
            {
                // 处理最后一个元素的情况
                output[i] = input.back();
            }
        }

        return output;
    }

    // Init 初始化函数
    int InitPathPlanning()
    {
        isFirst = true;
        width = 500;
        height = 500;
        resolution = 0.01;
        updateFlag = false;
        max_speed = 1.2;
        default_dp_config.unit_t = 1.0;
        default_dp_config.dense_dimension_s = 201;
        default_dp_config.dense_unit_s = 0.1;
        default_dp_config.sparse_unit_s = 0.5;
        default_dp_config.speed_weight = 0.0;
        default_dp_config.accel_weight = 10.0;
        default_dp_config.jerk_weight = 10.0;
        default_dp_config.obstacle_weight = 1.0;
        default_dp_config.reference_weight = 0.0;
        default_dp_config.go_down_buffer = 5.0;
        default_dp_config.go_up_buffer = 5.0;

        default_dp_config.default_obstacle_cost = 1e4;

        default_dp_config.default_speed_cost = 1.0e3;
        default_dp_config.exceed_speed_penalty = 1.0e3;
        default_dp_config.low_speed_penalty = 1.0;
        default_dp_config.reference_speed_penalty = 10.0;
        default_dp_config.keep_clear_low_speed_penalty = 10.0;
        default_dp_config.accel_penalty = 1.0;
        default_dp_config.decel_penalty = 1.0;

        default_dp_config.positive_jerk_coeff = 1.0;
        default_dp_config.negative_jerk_coeff = 1.0;

        default_dp_config.max_acceleration = max_acceleration;
        default_dp_config.max_deceleration = max_deceleration;
        default_dp_config.spatial_potential_penalty = 1.0e2;
        for (double s = 0; s < 200.0; s += 1.0)
        {
            speed_limit.AppendSpeedLimit(s, 1.2);
        }
        init_point_.path_point.x = 0.0;
        init_point_.path_point.y = 0.0;
        init_point_.path_point.z = 0.0;
        init_point_.path_point.kappa = 0.0;
        init_point_.v = 0.0;
        init_point_.a = 0.0;
        st_graph_data_ = StGraphData();
        imageId = 0;
        // Ptr<ml::TrainData> mlDataX, mlDataY;
        // mlDataX = ml::TrainData::loadFromCSV("/app/slam/mapX.csv", 1);
        // mlDataY = ml::TrainData::loadFromCSV("/app/slam/mapY.csv", 1);
        // dataX = mlDataX->getTrainSamples();
        // dataY = mlDataY->getTrainSamples();
        printf("InitPathPlanning, ----------end:");
        return PathPlan_OK;
    }


// ==================== 主函数 ====================

int main() 
{
    //先进行变量的初始化
    InitPathPlanning();
    
    // 参数配置
    int width = 1920;  //图像宽
    int height = 1080; //图像高
    int num_frames = 400;   //图像数量
    double resolution = 0.01;  //图像分辨率
    bool debug = false;
    int max_history = 15;  //最大参考帧，用于时序平滑中心点的寻找
    
    // 生成测试序列
    for(int i=100;i<500;i++)
    {
        width = 1920;  //图像宽
        height = 1080; //图像高
        resolution = 0.01;  //图像分辨率
        //选取某一帧作为测试图像
        cv::Mat image = cv::imread("/home/gq/guoqian/findPath/test_image/" + to_string(i) + ".jpg");
        //如果图像为空，直接返回规划失败
        if(image.empty())
        {
            std::cout<<"image is empty"<<std::endl;
            return PathPlan_FAILED;
        }
        //图像转换为灰度图，并进行二值化处理
        if (image.channels() == 3)
        {
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        }
        threshold(image, image, 120, 255, 0);
        // 道路宽度最多是6米，所以对图像超出6米范围的区域进行不可通行区域的处理，方便寻找中心线
        image.colRange(width/2+300,width).setTo(0);
        image.colRange(0,width/2-300).setTo(0);
        if(globalSequence.size() > max_size)
        {
            globalSequence.pop_front();
        }
        globalSequence.push_back(image);
    }
    std::cout << "  Generated " << globalSequence.size() << " frames" << std::endl;
    
    // 车辆位置（图像底部中央）
    cv::Point2d vehicle_pos(width / 2.0 * resolution, (height - centerBeginPixel) * resolution);
    std::cout << "  Vehicle position: (" << vehicle_pos.x << ", " << vehicle_pos.y << ") m" << std::endl;
    std::cout << "  Expected path length: ~" << (height - centerBeginPixel) * resolution << " m\n" << std::endl;
    
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
    std::vector<_lqrPose> lqrResultPose;
    
    for (int frame_id = 0; frame_id < globalSequence.size(); ++frame_id) {
        double timestamp = frame_id * 0.1;
        
        auto frame_start = std::chrono::high_resolution_clock::now();
        
        // 1. 提取中心线
        auto points = extractor.extract(globalSequence[frame_id]);
        
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
        frame.image = globalSequence[frame_id].clone();
        
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
        if (frame_id % 1 == 0) {
            visualizer.show(globalSequence[frame_id], fused.points, vehicle_pos, frame_id, fused.lateral_offset);
        }
        //打印找到的中心点信息
        //for(int i=0;i<fused.points.size();i++)
        //{
        //    std::cout<<"-------------------------------"<<std::endl;
        //    std::cout<<fused.points[i].position.x<<","<<fused.points[i].position.y<<std::endl;
        //    std::cout<<fused.points[i].confidence<<std::endl;
        //    std::cout<<fused.points[i].curvature<<std::endl;
        //    std::cout<<fused.points[i].heading<<std::endl;
        //    std::cout<<fused.points[i].road_width<<std::endl;
        //}
        //定义车辆的全局坐标信息，即里程计坐标
        T_CarState tCarState;
        if(LocalPathRefer.size() < 1)
        {
            tCarState.dX = 0;
            tCarState.dY = 0;
            tCarState.dTheta = 0;            
        }else
        {
            int over_start = std::max(0,(int)LocalPathRefer.size()-30);
            tCarState.dX = LocalPathRefer[over_start].x;
            tCarState.dY = LocalPathRefer[over_start].y;
            tCarState.dTheta = LocalPathRefer[over_start].heading;  
        }

        std::vector<T_Obs> Obs;
        //############################################begin path plan#########################
        vector<T_lqrPose> vectorLqrPose;
        T_FreeSpaceImg tFreeSpaceImg;
        //进行局部路径规划
        LocalPathPlanning(tFreeSpaceImg,globalSequence[frame_id], Obs, fused.points, tCarState,true, &vectorLqrPose);
        std::cout<<"vectorLqrPose size: "<<vectorLqrPose.size()<<std::endl;
        //clear the struct
        IpcSignal_Road_Det_TrackType *pTrack = (IpcSignal_Road_Det_TrackType*)malloc(sizeof(IpcSignal_Road_Det_TrackType));
        // IpcSignal_Road_Det_TrackType *pTrack = new IpcSignal_Road_Det_TrackType;
        if(pTrack != NULL)
        {
           memset(pTrack,0,sizeof(IpcSignal_Road_Det_TrackType)); 
        }
        // //set timestamp
        pTrack->u32TimeStamp = timestamp;
        // //set carPose
        pTrack->CarPoint.fPointX = tCarState.dX;
        pTrack->CarPoint.fPointY = tCarState.dY;
        pTrack->CarPoint.fAngle = tCarState.dTheta;
        pTrack->CarPoint.fkappa = 0.0f;
        pTrack->CarPoint.fv = 0.0f;
        pTrack->CarPoint.fa = 0.0f;
        //set path plan point
        for(int i=0;i<ROAD_DET_TRACK_POINTNUM && i< vectorLqrPose.size();i++)
        {
            pTrack->sDetTrackPoint[i].fPointX = vectorLqrPose[i].x;
            pTrack->sDetTrackPoint[i].fPointY = vectorLqrPose[i].y;
            pTrack->sDetTrackPoint[i].fAngle  = vectorLqrPose[i].theta;
            pTrack->sDetTrackPoint[i].fkappa  = vectorLqrPose[i].kappas;
            pTrack->sDetTrackPoint[i].fv      = vectorLqrPose[i].v;
            pTrack->sDetTrackPoint[i].fa      = vectorLqrPose[i].a;
        }
        pTrack->s8RoadDirection = 1;
        pTrack->s8isRoadWide = 1;
        pTrack->u16RoadWidth = fused.avg_width;


    }
    auto total_end = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double, std::milli>(total_end - total_start).count();
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Processing completed!" << std::endl;
    std::cout << "Total time: " << total_time / 1000.0 << " s" << std::endl;
    std::cout << "Average per frame: " << total_time / num_frames << " ms" << std::endl;
    std::cout << "FPS: " << 1000.0 / (total_time / num_frames) << std::endl;    
    waitKey(0);
    return 0;
}
    // 前视摄像头安装角度水平，鱼眼矫正后有0.7m盲区
    int LocalPathPlanning(T_FreeSpaceImg tFreeSpaceImg, const cv::Mat& image, std::vector<T_Obs> Obs, const std::vector<CenterlinePoint>& centerPoints,T_CarState tCarState,  bool needPath,vector<T_lqrPose> *vectorLqrPose)
    {
        clock_t time1 = clock();
        // std::cout << "Lidar info object num: " << std::endl;
        // if (g_sFreeSpaceGray.empty() && tFreeSpaceImg.uiHeight > 0)
        // {
        //     g_sFreeSpaceGray.create(tFreeSpaceImg.uiHeight, tFreeSpaceImg.uiWidth, CV_8UC1);
        // }

        // CopyMatImg(tFreeSpaceImg, g_sFreeSpaceGray);
        resolution = 0.01;
        imageId++;
        bool shortPath = false; //用于判断剩余路径是不是过短，及时进行局部路径的重规划
        // draw map
        Mat new_map;
        int index = 5; //参考路经的起点从第20个点开始
        localIndex = 0; //用于记录上次局部规划轨迹与当前车辆的位置关系
        // find the path respecting point
        globalPathPoints.clear(); //将找到的中心点放到全局规划的参考路径中，注意，中心点的坐标y是从上到下
        double totalLength = (image.rows-centerBeginPixel) * resolution;
        for(int i=0;i<centerPoints.size();i++)
        {
            Vec6d point;
            point[0] = centerPoints[i].position.x;
            point[1] = abs(centerPoints[i].position.y-totalLength);  //将中心点坐标改为从下到上
            point[2] = centerPoints[i].heading;
            point[3] = centerPoints[i].curvature;
            point[4] = centerPoints[i].confidence;
            point[5] = centerPoints[i].road_width;
            globalPathPoints.push_back(point);

        }
        //第一次轨迹规划，或者轨迹更新之后，进行初始车辆位置的刷新
        int iRet = PathPlan_OK;
        if (isFirst || true == updateFlag)
        {
            printf("To init state\n");
            init_state.x = tCarState.dX;
            init_state.y = tCarState.dY;
            updateFlag = false;
            isFirst = false;
        }
        int numSize = LocalPath.size();
        if (numSize > 0)
        {
            //判断当前车辆位置距离轨迹终点的位置，如果小于3米，则进行再次规划
            // std::cout << "local path: " << LocalPath.size() << "," << LocalPath[numSize - 1].x << "," << LocalPath[numSize - 1].y << std::endl;
            if ((pow((tCarState.dX - LocalPath[numSize - 1].x), 2) + pow((tCarState.dY - LocalPath[numSize - 1].y), 2)) < 9)
            {
                shortPath = true;
            }
        }
        //记录车辆行驶的距离
        double travelDistance = pow((init_state.x - tCarState.dX), 2) + pow((init_state.y - tCarState.dY), 2);

        std::cout << "planning :" << "," << needPath << "," << shortPath << "," << travelDistance << "," << std::endl;
        image.copyTo(new_map);
        if ((isFirst || needPath || travelDistance > 64 || shortPath)) //再次进行局部规划
        {
            std::cout << "imageId: " << imageId << std::endl;
            // imwrite("/app/slam/" + to_string(imageId) + "J3map.jpg", new_map); // 保存图
            // 4.补全车前的盲区
            cv::Rect roi(560,800,800,280);
            if(roi.x+roi.width <= new_map.cols && roi.y+roi.height <= new_map.rows)
            {
               new_map(roi).setTo(255); 
            }
            // cv::Mat warped_cut = new_map(cv::Rect(0, 100, 1920, 980));
            // cv::copyMakeBorder(warped_cut, new_map, 0, 100, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
            // cv::imshow("binaryImage", new_map);
            // cv::waitKey(0);
            // read original path
            //将全局参考路径拷贝到pathPoints中
            vector<Vec6d> pathPoints;
            pathPoints.assign(globalPathPoints.begin(), globalPathPoints.end());
            // std::reverse(pathPoints.begin(),pathPoints.end());
            int num = pathPoints.size();
            //如果全局参考轨迹小于40个点，即小于4米，则前面不可通行，返回规划失败
            if (num < 40)
            {
                printf("globalPathPoint is empty:\n");
                return PathPlan_FAILED;
            }
            // 2. binaryImage to get Areas of exercise
            vector<Point> points;
            double cropX;
            double cropY;
            cv::Mat img_src;
            findNonZero(new_map, points); //用于去除图像中黑色区域，加快规划速度，但会改变图像的大小
            double ori_height = new_map.size().height;
            double ori_width = new_map.size().width;
            if (points.empty())
            {
                img_src = new_map;
            }
            else
            {
                Rect bb = boundingRect(points);
                // std::cout << bb.x << bb.width << std::endl;
                img_src = new_map(bb);
                height = img_src.size().height;
                width = img_src.size().width;
                cropX = bb.x;
                cropY = bb.y;
                for(int i=0;i<pathPoints.size();i++)
                {
                    pathPoints[i][0] = pathPoints[i][0] - cropX*resolution;
                }
            }
            // cv::imshow("image",img_src);
            // cv::waitKey(0);
            cv::Size scaleSize(width/10,height/10);
            cv::resize(img_src,img_src,scaleSize,0,0,cv::INTER_AREA);
            width = width/10;
            height = height/10;
            resolution = resolution*10;
            // 6. 选取终点,设定候选目标半径为 高度减1米
            cv::Mat circleImage(img_src.size(), CV_8UC1, Scalar(0));
            int meter = 1/resolution;
            double targetPixel = height-meter;
            vector<Point2d> centerPoint;
            double downpixel = 0.5/resolution;
            while (centerPoint.size() == 0)
            {
                if (targetPixel - downpixel < meter)
                {
                    targetPixel = downpixel + meter;
                }

                circle(circleImage, Point(width / 2, height), targetPixel - downpixel, Scalar(255), 30);
                cv::Mat interSection(img_src.size(), CV_8UC1, Scalar(0));
                bitwise_and(circleImage, img_src, interSection);
                // cv::imshow("interSection", circleImage);
                // cv::waitKey(0);
                vector<vector<Point>> contours;
                vector<Vec4i> hierarchy;
                findContours(interSection, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

                for (int i = 0; i < contours.size(); i++)
                {
                    // std::cout << cv::contourArea(contours[i]) << std::endl;
                    if (cv::contourArea(contours[i]) > 50/resolution)
                    {
                        Moments M;
                        Point2d point;
                        M = moments(contours[i]);
                        point.x = double(M.m10 / M.m00);
                        point.y = double(M.m01 / M.m00);
                        int a = width/2+4/resolution;
                        int b = width/2-4/resolution;
                        if(point.x < a && point.x > b && point.y < height/2)
                            centerPoint.push_back(point);
                        // circle(interSection, point, 5, Scalar(0, 0, 0), -1);
                        // std::cout<<point<<std::endl;
                    }
                }
                downpixel = downpixel * 2;
                if (downpixel > 3/resolution)
                {
                    break;
                }
                // cv::imshow("interSection", interSection);
                // cv::waitKey(0);
            }
            if(centerPoint.size() < 1)  //找不到中心点，前面不可通行或者中心点偏一边，不进行局部规划
            {
                printf("centerPoint is empty:\n");
                return PathPlan_FAILED;
            }
            // Initialize grid map from image. 栅格地图的初始化
            grid_map::GridMap grid_map(std::vector<std::string>{"obstacle", "distance"});
            grid_map::GridMapCvConverter::initializeFromImage(
                img_src, resolution, grid_map, grid_map::Position::Zero());
            // Add obstacle layer.
            grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
                img_src, "obstacle", grid_map);
            PathOptimizationNS::Map map(grid_map);
            // Update distance layer.
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary = grid_map.get("obstacle").cast<unsigned char>();
            cv::distanceTransform(eigen2cv(binary), eigen2cv(grid_map.get("distance")),
                                  DIST_L2, DIST_MASK_PRECISE);
            grid_map.get("distance") *= resolution;
            printf("find the path respecting point\n");
            // 7. 选取终点
            car_state.x = tCarState.dX;
            car_state.y = tCarState.dY;
            car_state.heading = tCarState.dTheta;
            std::cout << "carState: " << car_state.x << "," << car_state.y << "," << car_state.heading << std::endl;
            // path splicing
            Point2d endpathPoint, endpathPoint_local;
            int endIndex = 0;
            double centerLength = (img_src.rows - centerPoint[0].y)*resolution;
            //找到全局参考轨迹中与图像中心点最接近的点
            for (int i = index; i < pathPoints.size(); i++)
            {
                double distance = sqrt(pow((pathPoints[0][0] - pathPoints[i][0]), 2) + pow((pathPoints[0][1] - pathPoints[i][1]), 2)) + centerBeginPixel*resolution/10;
                // printf(i<<","<<pathPoints[i]<<"distance: "<<distance);
                if (distance > centerLength)
                {
                    endIndex = i;
                    break;
                }
                else
                {
                    endIndex = pathPoints.size() - 20;
                }
                
            }
            //找到全局参考路径的参考终点，并且将x、y坐标调换
            endpathPoint.y = pathPoints[endIndex][0];
            endpathPoint.x = pathPoints[endIndex][1];
            // std::cout << "referPoint: " << pathPoints[index][0] << "," << pathPoints[index][1] << "," << pathPoints[index][2] << std::endl;
            std::cout << "endpathPoint: " << pathPoints[endIndex][0] << "," << pathPoints[endIndex][1] << "," << pathPoints[endIndex][2] << std::endl;
            //如果全局参考路径的参考点数量少于2个，直接返回规划失败
            if (endIndex - index < 2)
            {
                iRet = PathPlan_FAILED;
                return iRet;
            }
            // global to local
            Point2d pixelpathPoint, endCentrPoint, endCentrPoint1, endCentrPoint2;
            //将全局坐标转换为图像的像素坐标位置
            pixelpathPoint.x = endpathPoint.y/ resolution;
            double a = centerBeginPixel*resolution/10;
            pixelpathPoint.y = height - (endpathPoint.x+centerBeginPixel*resolution/10) / resolution;
            double minDis = 10000;
            double x = (height / 2 - pixelpathPoint.y) * resolution;
            double y = (width / 2 - pixelpathPoint.x) * resolution;
            grid_map::Position pose(x, y);
            // find the end point
            if (centerPoint.size() > 1)
            {
                //如果找到多个中心点，找出与全局参考路径距离最近的点作为最终的图像中心点
                Point2d centerPointend;
                for (int i = 0; i < centerPoint.size(); i++)
                {
                    double distY = sqrt(pow((centerPoint[i].x - pixelpathPoint.x), 2));
                    // std::cout << "dist:" << distY << std::endl;
                    if (distY < minDis)
                    {
                        minDis = distY;
                        centerPointend = centerPoint[i];
                    }
                }
                std::cout << "1centerPoint: " << centerPointend.x << "," << centerPointend.y << std::endl;
                double xCen = (height / 2 - centerPointend.y) * resolution;
                double yCen = (width / 2 - centerPointend.x) * resolution;
                grid_map::Position poseCen(xCen, yCen);
                //计算全局轨迹的参考终点与图像的参考终点与障碍物的距离和两个中心点相距的距离。
                double poseCenDistance = map.getObstacleDistance(poseCen);
                double poseDistance = map.getObstacleDistance(pose);
                double path_dis = sqrt(minDis) * resolution;
                double distY = sqrt(pow((centerPointend.y - pixelpathPoint.y), 2)) * resolution;
                std::cout << "poseDistance: " << poseDistance << "poseCenDistance:" << poseCenDistance << "path_dis:" << path_dis << std::endl;
                //如果全局参考轨迹的终点距离障碍物超过1.5米直接用轨迹终点作为局部轨迹规划的终点
                if (poseDistance > 1.5)
                {
                    endCentrPoint = pixelpathPoint;
                }
                else if (path_dis < 2.0 && distY < 2.0 && poseCenDistance > 1.5)
                {
                    endCentrPoint = centerPointend;
                }
                else
                {
                    endCentrPoint = pixelpathPoint;
                }
            }
            else
            {
                //只找到一个图像中心点
                double dist = pow((centerPoint[0].x - pixelpathPoint.x), 2); // + pow((centerPoint[0].y - pixelpathPoint.y), 2);
                double distY = sqrt(pow((centerPoint[0].y - pixelpathPoint.y), 2)) * resolution;
                double path_dis = sqrt(dist) * resolution;
                std::cout << "centerPoint: " << centerPoint[0].x << "," << centerPoint[0].y << "," << path_dis << std::endl;
                double xCen = (height / 2 - centerPoint[0].y) * resolution;
                double yCen = (width / 2 - centerPoint[0].x) * resolution;
                grid_map::Position poseCen(xCen, yCen);
                double poseCenDistance = map.getObstacleDistance(poseCen);
                double poseDistance = map.getObstacleDistance(pose);
                std::cout << "poseDistance: " << poseDistance << "poseCenDistance:" << poseCenDistance << "path_dis:" << path_dis << std::endl;
                //根据不同的条件选取局部轨迹规划的终点
                if (poseDistance > 1.5)
                {
                    endCentrPoint = pixelpathPoint;
                }
                else if (path_dis < 2.0 && distY < 2.0 && poseCenDistance > 1.5)
                {
                    endCentrPoint = centerPoint[0];
                }
                else
                {
                    endCentrPoint = pixelpathPoint;
                }
            }

            std::cout << "carState: " << car_state.x << "," << car_state.y << "," << car_state.heading << std::endl;
            // std::cout << "referPoint: " << referPoint << std::endl;
            std::cout << "endpathPoint: " << endpathPoint << std::endl;
            std::cout << "pixelpathPoint: " << pixelpathPoint << std::endl;
            std::cout << "endCentrPoint: " << endCentrPoint << std::endl;
            // convert endCentrPoint to pixle point将最终的终点转换为图像像素坐标
            Point2d endpixelPoint;
            endpixelPoint.x = height / 2 - endCentrPoint.y;
            endpixelPoint.y = width / 2 - endCentrPoint.x;
            // set start state
            //将起点向上挪2米，不然车辆会检测到碰撞
            start_state.x = (-height / 2) * resolution;
            start_state.y = (width / 2 - (ori_width/2 - cropX)/10) * resolution;
            // set end state 局部轨迹规划的终点做标
            end_state.x = endpixelPoint.x * resolution;
            end_state.y = endpixelPoint.y * resolution;
            // 找到全局参考轨迹上从起点到终点的参考点，要保证参数点数量大于6个
            referenceTurnPoint(endIndex, index, pathPoints, 0, 0);
            // reference_path_plot.erase(reference_path_plot.begin());
            reference_path_plot.emplace_back(end_state);
            int size = reference_path_plot.size();
            //终点距离障碍物的距离
            grid_map::Position poseEnd(end_state.x, end_state.y);
            double endDistance = map.getObstacleDistance(poseEnd);

            double theAngle = atan((reference_path_plot[1].y - start_state.y) / (reference_path_plot[1].x - start_state.x));
            double theAngle2 = regularize_angle(car_state.heading);
            //设置起点和终点的车辆角度，这里需要再考虑下
            start_state.heading = 0;
            end_state.heading = 0;//start_state.heading + pathPoints[endIndex][2] - pathPoints[index][2];
            std::cout << "theAngle and regulater angle :" << theAngle << "," << theAngle2 << "," << std::endl;
            std::cout << "turn start state: " << start_state.x << "," << start_state.y << "," << start_state.heading << std::endl;
            std::cout << "turn end state: " << end_state.x << "," << end_state.y << "," << end_state.heading << std::endl;

            //  8.路径规划目标,路径平滑
            // Calculate.调用局部规划算法
            std::vector<PathOptimizationNS::SlState> result_path;
            std::vector<PathOptimizationNS::State> smoothed_reference_path;
            double maxKappas = 0;
            if (reference_rcv)
            {
                int referDistancenum = 0;
                PathOptimizationNS::PathOptimizer path_optimizer(start_state, end_state, grid_map);
                // 计算全局参考点距离障碍物距离超过1米的数量
                for (int i = 0; i < reference_path_plot.size(); i++)
                {
                    double distance = path_optimizer.calDisObs(reference_path_plot[i].x, reference_path_plot[i].y);
                    if (abs(reference_path_plot[i].k) > maxKappas)
                    {
                        maxKappas = abs(reference_path_plot[i].k);
                    }
                    if (distance >= 1.0)
                    {
                        referDistancenum++;
                    }
                    else
                    {
                        break;
                    }
                }
                std::cout << "referDistancenum: " << referDistancenum << std::endl;
                if(referDistancenum < 6)
                {
                    std::cout<<"refer point is close to obstacle"<<std::endl;
                    iRet = PathPlan_FAILED;
                    return  iRet;
                }
                // calcualte distance from obstacle
                bool opt_ok = path_optimizer.solve(reference_path_plot, &result_path);
                std::cout << "plan state: " << opt_ok << std::endl;
                if (opt_ok)
                {
                    std::cout << "ok!" << std::endl;
                }
                else
                {
                    std::cout << "plan is failed!" << std::endl;
                    iRet = PathPlan_FAILED;
                    return iRet;
                }
                //轨迹平滑
                // reference_path_opt = path_optimizer.getReferencePath();
                // smoothed_reference_path.clear();
                // if (!PathOptimizationNS::isEqual(reference_path_opt.getLength(), 0.0))
                // {
                //     double s = 0.0;
                //     // std::cout<<reference_path_opt.getLength()<<std::endl;
                //     while (s < reference_path_opt.getLength())
                //     {
                //         smoothed_reference_path.emplace_back(reference_path_opt.getXS()(s), reference_path_opt.getYS()(s));
                //         s += 0.5;
                //     }
                // }
                // path_optimizer.printObstacleDistance2(smoothed_reference_path);
                // printf("-------------------------------------------------------");
                // path_optimizer.printObstacleDistance(result_path);
            }
            double angle = -car_state.heading;
            double disX, disY;
            PathOptimizationNS::State state;
            std::cout << "angle: " << angle << std::endl;
            LocalPath.clear();
            cv::Mat img_src2;
            cv::Size sizeOri(width*10,height*10);
            cv::resize(img_src,img_src2,sizeOri,0,0,cv::INTER_AREA);
            for (size_t i = 0; i < result_path.size() - 1; i = i + 1)
            {
                //将规划的局部轨迹转换到全局坐标系
                const auto path_point = result_path.at(i);
                disX = (path_point.x - start_state.x) * cos(angle) + (path_point.y - start_state.y) * sin(angle);
                disY = -(path_point.x - start_state.x) * sin(angle) + (path_point.y - start_state.y) * cos(angle);

                state.x = car_state.x + disX;
                state.y = car_state.y + disY;

                state.heading = path_point.heading + (car_state.heading - start_state.heading);
                LocalPath.push_back(state);
                // draw plan path to image
                // std::cout<<path_point.x<<","<<path_point.y<<std::endl;
                double y = -path_point.x / resolution + height / 2;
                double x = -path_point.y / resolution + width / 2;
                cv::Point p(x*10, y*10);
                circle(img_src2, p, 1, cv::Scalar(0, 0, 0), -1);
            }
            //// 轨迹拼接与曲率平滑
            std::vector<PathOptimizationNS::State> smoothed_trajectory = LocalPath;
            //如果LocalPathRefer不为0，说明上次局部轨迹存储到里面，找出当前车辆相对于上次轨迹的位置，用于多段轨迹的衔接
            //1. 找到当前车辆在上一轨迹中的位置
            double minLocalDis = 10000;
            for (int i = 0; i < LocalPathRefer.size(); i++)
            {
                double distance = pow((tCarState.dX - LocalPathRefer[i].x), 2) + pow((tCarState.dY - LocalPathRefer[i].y), 2);
                if (distance < minLocalDis)
                {
                    minLocalDis = distance;
                    localIndex = i;
                }
            }
            //localIndex暂时没用，后面补充
            if (localIndex > 0)
            {
                std::cout << "refer Local Point: " << LocalPathRefer[localIndex].x << "," << LocalPathRefer[localIndex].y << "," << LocalPathRefer[localIndex].k << std::endl;
                
                // 2. 截取上一轨迹的后段作为拼接基础
                std::vector<PathOptimizationNS::State> base_trajectory;
                for(int i = localIndex; i < LocalPathRefer.size(); ++i) 
                {
                    base_trajectory.push_back(LocalPathRefer[i]);
                }
                // 3. 创建重叠区域
                int overlap_length = 20;  // 重叠区域长度
                // int overlap_start = std::max(0, (int)base_trajectory.size() - overlap_length);
                int overlap_start = 0;
                int overlap_end = std::min(overlap_length, (int)base_trajectory.size());
                std::vector<PathOptimizationNS::State> overlap_region;
                for(int i = 0; i < overlap_end; ++i) 
                {
                    overlap_region.push_back(base_trajectory[i]);
                }
                // 4. 使用加权平均平滑重叠区域
                smoothed_trajectory = overlap_region;
            
                for(int i = 0; i < overlap_region.size() && i < LocalPath.size(); ++i) 
                {
                    double weight = (double)i / overlap_region.size();  // 线性权重
                
                    // 位置平滑
                    smoothed_trajectory[overlap_start + i].x = 
                        (1 - weight) * overlap_region[i].x + weight * LocalPath[i].x;
                    smoothed_trajectory[overlap_start + i].y = 
                        (1 - weight) * overlap_region[i].y + weight * LocalPath[i].y;
                
                    // 航向角平滑
                    smoothed_trajectory[overlap_start + i].heading = 
                        (1 - weight) * overlap_region[i].heading + weight * LocalPath[i].heading;
                
                    // 曲率平滑
                    smoothed_trajectory[overlap_start + i].k = 
                        (1 - weight) * overlap_region[i].k + weight * LocalPath[i].k;
                }

                // 5. 添加新轨迹的剩余部分
                if(smoothed_trajectory.size() > 0)
                {
                    double lastX = smoothed_trajectory[smoothed_trajectory.size()].x;
                    for(int i = overlap_region.size(); i < LocalPath.size(); ++i) 
                    {
                        if(LocalPath[i].x > lastX)
                            smoothed_trajectory.push_back(LocalPath[i]);
                    }
                }else
                {
                    smoothed_trajectory = LocalPath;
                }
                // 6. 全局平滑优化 
                // smoothed_trajectory = GlobalSmooth(smoothed_trajectory);
            }

            if (smoothed_trajectory.size() < 5)
            {
                if (iRet != PathPlan_LASTWRITE)
                {
                    iRet = PathPlan_PARKING;
                }
                return iRet;
            }
            else
            {
                //画出参考点位置
                for (size_t i = 0; i != reference_path_plot.size(); ++i)
                {
                    grid_map::Position pose(reference_path_plot[i].x, reference_path_plot[i].y);
                    // std::cout << "reference path obstacle distance: " << map.getObstacleDistance(pose) << std::endl;
                    double y = -reference_path_plot[i].x / resolution + height / 2;
                    double x = -reference_path_plot[i].y / resolution + width / 2;
                    cv::Point p(x*10, y*10);
                    // std::cout << "reference_path_plot: " << x << "," << y << std::endl;
                    circle(img_src2, p, 3, cv::Scalar(0, 0, 0), -1);
                }
                cv::imwrite("map_plan.jpg", img_src2);
                cv::imshow("image", img_src2);
                cv::waitKey(300);
                std::vector<PathPoint> points;
                geneatePathPoint(smoothed_trajectory, &points);
                SpeedData speed_data; 
                std::vector<T_lqrPose> resultPose;
                double cur_speed = tCarState.dV;
                generateSpeed(Obs, points, cur_speed, max_speed, &speed_data,&resultPose);
                for(int i=0;i<resultPose.size();i++)
                {
                    T_lqrPose lqrPose;
                    lqrPose = resultPose[i];
                    vectorLqrPose->push_back(lqrPose);
                }
            }

            // restore parameters
            updateFlag = true;
            if (iRet != PathPlan_LASTWRITE)
            {
                iRet = PathPlan_UPDATE;
            }
        }
        clock_t time2 = clock();
        cout << "LocalPathPlanning_time:" << double(time2 - time1) / 1000 << "ms" << std::endl;
        return iRet;
    }

    int  generateSpeed(std::vector<T_Obs> j3Obs, std::vector<PathPoint> points, double cur_speed, double max_speed, SpeedData *speed_data,vector<T_lqrPose> *resultPose)
    {
        LocalPathRefer.clear();
        PathOptimizationNS::State localRefer;
        std::cout << "generate speed: " << points.size() << std::endl;
        if(points.size() < 1)
        {
            return 0;
        }
        // create static obstacles
        STObstaclesProcessor st_obstacles_processor_;
        DiscretizedPath discretizedPath(points);
        double total_length = sqrt(pow((points[points.size() - 1].x - points[0].x), 2) + pow((points[points.size() - 1].y - points[0].y), 2));
        std::cout << "discretizedPath Length: " << total_length << std::endl;
        // Map all related obstacles onto ST-Graph.
        auto time1 = std::chrono::system_clock::now();
        st_obstacles_processor_.Init(discretizedPath.Length(),
                                     total_time, discretizedPath);
        // create  obstacle
        std::vector<Obstacle> Obstacles;
        for (int i = 0; i < j3Obs.size(); i++)
        {
            // create static obstacle
            if (j3Obs[i].obs_x < 4) //(j3Obs[i].is_static)
            {
                double rex, rey;

                rex = j3Obs[i].obs_x - j3Obs[i].obs_length / 2;
                rey = abs(j3Obs[i].obs_y - j3Obs[i].obs_width / 2);

                // if (j3Obs[i].obs_pose_type == 2) // 后中点
                {
                    rex = j3Obs[i].obs_x;
                    rey = abs(j3Obs[i].obs_y - j3Obs[i].obs_width / 2);
                }
                Box2d box({rex, rey}, j3Obs[i].obs_angle, j3Obs[i].obs_width, j3Obs[i].obs_length);
                std::unique_ptr<Obstacle> obs = Obstacle::CreateStaticVirtualObstacles("abc", box);
                Obstacle obstacle = *obs;
                Obstacles.push_back(obstacle);
            }

        }

        st_obstacles_processor_.MapObstaclesToSTBoundaries(Obstacles);
        auto time2 = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = time2 - time1;
        std::cout << "Time for ST Obstacles Processing = " << diff.count() * 1000 << " msec." << std::endl;

        std::list<Obstacle> obstacle_list_;
        for (int i = 0; i < Obstacles.size(); i++)
        {
            obstacle_list_.push_back(Obstacles[i]);
        }
        std::vector<const Obstacle *> obstacles_;
        if(obstacle_list_.size() > 0)
        {
           obstacles_.emplace_back(&(obstacle_list_.back())); 
        }
        
        obstacles_.erase(std::remove(obstacles_.begin(),obstacles_.end(),nullptr),obstacles_.end());

        auto all_st_boundaries = st_obstacles_processor_.GetAllSTBoundaries();
        std::vector<const STBoundary *> boundaries;
        std::vector<STBoundary> boundariesG;
        for (const auto &st_boundary : all_st_boundaries)
        {
            boundaries.push_back(&st_boundary.second);
            boundariesG.push_back(st_boundary.second);
        }
        std::cout << "boundaries size: " << boundaries.size() << std::endl;
        if (cur_speed < 0.5)
        {
            cur_speed = 0.5;
        }
        init_point_.v = cur_speed;
        std::cout << "speed information: " << cur_speed << "," << max_speed << std::endl;
        st_graph_data_.LoadData(boundaries, 30.0, init_point_, speed_limit, max_speed,
                                total_length, total_time);
        std::cout << "create dp_st_graph " << std::endl;
        GriddedPathTimeGraph dp_st_graph(st_graph_data_, default_dp_config, obstacles_,
                                         init_point_);

        std::cout << "begin search speed: " << std::endl;
        auto ret = dp_st_graph.Search(speed_data);
        std::cout << "begin process speed: " << std::endl;
        if (total_length > 5 && ret)
        {
            ret = Process(discretizedPath, init_point_, speed_data, boundariesG, max_speed, total_length);
        }
        SpeedData reference_speed_data = *speed_data;
        std::cout << "reference_speed_data size: " << reference_speed_data.size() << std::endl;
        std::vector<double> valitys;
        std::vector<double> acceles;
        for (int i = 0; i < reference_speed_data.size(); i++)
        {
            // std::cout<<"v: "<<speed_data[i].v<<" a: "<<speed_data[i].a<<std::endl;
            valitys.push_back(reference_speed_data[i].v);
            acceles.push_back(reference_speed_data[i].a);
        }
        if (reference_speed_data.size() < 2)
        {
            valitys.push_back(1.2);
            acceles.push_back(0);
        }
        std::vector<double> interValitys = linearInterpolation(valitys, points.size());
        std::vector<double> interAcceles = linearInterpolation(acceles, points.size());
        if (max_speed == 0)
        {
            interValitys[points.size() - 1] = 0;
            interAcceles[points.size() - 1] = 0;
        }
        // std::cout<<interpolated.size()<<std::endl;
        // for(int i=0;i<interpolated.size();i++)
        // {
        //   std::cout<<"interpolated size: "<<interpolated[i]<<std::endl;
        // }

        // string tempFilename2 = string(tempFile);
        // ofstream outFile2(tempFilename2.c_str(), ios::out | ios::binary);
        int id2 = 0;
        vector<T_lqrPose> vectorLqrPoseTest;
        for (size_t i = 0; i != points.size(); ++i)
        {
            T_lqrPose lqrpose;
            id2++;
            lqrpose.id = id2;
            lqrpose.x = points[i].x;
            lqrpose.y = points[i].y;
            lqrpose.z = 0;
            lqrpose.theta = points[i].theta;
            lqrpose.kappas = points[i].kappa;
            lqrpose.v = interValitys[i];
            lqrpose.a = interAcceles[i];
            localRefer.x = points[i].x;
            localRefer.y = points[i].y;
            localRefer.heading = points[i].theta;
            localRefer.k = points[i].kappa;
            LocalPathRefer.push_back(localRefer);
            // outFile2.write((char *)savepose, sizeof(savePose));
            vectorLqrPoseTest.push_back(lqrpose);
            resultPose->push_back(lqrpose);
        }
        return 1;
        // outFile2.close();
    }

    void geneatePathPoint(std::vector<PathOptimizationNS::State> resultPath, std::vector<PathPoint> *pointspath)
    {
        std::cout << "path length" << resultPath.size() << std::endl;
        // write result path to tempFile
        Eigen::Vector3d xyr;
        vector<Eigen::Vector2d> points;
        PiecewiseBezierCurve<2> curve;
        for (size_t i = 0; i != resultPath.size(); ++i)
        {
            Eigen::Vector2d p;
            p.x() = resultPath[i].x;
            p.y() = resultPath[i].y;
            points.push_back(p);
        }
        curve.fit(points, 0.1);
        // resample point
        vector<Point2f> result_path;
        for (Bezier<3, 2> b : curve.getPiecewiseBeziers())
        {
            for (auto &p : b.sampleWithArcLengthParameterized(0.2, true, 5))
            {
                // circle(img, {int(p[0]*10), int(p[1]*10+200)}, 0.2, {0, 0, 255}, -1);
                Point2f po;
                po.x = p[0];
                po.y = p[1];
                result_path.push_back(po);
            }
        }
        result_path.erase(unique(result_path.begin(), result_path.end()), result_path.end());
        std::cout << "remove duplicate elements" << result_path.size() << std::endl;
        vector<double> curvitys = calculate_curvature_difference(result_path);
        cout << "Saving trajectory ..." << endl;

        std::vector<double> x_set, y_set, s_set, heading;
        for (size_t i = 0; i != result_path.size(); ++i)
        {
            x_set.push_back(result_path[i].x);
            y_set.push_back(result_path[i].y);
        }
        s_set.push_back(0);
        double tmp_s = 0;
        for (size_t i = 1; i != result_path.size(); ++i)
        {
            double dis = sqrt(pow(result_path[i].x - result_path[i - 1].x, 2) + pow(result_path[i].y - result_path[i - 1].y, 2));
            tmp_s = tmp_s + dis;
            s_set.push_back(tmp_s);
            // std::cout<<tmp_s<<std::endl;
        }
        PathOptimizationNS::tk::spline x_s, y_s;
        x_s.set_points(s_set, x_set);
        y_s.set_points(s_set, y_set);
        for (int i = 0; i < s_set.size(); i++)
        {
            double h = PathOptimizationNS::getHeading(x_s, y_s, s_set[i]);
            heading.push_back(h);
        }
        for (size_t i = 0; i != result_path.size(); ++i)
        {
            PathPoint point;
            point.x = result_path[i].x;
            point.y = result_path[i].y;
            point.theta = heading[i]; //+ (car_state.heading - start_state.heading);
            point.kappa = curvitys[i];
            point.s = std::hypot(point.x, point.y);
            pointspath->push_back(point);
        }
    }

    static void ColorTransformation(cv::Mat cvImgIn, cv::Mat &cvImgOut)
    {
        if (cvImgIn.channels() == 3)
        {
            cvImgOut = cvImgIn.clone();
        }
        else if (cvImgIn.channels() == 4)
        {
            cv::cvtColor(cvImgIn, cvImgOut, COLOR_RGBA2RGB);
        }
        else if (cvImgIn.channels() == 1)
        {
            cv::cvtColor(cvImgIn, cvImgOut, COLOR_GRAY2RGB);
        }

        return;
    }
    // ##########################################################################################################

#ifdef __cplusplus
};
#endif