// ##########################################################################################################
// create by guoqian on 25-05-15
#include "DataType.hpp"
#include "tools/spline.h"
#include <iostream>
#include <vector>
#include <glob.h>
#include "Eigen/Dense"
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
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
#include "config/planning_flags.hpp"
#include "tools/Map.hpp"
#include "bezier.hpp"
#include <tinyspline_ros/tinysplinecpp.h>

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

    // ##########################################################################################################

    using namespace cv;
    using namespace std;
    using namespace Eigen;

    PathOptimizationNS::State start_state, end_state, car_state, init_state, car_state_local, init_stateObs;
    std::vector<PathOptimizationNS::State> reference_path_plot;
    std::vector<PathOptimizationNS::State> LocalPath;
    std::vector<PathOptimizationNS::State> LocalPathRefer;
    static int localIndex = 0;
    static bool iscurvityTurn = false;
    PathOptimizationNS::ReferencePath reference_path_opt;
    bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;
    std::vector<cv::Point2f> referencePoint;
    double resolution;
    int width, height;
    struct savePose
    {
        int id;
        double x;
        double y;
        double z;
        double theta;
        double kappas;
        double v;
        double a;
    };

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

#define LH_PATHPLANNING_ALG_VERSION ("LH-PAHTPLAN-V1.51") // pathplanning算法版本号

    static cv::Mat g_sFreeSpaceGray;      // 输入freespace, 只申请一次空间，避免频繁申请释放空间产生内存碎片
    static string g_sglobalPathName = ""; // global 路径名称
    static vector<Vec6d> globalPathPoints;
    static E_PathPlanAlgErrorType e_PathPlanAlgErrorType;
    static bool isFirst;
    static bool updateFlag;
    static double max_speed; // m/s pathPoints[endIndex].v
    static double normal_speed;
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
    static int CopyMatImg(T_FreeSpaceImg tFreeSpaceImg, cv::Mat &matDst);
    static bool IsParamLegal(cv::Mat map, T_CarState tCarState, const char *tempFile);
    static void ColorTransformation(cv::Mat cvImgIn, cv::Mat &cvImgOut);
    static void geneatePathPoint(std::vector<PathOptimizationNS::State> result_path, std::vector<PathPoint> *points);
    static void generateSpeed(std::vector<J3Obs> j3Obs, std::vector<PathPoint> points, double cur_speed, double max_speed, SpeedData *speed_data, const char *tempFile);
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
        if (iscurvityTurn)
        {
            // 应用B样条平滑
            int degree = 3;                                   // 三次B样条
            double lambda = 0.003;                            // 平滑参数
            int numControlPoints = curvature_list.size() / 3; // 控制点数量
            std::vector<double> params(curvature_list.size());
            // 创建参数化值 (0到1均匀分布)
            params[0] = 0.0;
            // 曲率自适应参数化
            double alpha = 0.7;       // 曲率权重 (0-1)
            double minSpacing = 0.01; // 最小参数间距
            double maxSpacing = 0.1;  // 最大参数间距

            int n = curvature_list.size();
            // 1. 计算曲率
            // 3. 计算基于曲率的权重
            std::vector<double> weights(n, 0.0);
            for (int i = 0; i < n; ++i)
            {
                weights[i] = (1.0 - alpha) + alpha * curvature_list[i];
            }
            std::vector<double> x;
            for (int i = 0; i < n; ++i)
            {
                double t = static_cast<double>(i) / (n - 1);
                x.push_back(t);
            }
            // 4. 计算弦长累积
            std::vector<double> chordLengths(n, 0.0);
            for (int i = 1; i < n; ++i)
            {
                double dx = x[i] - x[i - 1];
                double dy = curvature_list[i] - curvature_list[i - 1];
                chordLengths[i] = chordLengths[i - 1] + sqrt(dx * dx + dy * dy);
            }

            // 5. 计算加权累积距离
            std::vector<double> weightedDists(n, 0.0);
            for (int i = 1; i < n; ++i)
            {
                double dx = x[i] - x[i - 1];
                double dy = curvature_list[i] - curvature_list[i - 1];
                double dist = sqrt(dx * dx + dy * dy);
                weightedDists[i] = weightedDists[i - 1] + dist * weights[i];
            }

            // 6. 生成自适应参数化
            double totalWeightedDist = weightedDists.back();
            for (int i = 0; i < n; ++i)
            {
                params[i] = weightedDists[i] / totalWeightedDist;
            }

            // 7. 确保参数单调递增并满足间距约束
            for (int i = 1; i < n; ++i)
            {
                double delta = params[i] - params[i - 1];
                if (delta < minSpacing)
                {
                    params[i] = params[i - 1] + minSpacing;
                }
                else if (delta > maxSpacing)
                {
                    params[i] = params[i - 1] + maxSpacing;
                }
            }

            // 重新归一化到[0,1]
            double finalMax = params.back();
            for (double &p : params)
            {
                p /= finalMax;
            }
            std::vector<double> smoothed = bsplineCurvatureSmoothing(
                curvature_list, params, degree, lambda, numControlPoints);

            smoothed[smoothed.size() - 1] = smoothed[smoothed.size() - 2];
            return smoothed;
        }
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
    void referenceTurnPoint(int endIndex, int index, vector<Vec6d> pathPoints, double thX, double thY)
    {
        int num = (endIndex - index) / 10;
        reference_path_plot.clear();
        referencePoint.clear();
        std::vector<double> kappas;
        double refX, refY;
        for (size_t i = 0; i < endIndex - index + 1; i = i + num)
        {
            Point2d globalP, pathP;
            globalP.x = pathPoints[index + i][0] + thX;
            globalP.y = pathPoints[index + i][1] + thY;
            pathP.x = globalP.x * cos(car_state.heading) + globalP.y * sin(car_state.heading);
            pathP.y = -globalP.x * sin(car_state.heading) + globalP.y * cos(car_state.heading);
            // pathP.x = globalP.x;
            // pathP.y = globalP.y;

            Point2d imagepathP, pixelP;
            imagepathP.x = abs(pathP.x - car_state_local.x) - height / 2 * resolution;
            imagepathP.y = pathP.y - car_state_local.y;
            pixelP.x = height / 2 - imagepathP.y / resolution;
            pixelP.y = width / 2 - imagepathP.x / resolution;
            Point2d endpixelP;
            endpixelP.x = width / 2 - pixelP.y;
            endpixelP.y = height / 2 - pixelP.x;
            double xs = endpixelP.x * resolution;
            double ys = endpixelP.y * resolution;
            cv::Point2f point0(xs, ys);
            if (i == 0)
            {
                refX = xs;
                refY = ys;
            }
            referencePoint.push_back(point0);
            kappas.push_back(pathPoints[index + i][3]);
            // double y = -xs / resolution + 74;
            // double x = -ys / resolution + 96;
            // cv::Point p(x, y);
            // std::cout << "reference_path_plot111111: " << x << "," << y << std::endl;
        }

        double disX = refX - start_state.x;
        double disY = refY - start_state.y;
        for (int i = 0; i < referencePoint.size(); i++)
        {
            PathOptimizationNS::State reference_point;
            reference_point.x = referencePoint[i].x - disX + thX;
            reference_point.y = referencePoint[i].y - disY + thY;
            reference_point.k = kappas[i];
            reference_path_plot.emplace_back(reference_point);
            reference_rcv = reference_path_plot.size() >= 6;
            // std::cout << "received a reference point" <<referencePoint[i]<< std::endl;
        }
    }

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
                ;
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

    // Init
    int InitPathPlanning(const char *globalPath)
    {
        vector<Vec6d>().swap(globalPathPoints);
        globalPathPoints.clear();
        g_sglobalPathName = string(globalPath);
        if (NULL == globalPath)
        {
            printf("InitPathPlanning input param error, ----------globalPath: %s \n", globalPath);
            return PathPlan_INIT_FAILED;
        }

        savePose *savep = new savePose;
        ifstream inFile(g_sglobalPathName, ios::in | ios::binary);
        while (inFile.read((char *)savep, sizeof(savePose)))
        {
            Vec6d p;
            p[0] = savep->x;
            p[1] = savep->y;
            p[2] = savep->theta + 1.570796;
            p[3] = savep->kappas;
            p[4] = savep->v;
            p[5] = savep->a;
            globalPathPoints.push_back(p);
        }
        delete savep;
        isFirst = true;
        width = 500;
        height = 500;
        resolution = 0.1;
        updateFlag = false;
        normal_speed = 2.7;
        max_speed = normal_speed;
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
            speed_limit.AppendSpeedLimit(s, 3.0);
        }
        init_point_.path_point.x = 0.0;
        init_point_.path_point.y = 0.0;
        init_point_.path_point.z = 0.0;
        init_point_.path_point.kappa = 0.0;
        init_point_.v = 0.0;
        init_point_.a = 0.0;
        st_graph_data_ = StGraphData();
        imageId = 0;
        printf("InitPathPlanning, ----------globalPath: %s \n", globalPath);
        // Ptr<ml::TrainData> mlDataX, mlDataY;
        // mlDataX = ml::TrainData::loadFromCSV("/app/slam/mapX.csv", 1);
        // mlDataY = ml::TrainData::loadFromCSV("/app/slam/mapY.csv", 1);
        // dataX = mlDataX->getTrainSamples();
        // dataY = mlDataY->getTrainSamples();
        printf("InitPathPlanning, ----------end:");
        return PathPlan_OK;
    }

    int GetPathPlanAlgVersion(char pucAlgVersionArray[ARRAY_LEN])
    {

        snprintf(pucAlgVersionArray, ARRAY_LEN, "%s", LH_PATHPLANNING_ALG_VERSION);
        return PathPlan_OK;
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

    static bool IsParamLegal(cv::Mat map, T_CarState tCarState, const char *tempFile)
    {
        bool bRet = true;
        // printf("isPare tempFile name: %s\n", tempFile);
        if ((map.empty()) || (NULL == tempFile))
        {
            if (map.empty())
            {
                e_PathPlanAlgErrorType = PathPlan_NO_IMAGE_YET;
            }
            else if (NULL == tempFile)
            {
                e_PathPlanAlgErrorType = PathPlan_TEMPFILE_EMPTY;
            }

            bRet = false;
            printf("==========IsParamLegal check param is error, map.empty, tempFile: %s\n", tempFile);
        }
        return bRet;
    }

    int getReferPoint(T_CarState tCarState, float *referPointX, float *referPointY)
    {
        // car current pose in odom axis convert to camera axis
        tCarState.dTheta = tCarState.dTheta + 1.570796;
        tCarState.dX = tCarState.dX; //+ 2 * cos(-tCarState.dTheta);
        tCarState.dY = tCarState.dY; //- 2 * sin(-tCarState.dTheta);

        int iRet = PathPlan_OK;

        double minDistance = 10000;
        int index = 0;
        // find the path respecting point
        for (int i = 0; i < globalPathPoints.size(); i++)
        {
            double distance = pow((tCarState.dX - globalPathPoints[i][0]), 2) + pow((tCarState.dY - globalPathPoints[i][1]), 2);
            if (distance < minDistance)
            {
                minDistance = distance;
                index = i;
            }
        }
        Point2d referPoint;
        referPoint.x = globalPathPoints[index][0];
        referPoint.y = globalPathPoints[index][1];
        *referPointX = referPoint.x;
        *referPointY = referPoint.y;
        return iRet;
    }

    // 前视摄像头安装角度水平，鱼眼矫正后有0.7m盲区
    int LocalPathPlanning(J3PerceptionData *j3p_data, T_CarState tCarState, const char *tempFile, bool readOver, bool needPath, bool isTurn, double laneDis, bool backPath, float *referPointX, float *referPointY, double cur_speed, double turnDistance, char *lidarinfo)
    {
        clock_t time1 = clock();
        lidarInfo *tempLidarInfo = (lidarInfo *)lidarinfo;
        // std::cout << "Lidar info object num: " << std::endl;
        imageId++;
        bool needPathNow = needPath;
        bool shortPath = false;
        bool needPathObs = false;
        // lidar speed judge
        for (int i = 0; i < tempLidarInfo->objNum; i++)
        {
            if (tempLidarInfo->objPoints[i].pointNum < 2 || tempLidarInfo->objPoints[i].points[0].z < 0)
            {
                continue;
            }
            for (int j = 0; j < tempLidarInfo->objPoints[i].pointNum - 1; j++)
            {
                if (abs(tempLidarInfo->objPoints[i].points[j].y) < 0.8 && abs(tempLidarInfo->objPoints[i].points[j].x) < 2.0 && abs(tempLidarInfo->objPoints[i].points[j].x) > 0.0)
                {
                    max_speed = 0.0;
                    // needPathNow = true;
                    std::cout << "lidarinfo is close to obstacle 0.8: " << tempLidarInfo->objPoints[i].points[j].y << std::endl;
                }
                else if (abs(tempLidarInfo->objPoints[i].points[j].y) < 1.3 && abs(tempLidarInfo->objPoints[i].points[j].x) < 5.0)
                {
                    if (max_speed > 1.0)
                    {
                        max_speed = 1.2;
                    }
                    needPath = true;
                    std::cout << "lidarinfo is close to obstacle 1.3: " << tempLidarInfo->objPoints[i].points[j].y << std::endl;
                }
            }
        }
        // draw map
        Mat new_map;
        unsigned long time;
        std::vector<J3Obs> j3Obs;
        bool optok = true;
        double minDistance = 10000;
        double minLocalDis = 10000;
        int index = 0;
        localIndex = 0;
        // find the path respecting point
        for (int i = 0; i < globalPathPoints.size(); i++)
        {
            double distance = pow((tCarState.dX - globalPathPoints[i][0]), 2) + pow((tCarState.dY - globalPathPoints[i][1]), 2);
            if (distance < minDistance)
            {
                minDistance = distance;
                index = i;
            }
        }
        for (int i = 0; i < LocalPathRefer.size(); i++)
        {
            double distance = pow((tCarState.dX - LocalPathRefer[i].x), 2) + pow((tCarState.dY - LocalPathRefer[i].y), 2);
            if (distance < minLocalDis)
            {
                minLocalDis = distance;
                localIndex = i;
            }
        }
        if (turnDistance < 20 && turnDistance > 10)
        {
            double maxmK = 0;
            for (int i = index; i < index + 200; i++)
            {
                double kapps = globalPathPoints[i][3];
                if (abs(kapps) > maxmK)
                {
                    maxmK = abs(kapps);
                }
            }
            // if (maxmK > 0.06)
            // {
            //     optok = false;
            // }
            max_speed = 1.1; // m/s
            needPath = true;
            std::cout << "Too close to turn: " << max_speed << std::endl;
        }
        draw_map::draw_front_perception_gray(*j3p_data, new_map, time, j3Obs, optok, *tempLidarInfo);
        // std::cout << "draw J3data end" << std::endl;
        // car current pose in odom axis convert to camera axis
        tCarState.dTheta = tCarState.dTheta + 1.570796;
        tCarState.dX = tCarState.dX; //+ 2 * cos(-tCarState.dTheta);
        tCarState.dY = tCarState.dY; //- 2 * sin(-tCarState.dTheta);
        // target speed set
        double refery, referx, TTCtime;
        bool normalPath = false;
        for (int i = 0; i < j3Obs.size(); i++)
        {
            bool is_oblique = false; // 横向的车以中心画Rect
            if ((j3Obs[i].obs_angle > 42 && j3Obs[i].obs_angle < 160) ||
                (j3Obs[i].obs_angle < -42 && j3Obs[i].obs_angle > -160))
            {
                is_oblique = true;
            }
            if (is_oblique)
            {
                double w = j3Obs[i].obs_width;
                double h = j3Obs[i].obs_length;
                j3Obs[i].obs_width = h;
                j3Obs[i].obs_length = w;
            }
            referx = j3Obs[i].obs_x - j3Obs[i].obs_length / 2;
            refery = abs(j3Obs[i].obs_y - j3Obs[i].obs_width / 2);
            if (j3Obs[i].obs_pose_type == 2) // 后中点
            {
                referx = j3Obs[i].obs_x;
                refery = abs(j3Obs[i].obs_y - j3Obs[i].obs_width / 2);
            }
            double max_x_time = 5;
            double max_y_distance = 1.2;
            if (j3Obs[i].obs_class == 2) // 障碍物的类型，范围0~4。类别0=INVALID,1 = VEHICLE,2 = PEDESTRIAN,3 = CYCLIST,4 = BICYCLE
            {
                max_x_time = 6;
                max_y_distance = 1.5;
            }
            TTCtime = referx / (cur_speed + 2);
            if (refery < max_y_distance)
            {
                if ((referx < 20 || (TTCtime < max_x_time && TTCtime > 1)))
                {
                    if (max_speed > 1.0 && max_speed != 0)
                    {
                        max_speed = 1.2;
                    }
                    needPathObs = true;
                    needPath = true;
                    // normalPath = true;
                    std::cout << "Too close to an obstacle: " << TTCtime << "," << refery << std::endl;
                }
            }
            if (refery < 1)
            {
                if (referx < 2)
                {
                    max_speed = 0.0;
                    // needPath = true;
                    std::cout << "Too close to an obstacle < 1 stop: " << referx << "," << refery << std::endl;
                }
            }
            // std::cout << "j3 Obs: " << j3Obs[i].obs_x << "," << j3Obs[i].obs_y << "," << j3Obs[i].obs_width << "," << j3Obs[i].obs_length << std::endl;
        }
        if (needPathObs && true == updateFlag)
        {
            printf("To init obs state\n");
            init_stateObs.x = tCarState.dX;
            init_stateObs.y = tCarState.dY;
        }
        else if (needPathObs && false == updateFlag)
        {
            init_stateObs.x = tCarState.dX + 15;
            init_stateObs.y = tCarState.dY + 10;
        }
        double travelDistanceObs = pow((init_stateObs.x - tCarState.dX), 2) + pow((init_stateObs.y - tCarState.dY), 2);
        // printf("begin local path planning\n");
        int iRet = PathPlan_OK;
        // bool bIsParamLegal = false;
        // bIsParamLegal = IsParamLegal(new_map, tCarState, tempFile);
        // if (false == bIsParamLegal)
        // {
        //     printf("==========LocalPathPlanning input param is error!\n");
        //     return PathPlan_FAILED;
        // }

        if (isFirst || true == updateFlag)
        {
            printf("To init state\n");
            init_state.x = tCarState.dX;
            init_state.y = tCarState.dY;
            max_speed = normal_speed;
            updateFlag = false;
        }
        int numSize = LocalPath.size();
        if (numSize > 0)
        {
            // std::cout << "local path: " << LocalPath.size() << "," << LocalPath[numSize - 1].x << "," << LocalPath[numSize - 1].y << std::endl;
            if ((pow((tCarState.dX - LocalPath[numSize - 1].x), 2) + pow((tCarState.dY - LocalPath[numSize - 1].y), 2)) < 64)
            {
                shortPath = true;
            }
        }
        double travelDistance = pow((init_state.x - tCarState.dX), 2) + pow((init_state.y - tCarState.dY), 2);
        // 7. 选取终点
        double maxDis = 0;
        int maxIndex = 32;
        for (int i = 0; i < 64; i++)
        {
            double x = j3p_data->J3P_Freespace.contours_points[i].x;
            if (x > maxDis)
            {
                maxDis = x;
                maxIndex = i;
            }
        }
        double targetFreespaceX = j3p_data->J3P_Freespace.contours_points[maxIndex].x;
        double targetFreespaceY = j3p_data->J3P_Freespace.contours_points[maxIndex].y;
        double targetDistance = (targetFreespaceX) * (targetFreespaceX) + targetFreespaceY * targetFreespaceY;

        if (targetDistance > 1200)
        {
            targetDistance = 35 * 35;
        }
        Point2d referPoint;
        referPoint.x = globalPathPoints[index][0];
        referPoint.y = globalPathPoints[index][1];
        *referPointX = referPoint.x;
        *referPointY = referPoint.y;
        double targetDis = 20 * 20;
        if (isTurn || turnDistance < 20)
        {
            targetDis = 20 * 20;
        }
        else if (cur_speed < 2)
        {
            targetDis = 15 * 15;
        }
        std::cout << "planning :" << travelDistanceObs << "," << needPathObs << "," << needPath << "," << shortPath << "," << travelDistance << "," << std::endl;
        double carAngle = regularize_angle(tCarState.dTheta);
        if (!needPathNow && !shortPath && abs(carAngle > 0.2))
        {
            std::cout << "car theta is larger than 0.2" << std::endl;
            return PathPlan_OK;
        }
        if ((isFirst || needPathNow || (needPath && travelDistance > 100) || shortPath) || (travelDistance > targetDis && readOver == true))
        // if ((isFirst ||(needPathObs && travelDistanceObs > 100)|| needPathNow || shortPath) || (travelDistance > targetDis && readOver == true))
        {
            if (isTurn || turnDistance < 20)
            {
                iscurvityTurn = true;
            }
            std::cout << "imageId: " << imageId << std::endl;
            // imwrite("/app/slam/" + to_string(imageId) + "J3map.jpg", new_map); // 保存图
            // read original path
            vector<Vec6d> pathPoints;
            pathPoints.assign(globalPathPoints.begin(), globalPathPoints.end());
            int num = pathPoints.size();
            // std::cout << "global path :" << num << "," << globalPathPoints.size() << "," << pathPoints[num - 1][0] << "," << pathPoints[num - 1][1] << std::endl;
            if (num == 0)
            {
                printf("globalPathPoint is empty:\n");
                return PathPlan_FAILED;
            }
            // car current state pathPoint the last point is distance <8m the last path planning
            double pathDistance = pow((tCarState.dX - pathPoints[num - 1][0]), 2) + pow((tCarState.dY - pathPoints[num - 1][1]), 2);
            if (pathDistance < 65)
            {
                printf("the last path planning\n");
                max_speed = 0.0;
                iRet = PathPlan_LASTWRITE;
            }
            // 2. binaryImage to get Areas of exercise
            cv::Mat gray, binaryImage;
            if (new_map.channels() == 3)
            {
                cv::cvtColor(new_map, gray, COLOR_BGR2GRAY);
            }
            else
            {
                new_map.copyTo(gray);
            }
            threshold(gray, binaryImage, 120, 255, 0);
            // 111111111111111111111111111111111111111111111111111111111111111111111111111111111
            vector<Point> points;
            double cropX;
            cv::Mat img_src;
            findNonZero(binaryImage, points);
            if (points.empty())
            {
                img_src = binaryImage;
            }
            else
            {
                Rect bb = boundingRect(points);
                // std::cout << bb.x << bb.width << std::endl;
                img_src = binaryImage(bb);
                height = img_src.size().height;
                width = img_src.size().width;
                cropX = bb.x;
            }
            // 6. 选取终点,设定候选目标半径为 8m =
            cv::Mat circleImage(img_src.size(), CV_8UC1, Scalar(0));
            double targetPixel = targetFreespaceX / resolution;
            vector<Point2d> centerPoint;
            double downpixel = 50;
            while (centerPoint.size() == 0)
            {
                // std::cout << targetPixel << "," << maxIndex << std::endl;
                if (targetPixel > 500)
                {
                    targetPixel = 500;
                }
                else if (targetPixel - downpixel < 100)
                {
                    targetPixel = downpixel + 100;
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
                    if (cv::contourArea(contours[i]) > 500)
                    {
                        Moments M;
                        Point2d point;
                        M = moments(contours[i]);
                        point.x = double(M.m10 / M.m00);
                        point.y = double(M.m01 / M.m00);
                        centerPoint.push_back(point);
                        // circle(interSection, point, 5, Scalar(0, 0, 0), -1);
                        // std::cout<<point<<std::endl;
                    }
                }
                downpixel = downpixel * 2;
                if (downpixel > 300)
                {
                    break;
                }
                // cv::imshow("interSection", interSection);
                // cv::waitKey(0);
            }
            if (turnDistance < 30 && !isTurn)
            {
                targetDistance = 15 * 15;
            }
            if (centerPoint.size() > 0)
            {
                double centerX = (height - centerPoint[0].y) * resolution;
                if (sqrt(targetDistance) > centerX)
                {
                    targetDistance = centerX * centerX;
                }
            }

            // Initialize grid map from image.
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
            if (isFirst || backPath)
            {
                targetDistance = 100;
            }
            else if (isTurn || turnDistance < 20)
            {
                targetDistance = 40 * 40;
            }
            else if (max_speed < 1.5)
            {
                targetDistance = 15 * 15;
            }
            std::cout << "targetDistance distance: " << targetDistance << std::endl;
            for (int i = index; i < pathPoints.size(); i++)
            {
                double distance = pow((referPoint.x - pathPoints[i][0]), 2) + pow((referPoint.y - pathPoints[i][1]), 2);
                // printf(i<<","<<pathPoints[i]<<"distance: "<<distance);
                if (abs(abs(car_state.heading) - 1.57) < 0.5)
                {
                    if (pow((referPoint.x - pathPoints[i][0]), 2) > 400)
                    {
                        endIndex = i;
                        break;
                    }
                    else if (distance > targetDistance)
                    {
                        endIndex = i;
                        break;
                    }
                    else
                    {
                        endIndex = pathPoints.size() - 1;
                    }
                }
                else
                {
                    if (pow((referPoint.y - pathPoints[i][1]), 2) > 400)
                    {
                        endIndex = i;
                        break;
                    }
                    else if (distance > targetDistance)
                    {
                        endIndex = i;
                        break;
                    }
                    else
                    {
                        endIndex = pathPoints.size() - 1;
                    }
                }
            }
            int cca = index + endIndex / 3;
            if (pathPoints[cca][4] < 1.3 && pathPoints[endIndex][4] < 1.3)
            {
                if (max_speed > 1.0)
                {
                    max_speed = 1.2;
                }
                std::cout << "reference speed is <1.2: " << pathPoints[endIndex][4] << std::endl;
            }
            double thX = car_state.x - pathPoints[index][0];
            double thY = car_state.y - pathPoints[index][1];
            double thH = car_state.heading - pathPoints[index][2];
            double thXlocal, thYlocal;
            if (localIndex > 0)
            {
                thXlocal = LocalPathRefer[localIndex].x - pathPoints[index][0];
                thYlocal = LocalPathRefer[localIndex].y - pathPoints[index][1];
            }
            double thXL, thYL;
            if (abs(thX) > 1.0 || abs(thY) > 1.0 || abs(thH) > 0.3)
            {
                thXL = thX;
                thYL = thY;
            }
            else
            {
                thXL = 0;
                thYL = 0;
            }
            double addX = 0;
            double addY = 0;
            if (abs(thXlocal) > 0.2 || abs(thYlocal) > 0.2)
            {
                addX = thXlocal;
                addY = thYlocal;
            }
            std::cout << "compensate value:" << thX << "," << thY << "," << thXL << "," << thYL << "," << thXlocal << "," << thYlocal << std::endl;
            endpathPoint.x = pathPoints[endIndex][0];
            endpathPoint.y = pathPoints[endIndex][1];
            std::cout << "referPoint: " << pathPoints[index][0] << "," << pathPoints[index][1] << "," << pathPoints[index][2] << std::endl;
            if (localIndex > 0)
            {
                std::cout << "refer Local Point: " << LocalPathRefer[localIndex].x << "," << LocalPathRefer[localIndex].y << "," << LocalPathRefer[localIndex].k << std::endl;
            }
            std::cout << "endpathPoint: " << pathPoints[endIndex][0] << "," << pathPoints[endIndex][1] << "," << pathPoints[endIndex][2] << std::endl;
            if (endIndex - index < 2)
            {
                iRet = PathPlan_FAILED;
                return iRet;
            }
            // global to local
            endpathPoint_local.x = endpathPoint.x * cos(car_state.heading) + endpathPoint.y * sin(car_state.heading);
            endpathPoint_local.y = -endpathPoint.x * sin(car_state.heading) + endpathPoint.y * cos(car_state.heading);
            car_state_local.x = car_state.x * cos(car_state.heading) + car_state.y * sin(car_state.heading);
            car_state_local.y = -car_state.x * sin(car_state.heading) + car_state.y * cos(car_state.heading);

            Point2d imagepathPoint, pixelpathPoint, endCentrPoint, endCentrPoint1, endCentrPoint2;
            imagepathPoint.x = abs(endpathPoint_local.x - car_state_local.x) - height / 2 * resolution;
            imagepathPoint.y = endpathPoint_local.y - car_state_local.y;
            pixelpathPoint.x = width / 2 - imagepathPoint.y / resolution;
            pixelpathPoint.y = height / 2 - imagepathPoint.x / resolution;
            double minDis = 10000;
            double x = (height / 2 - pixelpathPoint.y) * resolution;
            double y = (width / 2 - pixelpathPoint.x) * resolution;
            grid_map::Position pose(x, y);
            double angleCh = abs(pathPoints[endIndex - 20][2] - pathPoints[index][2]);
            if (isFirst || backPath)
            {
                std::cout << "use mapping path curvity points0" << std::endl;
                LocalPath.clear();
                PathOptimizationNS::State state;
                for (size_t i = index; i < endIndex; i++)
                {
                    state.x = thXL + pathPoints[i][0];
                    state.y = thYL + pathPoints[i][1];
                    state.heading = pathPoints[i][2];
                    LocalPath.push_back(state);
                }
                std::vector<PathPoint> points;
                geneatePathPoint(LocalPath, &points);
                SpeedData speed_data;
                generateSpeed(j3Obs, points, cur_speed, max_speed, &speed_data, tempFile);
                max_speed = normal_speed;
                updateFlag = true;
                if (iRet != PathPlan_LASTWRITE)
                {
                    iRet = PathPlan_UPDATE;
                }
                isFirst = false;
                return iRet;
            }
            else if (isTurn && backPath)
            {
                std::cout << "use mapping path curvity points2" << std::endl;
                LocalPath.clear();
                PathOptimizationNS::State state;
                for (size_t i = index; i < endIndex; i++)
                {
                    state.x = addX + pathPoints[i][0];
                    state.y = addY + pathPoints[i][1];
                    state.heading = pathPoints[i][2];
                    LocalPath.push_back(state);
                }

                if (LocalPath.size() < 5)
                {
                    if (iRet != PathPlan_LASTWRITE)
                    {
                        iRet = PathPlan_PARKING;
                    }
                    return iRet;
                }
                std::vector<PathPoint> points;
                geneatePathPoint(LocalPath, &points);
                SpeedData speed_data;
                generateSpeed(j3Obs, points, cur_speed, max_speed, &speed_data, tempFile);
                max_speed = normal_speed;
                updateFlag = true;
                if (iRet != PathPlan_LASTWRITE)
                {
                    iRet = PathPlan_UPDATE;
                }
                return iRet;
            }
            else if (!isTurn && turnDistance < 30 && (abs(thXlocal) > 0.2 || abs(thYlocal) > 0.2))
            {
                normalPath = true;
                std::cout << "isTurn car is too far to line" << thXL << std::endl;
            }
            else if (isTurn || turnDistance < 20 || angleCh > 0.2)
            {
                double mK = 0;
                for (int i = index; i < endIndex; i++)
                {
                    double kapps = pathPoints[i][3];
                    if (abs(kapps) > mK)
                    {
                        mK = abs(kapps);
                    }
                }
                std::cout << "isTurn max kappas is: " << mK << std::endl;
                if ((abs(thXlocal) > 0.5 || abs(thYlocal) > 0.5) || ((abs(thXlocal) > 0.2 || abs(thYlocal) > 0.2) && turnDistance > 10) || (turnDistance < 10 && mK < 0.04))
                {
                    max_speed = 1.1;
                    std::cout << "isTurn thXL is too far to line" << mK << std::endl;
                }
                else
                {
                    std::cout << "use mapping path curvity points3" << std::endl;
                    LocalPath.clear();
                    PathOptimizationNS::State state;
                    for (size_t i = index; i < endIndex; i++)
                    {
                        state.x = addX + pathPoints[i][0];
                        state.y = addY + pathPoints[i][1];
                        state.heading = pathPoints[i][2];
                        LocalPath.push_back(state);
                    }

                    if (LocalPath.size() < 5)
                    {
                        if (iRet != PathPlan_LASTWRITE)
                        {
                            iRet = PathPlan_PARKING;
                        }
                        return iRet;
                    }
                    std::vector<PathPoint> points;
                    geneatePathPoint(LocalPath, &points);
                    SpeedData speed_data;
                    max_speed = 1.1;
                    generateSpeed(j3Obs, points, cur_speed, max_speed, &speed_data, tempFile);
                    max_speed = normal_speed;
                    updateFlag = true;
                    if (iRet != PathPlan_LASTWRITE)
                    {
                        iRet = PathPlan_UPDATE;
                    }
                    return iRet;
                }
            }
            else if (centerPoint.size() == 0)
            {
                std::cout << "use mapping path curvity points1" << std::endl;
                if (max_speed > 1.0)
                {
                    max_speed = 1.2;
                }
                LocalPath.clear();
                PathOptimizationNS::State state;
                if (index + 120 < endIndex)
                {
                    endIndex = index + 120;
                }
                for (size_t i = index; i < endIndex; i++)
                {
                    state.x = addX + pathPoints[i][0];
                    state.y = addY + pathPoints[i][1];
                    state.heading = pathPoints[i][2];
                    LocalPath.push_back(state);
                }

                if (LocalPath.size() < 5)
                {
                    if (iRet != PathPlan_LASTWRITE)
                    {
                        iRet = PathPlan_PARKING;
                    }
                    return iRet;
                }
                std::vector<PathPoint> points;
                geneatePathPoint(LocalPath, &points);
                SpeedData speed_data;
                generateSpeed(j3Obs, points, cur_speed, max_speed, &speed_data, tempFile);
                max_speed = normal_speed;
                updateFlag = true;
                if (iRet != PathPlan_LASTWRITE)
                {
                    iRet = PathPlan_UPDATE;
                }
                return iRet;
            }
            isFirst = false; // plan time and pose time is not sync in the first time
            // if (centerPoint.size() == 0)
            // {
            // printf("cannot find centerPoint\n");
            // return PathPlan_NOCENTERPOINT;
            // }
            // find the end point
            if (centerPoint.size() > 1)
            {
                Point2d centerPointend;
                for (int i = 0; i < centerPoint.size(); i++)
                {
                    double distY = pow((centerPoint[i].x - pixelpathPoint.x), 2);
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
                double poseCenDistance = map.getObstacleDistance(poseCen);
                double poseDistance = map.getObstacleDistance(pose);
                double path_dis = sqrt(minDis) * resolution;
                double distY = sqrt(pow((centerPointend.y - pixelpathPoint.y), 2)) * resolution;
                std::cout << "poseDistance: " << poseDistance << "poseCenDistance:" << poseCenDistance << "path_dis:" << path_dis << std::endl;

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
            // find the end point
            if (normalPath)
            {
                endCentrPoint = pixelpathPoint;
            }
            std::cout << "carState: " << car_state.x << "," << car_state.y << "," << car_state.heading << std::endl;
            std::cout << "referPoint: " << referPoint << std::endl;
            std::cout << "endpathPoint: " << endpathPoint << std::endl;
            // std::cout << "imagepathPoint: " << imagepathPoint << std::endl;
            std::cout << "pixelpathPoint: " << pixelpathPoint << std::endl;
            std::cout << "endCentrPoint: " << endCentrPoint << std::endl;
            // convert endCentrPoint to pixle point
            Point2d endpixelPoint;
            endpixelPoint.x = height / 2 - endCentrPoint.y;
            endpixelPoint.y = width / 2 - endCentrPoint.x;
            // set start state
            // start_state.x = -height / 2 * resolution;
            // start_state.y = 0.0 * resolution;
            start_state.x = (-height / 2) * resolution;
            start_state.y = (width / 2 - (250.0 - cropX)) * resolution;
            // set end state
            end_state.x = endpixelPoint.x * resolution;
            end_state.y = endpixelPoint.y * resolution;
            if (endIndex - index < 2)
            {
                iRet = PathPlan_FAILED;
                return iRet;
            }
            // car_state.heading = -0.066;
            if (true)
            {
                referenceTurnPoint(endIndex, index, pathPoints, thX, thY);
                reference_path_plot.erase(reference_path_plot.begin());
                int size = reference_path_plot.size();

                grid_map::Position poseEnd(end_state.x, end_state.y);
                double endDistance = map.getObstacleDistance(poseEnd);
                grid_map::Position poseEnd1(end_state.x, end_state.y + 1);
                double endDistance1 = map.getObstacleDistance(poseEnd1);
                grid_map::Position poseEnd2(end_state.x, end_state.y + 2);
                double endDistance2 = map.getObstacleDistance(poseEnd2);
                grid_map::Position poseEnd3(end_state.x, end_state.y - 1);
                double endDistance3 = map.getObstacleDistance(poseEnd3);
                grid_map::Position poseEnd4(end_state.x, end_state.y - 2);
                double endDistance4 = map.getObstacleDistance(poseEnd4);
                if (endDistance < 0.1)
                {
                    if (endDistance1 > 1)
                    {
                        end_state.x = end_state.x;
                        end_state.y = end_state.y + 1;
                    }
                    else if (endDistance2 > 1)
                    {
                        end_state.x = end_state.x;
                        end_state.y = end_state.y + 2;
                    }
                    else if (endDistance3 > 1)
                    {
                        end_state.x = end_state.x;
                        end_state.y = end_state.y - 1;
                    }
                    else if (endDistance4 > 1)
                    {
                        end_state.x = end_state.x;
                        end_state.y = end_state.y - 2;
                    }
                }
                double theAngle = atan((reference_path_plot[1].y - start_state.y) / (reference_path_plot[1].x - start_state.x));
                double theAngle2 = regularize_angle(car_state.heading);
                // if (abs(theAngle - theAngle2) < 0.2)
                // {
                //     start_state.heading = theAngle2;
                // }
                // else
                // {
                //     start_state.heading = theAngle;
                // }
                start_state.heading = 0;
                end_state.heading = start_state.heading + pathPoints[endIndex][2] - pathPoints[index][2];
                std::cout << "theAngle and regulater angle :" << theAngle << "," << theAngle2 << "," << std::endl;
                std::cout << "turn start state: " << start_state.x << "," << start_state.y << "," << start_state.heading << std::endl;
                std::cout << "turn end state: " << end_state.x << "," << end_state.y << "," << end_state.heading << std::endl;
            }

            //  8.路径规划目标,路径平滑
            // Calculate.
            std::vector<PathOptimizationNS::SlState> result_path;
            std::vector<PathOptimizationNS::State> smoothed_reference_path;
            double maxKappas = 0;
            if (reference_rcv)
            {
                int referDistancenum = 0;
                PathOptimizationNS::PathOptimizer path_optimizer(start_state, end_state, grid_map);
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
                std::cout << "reference path point distance num : " << referDistancenum << "," << maxKappas << std::endl;
                if (referDistancenum >= 5 && abs(thXlocal) < 0.2 && abs(thYlocal) < 0.2 && maxKappas < 0.03 && !normalPath)
                {
                    std::cout << "use mapping path curvity points4" << std::endl;
                    LocalPath.clear();
                    PathOptimizationNS::State state;
                    for (size_t i = index; i < endIndex; i++)
                    {
                        state.x = addX + pathPoints[i][0];
                        state.y = addY + pathPoints[i][1];
                        state.heading = pathPoints[i][2];
                        LocalPath.push_back(state);
                    }

                    if (LocalPath.size() < 5)
                    {
                        if (iRet != PathPlan_LASTWRITE)
                        {
                            iRet = PathPlan_PARKING;
                        }
                        return iRet;
                    }
                    std::vector<PathPoint> points;
                    geneatePathPoint(LocalPath, &points);
                    SpeedData speed_data;
                    generateSpeed(j3Obs, points, cur_speed, max_speed, &speed_data, tempFile);
                    max_speed = normal_speed;
                    updateFlag = true;
                    if (iRet != PathPlan_LASTWRITE)
                    {
                        iRet = PathPlan_UPDATE;
                    }
                    return iRet;
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
                reference_path_opt = path_optimizer.getReferencePath();
                smoothed_reference_path.clear();
                if (!PathOptimizationNS::isEqual(reference_path_opt.getLength(), 0.0))
                {
                    double s = 0.0;
                    // std::cout<<reference_path_opt.getLength()<<std::endl;
                    while (s < reference_path_opt.getLength())
                    {
                        smoothed_reference_path.emplace_back(reference_path_opt.getXS()(s), reference_path_opt.getYS()(s));
                        s += 0.5;
                    }
                }
                // path_optimizer.printObstacleDistance2(smoothed_reference_path);
                // printf("-------------------------------------------------------");
                // path_optimizer.printObstacleDistance(result_path);
            }
            double angle = -car_state.heading;
            double disX, disY;
            PathOptimizationNS::State state;
            std::cout << "angle: " << angle << std::endl;
            double localX = LocalPathRefer[localIndex + 3].x;
            double localY = LocalPathRefer[localIndex + 3].y;
            LocalPath.clear();
            for (size_t i = 0; i < result_path.size() - 1; i = i + 1)
            {
                const auto path_point = result_path.at(i);
                disX = (path_point.x - start_state.x) * cos(angle) + (path_point.y - start_state.y) * sin(angle);
                disY = -(path_point.x - start_state.x) * sin(angle) + (path_point.y - start_state.y) * cos(angle);

                if (localIndex > 0 && abs(thXlocal) < 0.8 && abs(thYlocal) < 0.8)
                {
                    state.x = localX + disX;
                    state.y = localY + disY;
                }
                else
                {
                    state.x = car_state.x + disX;
                    state.y = car_state.y + disY;
                }

                state.heading = path_point.heading + (car_state.heading - start_state.heading);
                LocalPath.push_back(state);
                // draw plan path to image
                double y = -path_point.x / resolution + height / 2;
                double x = -path_point.y / resolution + width / 2;
                cv::Point p(x, y);
                circle(img_src, p, 1, cv::Scalar(0, 0, 0), -1);
            }
            if (result_path.size() < 5)
            {
                if (iRet != PathPlan_LASTWRITE)
                {
                    iRet = PathPlan_PARKING;
                }
                return iRet;
            }
            else
            {
                for (size_t i = 0; i != reference_path_plot.size(); ++i)
                {
                    grid_map::Position pose(reference_path_plot[i].x, reference_path_plot[i].y);
                    // std::cout << "reference path obstacle distance: " << map.getObstacleDistance(pose) << std::endl;
                    double y = -reference_path_plot[i].x / resolution + height / 2;
                    double x = -reference_path_plot[i].y / resolution + width / 2;
                    cv::Point p(x, y);
                    // std::cout << "reference_path_plot: " << x << "," << y << std::endl;
                    circle(img_src, p, 3, cv::Scalar(0, 0, 0), -1);
                }
                cv::imwrite("/app/slam/map_plan.jpg", img_src);
                std::vector<PathPoint> points;
                geneatePathPoint(LocalPath, &points);
                SpeedData speed_data;
                generateSpeed(j3Obs, points, cur_speed, max_speed, &speed_data, tempFile);
            }

            // restore parameters
            max_speed = normal_speed;
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

    void generateSpeed(std::vector<J3Obs> j3Obs, std::vector<PathPoint> points, double cur_speed, double max_speed, SpeedData *speed_data, const char *tempFile)
    {
        LocalPathRefer.clear();
        PathOptimizationNS::State localRefer;
        std::cout << "generate speed: " << points.size() << std::endl;
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

                if (j3Obs[i].obs_pose_type == 2) // 后中点
                {
                    rex = j3Obs[i].obs_x;
                    rey = abs(j3Obs[i].obs_y - j3Obs[i].obs_width / 2);
                }
                Box2d box({rex, rey}, j3Obs[i].obs_angle, j3Obs[i].obs_width, j3Obs[i].obs_length);
                std::unique_ptr<Obstacle> obs = Obstacle::CreateStaticVirtualObstacles("abc", box);
                Obstacle obstacle = *obs;
                Obstacles.push_back(obstacle);
            }
            // if (false) // create dynamic obstacle
            // {
            //     PredictionObstacles prediction_obstacles;
            //     std::vector<PredictionObstacle> predictionObstacles;
            //     PredictionObstacle predictionObstacle;
            //     // first obstacle
            //     {
            //         PerceptionObstacle perception_obstacle;
            //         perception_obstacle.id = j3Obs[i].obs_id; // 2156;
            //         perception_obstacle.position.x = j3Obs[i].obs_x;
            //         perception_obstacle.position.y = j3Obs[i].obs_y;
            //         perception_obstacle.position.z = 0; // j3Obs[i].obs_z;
            //         perception_obstacle.theta = j3Obs[i].obs_angle;
            //         perception_obstacle.velocity.x = j3Obs[i].obs_x_abs_vel;
            //         perception_obstacle.velocity.y = j3Obs[i].obs_y_abs_vel;
            //         perception_obstacle.velocity.z = j3Obs[i].obs_z_abs_vel;
            //         perception_obstacle.length = j3Obs[i].obs_length;
            //         perception_obstacle.width = j3Obs[i].obs_width;
            //         perception_obstacle.height = j3Obs[i].obs_height;
            //         std::vector<Point3D> polygon_point;
            //         for (int i = 0; i < obs_contour_point_vaild_num; i++)
            //         {
            //             Point3D po;
            //             po.x = j3Obs[i].contour_point[i].x;
            //             po.y = j3Obs[i].contour_point[i].y;
            //             po.z = 0;
            //             polygon_point.push_back(po);
            //         }
            //         perception_obstacle.polygon_point = polygon_point;
            //         perception_obstacle.timestamp = 0;
            //         perception_obstacle.tracking_time = 0;
            //         perception_obstacle.type = PerceptionObstacle::VEHICLE;
            //         double timestamp = 1171064942.14;
            //         double predicted_period = 5000;
            //         // can have multiple trajectories per obstacle
            //         std::vector<TrajectoryObstacle> trajectory;
            //         TrajectoryObstacle TrObs;
            //         TrObs.probability = 0.614;
            //         std::vector<TrajectoryPoint> trajectory_point;
            //         // point1
            //         for (int i = 0; i < trajectory_point_vaild_num; i++)
            //         {
            //             TrajectoryPoint point;
            //             point.relative_time = j3Obs[i].trajectory[i].relative_time;
            //             point.v = j3Obs[i].trajectory[i].v_x;
            //             point.path_point.x = j3Obs[i].trajectory[i].x;
            //             point.path_point.y = j3Obs[i].trajectory[i].y;
            //             point.path_point.z = 0;
            //             point.path_point.theta = j3Obs[i].trajectory[i].theat;
            //             trajectory_point.push_back(point);
            //         }
            //         TrObs.trajectory_point = trajectory_point;
            //         // estimated obstacle intent
            //         trajectory.push_back(TrObs);
            //         ObstacleIntent intent;
            //         ObstaclePriority priority;
            //         bool is_static;

            //         predictionObstacle.perception_obstacle = perception_obstacle;
            //         predictionObstacle.timestamp = timestamp;
            //         predictionObstacle.predicted_period = predicted_period;
            //         predictionObstacle.trajectory = trajectory;
            //         predictionObstacle.is_static = false;
            //         //  predictionObstacle.priority
            //     }
            //     predictionObstacles.push_back(predictionObstacle);
            //     prediction_obstacles.prediction_obstacle = predictionObstacles;
            //     std::list<std::unique_ptr<Obstacle>> obs2 = Obstacle::CreateObstacles(prediction_obstacles);
            //     for (auto &obstacle : obs2)
            //     {
            //         Obstacle obstacle2 = *obstacle;
            //         Obstacles.push_back(obstacle2);
            //     }
            // }
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
        obstacles_.emplace_back(&(obstacle_list_.back()));

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
        if (total_length > 9 && ret)
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

        string tempFilename2 = string(tempFile);
        ofstream outFile2(tempFilename2.c_str(), ios::out | ios::binary);
        savePose *savepose = new savePose;
        int id2 = 0;
        for (size_t i = 0; i != points.size(); ++i)
        {
            id2++;
            savepose->id = id2;
            savepose->x = points[i].x;
            savepose->y = points[i].y;
            savepose->z = 0;
            savepose->theta = points[i].theta;
            savepose->kappas = points[i].kappa;
            savepose->v = interValitys[i];
            savepose->a = interAcceles[i];
            localRefer.x = points[i].x;
            localRefer.y = points[i].y;
            localRefer.heading = points[i].theta;
            localRefer.k = points[i].kappa;
            LocalPathRefer.push_back(localRefer);
            outFile2.write((char *)savepose, sizeof(savePose));
        }
        outFile2.close();
        delete savepose;
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
        curve.fit(points, 0.01);
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
        if (max_speed > 1.2)
        {
            if (result_path.size() < 70)
            {
                max_speed = 1.2;
            }
            else if (result_path.size() < 102)
            {
                max_speed = 2.0;
            }
        }
        vector<double> curvitys = calculate_curvature_difference(result_path);
        cout << "Saving trajectory ..." << endl;
        // write result path to tempFile
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
            if (abs(curvitys[i]) > 0.08)
            {
                // std::cout << "max curvitys is larger than 0.1" << std::endl;
                max_speed = 1.2;
            }
        }
    }

    int GetPathPlanningAlgErrorType(E_PathPlanAlgErrorType *pepathAlgErrorType)
    {

        *pepathAlgErrorType = e_PathPlanAlgErrorType;
        return PathPlan_OK;
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
// ##########################################################################################################
