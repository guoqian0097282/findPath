// ##########################################################################################################
// create by guoqian on 23-10-20
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <tuple>
#include <unistd.h>
#include <sys/stat.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include "Eigen/Dense"
#include "eigen2cv.hpp"
#include "reference_line_processor.h"
#include "path/data_structure.h"
#include "path/path_problem_manager.h"
#include "solver/solver.h"
#include "path/gflags.h"
#include "path/tool.h"
#include "DataType.hpp"
#include "bezier.hpp"
#include "tinyspline_ros/bspline.h"
#include <tinyspline_ros/tinysplinecpp.h>
#include "tools/tools.hpp"
// ##########################################################################################################

#ifdef __cplusplus
extern "C"
{
#endif

    // ##########################################################################################################

    using namespace cv;
    using namespace std;
    using namespace Eigen;
    using namespace PathPlanning;

    PathPlanning::PathPoint start_state, end_state, car_state, init_state, car_state_local;
    std::vector<PathPlanning::PathPoint> reference_path_plot;
    std::vector<PathPlanning::PathPoint> LocalPath;
    bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;
    std::vector<cv::Point2f> referencePoint;
    cv::Mat dataX, dataY;
    double resolution;
    int width, height;
    struct savePoseOri
    {
        int id;
        double x;
        double y;
        double z;
        double theta;
    };
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

#define LH_PATHPLANNING_ALG_VERSION ("LH-PAHTPLAN-V1.22") // pathplanning算法版本号

    static cv::Mat g_sFreeSpaceGray;      // 输入freespace, 只申请一次空间，避免频繁申请释放空间产生内存碎片
    static string g_sglobalPathName = ""; // global 路径名称
    static vector<Vec6d> globalPathPoints;
    static E_PathPlanAlgErrorType e_PathPlanAlgErrorType;
    static bool isFirst;
    static bool updateFlag;
    // ##########################################################################################################

    static int CopyMatImg(T_FreeSpaceImg tFreeSpaceImg, cv::Mat &matDst);
    static bool IsParamLegal(T_FreeSpaceImg tFreeSpaceImg, T_CarState tCarState, const char *tempFile);
    static void ColorTransformation(cv::Mat cvImgIn, cv::Mat &cvImgOut);
    static void writeTempFile(std::vector<PathPoint> result_path, const char *tempFile);
    static int writeMemory(T_lqrPose *lqrPose);
    // ##########################################################################################################
    // 线性插值函数
    std::vector<double> linearInterpolation(const std::vector<double> &input, int newSize)
    {
        if (input.empty())
            return {};
        if (newSize <= input.size())
            return input; // 如果新大小不大于原大小，直接返回

        std::vector<double> output(newSize);
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
    vector<double> calculate_curvature_difference(const vector<Point2f> &result_path, int interval = 5)
    {
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
        fill(curvature_list.begin(), curvature_list.begin() + 2 * interval, curvature_list[2 * interval]);
        fill(curvature_list.end() - 2 * interval, curvature_list.end(), curvature_list[result_path.size() - 2 * interval - 1]);

        return curvature_list;
    }

    vector<Vec3d> B_spline(vector<Vec3d> pathPoints)
    {
        // B spline smoothing.
        double length = 0;
        for (size_t i = 0; i != pathPoints.size() - 1; ++i)
        {
            length += std::sqrt(std::pow(pathPoints[i][0] - pathPoints[i + 1][0], 2) + std::pow(pathPoints[i][1] - pathPoints[i + 1][1], 2));
        }
        int degree = 3;
        double average_length = length / (pathPoints.size() - 1);
        if (average_length > 10)
            degree = 3;
        else if (average_length > 5)
            degree = 4;
        else
            degree = 5;

        tinyspline::BSpline b_spline_raw(pathPoints.size(), 2, degree);
        std::vector<tinyspline::real> ctrlp_raw = b_spline_raw.controlPoints();
        for (size_t i = 0; i != pathPoints.size(); ++i)
        {
            ctrlp_raw[2 * (i)] = pathPoints[i][0];
            ctrlp_raw[2 * (i) + 1] = pathPoints[i][1];
        }
        b_spline_raw.setControlPoints(ctrlp_raw);
        double delta_t = 1.0 / pathPoints.size();
        double tmp_t = 0;
        std::vector<double> x_list, y_list, s_list;
        while (tmp_t < 1)
        {
            auto result = b_spline_raw.eval(tmp_t).result();
            x_list.emplace_back(result[0]);
            y_list.emplace_back(result[1]);
            tmp_t += delta_t;
        }
        auto result = b_spline_raw.eval(1).result();
        x_list.emplace_back(result[0]);
        y_list.emplace_back(result[1]);
        s_list.emplace_back(0);
        for (size_t i = 1; i != x_list.size(); ++i)
        {
            double dis = sqrt(pow(x_list[i] - x_list[i - 1], 2) + pow(y_list[i] - y_list[i - 1], 2));
            s_list.emplace_back(s_list.back() + dis);
        }

        tk::spline x_s, y_s;
        x_s.set_points(s_list, x_list);
        y_s.set_points(s_list, y_list);
        vector<Vec3d> resultPoint;
        Vec3d point;
        for (size_t i = 0; i != x_list.size(); i++)
        {
            point[0] = x_list[i];
            point[1] = y_list[i];
            point[2] = s_list[i];
            resultPoint.push_back(point);
        }
        return resultPoint;
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
            PathPlanning::PathPoint reference_point;
            reference_point.x = referencePoint[i].x;
            reference_point.y = referencePoint[i].y;
            reference_path_plot.emplace_back(reference_point);
            reference_rcv = reference_path_plot.size() >= 6;
            std::cout << "referenceCb received a reference point" << referencePoint[i] << std::endl;
        }
    }

    void referenceTurnPoint(int endIndex, int index, double thX, double thY)
    {
        int num = (endIndex - index) / 6;
        if (num == 0)
        {
            num = 1;
        }
        reference_path_plot.clear();
        referencePoint.clear();
        double refX, refY;
        for (size_t i = 0; i < 5 * num + 1; i = i + num)
        {
            Point2d globalP, pathP;
            globalP.x = globalPathPoints[index + i][0] + thX;
            globalP.y = globalPathPoints[index + i][1] + thY;
            pathP.x = globalP.x * cos(car_state.theta) + globalP.y * sin(car_state.theta);
            pathP.y = -globalP.x * sin(car_state.theta) + globalP.y * cos(car_state.theta);
            // pathP.x = globalP.x;
            // pathP.y = globalP.y;

            Point2d imagepathP, pixelP;
            imagepathP.x = abs(pathP.x - car_state_local.x) - width / 2 * resolution;
            imagepathP.y = pathP.y - car_state_local.y;
            pixelP.x = width / 2 - imagepathP.y / resolution;
            pixelP.y = height / 2 - imagepathP.x / resolution;
            Point2d endpixelP;
            endpixelP.x = height / 2 - pixelP.y;
            endpixelP.y = width / 2 - pixelP.x;
            double xs = endpixelP.x * resolution + 100 * resolution;
            double ys = endpixelP.y * resolution;
            cv::Point2f point0(xs, ys);
            if (i == 0)
            {
                refX = xs;
                refY = ys;
            }
            referencePoint.push_back(point0);
        }

        double disX = refX - start_state.x;
        double disY = refY - start_state.y;
        for (int i = 0; i < referencePoint.size(); i++)
        {
            PathPlanning::PathPoint reference_point;
            reference_point.x = referencePoint[i].x - disX;
            reference_point.y = referencePoint[i].y - disY;
            reference_path_plot.emplace_back(reference_point);
            reference_rcv = reference_path_plot.size() >= 6;
            std::cout << "received a reference point" << referencePoint[i] << std::endl;
        }
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
        resolution = 0.02;
        updateFlag = false;
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

    static bool IsParamLegal(T_FreeSpaceImg tFreeSpaceImg, T_CarState tCarState, const char *tempFile)
    {
        bool bRet = true;
        // printf("isPare tempFile name: %s\n", tempFile);
        if ((NULL == tFreeSpaceImg.pucImg) || (NULL == tempFile))
        {
            if ((NULL == tFreeSpaceImg.pucImg))
            {
                e_PathPlanAlgErrorType = PathPlan_NO_IMAGE_YET;
            }
            else if (NULL == tempFile)
            {
                e_PathPlanAlgErrorType = PathPlan_TEMPFILE_EMPTY;
            }

            bRet = false;
            printf("==========IsParamLegal check param is error, tFreeSpaceImg.pucImg: %x, tempFile: %s\n", tFreeSpaceImg.pucImg, tempFile);
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
    int LocalPathPlanning(T_FreeSpaceImg tFreeSpaceImg, T_CarState tCarState, const char *tempFile, bool readOver, bool needPath, bool isTurn, double laneDis, bool backPath, float *referPointX, float *referPointY)
    {
        bool shortPath = false;
        int tempValue = 2;
        // car current pose in odom axis convert to camera axis
        tCarState.dTheta = tCarState.dTheta + 1.570796;
        tCarState.dX = tCarState.dX; //+ 2 * cos(-tCarState.dTheta);
        tCarState.dY = tCarState.dY; //- 2 * sin(-tCarState.dTheta);

        // printf("begin local path planning\n");
        int iRet = PathPlan_OK;
        bool bIsParamLegal = false;

        bIsParamLegal = IsParamLegal(tFreeSpaceImg, tCarState, tempFile);
        if (false == bIsParamLegal)
        {
            printf("==========LocalPathPlanning input param is error!\n");
            return PathPlan_FAILED;
        }
        if (isFirst || true == updateFlag)
        {
            printf("To init state\n");
            init_state.x = tCarState.dX;
            init_state.y = tCarState.dY;
            updateFlag = false;
        }
        int numSize = LocalPath.size();
        if (numSize > 0)
        {
            // std::cout << "local path: " << LocalPath.size() << "," << LocalPath[numSize - 1].x << "," << LocalPath[numSize - 1].y << std::endl;
            if ((pow((tCarState.dX - LocalPath[numSize - 1].x), 2) + pow((tCarState.dY - LocalPath[numSize - 1].y), 2)) < 2.5)
            {
                shortPath = true;
            }
        }
        double travelDistance = pow((init_state.x - tCarState.dX), 2) + pow((init_state.y - tCarState.dY), 2);
        double targetDis = 20;
        if (isTurn)
        {
            targetDis = 36;
        }
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
        if (1) //((isFirst || needPath || shortPath) || (travelDistance > targetDis && readOver == true))
        {
            std::cout << isFirst << "," << needPath << "," << shortPath << "," << travelDistance << "," << std::endl;
            // LocalPath.clear();
            if (g_sFreeSpaceGray.empty())
            {
                g_sFreeSpaceGray.create(tFreeSpaceImg.uiHeight, tFreeSpaceImg.uiWidth, CV_8UC1);
            }

            CopyMatImg(tFreeSpaceImg, g_sFreeSpaceGray);

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
                iRet = PathPlan_LASTWRITE;
            }
            // 0.DNN model get freespace and fisheye calibration
            // 1.Bev Tansform
            // printf("fisheye calibration and warp transform\n");
            Mat image = g_sFreeSpaceGray;
            // 2. binaryImage to get Areas of exercise
            cv::Mat gray, binaryImage;
            if (image.channels() == 3)
            {
                cv::cvtColor(image, gray, COLOR_BGR2GRAY);
            }
            else
            {
                image.copyTo(gray);
            }
            threshold(gray, binaryImage, 120, 255, 0);
            // cv::Mat warped_cut = binaryImage(cv::Rect(0, 200, 1920, 980));
            // cv::copyMakeBorder(warped_cut, binaryImage, 0, 200, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
            // 3. remove small object and dilate erode
            // printf("remove small object and dilate erode\n");
            Mat erodeImage, dilateImage;
            Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(10, 10));
            Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(19, 19));
            Mat element3 = getStructuringElement(MORPH_ELLIPSE, Size(19, 19));
            dilate(binaryImage, dilateImage, element1);
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;

            findContours(dilateImage, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
            int contSize = contours.size();
            contours.erase(remove_if(contours.begin(), contours.end(),
                                     [](const std::vector<cv::Point> &c)
                                     { return cv::contourArea(c) < 3000; }),
                           contours.end());
            dilateImage.setTo(0);
            cv::drawContours(dilateImage, contours, -1, cv::Scalar(255), cv::FILLED);
            erode(dilateImage, erodeImage, element2);
            dilate(erodeImage, dilateImage, element3);
            // cv::imshow("erodeImage",erodeImage);
            // cv::imshow("removeImage",dilateImage);
            // cv::waitKey(0);
            // 4.补全车前的盲区
            // for (int i = 760; i < 1160; i++)
            // {
            //     for (int j = 1000; j < 1080; j++)
            //     {
            //         binaryImage.at<uchar>(j, i) = 255;
            //     }
            // }
            cv::Mat warped_cut = dilateImage(cv::Rect(0, 40, width, height - 40));
            cv::copyMakeBorder(warped_cut, dilateImage, 0, 40, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
            // cv::imwrite("fullCar1.jpg", dilateImage);
            // 5.去掉与起点不连接的“可行驶区域”
            // findContours(dilateImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
            // // printf(contours.size());
            // int i = 0;
            // if (contours.size() > 1)
            // {
            //     vector<vector<Point>>::const_iterator itc = contours.begin();
            //     while (itc != contours.end())
            //     {
            //         if (pointPolygonTest(contours[i], cv::Point(960, 1078), false) == -1)
            //         {
            //             itc = contours.erase(itc);
            //         }
            //         else
            //         {
            //             ++itc;
            //             ++i;
            //         }
            //     }
            //     dilateImage.setTo(0);
            //     cv::drawContours(dilateImage, contours, -1, cv::Scalar(255), cv::FILLED);
            //     // cv::imwrite("fullCar.jpg", dilateImage);
            // }
            // 6. 选取终点,设定候选目标半径为 8m =
            // printf("choose target point with circle 8m\n");
            cv::Mat circleImage(dilateImage.size(), CV_8UC1, Scalar(0));
            double targetPixel = width - 50;
            circle(circleImage, Point(width / 2, height), targetPixel, Scalar(255), 30);
            // cv::imwrite("circleImage.jpg", circleImage);
            // cv::imshow("circleImage", circleImage);
            // cv::imshow("dilateImage", dilateImage);
            // cv::waitKey(0);
            cv::Mat interSection(dilateImage.size(), CV_8UC1, Scalar(0));
            bitwise_and(circleImage, dilateImage, interSection);
            // cv::imshow("interSection",interSection);
            // cv::waitKey(0);
            // Initialize grid map from image.
            cv::Mat img_src;
            // cv::Mat img_src = dilateImage;
            // dilateImage expand down 400 pixel for car length 4.9m
            cv::copyMakeBorder(dilateImage, img_src, 0, 200, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
            grid_map::GridMap grid_map(std::vector<std::string>{"obstacle", "distance"});
            grid_map::GridMapCvConverter::initializeFromImage(
                img_src, resolution, grid_map, grid_map::Position::Zero());
            // Add obstacle layer.
            unsigned char OCCUPY = 0;
            unsigned char FREE = 255;
            grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
                img_src, "obstacle", grid_map, OCCUPY, FREE, 0.5);
            // Update distance layer.
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary = grid_map.get("obstacle").cast<unsigned char>();
            cv::distanceTransform(eigen2cv(binary), eigen2cv(grid_map.get("distance")),
                                  DIST_L2, DIST_MASK_PRECISE);
            grid_map.get("distance") *= resolution;
            Test::Map map(grid_map);
            // 7. 选取终点
            findContours(interSection, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
            vector<Point2d> centerPoint;
            for (int i = 0; i < contours.size(); i++)
            {
                if (cv::contourArea(contours[i]) > 3000)
                {
                    Moments M;
                    Point2d point;
                    M = moments(contours[i]);
                    point.x = double(M.m10 / M.m00);
                    point.y = double(M.m01 / M.m00);
                    centerPoint.push_back(point);
                    // circle(interSection,point,5,Scalar(0,255,255),-1);
                    // printf(centerPoint);
                }
            }
            // cv::imshow("interSection",interSection);
            // cv::waitKey(0);
            printf("find the path respecting point\n");
            car_state.x = tCarState.dX;
            car_state.y = tCarState.dY;
            car_state.theta = tCarState.dTheta;
            // path splicing
            int mDis = 10000;
            int lIndex = 0;
            // if (LocalPath.size() > 10)
            // {
            //     for (int i = 0; i < LocalPath.size(); i++)
            //     {
            //         double disP = pow((car_state.x - LocalPath[i].x), 2) + pow((car_state.y - LocalPath[i].y), 2);
            //         if (disP < mDis)
            //         {
            //             mDis = disP;
            //             lIndex = i;
            //         }
            //     }
            //     if (sqrt(mDis) < 1)
            //     {
            //         car_state.x = LocalPath[lIndex].x;
            //         car_state.y = LocalPath[lIndex].y;
            //         // car_state.theta = LocalPath[lIndex].theta;
            //     }
            // }
            // printf("find the end path point\n");
            Point2d endpathPoint, referPoint_local, endpathPoint_local;
            int endIndex = 0;
            double targetPath = (targetPixel * resolution) * (targetPixel * resolution);
            if (isTurn && !backPath)
            {
                targetPath = 200.0;
            }
            for (int i = index; i < pathPoints.size(); i++)
            {
                double distance = pow((referPoint.x - pathPoints[i][0]), 2) + pow((referPoint.y - pathPoints[i][1]), 2);
                // printf(i<<","<<pathPoints[i]<<"distance: "<<distance);
                if (distance > targetPath)
                {
                    endIndex = i;
                    break;
                }
                else
                {
                    endIndex = pathPoints.size() - 1;
                }
            }
            double thX = car_state.x - pathPoints[index][0];
            double thY = car_state.y - pathPoints[index][1];
            double thXL, thYL;
            if (abs(thX) > 1.5 || abs(thY) > 1.5)
            {
                thXL = thX;
                thYL = thY;
            }
            else
            {
                thXL = 0;
                thYL = 0;
            }
            std::cout << "compensate value:" << thX << "," << thY << "," << thXL << "," << thYL << std::endl;
            endpathPoint.x = pathPoints[endIndex][0] + thXL;
            endpathPoint.y = pathPoints[endIndex][1] + thYL;
            std::cout << "referPoint: " << pathPoints[index][0] << "," << pathPoints[index][1] << "," << pathPoints[index][2] << std::endl;
            std::cout << "endpathPoint: " << pathPoints[endIndex][0] << "," << pathPoints[endIndex][1] << "," << pathPoints[endIndex][2] << std::endl;
            // global to local
            referPoint_local.x = referPoint.x * cos(car_state.theta) + referPoint.y * sin(car_state.theta);
            referPoint_local.y = -referPoint.x * sin(car_state.theta) + referPoint.y * cos(car_state.theta);
            endpathPoint_local.x = endpathPoint.x * cos(car_state.theta) + endpathPoint.y * sin(car_state.theta);
            endpathPoint_local.y = -endpathPoint.x * sin(car_state.theta) + endpathPoint.y * cos(car_state.theta);
            car_state_local.x = car_state.x * cos(car_state.theta) + car_state.y * sin(car_state.theta);
            car_state_local.y = -car_state.x * sin(car_state.theta) + car_state.y * cos(car_state.theta);

            Point2d imagepathPoint, pixelpathPoint, endCentrPoint, endCentrPoint1, endCentrPoint2;
            imagepathPoint.x = abs(endpathPoint_local.x - car_state_local.x) - width / 2 * resolution;
            imagepathPoint.y = endpathPoint_local.y - car_state_local.y;
            pixelpathPoint.x = width / 2 - imagepathPoint.y / resolution;
            pixelpathPoint.y = height / 2 - imagepathPoint.x / resolution;
            double minDis = 1000000;
            double x = (height / 2 - pixelpathPoint.y) * resolution + 100 * resolution;
            double y = (width / 2 - pixelpathPoint.x) * resolution;
            grid_map::Position pose(x, y);
            double angleCh = abs(pathPoints[endIndex + 5][2] - pathPoints[index][2]);
            grid_map::Position pose1(x - 0.5, y);
            grid_map::Position pose2(x + 0.5, y);
            double dis = map.getObstacleDistance(pose);
            double dis1 = map.getObstacleDistance(pose1);
            double dis2 = map.getObstacleDistance(pose2);
            std::cout << "map.getDistance pixelpathpoint: " << dis << "," << dis1 << "," << dis2 << std::endl;

            double turndisX = 0;
            double turndisY = 0;

            if (dis1 > dis && dis1 > dis2)
            {
                turndisX = -0.5 * cos(-car_state.theta);
                turndisY = 0;
            }
            else if (dis2 > dis && dis2 > dis1)
            {
                turndisX = 0.5 * cos(-car_state.theta);
                turndisY = 0;
            }

            if (isFirst)
            {
                std::cout << "use mapping path curvity points0" << std::endl;
                LocalPath.clear();
                PathPoint state;
                for (size_t i = 0; i < pathPoints.size() - 1; i++)
                {
                    state.x = pathPoints[i][0];
                    state.y = pathPoints[i][1];
                    state.theta = pathPoints[i][2];
                    state.kappa = pathPoints[i][3];
                    state.dl = pathPoints[i][4];
                    state.ddl = pathPoints[i][5];
                    LocalPath.push_back(state);
                }
                if (LocalPath.size() < 10)
                {
                    iRet = PathPlan_PARKING;
                    return iRet;
                }
                writeTempFile(LocalPath, tempFile);
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
                PathPoint state;
                for (size_t i = index; i < endIndex; i++)
                {
                    state.x = thX + pathPoints[i][0] + turndisX;
                    state.y = thY + pathPoints[i][1] + turndisY;
                    state.theta = pathPoints[i][2];
                    state.kappa = pathPoints[i][3];
                    state.dl = pathPoints[i][4];
                    state.ddl = pathPoints[i][5];
                    LocalPath.push_back(state);
                }
                if (LocalPath.size() < 10)
                {
                    iRet = PathPlan_PARKING;
                    return iRet;
                }
                writeTempFile(LocalPath, tempFile);
                updateFlag = true;
                if (iRet != PathPlan_LASTWRITE)
                {
                    iRet = PathPlan_UPDATE;
                }
                return iRet;
            }
            // else if (((angleCh > 0.3 || isTurn)&&dis>0) || needPath) // (&& dis > 1.0)
            else if (angleCh > 0.3 || (isTurn && angleCh > 0.1) || needPath)
            {
                std::cout << "use mapping path curvity points3" << std::endl;
                LocalPath.clear();
                PathPoint state;
                vector<Vec3d> pathPoint;
                if (true)
                {
                    if (needPath)
                    {
                        for (size_t i = 0; i < pathPoints.size() - 1; i++)
                        {
                            state.x = pathPoints[i][0];
                            state.y = pathPoints[i][1];
                            state.theta = pathPoints[i][2];
                            state.kappa = pathPoints[i][3];
                            state.dl = pathPoints[i][4];
                            state.ddl = pathPoints[i][5];
                            LocalPath.push_back(state);
                        }
                    }
                    else
                    {
                        for (size_t i = index; i < endIndex - (tempValue - 1); i = i + tempValue)
                        {
                            state.x = pathPoints[i][0];
                            state.y = pathPoints[i][1];
                            state.theta = pathPoints[i][2];
                            state.kappa = pathPoints[i][3];
                            state.dl = pathPoints[i][4];
                            state.ddl = pathPoints[i][5];
                            LocalPath.push_back(state);
                        }
                    }
                }
                if (LocalPath.size() < 10)
                {
                    iRet = PathPlan_PARKING;
                    return iRet;
                }
                writeTempFile(LocalPath, tempFile);
                updateFlag = true;
                if (iRet != PathPlan_LASTWRITE)
                {
                    iRet = PathPlan_UPDATE;
                }
                return iRet;
            }
            else if (centerPoint.size() == 0)
            {
                std::cout << "use mapping path curvity points1" << std::endl;
                LocalPath.clear();
                PathPoint state;
                for (size_t i = index; i < endIndex; i++)
                {
                    state.x = thX + pathPoints[i][0];
                    state.y = thY + pathPoints[i][1];
                    state.theta = pathPoints[i][2];
                    state.kappa = pathPoints[i][3];
                    state.dl = pathPoints[i][4];
                    state.ddl = pathPoints[i][5];
                    LocalPath.push_back(state);
                }
                if (LocalPath.size() < 10)
                {
                    iRet = PathPlan_PARKING;
                    return iRet;
                }
                writeTempFile(LocalPath, tempFile);
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
            //     printf("cannot find centerPoint\n");
            //     return PathPlan_NOCENTERPOINT;
            // }
            double xCen = (height / 2 - centerPoint[0].y) * resolution + 100 * resolution;
            double yCen = (width / 2 - centerPoint[0].x) * resolution;
            grid_map::Position poseCen(xCen, yCen);
            double poseCenDistance = map.getObstacleDistance(poseCen);
            double poseDistance = map.getObstacleDistance(pose);
            std::cout << "map.getDistance center point and pixel point: " << poseCenDistance << "," << poseDistance << std::endl;
            // find the end point
            if (centerPoint.size() > 1)
            {
                for (int i = 0; i < centerPoint.size(); i++)
                {
                    double dist = pow((centerPoint[i].x - pixelpathPoint.x), 2) + pow((centerPoint[i].y - pixelpathPoint.y), 2);
                    if (dist < minDis)
                    {
                        minDis = dist;
                        endCentrPoint = centerPoint[i];
                    }
                }
                std::cout << "1centerPoint: " << endCentrPoint.x << "," << endCentrPoint.y << std::endl;
                double path_dis = sqrt(minDis) * resolution;
                std::cout << "path_dis: " << path_dis << "laneDis: " << laneDis << std::endl;
                if (thXL > 0 || thYL > 0)
                {
                    if (poseDistance > 2.0)
                    {
                        std::cout << "thXL>0 and thYL>0 and poseDistance: " << poseDistance << std::endl;
                        endCentrPoint = pixelpathPoint;
                    }
                    else
                    {
                        std::cout << "thXL>0 and thYL>0 and poseCenDistance: " << poseCenDistance << std::endl;
                        endCentrPoint = endCentrPoint;
                    }
                }
                else
                {
                    if (path_dis > 1.5 && poseDistance > 1.0)
                    {
                        std::cout << "1path_dis is larger than 1.5: " << path_dis << std::endl;
                        endCentrPoint.x = pixelpathPoint.x;
                        endCentrPoint.y = pixelpathPoint.y;
                    }
                    else if (laneDis >= 1.5 && pixelpathPoint.x > 250 && poseDistance > 0.5)
                    {
                        std::cout << "2laneDis is larger than 1.2: " << laneDis << std::endl;
                        endCentrPoint.x = pixelpathPoint.x;
                        endCentrPoint.y = pixelpathPoint.y;
                    }
                    else if (laneDis <= 0.8 && pixelpathPoint.x < 250 && poseDistance > 0.5)
                    {
                        std::cout << "3laneDis is small than 0.5: " << laneDis << std::endl;
                        endCentrPoint.x = pixelpathPoint.x;
                        endCentrPoint.y = pixelpathPoint.y;
                    }
                    else if (path_dis < 1.5 && laneDis >= 1.5 && centerPoint[0].x > 250 && poseCenDistance > 0.8)
                    {
                        std::cout << "4laneDis is larger than 1.2: " << laneDis << std::endl;
                        endCentrPoint.x = endCentrPoint.x;
                        endCentrPoint.y = endCentrPoint.y;
                    }
                    else if (path_dis < 1.5 && laneDis <= 0.8 && endCentrPoint.x < 250 && poseCenDistance > 0.8)
                    {
                        std::cout << "5laneDis is small than 0.5: " << laneDis << std::endl;
                        endCentrPoint.x = endCentrPoint.x;
                        endCentrPoint.y = endCentrPoint.y;
                    }
                    else if (angleCh < 0.5 && poseDistance > 0.8 && path_dis < 1.0)
                    {
                        std::cout << "6poseDistance is larger 0.8: " << poseDistance << std::endl;
                        endCentrPoint = pixelpathPoint;
                    }
                    else if (path_dis > 1.0 && poseCenDistance > 1.0)
                    {
                        std::cout << "7poseCenDistance is larger than 1: " << poseCenDistance << std::endl;
                        endCentrPoint.x = endCentrPoint.x;
                        endCentrPoint.y = endCentrPoint.y;
                    }
                    else if (path_dis > 1.0 && poseDistance > 0.5)
                    {
                        std::cout << "8path_dis is larger than 1: " << path_dis << std::endl;
                        endCentrPoint.x = pixelpathPoint.x;
                        endCentrPoint.y = pixelpathPoint.y;
                    }
                }
            }
            else
            {
                double dist = pow((centerPoint[0].x - pixelpathPoint.x), 2) + pow((centerPoint[0].y - pixelpathPoint.y), 2);
                double path_dis = sqrt(dist) * resolution;
                std::cout << "centerPoint: " << centerPoint[0].x << "," << centerPoint[0].y << "," << path_dis << std::endl;
                if (thXL > 0 || thYL > 0)
                {
                    if (poseDistance > 2.0)
                    {
                        std::cout << "thXL>0 thYL>0 and poseDistance >0.5" << poseDistance << std::endl;
                        endCentrPoint = pixelpathPoint;
                    }
                    else
                    {
                        std::cout << "thXL>0 thYL>0 and poseCenDistance >0.5" << poseCenDistance << std::endl;
                        endCentrPoint.x = centerPoint[0].x;
                        endCentrPoint.y = centerPoint[0].y;
                    }
                }
                else
                {
                    if (angleCh < 0.3 && (poseDistance > 2.0))
                    {
                        std::cout << "1path_dis is larger: " << path_dis << std::endl;
                        endCentrPoint = pixelpathPoint;
                    }
                    else if (path_dis < 0.2)
                    {
                        if (poseCenDistance > poseDistance)
                        {
                            endCentrPoint = centerPoint[0];
                        }
                        else
                        {
                            endCentrPoint = pixelpathPoint;
                        }
                    }
                    else if (poseDistance > 1.0)
                    {
                        std::cout << "1path_dis is larger: " << path_dis << std::endl;
                        endCentrPoint = pixelpathPoint;
                    }
                    else if (poseCenDistance > poseDistance)
                    {
                        std::cout << "2path_dis is larger: " << path_dis << std::endl;
                        endCentrPoint = centerPoint[0];
                    }
                    else if (poseDistance > poseCenDistance)
                    {
                        std::cout << "3path_dis is larger: " << path_dis << std::endl;
                        endCentrPoint = pixelpathPoint;
                    }
                    else if (laneDis >= 1.5 && pixelpathPoint.x > 250 && poseDistance > 0.8)
                    {
                        std::cout << "4path_dis is larger than 1.2: " << path_dis << std::endl;
                        endCentrPoint.x = pixelpathPoint.x;
                        endCentrPoint.y = pixelpathPoint.y;
                    }
                    else if (laneDis <= 0.8 && laneDis > 0.1 && pixelpathPoint.x < 240 && poseDistance > 0.8)
                    {
                        std::cout << "5laneDis is small than 0.5: " << laneDis << std::endl;
                        endCentrPoint.x = pixelpathPoint.x;
                        endCentrPoint.y = pixelpathPoint.y;
                    }
                    else if (path_dis < 1.5 && laneDis >= 1.5 && centerPoint[0].x > 250 && poseCenDistance > 0.8)
                    {
                        std::cout << "6laneDis is larger than 1.2: " << laneDis << std::endl;
                        endCentrPoint.x = centerPoint[0].x;
                        endCentrPoint.y = centerPoint[0].y;
                    }
                    else if (path_dis < 1.5 && laneDis <= 0.8 && laneDis > 0.1 && centerPoint[0].x < 240 && poseCenDistance > 0.8)
                    {
                        std::cout << "7laneDis is small than 0.5: " << laneDis << std::endl;
                        endCentrPoint.x = centerPoint[0].x;
                        endCentrPoint.y = centerPoint[0].y;
                    }
                    else if (path_dis > 1.0 && poseDistance > 1.5)
                    {
                        std::cout << "8path_dis is larger than 1: " << path_dis << std::endl;
                        endCentrPoint.x = pixelpathPoint.x;
                        endCentrPoint.y = pixelpathPoint.y;
                    }
                    else if (path_dis > 1.0 && poseCenDistance > 1.5)
                    {
                        std::cout << "9path_dis is larger than 1: " << path_dis << std::endl;
                        endCentrPoint.x = centerPoint[0].x;
                        endCentrPoint.y = centerPoint[0].y;
                    }
                    else if (laneDis <= 1.5 && laneDis > 0.1)
                    {
                        if (pixelpathPoint.x < centerPoint[0].x && path_dis < 1.0 && poseDistance > 0.5)
                        {
                            endCentrPoint.x = pixelpathPoint.x;
                            endCentrPoint.y = pixelpathPoint.y;
                        }
                        else if (pixelpathPoint.x < 252 && centerPoint[0].x > 250)
                        {
                            endCentrPoint.x = pixelpathPoint.x;
                            endCentrPoint.y = pixelpathPoint.y;
                        }
                        else if (pixelpathPoint.x > 250 && centerPoint[0].x < 250)
                        {
                            endCentrPoint.x = centerPoint[0].x;
                            endCentrPoint.y = centerPoint[0].y;
                        }
                        else if (path_dis > 1.5 && pixelpathPoint.x < 250 && poseDistance > 1.0)
                        {
                            endCentrPoint.x = pixelpathPoint.x;
                            endCentrPoint.y = pixelpathPoint.y;
                        }
                        else if (poseDistance > poseCenDistance)
                        {
                            endCentrPoint.x = pixelpathPoint.x;
                            endCentrPoint.y = pixelpathPoint.y;
                        }
                        else if (centerPoint[0].x < 250 && poseCenDistance > 1.0)
                        {
                            endCentrPoint.x = centerPoint[0].x;
                            endCentrPoint.y = centerPoint[0].y;
                        }
                        else if (pixelpathPoint.x < centerPoint[0].x)
                        {
                            endCentrPoint = pixelpathPoint;
                        }
                        else
                        {
                            endCentrPoint = centerPoint[0];
                        }
                    }
                    else if (laneDis > 1.5)
                    {
                        if (pixelpathPoint.x > centerPoint[0].x && path_dis < 1.0 && poseDistance > 1.0)
                        {
                            endCentrPoint.x = pixelpathPoint.x;
                            endCentrPoint.y = pixelpathPoint.y;
                        }
                        else if (pixelpathPoint.x > 250 && centerPoint[0].x < 250)
                        {
                            endCentrPoint.x = pixelpathPoint.x;
                            endCentrPoint.y = pixelpathPoint.y;
                        }
                        else if (pixelpathPoint.x < 250 && centerPoint[0].x > 250)
                        {
                            endCentrPoint = centerPoint[0];
                        }
                        else if (path_dis > 1.5 && pixelpathPoint.x > 250 && poseDistance > 1.0)
                        {
                            endCentrPoint.x = pixelpathPoint.x;
                            endCentrPoint.y = pixelpathPoint.y;
                        }
                        else if (poseDistance > poseCenDistance)
                        {
                            endCentrPoint.x = pixelpathPoint.x;
                            endCentrPoint.y = pixelpathPoint.y;
                        }
                        else if (centerPoint[0].x > 250 && poseCenDistance > 1.0)
                        {
                            endCentrPoint.x = centerPoint[0].x;
                            endCentrPoint.y = centerPoint[0].y;
                        }
                        else if (centerPoint[0].x > pixelpathPoint.x)
                        {
                            endCentrPoint = centerPoint[0];
                        }
                        else
                        {
                            endCentrPoint = pixelpathPoint;
                        }
                    }
                    else
                    {
                        endCentrPoint.x = centerPoint[0].x;
                        endCentrPoint.y = centerPoint[0].y;
                    }
                }
            }

            // double centerD = 1.0;
            // if (isTurn)
            // {
            //     centerD = 2.0;
            // }
            // if (poseDistance > centerD)
            // {
            //     endCentrPoint = pixelpathPoint;
            // }
            std::cout << "carState: " << car_state.x << "," << car_state.y << "," << car_state.theta << std::endl;
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
            start_state.x = -height / 2 * resolution + 100 * resolution;
            start_state.y = 0.0 * resolution;
            // set end state
            end_state.x = endpixelPoint.x * resolution + 100 * resolution;
            end_state.y = endpixelPoint.y * resolution;
            double disx = (end_state.x - start_state.x) / 6;
            double disy = (end_state.y - start_state.y) / 6;
            double path_startTheta = 0;
            if ((isTurn && angleCh > 0.1))
            {
                referenceTurnPoint(endIndex, index, thX, thY);
                // double distancel;
                // for (size_t i = 0; i != reference_path_plot.size(); ++i)
                // {
                //     grid_map::Position pose(reference_path_plot[i].x, reference_path_plot[i].y);
                //     distancel = map.getObstacleDistance(pose);
                //     if (distancel < 2.0)
                //     {
                //         reference_path_plot[i].x = start_state.x + disx * i;
                //         reference_path_plot[i].y = start_state.y + disy * i;
                //     }
                // }
                int size = reference_path_plot.size();
                // double theL = atan((reference_path_plot[size - 1].y - end_state.y) / (reference_path_plot[size - 1].x - end_state.x));
                // if ((abs(theL - 1.56) < 0.1 && distancel > 1.0))
                // {
                //     end_state.x = reference_path_plot[size - 1].x;
                //     end_state.y = reference_path_plot[size - 1].y;
                // }
                // else if (abs(theL - 1.56) < 0.1)
                // {
                //     reference_path_plot[size - 1].x = end_state.x;
                //     reference_path_plot[size - 1].y = end_state.y;
                // }
                // else
                // {
                //     reference_path_plot.emplace_back(end_state);
                // }

                double the_start = atan((reference_path_plot[3].y - start_state.y) / (reference_path_plot[3].x - start_state.x));
                // double the_end = atan((reference_path_plot[size - 1].y - reference_path_plot[size - 4].y) / (reference_path_plot[size - 1].x - reference_path_plot[size - 4].x));
                // if ((the_end > 0 && pathPoints[endIndex][2] < 0) || (the_end < 0 && pathPoints[endIndex][2] > 0))
                // {
                //     the_end = pathPoints[endIndex][2];
                // }
                start_state.theta = path_startTheta;
                end_state.theta = path_startTheta + pathPoints[endIndex][2] - pathPoints[index][2];
                std::cout << "turn start state: " << start_state.x << "," << start_state.y << "," << start_state.theta << std::endl;
                std::cout << "turn end state: " << end_state.x << "," << end_state.y << "," << end_state.theta << std::endl;
            }
            else
            {
                double the = atan((end_state.y - start_state.y) / (end_state.x - start_state.x));
                end_state.theta = path_startTheta + pathPoints[endIndex][2] - pathPoints[index][2];
                start_state.theta = path_startTheta; // pathPoints[index][2];
                std::cout << "start state: " << start_state.x << "," << start_state.y << "," << start_state.theta << std::endl;
                std::cout << "end state: " << end_state.x << "," << end_state.y << "," << end_state.theta << std::endl;

                // referenceTurnPoint(endIndex, index, thX, thY);
                referenceCb();
                /**for (size_t ii = 1; ii != 2; ++ii)
                {
                    // int ii = 2;
                    grid_map::Position pose2(reference_path_plot[ii].x, reference_path_plot[ii].y);
                    double distance = map.getObstacleDistance(pose2);
                    // std::cout << "reference path obstacle distance: " <<reference_path_plot[ii].x<<","<<reference_path_plot[ii].y<<","<< distance << std::endl;
                    double right_y, left_y;
                    if (distance < 1.0)
                    {
                        double x = reference_path_plot[ii].x;
                        for (double y = -9.7; y < 9.7; y = y + 0.1)
                        {
                            grid_map::Position pose1(x, y);
                            // std::cout<<x<<","<<y<<","<<map.getObstacleDistance(pose1)<<std::endl;
                            if (map.getObstacleDistance(pose1) > 0)
                            {
                                right_y = y;
                                break;
                            }
                        }
                        for (double y = 9.7; y > -9.7; y = y - 0.1)
                        {
                            grid_map::Position pose1(x, y);
                            // std::cout<<x<<","<<y<<","<<map.getObstacleDistance(pose1)<<std::endl;
                            if (map.getObstacleDistance(pose1) > 0)
                            {
                                left_y = y;
                                break;
                            }
                        }
                        reference_path_plot[ii].y = right_y + (left_y - right_y) / 2;
                        double the = atan((reference_path_plot[ii].y - start_state.y) / (reference_path_plot[ii].x - start_state.x));
                        start_state.theta = the; // car_state.theta;pathPoints[index][2];
                    }
                }**/
            }

            //  8.路径规划目标,路径平滑
            //  grid map = dilateImage centerPoint
            Test::ReferenceLineProcessor reference_line_processor(reference_path_plot, map, start_state);
            std::shared_ptr<PathPlanning::ReferenceLine> ref_line_ptr = std::make_shared<PathPlanning::ReferenceLine>();
            std::shared_ptr<PathPlanning::FreeSpace> free_space_ptr = std::make_shared<PathPlanning::FreeSpace>();
            reference_line_processor.solve(ref_line_ptr, free_space_ptr);

            // solve
            PathPlanning::PathProblemManager path_problem_manager;
            path_problem_manager.formulate_path_problem(*free_space_ptr, *ref_line_ptr, start_state, end_state);
            Solver::ILQRSolver<PathPlanning::N_PATH_STATE, PathPlanning::N_PATH_CONTROL> ilqr_solver(path_problem_manager);
            ILQRSolveStatus solve_status = ilqr_solver.solve();
            const auto &opt_path_raw = ilqr_solver.final_trajectory();
            const auto result = PathProblemManager::transform_to_path_points(*ref_line_ptr, opt_path_raw);
            if (solve_status != ILQRSolveStatus::SOLVED)
            {
                std::cout << "solver status is failed!" << std::endl;
                iRet = PathPlan_FAILED;
                return iRet;
            }

            double angle = -car_state.theta;
            LocalPath.clear();
            double disX, disY;
            PathPoint state;
            std::cout << "angle: " << angle << std::endl;
            for (size_t i = 0; i < result.size() - (tempValue - 1); i = i + tempValue)
            {
                const auto path_point = result.at(i);
                disX = (path_point.x + (width / 2 - 100) * resolution) * cos(angle) + path_point.y * sin(angle);
                disY = -(path_point.x + (width / 2 - 100) * resolution) * sin(angle) + path_point.y * cos(angle);
                state.x = car_state.x + disX;
                state.y = car_state.y + disY;
                state.theta = path_point.theta - angle;
                LocalPath.push_back(state);
            }
            if (LocalPath.size() < 10)
            {
                if (iRet != PathPlan_LASTWRITE)
                {
                    iRet = PathPlan_FAILED;
                }
                else
                {
                    iRet = PathPlan_PARKING;
                }
                return iRet;
            }
            writeTempFile(LocalPath, tempFile);
            updateFlag = true;
        }
        return iRet;
    }

    void writeTempFile(std::vector<PathPoint> resultPath, const char *tempFile)
    {
        string tempFilename = string(tempFile) + "lqr.txt";
        ofstream outFile2(tempFilename.c_str(), ios::out | ios::binary);
        savePose *savepose = new savePose;
        int id = 0;
        std::cout << "path length" << resultPath.size() << std::endl;
        // write result path to tempFile
        vector<Eigen::Vector2d> points;
        PiecewiseBezierCurve<2> curve;
        std::vector<double> valutiys;
        std::vector<double> acceles;
        for (size_t i = 0; i != resultPath.size(); ++i)
        {
            Eigen::Vector2d p;
            p.x() = resultPath[i].x;
            p.y() = resultPath[i].y;
            valutiys.push_back(resultPath[i].dl);
            acceles.push_back(resultPath[i].ddl);
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
        vector<double> curvitys = calculate_curvature_difference(result_path);
        int id2 = 0;
        cout << "Saving trajectory ..." << endl;

        std::vector<double> interacceleras = linearInterpolation(acceles, result_path.size());
        std::vector<double> interValitys = linearInterpolation(valutiys, result_path.size());
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
        tk::spline x_s, y_s;
        x_s.set_points(s_set, x_set);
        y_s.set_points(s_set, y_set);
        for (int i = 0; i < s_set.size(); i++)
        {
            double h = path::getHeading(x_s, y_s, s_set[i]);
            heading.push_back(h);
        }
        for (size_t i = 0; i != result_path.size(); ++i)
        {
            id2++;
            savepose->id = id2;
            savepose->x = result_path[i].x;
            savepose->y = result_path[i].y;
            savepose->z = 0;
            savepose->theta = heading[i];// + car_state.theta;
            savepose->kappas = curvitys[i];
            savepose->v = interValitys[i];
            savepose->a = interacceleras[i];
            outFile2.write((char *)savepose, sizeof(savePose));
        }
        outFile2.close();
        delete savepose;
    }

    int writeMemory(T_lqrPose *lqrPose)
    {
        std::cout << "path length" << LocalPath.size() << std::endl;
        // write result path to tempFile
        vector<Eigen::Vector2d> points;
        PiecewiseBezierCurve<2> curve;
        for (size_t i = 0; i != LocalPath.size(); ++i)
        {
            Eigen::Vector2d p;
            p.x() = LocalPath[i].x;
            p.y() = LocalPath[i].y;
            points.push_back(p);
        }
        curve.fit(points, 0.01);
        // resample point
        vector<Point2f> result_path;
        for (Bezier<3, 2> b : curve.getPiecewiseBeziers())
        {
            for (auto &p : b.sampleWithArcLengthParameterized(0.1, true, 5))
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
        int id2 = 0;
        cout << "Saving trajectory ..." << endl;
        vector<double> valuity(result_path.size(), 0.83);
        // for (size_t i = 0; i < result_path.size(); ++i)
        // {
        //     if (curvitys[i] > 0.03)
        //     {
        //         for (int j = 0; j < 80; j++)
        //         {
        //             valuity[i + j] = 1.94 - 0.014 * j;
        //         }
        //         break;
        //     }
        // }
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
        tk::spline x_s, y_s;
        x_s.set_points(s_set, x_set);
        y_s.set_points(s_set, y_set);
        for (int i = 0; i < s_set.size(); i++)
        {
            double h = path::getHeading(x_s, y_s, s_set[i]);
            heading.push_back(h);
        }
        for (size_t i = 0; i != result_path.size(); ++i)
        {
            id2++;
            (lqrPose + i)->id = id2;
            (lqrPose + i)->x = result_path[i].x;
            (lqrPose + i)->y = result_path[i].y;
            (lqrPose + i)->z = 0;
            (lqrPose + i)->theta = heading[i];// + car_state.theta;
            (lqrPose + i)->kappas = curvitys[i];
            (lqrPose + i)->v = valuity[i];
            (lqrPose + i)->a = 0;
        }
        return result_path.size();
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
