#include <iostream>
#include <vector>
#include <tuple>
#include <unistd.h>
#include <sys/stat.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include "eigen3/Eigen/Dense"
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include "eigen2cv.hpp"
#include "reference_line_processor.h"
#include "path/data_structure.h"
#include "path/path_problem_manager.h"
#include "solver/solver.h"
#include "path/gflags.h"
#include "path/tool.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml/ml.hpp"
// TODO: this file is a mess.
#include "bspline.h"
#include <tinyspline_ros/tinysplinecpp.h>
#include "j3_perception_data_api.h"
#include "run_code.h"

#include <filesystem> // 要求C++17标准

namespace fs = std::filesystem;
using namespace PathPlanning;
PathPlanning::PathPoint start_state, end_state, car_state, car_state_local;
bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;
std::vector<PathPlanning::PathPoint> reference_path_plot;
std::vector<cv::Point2f> referencePoint;
bool isFirst = true;
double resolution = 0.1;
int width = 500;
int height = 500;
using namespace cv;
using namespace std;

struct savePose
{
    int id;
    double x;
    double y;
    double z;
    double theta;
};

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
    double delta_t = 1.0 / length;
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
    // std::cout<<disx<<","<<disy<<std::endl;
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
        std::cout << "received a reference point" << referencePoint[i] << std::endl;
    }
}
void referenceTurnPoint(int endIndex, int index, vector<Vec3d> pathPoints, double thX, double thY)
{
    int num = (endIndex - index) / 6;
    reference_path_plot.clear();
    referencePoint.clear();
    double refX, refY;
    for (size_t i = 0; i < 5 * num + 1; i = i + num)
    {
        Point2d globalP, pathP;
        globalP.x = pathPoints[index + i][0] + thX;
        globalP.y = pathPoints[index + i][1] + thY;
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
        // double y = -xs / resolution + 74;
        // double x = -ys / resolution + 96;
        // cv::Point p(x, y);
        // std::cout << "reference_path_plot111111: " << x << "," << y << std::endl;
        // circle(img_src, p, 1, cv::Scalar(0, 0, 0), -1);
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

int main(int argc, char **argv)
{
    bool isTurn = false;
    // 注意改成你对应的路径
    string root_path = "/home/gq/guoqian/PathPlanning/J3map/";
    string data_id = "0415-02";
    string xml_file = root_path + data_id + "/photo/";

    // step1 先扫描json文件，获取文件名和文件数量
    cout << "@yang: [step 1]: scan xml folder" << "\n";
    vector<string> all_file_names;
    string mea = xml_file;
    if (!fs::exists(mea))
    {
        std::cerr << "目录不存在: " << mea << std::endl;
        return -1;
    }
    else if (!fs::is_directory(mea))
    {
        std::cerr << "路径不是目录: " << mea << std::endl;
        return -1;
    }

    // 递归遍历目录（包含子目录）
    for (const auto &entry : fs::recursive_directory_iterator(
             mea,
             fs::directory_options::skip_permission_denied))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".xml")
        {
            // 获取不带后缀的文件名
            std::string filename = entry.path().stem().string();
            all_file_names.emplace_back(filename);
            // std::cout << filename << std::endl;
        }
    }

    // step2 读取xml文件，解析数据，并绘制地图
    cout << "@yang: [step 2]: read and draw map" << "\n";
    string save_new_map = root_path + data_id + "/new_map/";
    fs::create_directories(save_new_map); // 生成保存地图文件的文件夹
    for (int i = 0; i < 1; i++)           // all_file_names.size(); i++)
    {
        string read_xml_path = mea + all_file_names[i] + ".xml";
        string save_map_path = save_new_map + all_file_names[i] + ".jpg";
        J3PerceptionData j3p_data;
        bool load_ret = tinyxml2_case02::loadPerceptionDataFromXML(j3p_data, read_xml_path); // 加载xml文件为结构体变量
        if (load_ret)
        {
            Mat new_map;
            unsigned long time;
            // draw_map::draw_front_perception_img(&j3p_data, new_map, time);//把结构体变量画成地图mat，此函数参数使用默认地图参数

            // draw_map::draw_front_perception_bgr(j3p_data, new_map, time);
            draw_map::draw_front_perception_gray(j3p_data, new_map, time);

            imwrite(save_map_path, new_map); // 保存图
            cout << "i: " << i << " -> " << all_file_names[i] << endl;
            imshow("new_map", new_map);
            waitKey(0);

            vector<Vec3d> pathPoints;
            int num = 0;
            if (false)
            {
                ifstream fOdom;
                fOdom.open("/home/gq/guoqian/data/1horn/dvr_plan/camera_odom.txt");
                double ts;
                while (!fOdom.eof())
                {
                    Vec3d p;
                    string s;
                    getline(fOdom, s);
                    if (s[0] == '#')
                        continue;
                    stringstream ss(s);
                    ss >> ts;
                    ss >> p(0) >> p(1) >> p(2);
                    // std::cout<<p(0)<<","<<p(1)<<std::endl;
                    pathPoints.push_back(p);
                    num++;
                }
            }
            else
            {
                // read original path
                savePose *savep = new savePose;
                ifstream inFile("/home/gq/guoqian/data/save_slam_1.txt", ios::in | ios::binary);

                while (inFile.read((char *)savep, sizeof(savePose)))
                {
                    Vec3d p;
                    p[0] = savep->x;
                    p[1] = savep->y;
                    p[2] = savep->theta + 1.57096;
                    pathPoints.push_back(p);
                    num++;
                    // std::cout << p[0]<<","<<p[1]<<","<<savep->theta+1.57 << std::endl;
                }
                delete savep;
            }

            // car current state pathPoint the last point is distance <8m the last path planning
            // double pathDistance = pow((car_state.x - pathPoints[num][0]), 2) + pow((car_state.y - pathPoints[num][1]), 2);
            // if (!isFirst && pathDistance < 65)
            // {
            //     std::cout << "the last path planning" << std::endl;
            // }
            cv::Mat image = new_map;
            for (int j = 0; j < 1; j++)
            {
                car_state.x = 0;              //  Y: T:-1.643736
                car_state.y = -0.003582;              // currentPose[j][1];              //-78.2019;//0.000;//-7.16728;
                car_state.theta = -1.589395 + 1.5706; // currentPose[j][2] + 1.5706; // 4.70266;//-0.000796446;//-1.31224;

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
                // Initialize grid map from image.
                cv::Mat img_src = binaryImage;
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
                double targetFreespaceX = j3p_data.J3P_Freespace.contours_points[32].x;
                double targetFreespaceY = j3p_data.J3P_Freespace.contours_points[32].y;
                double targetDistance = targetFreespaceX*targetFreespaceX+targetFreespaceY*targetFreespaceY;
                // cv::imshow("interSection",interSection);
                // cv::waitKey(0);
                printf("find the path respecting point\n");
                // path splicing
                double mDis = 10000;
                int index = 0;
                for (int i = 0; i < pathPoints.size(); i++)
                {
                    double distance = pow((car_state.x - pathPoints[i][0]), 2) + pow((car_state.y - pathPoints[i][1]), 2);
                    if (distance < mDis)
                    {
                        mDis = distance;
                        index = i;
                    }
                }
                Point2d referPoint;
                referPoint.x = pathPoints[index][0];
                referPoint.y = pathPoints[index][1];
                // printf("find the end path point\n");
                Point2d endpathPoint, referPoint_local, endpathPoint_local;
                int endIndex = 0;
                if (isTurn)
                {
                    targetDistance = 100.0;
                }
                for (int i = index; i < pathPoints.size(); i++)
                {
                    double distance = pow((referPoint.x - pathPoints[i][0]), 2) + pow((referPoint.y - pathPoints[i][1]), 2);
                    // printf(i<<","<<pathPoints[i]<<"distance: "<<distance);
                    if (distance > targetDistance)
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
                double x = (height / 2 - pixelpathPoint.y) * resolution;
                double y = (width / 2 - pixelpathPoint.x) * resolution;
                grid_map::Position pose(x, y);
                double angleCh = abs(pathPoints[endIndex][2] - pathPoints[index][2]);
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

                if (isTurn)
                {
                    std::cout << "use mapping path curvity points2" << std::endl;
                    PathPoint state;
                    for (size_t i = index; i < endIndex; i++)
                    {
                        state.x = thX + pathPoints[i][0] + turndisX;
                        state.y = thY + pathPoints[i][1] + turndisY;
                        state.theta = pathPoints[i][2];
                    }
                }
                // else if (((angleCh > 0.3 || isTurn)&&dis>0) || needPath) // (&& dis > 1.0)
                else if (angleCh > 0.3 || isTurn)
                {
                    std::cout << "use mapping path curvity points3" << std::endl;
                    PathPoint state;
                    vector<Vec3d> pathPoint;
        
                        for (size_t i = index; i < endIndex; i++)
                        {
                            state.x = thX + pathPoints[i][0];
                            state.y = thY + pathPoints[i][1];
                            state.theta = pathPoints[i][2];
                        }
                    

                }
                // find the end point
                endCentrPoint = pixelpathPoint;
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
                start_state.x = -height / 2 * resolution;
                start_state.y = 0.0 * resolution;
                // set end state
                end_state.x = endpixelPoint.x * resolution;
                end_state.y = endpixelPoint.y * resolution;
                // car_state.theta = -0.066;
                if (true)
                {
                    referenceTurnPoint(endIndex, index, pathPoints, thX, thY);
                    double distancel;
                    for (size_t i = 0; i != reference_path_plot.size(); ++i)
                    {
                        grid_map::Position pose(reference_path_plot[i].x, reference_path_plot[i].y);
                        distancel = map.getObstacleDistance(pose);
                        // if (distancel < 2.0)
                        // {
                        //     reference_path_plot[i].x = start_state.x + disx * i;
                        //     reference_path_plot[i].y = start_state.y + disy * i;
                        // }
                        std::cout<<distancel<<std::endl;
                    }
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
                    // double the_end = atan((reference_path_plot[size - 1].y - reference_path_plot[size - 4].y) / (reference_path_plot[size - 1].x - reference_path_plot[size - 4].x));
                    // if ((the_end > 0 && pathPoints[endIndex][2] < 0) || (the_end < 0 && pathPoints[endIndex][2] > 0))
                    // {
                    //     the_end = pathPoints[endIndex][2];
                    // }
                    start_state.theta = car_state.theta;
                    end_state.theta = car_state.theta + pathPoints[endIndex][2] - pathPoints[index][2];
                    std::cout << "turn start state: " << start_state.x << "," << start_state.y << "," << start_state.theta << std::endl;
                    std::cout << "turn end state: " << end_state.x << "," << end_state.y << "," << end_state.theta << std::endl;
                }
                else
                {
                    double the = atan((end_state.y - start_state.y) / (end_state.x - start_state.x));
                    end_state.theta = car_state.theta + pathPoints[endIndex][2] - pathPoints[index][2];
                    start_state.theta = car_state.theta; // pathPoints[index][2];
                    std::cout << "start state: " << start_state.x << "," << start_state.y << "," << start_state.theta << std::endl;
                    std::cout << "end state: " << end_state.x << "," << end_state.y << "," << end_state.theta << std::endl;

                    // referenceTurnPoint(endIndex, index, pathPoints, thX, thY);
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
                }

                // path_optimizer.printObstacleDistance2(smoothed_reference_path);
                // printf("-------------------------------------------------------");
                // path_optimizer.printObstacleDistance(result_path);

                double angle = -car_state.theta;
                std::cout << "angle: " << angle << std::endl;
                cv::Mat img_src2(cv::Size(img_src.cols * 10, img_src.rows * 10), CV_8UC3, cv::Scalar(255, 255, 255));
                double disX, disY;
                for (size_t i = 0; i < result.size() - 1; i = i + 2)
                {

                    const auto path_point = result.at(i);
                    disX = (path_point.x + (width / 2) * resolution) * cos(angle) + path_point.y * sin(angle);
                    disY = -(path_point.x + (width / 2) * resolution) * sin(angle) + path_point.y * cos(angle);

                    std::cout << "result_path: " << car_state.x + disX << "," << car_state.y + disY << "," << path_point.theta - angle << std::endl;

                    double pathX = (car_state.x + disX);
                    double pathY = (car_state.y + disY);
                    cv::Point p(-pathY + height, -pathX + width);
                    // std::cout << "result point: " << pathX << "," << pathY << std::endl;
                    circle(img_src2, p, 1, cv::Scalar(0, 0, 0), -1);
                }
                for (size_t i = 0; i != result.size(); ++i)
                {
                    const auto path_point = result.at(i);
                    double y = -path_point.x / resolution + height / 2;
                    double x = -path_point.y / resolution + width / 2;
                    cv::Point p(x, y);
                    // std::cout << "result pixle: " << x << "," << y << std::endl;
                    circle(img_src, p, 1, cv::Scalar(0, 0, 0), -1);
                }
                for (size_t i = 0; i != reference_path_plot.size(); ++i)
                {
                    grid_map::Position pose(reference_path_plot[i].x, reference_path_plot[i].y);
                    // std::cout << "reference path obstacle distance: " << map.getObstacleDistance(pose) << std::endl;
                    double y = -reference_path_plot[i].x / resolution + height / 2;
                    double x = -reference_path_plot[i].y / resolution + width / 2;
                    cv::Point p(x, y);
                    // std::cout << "reference_path_plot: " << x << "," << y << std::endl;
                    circle(img_src, p, 1, cv::Scalar(255, 0, 0), -1);
                }
                isFirst = false;
                cv::imwrite("img.jpg", img_src);
                cv::imshow("image", img_src);
                cv::waitKey(0);
            }
        }
    }
}