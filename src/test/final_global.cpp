#include <fstream>
#include <iostream>
#include <vector>
#include <glob.h>
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
#include "path_optimizer_2/path_optimizer.hpp"
#include "tools/eigen2cv.hpp"
#include "data_struct/data_struct.hpp"
#include "tools/tools.hpp"
#include "data_struct/reference_path.hpp"
#include "tools/spline.h"
#include "config/planning_flags.hpp"

PathOptimizationNS::State start_state, end_state, car_state, car_state_local;
std::vector<PathOptimizationNS::State> reference_path_plot;
PathOptimizationNS::ReferencePath reference_path_opt;
bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;
std::vector<cv::Point2f> referencePoint;
bool isFirst = true;
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

int main(int argc, char **argv)
{
    double resolution = 0.1; // 1 pixel match 10cm

    cv::Mat imageTest = cv::imread("/home/gq/guoqian/PathPlanning/hybridAstar/hybridAstar_lbfgsSmooth/src/create_maps/map2.jpg");
    cv::resize(imageTest, imageTest, cv::Size(imageTest.cols, imageTest.rows));
    cv::cvtColor(imageTest, imageTest, COLOR_BGR2GRAY);
    threshold(imageTest, imageTest, 120, 255, 0);
    // vector<vector<Point>> contours;
    // vector<Vec4i> hierarchy;
    // findContours(imageTest, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    // cv::Mat mapImage(imageTest.size(), CV_8UC1, Scalar(255));
    // std::cout << contours.size() << std::endl;
    // int i = 0;
    // if (contours.size() > 1)
    // {
    //     cv::drawContours(mapImage, contours, -1, cv::Scalar(0), 5);
    // }

    // std::cout << imageTest.size() << std::endl;
    cv::Mat mapImage = imageTest;
    // cv::imshow("image",mapImage);
    // cv::imwrite("mapTest.jpg",mapImage);
    // cv::waitKey(0);
    ifstream filePixel;
    double oriPixelPointX = 75;
    double oriPixelPointY = 925;
    std::cout << oriPixelPointX << "," << oriPixelPointY << std::endl;

    vector<Vec3d> pathPoints;
    int num = 0;
    // Convert pose to current pose
    ifstream file;
    string poseFile = "/home/gq/guoqian/PathPlanning/hybridAstar/hybridAstar_lbfgsSmooth/src/create_maps/Read2dMap/data/PPdata/0930-01/Map2D_carPoseData3D1.txt";
    file.open(poseFile.c_str());
    while (!file.eof())
    {
        string s;
        getline(file, s);

        stringstream ss(s);
        double id, x, y, angle;
        ss >> id;
        ss >> x;
        ss >> y;
        ss >> angle;
        Vec3d p;
        p[0] = x / 100;
        p[1] = y / 100;
        p[2] = angle * 3.14 / 180;
        pathPoints.push_back(p);
    }
    file.close();
    // Convert pathPoint to pixelPoint
    cv::Mat drawImage = mapImage;
    // for (int i=0;i<pathPoints.size();i++)
    // {
    //     cv::Point p((pathPoints[i][0]+200),(-pathPoints[i][1]+300));
    //     circle(mapImage,p,1, cv::Scalar(0, 0, 0), -1);
    // }
    double oriPixelX = oriPixelPointX;
    double oriPixelY = oriPixelPointY;
    double firstPosX = pathPoints[0][0];
    double firstPosY = pathPoints[0][1];

    for (int i = 0; i < pathPoints.size(); i++)
    {
        cv::Point point((oriPixelX - pathPoints[i][1] / resolution), (oriPixelY - pathPoints[i][0] / resolution));
        // std::cout<<point<<std::endl;
        // circle(drawImage,point,1, cv::Scalar(0, 0, 0), -1);
    }

    cv::imshow("image", drawImage);
    cv::waitKey(0);

    cv::Mat img_src = mapImage;
    // dilateImage expand down 400 pixel for car length 4.9m
    // cv::copyMakeBorder(img_src, img_src, 0, 500, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

    grid_map::GridMap grid_map(std::vector<std::string>{"obstacle", "distance"});
    grid_map::GridMapCvConverter::initializeFromImage(img_src, resolution, grid_map, grid_map::Position::Zero());
    // Add obstacle layer.
    unsigned char OCCUPY = 0;
    unsigned char FREE = 255;
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(img_src, "obstacle", grid_map, OCCUPY, FREE, 0.25);
    // Update distance layer.
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary = grid_map.get("obstacle").cast<unsigned char>();
    cv::distanceTransform(eigen2cv(binary), eigen2cv(grid_map.get("distance")), CV_DIST_L2, CV_DIST_MASK_PRECISE);
    grid_map.get("distance") *= resolution;

    std::cout << img_src.cols << "," << img_src.rows << std::endl;
    // set start state
    int numPoint = pathPoints.size();
    start_state.y = -((oriPixelX - pathPoints[0][1] / resolution) - img_src.cols / 2) * resolution;
    start_state.x = -((oriPixelY - pathPoints[0][0] / resolution) - img_src.rows / 2) * resolution;
    start_state.heading = pathPoints[0][2];
    // set end state
    end_state.x = -((oriPixelY - pathPoints[numPoint - 1][0] / resolution) - img_src.rows / 2) * resolution;
    end_state.y = -((oriPixelX - pathPoints[numPoint - 1][1] / resolution) - img_src.cols / 2) * resolution;
    end_state.heading = pathPoints[numPoint - 1][2];
    ;
    // referenceTurnPoint(endIndex, index, pathPoints, thX, thY);
    reference_path_plot.clear();
    referencePoint.clear();
    double refX, refY;
    for (size_t i = 0; i < numPoint; i = i + 20)
    {
        Point2d pixelP;
        pixelP.x = img_src.rows / 2 - (oriPixelY - pathPoints[i][0] / resolution);
        pixelP.y = img_src.cols / 2 - (oriPixelX - pathPoints[i][1] / resolution);
        double xs = pixelP.x * resolution;
        double ys = pixelP.y * resolution;
        cv::Point2f point0(xs, ys);
        if (i == 0)
        {
            refX = xs;
            refY = ys;
        }
        referencePoint.push_back(point0);
    }

    for (int i = 0; i < referencePoint.size(); i++)
    {
        PathOptimizationNS::State reference_point;
        reference_point.x = referencePoint[i].x - (refX - start_state.x);
        reference_point.y = referencePoint[i].y - (refY - start_state.y);
        reference_path_plot.emplace_back(reference_point);
        reference_rcv = reference_path_plot.size() >= 6;
        // std::cout << "received a reference point" <<referencePoint[i]<< std::endl;
    }

    std::cout << "start state: " << start_state.x << "," << start_state.y << "," << start_state.heading << std::endl;
    std::cout << "end state: " << end_state.x << "," << end_state.y << "," << end_state.heading << std::endl;
    // Calculate.
    std::vector<PathOptimizationNS::SlState> result;

    bool opt_ok = false;
    if (reference_rcv)
    {
        PathOptimizationNS::PathOptimizer path_optimizer(start_state, end_state, grid_map);
        // calcualte distance from obstacle
        for (int i = 0; i < reference_path_plot.size(); i++)
        {
            // std::cout << reference_path_plot[i].x << "," << reference_path_plot[i].y << std::endl;
            double distance = path_optimizer.calDisObs(reference_path_plot[i].x, reference_path_plot[i].y);
            // std::cout<< "reference path point distance: "<<distance<<std::endl;
            if (distance < 1.0)
            {
                if (path_optimizer.calDisObs(reference_path_plot[i].x, reference_path_plot[i].y) < 1.0)
                {
                    for(double j = -2; j < 2; j = j + 0.5)
                    {
                        double dis = 1.0;
                        if (path_optimizer.calDisObs(reference_path_plot[i].x, reference_path_plot[i].y + j) > dis)
                        {
                            reference_path_plot[i].y = reference_path_plot[i].y + j;
                            dis = path_optimizer.calDisObs(reference_path_plot[i].x, reference_path_plot[i].y + j);
                        }
                    }
                }
                if (path_optimizer.calDisObs(reference_path_plot[i].x, reference_path_plot[i].y) < 1.0)
                {
                    double dis = 1.0;
                    for(double j = -2; j < 2; j = j + 0.5)
                    {
                        if (path_optimizer.calDisObs(reference_path_plot[i].x+j, reference_path_plot[i].y) > dis)
                        {
                            reference_path_plot[i].x = reference_path_plot[i].x + j;
                            dis = path_optimizer.calDisObs(reference_path_plot[i].x+j, reference_path_plot[i].y);
                        }
                    }
                }
            }
        }
        // for (int i = 0; i < reference_path_plot.size(); i++)
        // {
        //     // std::cout << reference_path_plot[i].x << "," << reference_path_plot[i].y << std::endl;
        //     double distance = path_optimizer.calDisObs(reference_path_plot[i].x, reference_path_plot[i].y);
        //     std::cout << "reference path point distance: " << distance << std::endl;
        // }
        opt_ok = path_optimizer.solve(reference_path_plot, &result);
        reference_path_opt = path_optimizer.getReferencePath();

        if (opt_ok)
        {
            std::cout << "ok!" << std::endl;
        }
    }
    double angle = 0.0;
    cv::Mat img_src2(cv::Size(img_src.cols, img_src.rows), CV_8UC3, cv::Scalar(255, 255, 255));
    double disX, disY;
    for (size_t i = 0; i != result.size(); ++i)
    {
        const auto path_point = result.at(i);
        double y = -path_point.x / resolution + img_src.rows / 2;
        double x = -path_point.y / resolution + img_src.cols / 2;
        cv::Point p(x, y);
        std::cout << "result pixle: " << x << "," << y <<","<<path_point.heading<< std::endl;
        circle(img_src, p, 1, cv::Scalar(0, 0, 0), -1);
    }
    for (size_t i = 0; i != reference_path_plot.size(); ++i)
    {
        grid_map::Position pose(reference_path_plot[i].x, reference_path_plot[i].y);
        // std::cout << "reference path obstacle distance: " << map.getObstacleDistance(pose) << std::endl;
        double y = -reference_path_plot[i].x / resolution + img_src.rows / 2;
        double x = -reference_path_plot[i].y / resolution + img_src.cols / 2;
        cv::Point p(x, y);
        // std::cout << "reference_path_plot: " << x << "," << y << std::endl;
        circle(img_src, p, 1, cv::Scalar(0, 0, 0), -1);
    }
    cv::imshow("image", img_src);
    cv::waitKey(0);
}