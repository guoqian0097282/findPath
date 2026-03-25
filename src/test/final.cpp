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
#include "tools/Map.hpp"

PathOptimizationNS::State start_state, end_state, car_state, car_state_local;
bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;
std::vector<PathOptimizationNS::State> reference_path_plot;
PathOptimizationNS::ReferencePath reference_path_opt;
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
        PathOptimizationNS::State reference_point;
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
        pathP.x = globalP.x * cos(car_state.heading) + globalP.y * sin(car_state.heading);
        pathP.y = -globalP.x * sin(car_state.heading) + globalP.y * cos(car_state.heading);
        // pathP.x = globalP.x;
        // pathP.y = globalP.y;

        Point2d imagepathP, pixelP;
        imagepathP.x = abs(pathP.x - car_state_local.x) - 5.4;
        imagepathP.y = pathP.y - car_state_local.y;
        pixelP.x = 960 - imagepathP.y / 0.1 * 10;
        pixelP.y = 540 - imagepathP.x / 0.1 * 10;
        Point2d endpixelP;
        endpixelP.x = 540 - pixelP.y;
        endpixelP.y = 960 - pixelP.x;
        double xs = endpixelP.x / 10 * 0.1 + 20 * 0.1;
        double ys = endpixelP.y / 10 * 0.1;
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
        PathOptimizationNS::State reference_point;
        reference_point.x = referencePoint[i].x - disX;
        reference_point.y = referencePoint[i].y - disY;
        reference_path_plot.emplace_back(reference_point);
        reference_rcv = reference_path_plot.size() >= 6;
        // std::cout << "received a reference point" <<referencePoint[i]<< std::endl;
    }
}

int main(int argc, char **argv)
{
    bool isTurn = false;
    Ptr<ml::TrainData> mlDataX, mlDataY;
    mlDataX = ml::TrainData::loadFromCSV("/home/gq/guoqian/PathPlanning/Findpath2/build/mapX.csv", 1);
    mlDataY = ml::TrainData::loadFromCSV("/home/gq/guoqian/PathPlanning/Findpath2/build/mapY.csv", 1);

    cv::Mat dataX = mlDataX->getTrainSamples();
    cv::Mat dataY = mlDataY->getTrainSamples();
    cv::Mat image = cv::imread("/home/gq/guoqian/PathPlanning/DNN_test/DNN_result/0.jpg");
    // cv::Mat image = cv::imread("/home/gq/guoqian/data/1horn/dvr_plan/plan_38.jpg");
    cv::Mat im, img_src;
    cv::remap(image, im, dataX, dataY, INTER_LINEAR);
    imwrite("warped.jpg", im);

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
        /* b_spline with pathpoints
        cv::Mat imgTest(cv::Size(300, 300), CV_8UC3, cv::Scalar(255, 255, 255));
        vector<Vec3d> pathPoints2 = B_spline(pathPoints);
        for(int i=0;i<pathPoints.size();i++)
        {
            cv::Point point(pathPoints[i][0]*10,pathPoints[i][1]*10+300);
            circle(imgTest,point,2,cv::Scalar(0, 0, 0), -1);
        }
        for(int i=0;i<pathPoints2.size();i++)
        {
            cv::Point point(pathPoints2[i][0]*10,pathPoints2[i][1]*10+300);
            circle(imgTest,point,2,cv::Scalar(255, 255, 0), -1);
        }

        imshow("imgT",imgTest);
        cv::waitKey(0);
        std::cout << "b spline pathPoints:"<<pathPoints.size()<<","<<pathPoints2.size()<<"," << std::endl;
        */
        delete savep;
    }

    // car current state pathPoint the last point is distance <8m the last path planning
    double pathDistance = pow((car_state.x - pathPoints[num][0]), 2) + pow((car_state.y - pathPoints[num][1]), 2);
    if (!isFirst && pathDistance < 65)
    {
        std::cout << "the last path planning" << std::endl;
    }
    // 0.DNN model get freespace and fisheye calibration
    // 1.Bev Tansform
    // Mat image = cv::imread("../0.jpg");
    // ifstream inFile;
    // inFile.open("/home/gq/guoqian/data/1horn/YUV2RGB/dvr_1218/carinfo.txt");
    // int id;
    // vector<Vec3d> currentPose;
    // while (!inFile.eof())
    // {
    //     string s;
    //     getline(inFile, s);
    //     stringstream ss(s);
    //     ss >> id;
    //     Vec3d p;
    //     ss >> p[0] >> p[1] >> p[2];
    //     currentPose.push_back(p);
    //     // std::cout << p[0]<<","<<p[1]<<","<<savep->theta+1.57 << std::endl;
    // }

    for (int j = 0; j < 1; j++)
    {
        double resolution = 0.01;              // in meter per pixel
        car_state.x = -36.633717;              //  Y: T:-1.643736
        car_state.y = -97.2294;                // currentPose[j][1];              //-78.2019;//0.000;//-7.16728;
        car_state.heading = 1.539257 + 1.5706; // currentPose[j][2] + 1.5706; // 4.70266;//-0.000796446;//-1.31224;
        // Mat warped_part;
        // warpPerspective(image, warped_part, Mtrans, image.size(), INTER_LINEAR);
        Mat warped = cv::imread("./warped.jpg");
        // Mat warped = warped_part(cv::Rect(0, 100, 1920, 980));
        // imwrite("warped.jpg", warped_part);
        // imshow("image1", warped);
        // waitKey(0);
        // 2. binaryImage to get Areas of exercise
        Point2f src[4] = {Point2f(837, 491), Point2f(881, 416), Point2f(1056, 416), Point2f(1094, 494)};
        Point2f dst[4] = {Point2f(930, 800), Point2f(930, 650), Point2f(1080, 650), Point2f(1080, 800)};
        Mat Mtrans = getPerspectiveTransform(src, dst);
        Mat warped_tran;
        warpPerspective(warped, warped_tran, Mtrans, image.size(), INTER_LINEAR);
        // imwrite("/home/gq/guoqian/PathPlanning/Findpath探索前进/test_image/0.jpg", warped_tran);
        // resolution = resolution * 10;
        // cv::resize(warped, warped, cv::Size(warped.cols / 10, warped.rows / 10));
        // 2. binaryImage to get Areas of exercise
        cv::Mat gray, binaryImage;
        if (warped_tran.channels() == 3)
        {
            cv::cvtColor(warped_tran, gray, COLOR_BGR2GRAY);
        }
        else
        {
            warped_tran.copyTo(gray);
        }
        threshold(gray, binaryImage, 120, 255, 0);
        // lane detect
        double laneDis = 1.9;
        if (!isTurn && abs(laneDis) < 1.8 && laneDis > 0.1)
        {
            double pixel = (laneDis + 1.5) / resolution;
            double pixel2 = (3.5 - laneDis) / resolution;
            double pixel3 = (2.5 - laneDis) / resolution;
            for (int i = 960 + pixel; i < 1920; i++)
            {
                for (int j = 0; j < 1080; j++)
                {
                    binaryImage.at<uchar>(j, i) = 0;
                }
            }
            for (int i = 0; i < 960 - pixel2; i++)
            {
                for (int j = 0; j < 1080; j++)
                {
                    binaryImage.at<uchar>(j, i) = 0;
                }
            }
            // for (int i = 960 - pixel3; i < 960 + pixel; i++)
            // {
            //     for (int j = 0; j < 1080; j++)
            //     {
            //         binaryImage.at<uchar>(j, i) = 255;
            //     }
            // }
        }
        // cv::Mat warped_cut = binaryImage(cv::Rect(0, 200, 1920, 980));
        // cv::copyMakeBorder(warped_cut, binaryImage, 0, 200, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
        // 3. remove small object and dilate erode
        // printf("remove small object and dilate erode\n");
        // Mat erodeImage, dilateImage;
        // Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
        // Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(19, 19));
        // Mat element3 = getStructuringElement(MORPH_ELLIPSE, Size(19, 19));
        // dilate(binaryImage, dilateImage, element1);
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        // findContours(dilateImage, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
        // int contSize = contours.size();
        // contours.erase(remove_if(contours.begin(), contours.end(),
        //                          [](const std::vector<cv::Point> &c)
        //                          { return cv::contourArea(c) < 4000; }),
        //                contours.end());
        // dilateImage.setTo(0);
        // cv::drawContours(dilateImage, contours, -1, cv::Scalar(255), cv::FILLED);
        // erode(dilateImage, erodeImage, element2);
        // dilate(erodeImage, dilateImage, element3);
        // cv::imshow("erodeImage",erodeImage);
        // cv::imshow("removeImage",dilateImage);
        // cv::waitKey(0);
        // 4.补全车前的盲区
        for (int i = 760; i < 1160; i++)
        {
            for (int j = 1000; j < 1080; j++)
            {
                binaryImage.at<uchar>(j, i) = 255;
            }
        }
        cv::Mat warped_cut = binaryImage(cv::Rect(0, 100, 1920, 980));
        cv::copyMakeBorder(warped_cut, binaryImage, 0, 100, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
        cv::imshow("binaryImage", binaryImage);
        cv::waitKey(0);
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
        cv::Mat circleImage(binaryImage.size(), CV_8UC1, Scalar(0));
        double targetPixel = 900;
        circle(circleImage, Point(960, 1080), targetPixel, Scalar(255), 30);
        // cv::imwrite("circleImage.jpg", circleImage);
        // cv::imshow("circleImage", circleImage);
        // cv::imshow("dilateImage", dilateImage);
        // cv::waitKey(0);
        cv::Mat interSection(binaryImage.size(), CV_8UC1, Scalar(0));
        bitwise_and(circleImage, binaryImage, interSection);
        // cv::imshow("interSection",interSection);
        // cv::waitKey(0);
        // Initialize grid map from image.
        cv::Mat img_src;
        // cv::Mat img_src = dilateImage;
        // dilateImage expand down 400 pixel for car length 4.9m
        cv::copyMakeBorder(binaryImage, img_src, 0, 400, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
        resolution = resolution * 10;
        cv::resize(img_src, img_src, cv::Size(img_src.cols / 10, img_src.rows / 10));

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
        // 7. 选取终点
        findContours(interSection, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
        vector<Point2d> centerPoint;
        for (int i = 0; i < contours.size(); i++)
        {
            if (cv::contourArea(contours[i]) > 5000)
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
        // path splicing
        int mDis = 10000;
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
        double targetPath = 78.0;
        if (isTurn)
        {
            targetPath = 100.0;
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
        referPoint_local.x = referPoint.x * cos(car_state.heading) + referPoint.y * sin(car_state.heading);
        referPoint_local.y = -referPoint.x * sin(car_state.heading) + referPoint.y * cos(car_state.heading);
        endpathPoint_local.x = endpathPoint.x * cos(car_state.heading) + endpathPoint.y * sin(car_state.heading);
        endpathPoint_local.y = -endpathPoint.x * sin(car_state.heading) + endpathPoint.y * cos(car_state.heading);
        car_state_local.x = car_state.x * cos(car_state.heading) + car_state.y * sin(car_state.heading);
        car_state_local.y = -car_state.x * sin(car_state.heading) + car_state.y * cos(car_state.heading);

        Point2d imagepathPoint, pixelpathPoint, endCentrPoint, endCentrPoint1, endCentrPoint2;
        imagepathPoint.x = abs(endpathPoint_local.x - car_state_local.x) - 54 * resolution;
        imagepathPoint.y = endpathPoint_local.y - car_state_local.y;
        pixelpathPoint.x = 960 - imagepathPoint.y / resolution * 10;
        pixelpathPoint.y = 540 - imagepathPoint.x / resolution * 10;
        double minDis = 1000000;
        double x = (540 - pixelpathPoint.y) / 10 * resolution + 20 * resolution;
        double y = (960 - pixelpathPoint.x) / 10 * resolution;
        grid_map::Position pose(x, y);
        double angleCh = abs(pathPoints[endIndex][2] - pathPoints[index][2]);
        grid_map::Position pose1(x - 0.5, y);
        grid_map::Position pose2(x + 0.5, y);

        PathOptimizationNS::Map map(grid_map);
        double dis = map.getObstacleDistance(pose);
        double dis1 = map.getObstacleDistance(pose1);
        double dis2 = map.getObstacleDistance(pose2);
        std::cout << "grid_map.getDistance pixelpathpoint: " << dis << "," << dis1 << "," << dis2 << std::endl;

        double turndisX = 0;
        double turndisY = 0;

        if (dis1 > dis && dis1 > dis2)
        {
            turndisX = -0.5 * cos(-car_state.heading);
            turndisY = 0;
        }
        else if (dis2 > dis && dis2 > dis1)
        {
            turndisX = 0.5 * cos(-car_state.heading);
            turndisY = 0;
        }

        if (centerPoint.size() == 0)
        {
            std::cout << "use mapping path curvity points1" << std::endl;
            PathOptimizationNS::State state;
            for (size_t i = index; i < endIndex - 3; i = i + 4)
            {
                state.x = thX + pathPoints[i][0];
                state.y = thY + pathPoints[i][1];
                state.heading = pathPoints[i][2];
            }
        }
        else if (isTurn)
        {
            std::cout << "use mapping path curvity points2" << std::endl;
            PathOptimizationNS::State state;
            for (size_t i = index; i < endIndex - 3; i = i + 4)
            {
                state.x = thX + pathPoints[i][0] + turndisX;
                state.y = thY + pathPoints[i][1] + turndisY;
                state.heading = pathPoints[i][2];
            }
        }
        // else if (((angleCh > 0.3 || isTurn)&&dis>0) || needPath) // (&& dis > 1.0)
        else if (angleCh > 0.3 || isTurn)
        {
            std::cout << "use mapping path curvity points3" << std::endl;
            PathOptimizationNS::State state;
            vector<Vec3d> pathPoint;
            if (true)
            {
                for (size_t i = index; i < endIndex; i++)
                {
                    state.x = thX + pathPoints[i][0];
                    state.y = thY + pathPoints[i][1];
                    state.heading = pathPoints[i][2];
                }
            }
        }
        // if (centerPoint.size() == 0)
        // {
        //     printf("cannot find centerPoint\n");
        //     return PathPlan_NOCENTERPOINT;
        // }
        double xCen = (540 - centerPoint[0].y) / 10 * resolution + 20 * resolution;
        double yCen = (960 - centerPoint[0].x) / 10 * resolution;
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
            double path_dis = sqrt(minDis) * resolution / 10;
            std::cout << "path_dis: " << path_dis << "laneDis: " << laneDis << std::endl;
            if (thXL > 0 || thYL > 0)
            {
                if (poseDistance > 0.5)
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
                if (path_dis > 1.5 && poseDistance > 0.5)
                {
                    std::cout << "1path_dis is larger than 1.5: " << path_dis << std::endl;
                    endCentrPoint.x = pixelpathPoint.x;
                    endCentrPoint.y = pixelpathPoint.y;
                }
                else if (laneDis >= 1.2 && pixelpathPoint.x > 960 && poseDistance > 0.5)
                {
                    std::cout << "2laneDis is larger than 1.2: " << laneDis << std::endl;
                    endCentrPoint.x = pixelpathPoint.x;
                    endCentrPoint.y = pixelpathPoint.y;
                }
                else if (laneDis <= 0.5 && laneDis > 0.1 && pixelpathPoint.x < 960 && poseDistance > 0.5)
                {
                    std::cout << "3laneDis is small than 0.5: " << laneDis << std::endl;
                    endCentrPoint.x = pixelpathPoint.x;
                    endCentrPoint.y = pixelpathPoint.y;
                }
                else if (path_dis < 1.5 && laneDis >= 1.2 && centerPoint[0].x > 960 && poseCenDistance > 0.8)
                {
                    std::cout << "4laneDis is larger than 1.2: " << laneDis << std::endl;
                    endCentrPoint.x = endCentrPoint.x;
                    endCentrPoint.y = endCentrPoint.y;
                }
                else if (path_dis < 1.5 && laneDis <= 0.5 && laneDis > 0.1 && endCentrPoint.x < 960 && poseCenDistance > 0.8)
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
            double path_dis = sqrt(dist) * resolution / 10;
            std::cout << "centerPoint: " << centerPoint[0].x << "," << centerPoint[0].y << "," << path_dis << std::endl;
            if (thXL > 0 || thYL > 0)
            {
                if (poseDistance > 0.5)
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
                if (angleCh < 0.3 && (poseDistance > 0.8 || path_dis < 0.5))
                {
                    std::cout << "1path_dis is larger: " << path_dis << std::endl;
                    endCentrPoint = pixelpathPoint;
                }
                else if (laneDis <= 0.1 && poseCenDistance > poseDistance)
                {
                    std::cout << "2path_dis is larger: " << path_dis << std::endl;
                    endCentrPoint = centerPoint[0];
                }
                else if (path_dis > 1.5 && poseDistance > 0.5)
                {
                    std::cout << "3path_dis is larger: " << path_dis << std::endl;
                    endCentrPoint = pixelpathPoint;
                }
                else if (laneDis >= 1.2 && pixelpathPoint.x > 960 && poseDistance > 0.5)
                {
                    std::cout << "4path_dis is larger than 1.2: " << path_dis << std::endl;
                    endCentrPoint.x = pixelpathPoint.x;
                    endCentrPoint.y = pixelpathPoint.y;
                }
                else if (laneDis <= 0.5 && laneDis > 0.1 && pixelpathPoint.x < 960 && poseDistance > 0.5)
                {
                    std::cout << "5laneDis is small than 0.5: " << laneDis << std::endl;
                    endCentrPoint.x = pixelpathPoint.x;
                    endCentrPoint.y = pixelpathPoint.y;
                }
                else if (path_dis < 1.5 && laneDis >= 1.2 && centerPoint[0].x > 960 && poseCenDistance > 0.8)
                {
                    std::cout << "6laneDis is larger than 1.2: " << laneDis << std::endl;
                    endCentrPoint.x = centerPoint[0].x;
                    endCentrPoint.y = centerPoint[0].y;
                }
                else if (path_dis < 1.5 && laneDis <= 0.5 && laneDis > 0.1 && centerPoint[0].x < 960 && poseCenDistance > 0.8)
                {
                    std::cout << "7laneDis is small than 0.5: " << laneDis << std::endl;
                    endCentrPoint.x = centerPoint[0].x;
                    endCentrPoint.y = centerPoint[0].y;
                }
                else if (path_dis > 1.0 && poseDistance > 0.5)
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
                else if (laneDis <= 1.0)
                {
                    if (pixelpathPoint.x < centerPoint[0].x && path_dis < 1.0)
                    {
                        endCentrPoint.x = pixelpathPoint.x;
                        endCentrPoint.y = pixelpathPoint.y;
                    }
                    else if (path_dis > 1.5 && pixelpathPoint.x < 960)
                    {
                        endCentrPoint.x = pixelpathPoint.x;
                        endCentrPoint.y = pixelpathPoint.y;
                    }
                    else
                    {
                        endCentrPoint.x = centerPoint[0].x;
                        endCentrPoint.y = centerPoint[0].y;
                    }
                }
                else if (laneDis > 1.0)
                {
                    if (pixelpathPoint.x > centerPoint[0].x && path_dis < 1.0)
                    {
                        endCentrPoint.x = pixelpathPoint.x;
                        endCentrPoint.y = pixelpathPoint.y;
                    }
                    else if (path_dis > 1.5 && pixelpathPoint.x > 960)
                    {
                        endCentrPoint.x = pixelpathPoint.x;
                        endCentrPoint.y = pixelpathPoint.y;
                    }
                    else
                    {
                        endCentrPoint.x = centerPoint[0].x;
                        endCentrPoint.y = centerPoint[0].y;
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
        std::cout << "carState: " << car_state.x << "," << car_state.y << "," << car_state.heading << std::endl;
        std::cout << "referPoint: " << referPoint << std::endl;
        std::cout << "endpathPoint: " << endpathPoint << std::endl;
        // std::cout << "imagepathPoint: " << imagepathPoint << std::endl;
        std::cout << "pixelpathPoint: " << pixelpathPoint << std::endl;
        std::cout << "endCentrPoint: " << endCentrPoint << std::endl;
        // convert endCentrPoint to pixle point
        Point2d endpixelPoint;
        endpixelPoint.x = 540 - endCentrPoint.y;
        endpixelPoint.y = 960 - endCentrPoint.x;
        // set start state
        start_state.x = -54 * resolution + 20 * resolution;
        start_state.y = 0.0 * resolution;
        // set end state
        end_state.x = endpixelPoint.x / 10 * resolution + 20 * resolution;
        end_state.y = endpixelPoint.y / 10 * resolution;
        double disx = (end_state.x - start_state.x) / 6;
        double disy = (end_state.y - start_state.y) / 6;
        double path_startTheta = 0;
        if (isTurn)
        {
            referenceTurnPoint(endIndex, index, pathPoints, thX, thY);
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
            start_state.heading = path_startTheta;
            end_state.heading = path_startTheta + pathPoints[endIndex][2] - pathPoints[index][2];
            std::cout << "turn start state: " << start_state.x << "," << start_state.y << "," << start_state.heading << std::endl;
            std::cout << "turn end state: " << end_state.x << "," << end_state.y << "," << end_state.heading << std::endl;
        }
        else
        {
            double the = atan((end_state.y - start_state.y) / (end_state.x - start_state.x));
            end_state.heading = path_startTheta + pathPoints[endIndex][2] - pathPoints[index][2];
            start_state.heading = path_startTheta; // pathPoints[index][2];
            std::cout << "start state: " << start_state.x << "," << start_state.y << "," << start_state.heading << std::endl;
            std::cout << "end state: " << end_state.x << "," << end_state.y << "," << end_state.heading << std::endl;

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
                    start_state.heading = the; // car_state.heading;pathPoints[index][2];
                }
            }**/
        }

        // Calculate.
        std::vector<PathOptimizationNS::SlState> result_path;
        std::vector<PathOptimizationNS::State> smoothed_reference_path;

        bool opt_ok = false;
        if (reference_rcv)
        {
            PathOptimizationNS::PathOptimizer path_optimizer(start_state, end_state, grid_map);
            // calcualte distance from obstacle
            for (int i = 0; i < reference_path_plot.size(); i++)
            {
                std::cout << reference_path_plot[i].x << "," << reference_path_plot[i].y << std::endl;
                double distance = path_optimizer.calDisObs(reference_path_plot[i].x, reference_path_plot[i].y);
                std::cout << "reference path point distance: " << distance << std::endl;
            }
            opt_ok = path_optimizer.solve(reference_path_plot, &result_path);
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
            if (opt_ok)
            {
                std::cout << "ok!" << std::endl;
            }
            // path_optimizer.printObstacleDistance2(smoothed_reference_path);
            // printf("-------------------------------------------------------");
            // path_optimizer.printObstacleDistance(result_path);
        }
        double angle = -car_state.heading;
        std::cout << "angle: " << angle << std::endl;
        cv::Mat img_src2(cv::Size(img_src.cols * 10, img_src.rows * 10), CV_8UC3, cv::Scalar(255, 255, 255));
        double disX, disY;
        for (size_t i = 0; i < result_path.size() - 1; i = i + 2)
        {

            const auto path_point = result_path.at(i);
            disX = (path_point.x + 3.4) * cos(angle) + path_point.y * sin(angle);
            disY = -(path_point.x + 3.4) * sin(angle) + path_point.y * cos(angle);

            std::cout << "result_path: " << car_state.x + disX << "," << car_state.y + disY << "," << path_point.heading - angle << std::endl;

            double pathX = (car_state.x + disX) * 10;
            double pathY = (car_state.y + disY) * 10 - 560;
            cv::Point p(-pathY + 1080, -pathX + 1920);
            // std::cout << "result point: " << pathX << "," << pathY << std::endl;
            circle(img_src2, p, 1, cv::Scalar(0, 0, 0), -1);
        }
        for (size_t i = 0; i != result_path.size(); ++i)
        {
            const auto path_point = result_path.at(i);
            double y = -path_point.x / resolution + 74;
            double x = -path_point.y / resolution + 96;
            cv::Point p(x, y);
            // std::cout << "result pixle: " << x << "," << y << std::endl;
            circle(img_src, p, 1, cv::Scalar(0, 0, 0), -1);
        }
        for (size_t i = 0; i != reference_path_plot.size(); ++i)
        {
            grid_map::Position pose(reference_path_plot[i].x, reference_path_plot[i].y);
            // std::cout << "reference path obstacle distance: " << map.getObstacleDistance(pose) << std::endl;
            double y = -reference_path_plot[i].x / resolution + 74;
            double x = -reference_path_plot[i].y / resolution + 96;
            cv::Point p(x, y);
            // std::cout << "reference_path_plot: " << x << "," << y << std::endl;
            circle(img_src, p, 1, cv::Scalar(0, 0, 0), -1);
        }
        isFirst = false;
        cv::imwrite("img.jpg", img_src);
        cv::imshow("image", img_src);
        cv::waitKey(0);
    }
}