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
#include "time.h"

using namespace cv;
using namespace std;

// 灰度化图像
Mat gray_Img(Mat src)
{
    Mat dst = Mat::zeros(src.size(), src.type());
    cvtColor(src, dst, CV_BGR2GRAY);
    return dst;
}

// 自适应阈值(二值化图像)
Mat threshold_Img(Mat src)
{
    Mat dst = Mat::zeros(src.size(), src.type());
    // 参数：输入，       输出，       二值图像的最大值 ，      在一个邻域内计算阈值所采用的算法，有两个取值分别为 ADAPTIVE_THRESH_MEAN_C 和 ADAPTIVE_THRESH_GAUSSIAN_C ，     阈值类型只有两个取值，分别为 THRESH_BINARY 和THRESH_BINARY_INV，（blockSize）adaptiveThreshold的计算单位是像素的邻域块，邻域块取多大就由这个值作决定，    偏移值调整量
    adaptiveThreshold(~src, dst, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15, -2);
    return dst;
}

// 结构元素(获取垂直算子)
Mat get_Vertical(Mat src)
{
    Mat dst = Mat::zeros(src.size(), src.type());
    return getStructuringElement(MORPH_RECT, Size(src.cols / 32, 1), Point(-1, -1));
}

// 结构元素（获取水平算子）
Mat get_Horizontal(Mat src)
{
    Mat dst = Mat::zeros(src.size(), src.type());
    return getStructuringElement(MORPH_RECT, Size(1, src.rows / 32), Point(-1, -1));
}

// 腐蚀
Mat erode_Img(Mat src, Mat kernel)
{
    Mat dst = Mat::zeros(src.size(), src.type());
    erode(src, dst, kernel);
    return dst;
}

// 膨胀
Mat dilate_Img(Mat src, Mat kernel)
{
    Mat dst = Mat::zeros(src.size(), src.type());
    dilate(src, dst, kernel);
    return dst;
}
static bool isLineIntersectRectangle(float linePointX1,
                                     float linePointY1,
                                     float linePointX2,
                                     float linePointY2,
                                     float rectangleLeftTopX,
                                     float rectangleLeftTopY,
                                     float rectangleRightBottomX,
                                     float rectangleRightBottomY)
{
    float lineHeight = linePointY1 - linePointY2;
    float lineWidth = linePointX2 - linePointX1; // 计算叉乘
    float c = linePointX1 * linePointY2 - linePointX2 * linePointY1;
    if ((lineHeight * rectangleLeftTopX + lineWidth * rectangleLeftTopY + c >= 0 && lineHeight * rectangleRightBottomX + lineWidth * rectangleRightBottomY + c <= 0) || (lineHeight * rectangleLeftTopX + lineWidth * rectangleLeftTopY + c <= 0 && lineHeight * rectangleRightBottomX + lineWidth * rectangleRightBottomY + c >= 0) || (lineHeight * rectangleLeftTopX + lineWidth * rectangleRightBottomY + c >= 0 && lineHeight * rectangleRightBottomX + lineWidth * rectangleLeftTopY + c <= 0) || (lineHeight * rectangleLeftTopX + lineWidth * rectangleRightBottomY + c <= 0 && lineHeight * rectangleRightBottomX + lineWidth * rectangleLeftTopY + c >= 0))
    {

        if (rectangleLeftTopX > rectangleRightBottomX)
        {
            float temp = rectangleLeftTopX;
            rectangleLeftTopX = rectangleRightBottomX;
            rectangleRightBottomX = temp;
        }
        if (rectangleLeftTopY < rectangleRightBottomY)
        {
            float temp1 = rectangleLeftTopY;
            rectangleLeftTopY = rectangleRightBottomY;
            rectangleRightBottomY = temp1;
        }
        if ((linePointX1 < rectangleLeftTopX && linePointX2 < rectangleLeftTopX) || (linePointX1 > rectangleRightBottomX && linePointX2 > rectangleRightBottomX) || (linePointY1 > rectangleLeftTopY && linePointY2 > rectangleLeftTopY) || (linePointY1 < rectangleRightBottomY && linePointY2 < rectangleRightBottomY))
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

int main(int argc, char **argv)
{

    std::cout<<isLineIntersectRectangle(231,0,227,376,211,122,226,80)<<std::endl;
    string line;
    stringstream ss;
    ifstream ifs("/home/gq/guoqian/AVP_SLAM/RoadLib-Noted/test/stamp.txt");

    double sow;
    string imagename;
    while (ifs.good())
    {
        getline(ifs, line);
        if (line.size() < 1)
            continue;
        if (line[0] == '#')
            continue;
        for (int ii = 0; ii < line.size(); ii++)
        {
            if (line[ii] == ',')
                line[ii] = ' ';
        }
        ss.clear();
        ss.str("");
        ss << line;
        ss >> sow >> imagename;
        // std::cout<<imagename<<std::endl;;

        // read original path
        // Mat image = cv::imread("/home/gq/guoqian/data/2ddata/mask/" + imagename);
        Mat image = cv::imread("/home/gq/guoqian/data/test/155647_1.jpg");
        // 2. binaryImage to get Areas of exercise

        clock_t time1 = clock();
        cv::resize(image, image, Size(image.rows / 2, image.cols / 2));
        cv::Mat gray, binaryImage;
        cv::cvtColor(image, gray, COLOR_BGR2GRAY);
        threshold(gray, binaryImage, 120, 255, 0);

        Mat verticalLine = getStructuringElement(MORPH_RECT, Size(binaryImage.cols / 32, 1), Point(-1, -1));
        Mat horizontalLine = getStructuringElement(MORPH_RECT, Size(1, binaryImage.rows / 32), Point(-1, -1));
        // 先腐蚀再膨胀
        Mat vertical_Line_erode = erode_Img(binaryImage, verticalLine);
        Mat vertical_Line_dilate = dilate_Img(vertical_Line_erode, verticalLine);

        Mat horizontal_Line_erode = erode_Img(binaryImage, horizontalLine);
        Mat horizontal_Line_dilate = dilate_Img(horizontal_Line_erode, horizontalLine);

        vector<vector<Point>> contours_v;
        vector<Vec4i> hierarchy_v;

        findContours(vertical_Line_dilate, contours_v, hierarchy_v, RETR_LIST, CHAIN_APPROX_NONE);

        vector<vector<Point>> contours_h;
        vector<Vec4i> hierarchy_h;

        findContours(horizontal_Line_dilate, contours_h, hierarchy_h, RETR_LIST, CHAIN_APPROX_NONE);

        vector<vector<Point>> contours_guide;
        for (int i = 0; i < contours_h.size(); i++)
        {
            Moments M;
            M = moments(contours_h[i]);
            double x = double(M.m10 / M.m00);
            double y = double(M.m01 / M.m00);
            if (abs(x - 160) < 25 && cv::contourArea(contours_h[i]) < 500 && cv::contourArea(contours_h[i]) > 100)
            {
                std::cout << x << "," << y << std::endl;
                contours_guide.push_back(contours_h[i]);
            }
        }
        for (int i = 0; i < contours_h.size(); i++)
        {
            Moments M;
            M = moments(contours_h[i]);
            double x = double(M.m10 / M.m00);
            // std::cout << x << std::endl;
            if (x > 270 || x < 50)
            {
                contours_h.erase(contours_h.begin() + i);
            }
        }
        contours_h.erase(remove_if(contours_h.begin(), contours_h.end(),
                                   [](const std::vector<cv::Point> &c)
                                   { return cv::contourArea(c) < 600; }),
                         contours_h.end());

        gray.setTo(0);
        Mat gray1 = Mat::zeros(Size(gray.cols, gray.rows), CV_8UC1);
        Mat gray2 = Mat::zeros(Size(gray.cols, gray.rows), CV_8UC1);
        Mat gray3 = Mat::zeros(Size(gray.cols, gray.rows), CV_8UC1);
        // cv::line(gray3, Point(226 / 2 * 0.91, 258 / 2 * 0.91), Point(26 / 2 * 0.91, 245 / 2 * 0.91), Scalar(255, 255, 255), 2, cv::LINE_AA);
        // cv::line(gray3, Point(222 / 2 * 0.91, 386 / 2 * 0.91), Point(22 / 2 * 0.91, 373 / 2 * 0.91), Scalar(255, 255, 255), 2, cv::LINE_AA);

        cv::drawContours(gray, contours_h, -1, cv::Scalar(255), cv::FILLED);
        // Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
        // erode(gray, gray, element);
        cv::drawContours(gray1, contours_h, -1, cv::Scalar(255), cv::FILLED);
        cv::drawContours(gray, contours_v, -1, cv::Scalar(255), cv::FILLED);
        cv::drawContours(gray2, contours_v, -1, cv::Scalar(255), cv::FILLED);

        cv::drawContours(gray, contours_guide, -1, cv::Scalar(125, 125, 125), cv::FILLED);
        // cv::line(gray, Point(226 / 2 * 0.91-10, 258 / 2 * 0.91), Point(26 / 2 * 0.91, 245 / 2 * 0.91), Scalar(255, 255, 255), 2, cv::LINE_AA);
        // cv::line(gray, Point(222 / 2 * 0.91-10, 386 / 2 * 0.91), Point(22 / 2 * 0.91, 373 / 2 * 0.91), Scalar(255, 255, 255), 2, cv::LINE_AA);
        Mat image2 = cv::imread("/home/gq/guoqian/data/test/98384_1.jpg");
        cv::rectangle(image2, Point((631 - 288) * 0.91, (128 - 128) * 0.91), Point((653 - 288) * 0.91, (275 - 128) * 0.91), Scalar(125, 125, 125), 2);
        // cv::rectangle(image2, Point((620-288) * 0.91, (733-128) *0.91), Point((647-288) * 0.91, (826-128) *0.91), Scalar(0, 255, 255), 2);
        // cv::rectangle(image2, Point((621-288) * 0.91, (740-128) *0.91), Point((648-288) * 0.91, (830-128) *0.91), Scalar(255, 0, 255), 2);
        // cv::line(image2, Point(500 * 0.91, 195 * 0.91), Point(659 * 0.91, 192 * 0.91), Scalar(255, 255, 255), 2, cv::LINE_AA);
        // cv::circle(image2,Point(175*2,302*2),1,cv::Scalar(255,0,0));
        cv::imshow("image", image2);
        cv::waitKey(0);

        for (int i = 0; i < gray.rows; i++)
        {
            for (int j = 0; j < gray.cols; j++)
            {
                if (255 == gray1.at<uchar>(i, j) && 255 == gray2.at<uchar>(i, j))
                {
                    gray.at<uchar>(i, j) = 0;
                }
            }
        }
        cv::resize(gray, gray, Size(image.rows, image.cols));
        clock_t time2 = clock();
        double mapping_time = double(time2 - time1) / 1000;
        cout << "close time:" << mapping_time << "ms" << std::endl;
        cv::imshow("gray", gray);
        cv::waitKey(0);
        cv::imwrite("/home/gq/guoqian/AVP_SLAM/RoadLib-Noted/test/mask2/" + imagename, gray);
    }
}