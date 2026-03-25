#include "j3_perception_data_api.h"

// #include "tinyxml2.h"
#include <cstring>

#include <string>
#include <array>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

namespace draw_map
{

#define J3P_PI 3.14159265358979323846

#if 0
    /*
    * @brief   (整型-底边)计算Rect旋转后的坐标,用于绘制有角度的障碍物
    * @param   old_rect     [in]     原始Rect
    * @param   yaw_rad      [in]     旋转角度，弧度制
    * @param   new_rect     [out]    旋转后的点坐标
    */
    void rotate_rect(const cv::Rect& old_rect, float yaw_rad, std::vector<cv::Point>& new_rect) {
        // 计算旋转中心（底边中点）
        cv::Point2f center(old_rect.x + old_rect.width / 2.0f, old_rect.y + old_rect.height);

        // 计算旋转矩阵
        cv::Mat rot_mat = cv::getRotationMatrix2D(center, yaw_rad * 180.0f / CV_PI, 1.0);

        // 原始矩形的四个角点
        std::vector<cv::Point2f> rect_pts = {
            cv::Point2f(old_rect.x, old_rect.y),
            cv::Point2f(old_rect.x + old_rect.width, old_rect.y),
            cv::Point2f(old_rect.x + old_rect.width, old_rect.y + old_rect.height),
            cv::Point2f(old_rect.x, old_rect.y + old_rect.height)
        };

        // 计算旋转后的点
        std::vector<cv::Point2f> rotated_pts(4);
        cv::transform(rect_pts, rotated_pts, rot_mat);

        // 转换为整数点
        new_rect.assign(rotated_pts.begin(), rotated_pts.end());
    }

    /*
    * @brief   (整型-中心)计算Rect旋转后的坐标,用于绘制有角度的障碍物
    * @param   old_rect     [in]     原始Rect
    * @param   yaw_rad      [in]     旋转角度，弧度制
    * @param   new_rect     [out]    旋转后的点坐标
    */
    void rotate_rect_center(const cv::Rect& old_rect, float yaw_rad, std::vector<cv::Point>& new_rect) {
        // 计算旋转中心（矩形中点）
        cv::Point2f center(old_rect.x + old_rect.width / 2.0f, old_rect.y + old_rect.height / 2);

        // 计算旋转矩阵
        cv::Mat rot_mat = cv::getRotationMatrix2D(center, yaw_rad * 180.0f / CV_PI, 1.0);

        // 原始矩形的四个角点
        std::vector<cv::Point2f> rect_pts = {
            cv::Point2f(old_rect.x, old_rect.y),
            cv::Point2f(old_rect.x + old_rect.width, old_rect.y),
            cv::Point2f(old_rect.x + old_rect.width, old_rect.y + old_rect.height),
            cv::Point2f(old_rect.x, old_rect.y + old_rect.height)
        };

        // 计算旋转后的点
        std::vector<cv::Point2f> rotated_pts(4);
        cv::transform(rect_pts, rotated_pts, rot_mat);

        // 转换为整数点
        new_rect.assign(rotated_pts.begin(), rotated_pts.end());
    }

    /*
    * @brief   计算Rect旋转后的坐标,用于绘制有角度的障碍物
    * @param   old_rect     [in]     原始Rect
    * @param   yaw_rad      [in]     旋转角度，弧度制
    * @param   new_rect     [out]    旋转后的点坐标
    */
    void rotate_rect_f(const cv::Rect2f& old_rect, float yaw_rad, std::vector<cv::Point2f>& new_rect) {
        // 计算旋转中心（底边中点）
        cv::Point2f center(old_rect.x + old_rect.width / 2.0f, old_rect.y + old_rect.height);

        // 计算旋转矩阵
        cv::Mat rot_mat = cv::getRotationMatrix2D(center, yaw_rad * 180.0f / CV_PI, 1.0);

        // 原始矩形的四个角点
        std::vector<cv::Point2f> rect_pts = {
            cv::Point2f(old_rect.x, old_rect.y),
            cv::Point2f(old_rect.x + old_rect.width, old_rect.y),
            cv::Point2f(old_rect.x + old_rect.width, old_rect.y + old_rect.height),
            cv::Point2f(old_rect.x, old_rect.y + old_rect.height)
        };

        // 计算旋转后的点
        std::vector<cv::Point2f> rotated_pts(4);
        cv::transform(rect_pts, rotated_pts, rot_mat);

        // 转换为整数点
        new_rect.assign(rotated_pts.begin(), rotated_pts.end());
    }
#endif // #if 0

    /*
     * @brief   以指定中心点计算Rect旋转后的坐标,用于绘制有角度的障碍物
     * @param   old_rect     [in]     原始Rect
     * @param   center_point [in]     指定的旋转中心点
     * @param   yaw_rad      [in]     旋转角度，弧度制
     * @param   new_rect     [out]    旋转后的点坐标
     */
    void rotate_rect_with_center(const cv::Rect &old_rect, const cv::Point &center_point, float yaw_rad, std::vector<cv::Point> &new_rect)
    {
        // 计算旋转矩阵
        cv::Mat rot_mat = cv::getRotationMatrix2D(center_point, yaw_rad * 180.0f / CV_PI, 1.0);

        // 原始矩形的四个角点
        std::vector<cv::Point2f> rect_pts = {
            cv::Point2f(old_rect.x, old_rect.y),
            cv::Point2f(old_rect.x + old_rect.width, old_rect.y),
            cv::Point2f(old_rect.x + old_rect.width, old_rect.y + old_rect.height),
            cv::Point2f(old_rect.x, old_rect.y + old_rect.height)};

        // 计算旋转后的点
        std::vector<cv::Point2f> rotated_pts(4);
        cv::transform(rect_pts, rotated_pts, rot_mat);

        // 转换为整数点
        new_rect.assign(rotated_pts.begin(), rotated_pts.end());
    }

    const J3Line *get_front_data_all_line_ptr_apicpp(const J3PerceptionData &front_data_, int index)
    {
        switch (index)
        {
        case 0:
            return &(front_data_.J3P_LKA_Left_Lane);
        case 1:
            return &(front_data_.J3P_LKA_Right_Lane);
        case 2:
            return &(front_data_.J3P_Next_Left_Lane);
        case 3:
            return &(front_data_.J3P_Next_Right_Lane);
        case 4:
            return &(front_data_.J3P_Left_Outer_Lane);
        case 5:
            return &(front_data_.J3P_Right_Outer_Lane);
        case 6:
            return &(front_data_.J3P_Left_Cone_Road_Edge);
        case 7:
            return &(front_data_.J3P_Right_Cone_Road_Edge);
        case 8:
            return &(front_data_.J3P_Left_Road_Edge);
        case 9:
            return &(front_data_.J3P_Right_Road_Edge);
        default:
            return nullptr;
        }
    }

    static cv::Rect2f calculate_obs_rect(const cv::Point &obs_img_pose,
                                         float width, float height,
                                         float resolution_m,
                                         unsigned short pose_type)
    {
        cv::Rect2f obs_rect;
        switch (pose_type)
        {
        case 0: // Unknown - 使用中心点
            obs_rect = cv::Rect2f(
                obs_img_pose.x - width / 2,
                obs_img_pose.y - height / 2,
                width,
                height);
            break;

        case 1: // RearLeft - 后左点
            obs_rect = cv::Rect2f(
                obs_img_pose.x,
                obs_img_pose.y - height,
                width,
                height);
            break;

        case 2: // RearCenter - 后中点
            obs_rect = cv::Rect2f(
                obs_img_pose.x - width / 2,
                obs_img_pose.y - height,
                width,
                height);
            break;
        case 3: // RearRight - 后右点
            obs_rect = cv::Rect2f(
                obs_img_pose.x - width,
                obs_img_pose.y - height,
                width,
                height);
            break;

        case 4: // FrontLeft - 前左点
            obs_rect = cv::Rect2f(
                obs_img_pose.x,
                obs_img_pose.y,
                width,
                height);
            break;

        case 5: // FrontCenter - 前中点
            obs_rect = cv::Rect2f(
                obs_img_pose.x - width / 2,
                obs_img_pose.y,
                width,
                height);
            break;

        case 6: // FrontRight - 前右点
            obs_rect = cv::Rect2f(
                obs_img_pose.x - width,
                obs_img_pose.y,
                width,
                height);
            break;

        case 7: // LeftCenter - 左中点
            obs_rect = cv::Rect2f(
                obs_img_pose.x,
                obs_img_pose.y - height / 2,
                width,
                height);
            break;

        case 8: // RightCenter - 右中点
            obs_rect = cv::Rect2f(
                obs_img_pose.x - width,
                obs_img_pose.y - height / 2,
                width,
                height);
            break;

        case 9: // Center - 中心点
        default:
            obs_rect = cv::Rect2f(
                obs_img_pose.x - width / 2,
                obs_img_pose.y - height / 2,
                width,
                height);
            break;
        }
        return obs_rect;
    }

    cv::Point world2img_param(cv::Point2d world_point, cv::Point origin_point, float resolution_m)
    {
        return cv::Point(std::round(origin_point.x - world_point.y / resolution_m), std::round(origin_point.y - world_point.x / resolution_m));
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

    int draw_front_perception_bgr(const J3PerceptionData &front_data_, cv::Mat &fornt_perception_img, unsigned long &time_stamp, float map_resolution_m, float map_width_m, float map_height_m)
    {
        if (front_data_.data_valid != 1)
        {

            return -1;
        }

        float resolution_m = 0.1;
        float width = 60.0;
        float height = 110.0;

        // step1 地图参数合法性判断,以及设置地图参数
        if (map_resolution_m < 0.01)
        {
            resolution_m = 0.1;
        }
        else
        {
            resolution_m = map_resolution_m;
        }
        float resolution_cm = resolution_m * 100;

        if (map_width_m < 20)
        {
            width = 60.0;
        }
        else
        {
            width = map_width_m;
        }

        if (map_height_m < 30)
        {
            map_height_m = 110.0;
        }
        else
        {
            height = map_height_m;
        }

        // step2 生成地图Mat
        int map_img_width = static_cast<int>(std::round(width / resolution_m));
        int map_img_height = static_cast<int>(std::round(height / resolution_m));
        cv::Point img_origin(map_img_width / 2, map_img_height);                  // 地图原点即自车后轴中心，始终定义在地图的最下方中间
        cv::Mat map_img = cv::Mat::zeros(map_img_height, map_img_width, CV_8UC3); // TODO:要改成单通道

        // step3 绘制地图

        // std::lock_guard<std::mutex> lock(data_mutex_);
        // step3.1 绘制freespace，确定区域
        do
        {
            std::vector<cv::Point> freespace_area_contours;
            std::vector<std::vector<cv::Point>> draw_fs_polygons;
            freespace_area_contours.emplace_back(world2img_param(cv::Point2d(0, 2), img_origin, resolution_m)); // 起点为自车后轴左侧2米
            for (int i = 0; i < 64; i++)
            {
                freespace_area_contours.emplace_back(world2img_param(cv::Point2d(front_data_.J3P_Freespace.contours_points[i].x, front_data_.J3P_Freespace.contours_points[i].y), img_origin, resolution_m));
            }
            freespace_area_contours.emplace_back(world2img_param(cv::Point2d(0, -2), img_origin, resolution_m)); // 终点点为自车后轴右侧2米
            freespace_area_contours.emplace_back(world2img_param(cv::Point2d(0, 2), img_origin, resolution_m));  // 保证轮廓闭合
            draw_fs_polygons.emplace_back(freespace_area_contours);
            cv::fillPoly(map_img, draw_fs_polygons, cv::Scalar(255, 255, 255));

            for (int i = 0; i < freespace_area_contours.size(); i++)
            {
                cv::circle(map_img, freespace_area_contours[i], 1, cv::Scalar(0, 0, 255), -1);
            }
            //
        } while (0);

        do
        {
            // step3.2 绘制障碍物，
            for (int i = 0; i < front_data_.J3P_obs_num; i++)
            {
                if (front_data_.J3P_obs[i].obs_x >= 0 && front_data_.J3P_obs[i].obs_x < map_height_m)
                { // x超出地图边界障碍物，此处不绘制
                    if (front_data_.J3P_obs[i].obs_class == 0)
                    { // 0是无效
                        char do_somesthing = 0;
                    }
                    else if (front_data_.J3P_obs[i].obs_class == 2)
                    { // 2是行人，这里画一个半径3个像素的黑色圈

                        cv::Point2d pede_world_pose(front_data_.J3P_obs[i].obs_x, front_data_.J3P_obs[i].obs_y);
                        cv::Point pede_img_pose = world2img_param(pede_world_pose, img_origin, resolution_m);
                        cv::circle(map_img, pede_img_pose, 3, cv::Scalar(0, 0, 255), -1);
                    }
                    else
                    { // 剩下的障碍物认为有宽高的物体，例如车辆

                        // 根据角度区分可能的障碍物
                        cv::Point2d obs_world_pose(front_data_.J3P_obs[i].obs_x, front_data_.J3P_obs[i].obs_y);
                        cv::Point obs_img_pose = world2img_param(obs_world_pose, img_origin, resolution_m);

                        cv::Rect2f obs_rect;
                        std::vector<cv::Point> rotate_rect_point;

                        obs_rect = calculate_obs_rect(
                            obs_img_pose,
                            front_data_.J3P_obs[i].obs_width / resolution_m,
                            front_data_.J3P_obs[i].obs_length / resolution_m,
                            resolution_m,
                            front_data_.J3P_obs[i].obs_pose_type);
                        rotate_rect_with_center(obs_rect, obs_img_pose, front_data_.J3P_obs[i].obs_angle * J3P_PI / 180.0, rotate_rect_point);

                        std::vector<std::vector<cv::Point>> draw_obs_polygons;
                        draw_obs_polygons.emplace_back(rotate_rect_point);
                        // cv::fillPoly(map_img, draw_obs_polygons, cv::Scalar(0, 0, 255));
                        cv::polylines(map_img, draw_obs_polygons, true, cv::Scalar(0, 0, 255), 2);
                        cv::circle(map_img, obs_img_pose, 3, cv::Scalar(0, 255, 0), 1);
                    }
                }
            }

            for (int i = front_data_.J3P_obs_num; i < 20; i++)
            { // 测试发现，部分行人的障碍物数据会在有效个数以外，通过判断坐标来补充
                if (/*front_data_.J3P_obs[i].obs_x != 0 &&*/ front_data_.J3P_obs[i].obs_class != 0 && front_data_.J3P_obs[i].obs_height > 0)
                {
                    if (front_data_.J3P_obs[i].obs_x >= 0 && front_data_.J3P_obs[i].obs_x < map_height_m)
                    { // x超出地图边界障碍物，此处不绘制
                        if (front_data_.J3P_obs[i].obs_class == 0)
                        { // 0是无效
                            char do_somesthing = 0;
                        }
                        else if (front_data_.J3P_obs[i].obs_class == 2)
                        { // 2是行人，这里画一个半径3个像素的黑色圈
                            cv::Point2d pede_world_pose(front_data_.J3P_obs[i].obs_x, front_data_.J3P_obs[i].obs_y);
                            cv::Point pede_img_pose = world2img_param(pede_world_pose, img_origin, resolution_m);
                            cv::circle(map_img, pede_img_pose, 3, cv::Scalar(0, 0, 255), -1);
                        }
                        else
                        { // 剩下的障碍物认为有宽高的物体，例如车辆
                            // 根据角度区分可能的障碍物
                            cv::Point2d obs_world_pose(front_data_.J3P_obs[i].obs_x, front_data_.J3P_obs[i].obs_y);
                            cv::Point obs_img_pose = world2img_param(obs_world_pose, img_origin, resolution_m);

                            cv::Rect2f obs_rect;
                            std::vector<cv::Point> rotate_rect_point;

                            obs_rect = calculate_obs_rect(
                                obs_img_pose,
                                front_data_.J3P_obs[i].obs_width / resolution_m,
                                front_data_.J3P_obs[i].obs_length / resolution_m,
                                resolution_m,
                                front_data_.J3P_obs[i].obs_pose_type);
                            rotate_rect_with_center(obs_rect, obs_img_pose, front_data_.J3P_obs[i].obs_angle * J3P_PI / 180.0, rotate_rect_point);

                            std::vector<std::vector<cv::Point>> draw_obs_polygons;
                            draw_obs_polygons.emplace_back(rotate_rect_point);
                            // cv::fillPoly(map_img, draw_obs_polygons, cv::Scalar(0, 0, 255));
                            cv::polylines(map_img, draw_obs_polygons, true, cv::Scalar(0, 0, 255), 2);
                            cv::circle(map_img, obs_img_pose, 3, cv::Scalar(0, 255, 0), 1);
                        }
                    }
                }
            }
        } while (0);

        // step3.2 绘制车道线
        do
        {

            float lane_gap = 0.1;
            cv::Scalar lane_color(242, 167, 34); // TODO:当前先用3通道图像的颜色
            for (int i = 0; i < 10; i++)
            {
                const J3Line *Lane_line = get_front_data_all_line_ptr_apicpp(front_data_, i);
                if (Lane_line != nullptr)
                {
                    if (Lane_line->is_valid == 1)
                    {
                        float lane_c0 = Lane_line->line_c0;
                        float lane_c1 = Lane_line->line_c1;
                        float lane_c2 = Lane_line->line_c2;
                        float lane_c3 = Lane_line->line_c3;

                        float lane_strat = Lane_line->line_start;

                        if (i < 6)
                        {
                            // lane_color = cv::Scalar(242, 167, 34);
                            lane_color = cv::Scalar(0, 255, 0);
                        }
                        else if (i < 8)
                        {
                            // lane_color = cv::Scalar(203, 65, 185);
                            lane_color = cv::Scalar(0, 0, 255);
                        }
                        else
                        {
                            lane_color = cv::Scalar(255, 0, 0);
                        }

                        cv::Point2d lane_p0(lane_strat, lane_c0 + lane_c1 * lane_strat + lane_c2 * lane_strat * lane_strat + lane_c3 * lane_strat * lane_strat * lane_strat);
                        cv::Point lane_img_p0 = world2img_param(lane_p0, img_origin, resolution_m);
                        for (double x = lane_strat + lane_gap; x < Lane_line->line_end; x += lane_gap)
                        {
                            double y = lane_c0 + lane_c1 * x + lane_c2 * x * x + lane_c3 * x * x * x;
                            cv::Point2d lane_p1(x, y);
                            cv::Point lane_img_p1 = world2img_param(lane_p1, img_origin, resolution_m);
                            cv::line(map_img, lane_img_p0, lane_img_p1, lane_color, 2);
                            lane_img_p0 = lane_img_p1;
                        }
                    }
                }
                else
                {
                    std::cout << "@j3p: i = " << i << ", Lane_line == nullptr" << std::endl;
                }
            }
        } while (0);

        fornt_perception_img = map_img;
        time_stamp = front_data_.vm_muc_timeStamp;
        return 0;
    }

    int draw_front_perception_gray(const J3PerceptionData &front_data_, cv::Mat &fornt_perception_img, unsigned long &time_stamp, std::vector<J3Obs> &j3Obs, bool opt_ok, lidarInfo lidarinfo, float map_resolution_m, float map_width_m, float map_height_m)
    {
        if (front_data_.data_valid != 1)
        {

            return -1;
        }

        float resolution_m = 0.1;
        float width = 60.0;
        float height = 110.0;

        // step1 地图参数合法性判断,以及设置地图参数
        if (map_resolution_m < 0.01)
        {
            resolution_m = 0.1;
        }
        else
        {
            resolution_m = map_resolution_m;
        }
        float resolution_cm = resolution_m * 100;

        if (map_width_m < 20)
        {
            width = 60.0;
        }
        else
        {
            width = map_width_m;
        }

        if (map_height_m < 30)
        {
            map_height_m = 110.0;
        }
        else
        {
            height = map_height_m;
        }

        // step2 生成地图Mat
        int map_img_width = static_cast<int>(std::round(width / resolution_m));
        int map_img_height = static_cast<int>(std::round(height / resolution_m));
        cv::Point img_origin(map_img_width / 2, map_img_height);                  // 地图原点即自车后轴中心，始终定义在地图的最下方中间
        cv::Mat map_img = cv::Mat::zeros(map_img_height, map_img_width, CV_8UC1); // TODO:要改成单通道
        map_img.setTo(100);
        // std::cout << map_img_width << "," << map_img_height << std::endl;
        // step3 绘制地图

        // std::lock_guard<std::mutex> lock(data_mutex_);
        // step3.1 绘制freespace，确定区域
        do
        {
            std::vector<cv::Point> freespace_area_contours;
            std::vector<std::vector<cv::Point>> draw_fs_polygons;
            if (!opt_ok)
            {

                freespace_area_contours.emplace_back(world2img_param(cv::Point2d(0, 40), img_origin, resolution_m));
                freespace_area_contours.emplace_back(world2img_param(cv::Point2d(100, 40), img_origin, resolution_m));
                freespace_area_contours.emplace_back(world2img_param(cv::Point2d(100, -40), img_origin, resolution_m));
                freespace_area_contours.emplace_back(world2img_param(cv::Point2d(0, -40), img_origin, resolution_m));
                for (int i = 4; i < 64; i++)
                {
                    freespace_area_contours.emplace_back(world2img_param(cv::Point2d(0, -40), img_origin, resolution_m));
                }
                draw_fs_polygons.emplace_back(freespace_area_contours);
                cv::fillPoly(map_img, draw_fs_polygons, cv::Scalar(255)); // 白色可行驶区域
                fornt_perception_img = map_img;
                time_stamp = front_data_.vm_muc_timeStamp;
                return 0;
            }
            freespace_area_contours.emplace_back(world2img_param(cv::Point2d(0, 2), img_origin, resolution_m)); // 起点为自车后轴左侧2米
            for (int i = 10; i < 60; i++)
            {
                freespace_area_contours.emplace_back(world2img_param(cv::Point2d(front_data_.J3P_Freespace.contours_points[i].x, front_data_.J3P_Freespace.contours_points[i].y), img_origin, resolution_m));
            }
            freespace_area_contours.emplace_back(world2img_param(cv::Point2d(0, -2), img_origin, resolution_m)); // 终点点为自车后轴右侧2米
            freespace_area_contours.emplace_back(world2img_param(cv::Point2d(0, 2), img_origin, resolution_m));  // 保证轮廓闭合
            draw_fs_polygons.emplace_back(freespace_area_contours);
            cv::fillPoly(map_img, draw_fs_polygons, cv::Scalar(255)); // 白色可行驶区域
        } while (0);

        std::vector<cv::Rect2f> obs_rects;
        do
        {
            // step3.2 绘制障碍物，
            for (int i = 0; i < front_data_.J3P_obs_num; i++)
            {
                if (front_data_.J3P_obs[i].obs_x >= 0 && front_data_.J3P_obs[i].obs_x < map_height_m)
                { // x超出地图边界障碍物，此处不绘制
                    if (front_data_.J3P_obs[i].obs_class == 0)
                    { // 0是无效
                        char do_somesthing = 0;
                    }
                    else if (front_data_.J3P_obs[i].obs_class == 2)
                    { // 2是行人，这里画一个半径3个像素的黑色圈

                        cv::Point2d pede_world_pose(front_data_.J3P_obs[i].obs_x, front_data_.J3P_obs[i].obs_y);
                        cv::Point pede_img_pose = world2img_param(pede_world_pose, img_origin, resolution_m);
                        cv::circle(map_img, pede_img_pose, 5, cv::Scalar(0), -1); // 黑色行人点
                        j3Obs.push_back(front_data_.J3P_obs[i]);
                    }
                    else
                    { // 剩下的障碍物认为有宽高的物体，例如车辆

                        // 根据角度区分可能的障碍物
                        cv::Point2d obs_world_pose(front_data_.J3P_obs[i].obs_x, front_data_.J3P_obs[i].obs_y);
                        cv::Point obs_img_pose = world2img_param(obs_world_pose, img_origin, resolution_m);

                        cv::Rect2f obs_rect;
                        std::vector<cv::Point> rotate_rect_point;

                        obs_rect = calculate_obs_rect(
                            obs_img_pose,
                            front_data_.J3P_obs[i].obs_width / resolution_m,
                            front_data_.J3P_obs[i].obs_length / resolution_m,
                            resolution_m,
                            front_data_.J3P_obs[i].obs_pose_type);
                        rotate_rect_with_center(obs_rect, obs_img_pose, front_data_.J3P_obs[i].obs_angle * J3P_PI / 180.0, rotate_rect_point);

                        std::vector<std::vector<cv::Point>> draw_obs_polygons;
                        draw_obs_polygons.emplace_back(rotate_rect_point);
                        // cv::fillPoly(map_img, draw_obs_polygons, cv::Scalar(0));//黑色障碍物
                        cv::polylines(map_img, draw_obs_polygons, true, cv::Scalar(0), 2); // 黑色障碍物框
                        cv::circle(map_img, obs_img_pose, 3, cv::Scalar(125), 1);
                        J3Obs obs;
                        obs = front_data_.J3P_obs[i];
                        obs.obs_x = (500 - obs_img_pose.y) * resolution_m;
                        obs.obs_y = (250 - obs_img_pose.x) * resolution_m;
                        // std::cout << "j3 origin Obs: " << front_data_.J3P_obs[i].obs_x << "," << front_data_.J3P_obs[i].obs_y << "," << front_data_.J3P_obs[i].obs_width << "," << front_data_.J3P_obs[i].obs_length << std::endl;
                        j3Obs.push_back(obs);
                    }
                }
            }

            for (int i = front_data_.J3P_obs_num; i < 20; i++)
            { // 测试发现，部分行人的障碍物数据会在有效个数以外，通过判断坐标来补充
                if (/*front_data_.J3P_obs[i].obs_x != 0 &&*/ front_data_.J3P_obs[i].obs_class != 0 && front_data_.J3P_obs[i].obs_height > 0)
                {
                    if (front_data_.J3P_obs[i].obs_x >= 1 && abs(front_data_.J3P_obs[i].obs_y) >= 0.5 && front_data_.J3P_obs[i].obs_x < map_height_m)
                    { // x超出地图边界障碍物，此处不绘制
                        if (front_data_.J3P_obs[i].obs_class == 0)
                        { // 0是无效
                            char do_somesthing = 0;
                        }
                        else if (front_data_.J3P_obs[i].obs_class == 2)
                        { // 2是行人，这里画一个半径3个像素的黑色圈
                            cv::Point2d pede_world_pose(front_data_.J3P_obs[i].obs_x, front_data_.J3P_obs[i].obs_y);
                            cv::Point pede_img_pose = world2img_param(pede_world_pose, img_origin, resolution_m);
                            cv::circle(map_img, pede_img_pose, 3, cv::Scalar(0), -1);
                            j3Obs.push_back(front_data_.J3P_obs[i]);
                        }
                        else
                        { // 剩下的障碍物认为有宽高的物体，例如车辆
                            // 根据角度区分可能的障碍物
                            cv::Point2d obs_world_pose(front_data_.J3P_obs[i].obs_x, front_data_.J3P_obs[i].obs_y);
                            cv::Point obs_img_pose = world2img_param(obs_world_pose, img_origin, resolution_m);

                            cv::Rect2f obs_rect;
                            std::vector<cv::Point> rotate_rect_point;

                            obs_rect = calculate_obs_rect(
                                obs_img_pose,
                                front_data_.J3P_obs[i].obs_width / resolution_m,
                                front_data_.J3P_obs[i].obs_length / resolution_m,
                                resolution_m,
                                front_data_.J3P_obs[i].obs_pose_type);
                            rotate_rect_with_center(obs_rect, obs_img_pose, front_data_.J3P_obs[i].obs_angle * J3P_PI / 180.0, rotate_rect_point);

                            std::vector<std::vector<cv::Point>> draw_obs_polygons;
                            draw_obs_polygons.emplace_back(rotate_rect_point);
                            // cv::fillPoly(map_img, draw_obs_polygons, cv::Scalar(0));
                            cv::polylines(map_img, draw_obs_polygons, true, cv::Scalar(0), 2); // 黑色障碍物框
                            cv::circle(map_img, obs_img_pose, 3, cv::Scalar(125), 1);
                            J3Obs obs;
                            obs = front_data_.J3P_obs[i];
                            obs.obs_x = (500 - obs_img_pose.y) * resolution_m;
                            obs.obs_y = (250 - obs_img_pose.x) * resolution_m;
                            // std::cout << "j3 origin Obs: " << front_data_.J3P_obs[i].obs_x << "," << front_data_.J3P_obs[i].obs_y << "," << front_data_.J3P_obs[i].obs_width << "," << front_data_.J3P_obs[i].obs_length << std::endl;
                            j3Obs.push_back(obs);
                        }
                    }
                }
            }
        } while (0);
        // for (int i = 0; i < obs_rects.size(); i++)
        // {
        //     cv::Rect2f rect = obs_rects[i];
        //     std::cout << "J3obs: " << rect.x << "," << rect.y << "," << rect.width << "," << rect.height << std::endl;
        //     // std::cout<<j3Obs[i].obs_x<<","<<j3Obs[i].obs_y<<","<<j3Obs[i].obs_width<<","<<j3Obs[i].obs_height<<std::endl;
        // }
        // step3.2 绘制车道线
        float lane_gap = 0.1;
        cv::Scalar lane_color(0); // TODO:当前先用3通道图像的颜色
        const J3Line *Lane_line;
        const J3Line *Lane_line0 = get_front_data_all_line_ptr_apicpp(front_data_, 0);
        const J3Line *Lane_line1 = get_front_data_all_line_ptr_apicpp(front_data_, 1);
        double y0 = Lane_line0->line_c0 + Lane_line0->line_c1 * Lane_line0->line_start + Lane_line0->line_c2 * Lane_line0->line_start * Lane_line0->line_start + Lane_line0->line_c3 * Lane_line0->line_start * Lane_line0->line_start * Lane_line0->line_start;
        double x0 = Lane_line0->line_start;
        double y1 = Lane_line1->line_c0 + Lane_line1->line_c1 * Lane_line1->line_start + Lane_line1->line_c2 * Lane_line1->line_start * Lane_line1->line_start + Lane_line1->line_c3 * Lane_line1->line_start * Lane_line1->line_start * Lane_line1->line_start;
        double x1 = Lane_line1->line_start;
        double y2 = Lane_line0->line_c0 + Lane_line0->line_c1 * Lane_line0->line_end + Lane_line0->line_c2 * Lane_line0->line_end * Lane_line0->line_end + Lane_line0->line_c3 * Lane_line0->line_end * Lane_line0->line_end * Lane_line0->line_end;
        double x2 = Lane_line0->line_end;
        double y3 = Lane_line1->line_c0 + Lane_line1->line_c1 * Lane_line1->line_end + Lane_line1->line_c2 * Lane_line1->line_end * Lane_line1->line_end + Lane_line1->line_c3 * Lane_line1->line_end * Lane_line1->line_end * Lane_line1->line_end;
        double x3 = Lane_line1->line_end;
        cv::Point p0 = world2img_param(cv::Point2d(x0, y0), img_origin, resolution_m);
        cv::Point p1 = world2img_param(cv::Point2d(x1, y1), img_origin, resolution_m);
        cv::Point p2 = world2img_param(cv::Point2d(x2, y2), img_origin, resolution_m);
        cv::Point p3 = world2img_param(cv::Point2d(x3, y3), img_origin, resolution_m);
        // std::cout << "lane line0:" << x0 << "," << y0 << std::endl;
        // std::cout << "lane line0:" << x1 << "," << y1 << std::endl;
        // std::cout << "lane line1:" << x2 << "," << y2 << std::endl;
        // std::cout << "lane line1:" << x3 << "," << y3 << std::endl;
        bool isInter = false;
        // for (int i = 0; i < obs_rects.size(); i++)
        // {
        //     cv::Rect2f rect = obs_rects[i];
        //     bool ret = isLineIntersectRectangle(p0.x, 500 - p0.y, p2.x, 500 - p2.y, rect.x, 500 - rect.y, rect.x + rect.width, 500 - rect.y - rect.height);
        //     bool ret1 = isLineIntersectRectangle(p1.x, 500 - p1.y, p3.x, 500 - p3.y, rect.x, 500 - rect.y, rect.x + rect.width, 500 - rect.y - rect.height);
        //     // std::cout << ret << ret1 << std::endl;
        //     if (ret || ret1)
        //     {
        //         isInter = true;
        //     }
        // }
        if (Lane_line0 != nullptr && Lane_line1 != nullptr)
        {
            if (Lane_line0->is_valid == 1 && Lane_line1->is_valid == 1 && abs(y2 - y3) > 4 && opt_ok && !isInter) // && x0 == 0 && x1 == 0)
            {
                for (int i = 0; i < 2; i++)
                {
                    if (i == 0)
                    {
                        Lane_line = Lane_line0;
                    }
                    else
                    {
                        Lane_line = Lane_line1;
                    }
                    float lane_c0 = Lane_line->line_c0;
                    float lane_c1 = Lane_line->line_c1;
                    float lane_c2 = Lane_line->line_c2;
                    float lane_c3 = Lane_line->line_c3;

                    float lane_strat = Lane_line->line_start;

                    cv::Point2d lane_p0(lane_strat, lane_c0 + lane_c1 * lane_strat + lane_c2 * lane_strat * lane_strat + lane_c3 * lane_strat * lane_strat * lane_strat);
                    cv::Point lane_img_p0 = world2img_param(lane_p0, img_origin, resolution_m);
                    for (double x = lane_strat + lane_gap; x < Lane_line->line_end; x += lane_gap)
                    {
                        double y = lane_c0 + lane_c1 * x + lane_c2 * x * x + lane_c3 * x * x * x;
                        cv::Point2d lane_p1(x, y);
                        cv::Point lane_img_p1 = world2img_param(lane_p1, img_origin, resolution_m);
                        cv::line(map_img, lane_img_p0, lane_img_p1, lane_color, 2);
                        lane_img_p0 = lane_img_p1;
                    }
                }
            }
        }

        for (int i = 2; i < 10; i++)
        {
            const J3Line *Lane_line = get_front_data_all_line_ptr_apicpp(front_data_, i);
            if (Lane_line != nullptr)
            {
                if (Lane_line->is_valid == 1)
                {
                    float lane_c0 = Lane_line->line_c0;
                    float lane_c1 = Lane_line->line_c1;
                    float lane_c2 = Lane_line->line_c2;
                    float lane_c3 = Lane_line->line_c3;

                    float lane_strat = Lane_line->line_start;

                    if (i < 6)
                    {
                        lane_color = cv::Scalar(128);
                    }
                    else if (i < 8)
                    {
                        lane_color = cv::Scalar(0);
                    }
                    else
                    {
                        lane_color = cv::Scalar(0);
                    }

                    cv::Point2d lane_p0(lane_strat, lane_c0 + lane_c1 * lane_strat + lane_c2 * lane_strat * lane_strat + lane_c3 * lane_strat * lane_strat * lane_strat);
                    cv::Point lane_img_p0 = world2img_param(lane_p0, img_origin, resolution_m);
                    for (double x = lane_strat + lane_gap; x < Lane_line->line_end; x += lane_gap)
                    {
                        double y = lane_c0 + lane_c1 * x + lane_c2 * x * x + lane_c3 * x * x * x;
                        cv::Point2d lane_p1(x, y);
                        cv::Point lane_img_p1 = world2img_param(lane_p1, img_origin, resolution_m);
                        cv::line(map_img, lane_img_p0, lane_img_p1, lane_color, 2);
                        lane_img_p0 = lane_img_p1;
                    }
                }
            }
        }
        // for (int i = 0; i < lidarinfo.fieldNum; i++)
        // {
        //     if (lidarinfo.lidarfield[i].pointNum < 2 || lidarinfo.lidarfield[i].point[0].z < 0)
        //     {
        //         continue;
        //     }
        //     for (int j = 0; j < lidarinfo.lidarfield[i].pointNum - 1; j++)
        //     {
        //         cv::Point2d lidarp0(lidarinfo.lidarfield[i].point[j].x, lidarinfo.lidarfield[i].point[j].y);
        //         cv::Point lidar_p0 = world2img_param(lidarp0, img_origin, resolution_m);
        //         cv::Point2d lidarp1(lidarinfo.lidarfield[i].point[j + 1].x, lidarinfo.lidarfield[i].point[j + 1].y);
        //         cv::Point lidar_p1 = world2img_param(lidarp1, img_origin, resolution_m);
        //         cv::line(map_img, lidar_p0, lidar_p1, lane_color, 2);
        //     }
        // }

        fornt_perception_img = map_img;
        time_stamp = front_data_.vm_muc_timeStamp;
        return 0;
    }
}

//============================================================================

namespace tinyxml2_case02
{

    using namespace tinyxml2;

    // 辅助函数：将J3Point2f保存为XML元素
    void saveJ3Point2f(XMLElement *parent, const char *name, const J3Point2f &point)
    {
        XMLElement *pointElement = parent->GetDocument()->NewElement(name);
        pointElement->SetAttribute("x", point.x);
        pointElement->SetAttribute("y", point.y);
        parent->LinkEndChild(pointElement);
    }

    // 辅助函数：从XML元素加载J3Point2f
    bool loadJ3Point2f(XMLElement *parent, const char *name, J3Point2f &point)
    {
        XMLElement *pointElement = parent->FirstChildElement(name);
        if (!pointElement)
        {
            return false;
        }
        pointElement->QueryFloatAttribute("x", &point.x);
        pointElement->QueryFloatAttribute("y", &point.y);
        return true;
    }

    //=======================================================================================

    // 一次保存20个函数
    void saveJ3ObsArray(XMLElement *parent, const J3Obs (&obsArray)[20])
    {
        XMLDocument *doc = parent->GetDocument();
        XMLElement *obstaclesElement = doc->NewElement("Obstacles");

        for (int i = 0; i < 20; ++i)
        {
            XMLElement *obsElement = doc->NewElement("J3Obs");

            // 保存基础属性
            obsElement->SetAttribute("index", i);

            obsElement->SetAttribute("obs_id", obsArray[i].obs_id);
            obsElement->SetAttribute("obs_conf", (unsigned int)obsArray[i].obs_conf);
            obsElement->SetAttribute("obs_class", (unsigned int)obsArray[i].obs_class);

            obsElement->SetAttribute("obs_x", obsArray[i].obs_x);
            obsElement->SetAttribute("obs_y", obsArray[i].obs_y);
            obsElement->SetAttribute("obs_angle", obsArray[i].obs_angle);
            obsElement->SetAttribute("obs_length", obsArray[i].obs_length);
            obsElement->SetAttribute("obs_width", obsArray[i].obs_width);
            obsElement->SetAttribute("obs_height", obsArray[i].obs_height);

            // obsElement->SetAttribute("obs_contour_point_max_num", 4/*obsArray[i].obs_contour_point_max_num*/);//这个值默认是4
            // obsElement->SetAttribute("obs_contour_point_vaild_num", obsArray[i].obs_contour_point_vaild_num);

            obsElement->SetAttribute("obs_vaild", (unsigned int)obsArray[i].obs_vaild);
            obsElement->SetAttribute("obs_id0", (unsigned int)obsArray[i].obs_id0);
            // obsElement->SetAttribute("obs_enum_class", (int)obsArray[i].obs_enum_class);
            // obsElement->SetAttribute("obs_pose_type", (unsigned int)obsArray[i].obs_pose_type);

            //// 保存轮廓点（完整保存4个点）
            // for (unsigned int j = 0; j < 4/*obsArray[i].obs_contour_point_max_num*/; ++j) {
            //     XMLElement* pointElement = doc->NewElement("contour_point");
            //     pointElement->SetAttribute("x", obsArray[i].contour_point[j].x);
            //     pointElement->SetAttribute("y", obsArray[i].contour_point[j].y);
            //     obsElement->LinkEndChild(pointElement);
            // }

            obstaclesElement->LinkEndChild(obsElement);
        }

        parent->LinkEndChild(obstaclesElement);
    }

    // 从XML加载所有20个J3Obs（内部循环）
    bool loadJ3ObsArray(XMLElement *parent, J3Obs (&obsArray)[20])
    {
        XMLElement *obstaclesElement = parent->FirstChildElement("Obstacles");
        if (!obstaclesElement)
            return false;

        XMLElement *obsElement = obstaclesElement->FirstChildElement("J3Obs");
        int loadedCount = 0;

        // 严格限制最多加载20个
        while (obsElement && loadedCount < 20)
        {
            // 读取索引（用于校验）
            int index = -1;
            obsElement->QueryIntAttribute("index", &index);

            if (index != loadedCount)
            {
                // 处理索引不连续的情况
                break;
            }

            // 加载基础属性
            // obsElement->QueryUnsignedAttribute("obs_id", &obsArray[loadedCount].obs_id);
            unsigned int tmp_ui = 0;

            obsElement->QueryUnsignedAttribute("obs_id", &obsArray[loadedCount].obs_id);
            obsElement->QueryUnsignedAttribute("obs_conf", &tmp_ui);
            obsArray[loadedCount].obs_conf = tmp_ui;
            obsElement->QueryUnsignedAttribute("obs_class", &tmp_ui);
            obsArray[loadedCount].obs_class = tmp_ui;

            obsElement->QueryFloatAttribute("obs_x", &obsArray[loadedCount].obs_x);
            obsElement->QueryFloatAttribute("obs_y", &obsArray[loadedCount].obs_y);
            obsElement->QueryFloatAttribute("obs_angle", &obsArray[loadedCount].obs_angle);
            obsElement->QueryFloatAttribute("obs_length", &obsArray[loadedCount].obs_length);
            obsElement->QueryFloatAttribute("obs_width", &obsArray[loadedCount].obs_width);
            obsElement->QueryFloatAttribute("obs_height", &obsArray[loadedCount].obs_height);
            // obsElement->QueryUnsignedAttribute("obs_contour_point_max_num", &obsArray[loadedCount].obs_contour_point_max_num);
            // obsElement->QueryUnsignedAttribute("obs_contour_point_vaild_num", &obsArray[loadedCount].obs_contour_point_vaild_num);

            obsElement->QueryUnsignedAttribute("obs_id0", &tmp_ui);
            obsArray[loadedCount].obs_id0 = tmp_ui;
            // obsElement->QueryIntAttribute("obs_enum_class", (int*)&obsArray[loadedCount].obs_enum_class);
            // obsElement->QueryUnsignedAttribute("obs_pose_type", &tmp_ui); obsArray[loadedCount].obs_pose_type = tmp_ui;

            // 加载轮廓点（完整加载4个点）
            // XMLElement* pointElement = obsElement->FirstChildElement("contour_point");
            // unsigned int pointIndex = 0;
            // while (pointElement && pointIndex < 4/*obsArray[loadedCount].obs_contour_point_max_num*/) {
            //    pointElement->QueryFloatAttribute("x", &obsArray[loadedCount].contour_point[pointIndex].x);
            //    pointElement->QueryFloatAttribute("y", &obsArray[loadedCount].contour_point[pointIndex].y);
            //    pointElement = pointElement->NextSiblingElement("contour_point");
            //    pointIndex++;
            //}

            obsElement = obsElement->NextSiblingElement("J3Obs");
            loadedCount++;
        }

        // 填充未加载的障碍物为默认值
        for (; loadedCount < 20; ++loadedCount)
        {
            obsArray[loadedCount] = J3Obs{};
        }

        return true;
    }
    //=======================================================================================

    // 辅助函数：将J3Line保存为XML元素
    void saveJ3Line(XMLElement *parent, const char *name, const J3Line &line)
    {
        XMLElement *lineElement = parent->GetDocument()->NewElement(name);
        lineElement->SetAttribute("is_valid", (int)line.is_valid);
        lineElement->SetAttribute("line_id", (unsigned int)line.line_id);
        lineElement->SetAttribute("line_conf", (unsigned int)line.line_conf);
        lineElement->SetAttribute("line_status", (unsigned int)line.line_status);
        // lineElement->SetAttribute("line_enum_status", (int)line.line_enum_status);
        lineElement->SetAttribute("line_class", (unsigned int)line.line_class);
        // lineElement->SetAttribute("line_enum_class", (int)line.line_enum_class);

        lineElement->SetAttribute("line_start", line.line_start);
        lineElement->SetAttribute("line_end", line.line_end);
        lineElement->SetAttribute("line_c0", line.line_c0);
        lineElement->SetAttribute("line_c1", line.line_c1);
        lineElement->SetAttribute("line_c2", line.line_c2);
        lineElement->SetAttribute("line_c3", line.line_c3);
        parent->LinkEndChild(lineElement);
    }

    // 辅助函数：从XML元素加载J3Line
    bool loadJ3Line(XMLElement *parent, const char *name, J3Line &line)
    {
        XMLElement *lineElement = parent->FirstChildElement(name);
        if (!lineElement)
        {
            return false;
        }
        int tmp_i = 0;
        unsigned int tmp_ui = 0;

        lineElement->QueryIntAttribute("is_valid", &tmp_i);
        line.is_valid = tmp_i;
        lineElement->QueryUnsignedAttribute("line_id", &tmp_ui);
        line.line_id = tmp_ui;
        lineElement->QueryUnsignedAttribute("line_conf", &tmp_ui);
        line.line_conf = tmp_ui;
        lineElement->QueryUnsignedAttribute("line_status", &tmp_ui);
        line.line_status = tmp_ui;
        // lineElement->QueryIntAttribute("line_enum_status", (int*)&line.line_enum_status);
        lineElement->QueryUnsignedAttribute("line_class", &tmp_ui);
        line.line_class = tmp_ui;
        // lineElement->QueryIntAttribute("line_enum_class", (int*)&line.line_enum_class);

        lineElement->QueryFloatAttribute("line_start", &line.line_start);
        lineElement->QueryFloatAttribute("line_end", &line.line_end);
        lineElement->QueryFloatAttribute("line_c0", &line.line_c0);
        lineElement->QueryFloatAttribute("line_c1", &line.line_c1);
        lineElement->QueryFloatAttribute("line_c2", &line.line_c2);
        lineElement->QueryFloatAttribute("line_c3", &line.line_c3);
        return true;
    }

    //=======================================================================================

    // 辅助函数：将J3Freespace保存为XML元素
    void saveJ3Freespace(XMLElement *parent, const char *name, const J3Freespace &freespace)
    {
        XMLDocument *doc = parent->GetDocument();
        XMLElement *freespaceElement = doc->NewElement(name);

        // 创建 contours_points 的父节点
        XMLElement *contoursElement = doc->NewElement("Contours");

        for (unsigned int i = 0; i < 64; ++i)
        {
            // 每个点单独作为一个节点，包含坐标和类型
            XMLElement *pointElement = doc->NewElement("Point");
            pointElement->SetAttribute("index", i);
            pointElement->SetAttribute("x", freespace.contours_points[i].x);
            pointElement->SetAttribute("y", freespace.contours_points[i].y);
            pointElement->SetAttribute("type", freespace.point_type[i]); // 注意：这里假设 point_type 是 uint32_t
            contoursElement->LinkEndChild(pointElement);
        }

        freespaceElement->LinkEndChild(contoursElement);
        parent->LinkEndChild(freespaceElement);
    }

    // 辅助函数：从XML元素加载J3Freespace
    bool loadJ3Freespace(XMLElement *parent, const char *name, J3Freespace &freespace)
    {
        XMLElement *freespaceElement = parent->FirstChildElement(name);
        if (!freespaceElement)
        {
            return false;
        }

        // 查找 Contours 节点
        XMLElement *contoursElement = freespaceElement->FirstChildElement("Contours");
        if (!contoursElement)
            return false;

        // 遍历所有 Point 节点（最多64个）
        unsigned int pointIndex = 0;
        XMLElement *pointElement = contoursElement->FirstChildElement("Point");
        while (pointElement && pointIndex < 64)
        {
            // 读取坐标
            pointElement->QueryFloatAttribute("x", &freespace.contours_points[pointIndex].x);
            pointElement->QueryFloatAttribute("y", &freespace.contours_points[pointIndex].y);

            // 读取类型（如果属性不存在则默认为0）
            if (pointElement->FindAttribute("type"))
            {
                pointElement->QueryUnsignedAttribute("type", &freespace.point_type[pointIndex]);
            }
            else
            {
                freespace.point_type[pointIndex] = 0;
            }

            // 移动到下一个点（忽略index属性）
            pointElement = pointElement->NextSiblingElement("Point");
            pointIndex++;
        }

        return true;
    }
    //=======================================================================================

    // 主函数：保存J3PerceptionData为XML文件
    bool savePerceptionDataToXML(const J3PerceptionData &data, const std::string &filename)
    {
        XMLDocument doc;
        XMLElement *rootElement = doc.NewElement("J3PerceptionData");
        doc.LinkEndChild(rootElement);

        // 基本数据
        rootElement->SetAttribute("data_valid", (int)data.data_valid);
        rootElement->SetAttribute("command", (unsigned int)data.command);
        rootElement->SetAttribute("vm_muc_timeStamp", (unsigned int)data.vm_muc_timeStamp);
        rootElement->SetAttribute("J3_image_time", (unsigned int)data.J3_image_time);
        rootElement->SetAttribute("J3_frame_index", data.J3_frame_index);

        // 障碍物数据
        rootElement->SetAttribute("J3P_obs_num", data.J3P_obs_num);
        saveJ3ObsArray(rootElement, data.J3P_obs);

        // 车道线数据
        saveJ3Line(rootElement, "J3P_LKA_Left_Lane", data.J3P_LKA_Left_Lane);
        saveJ3Line(rootElement, "J3P_LKA_Right_Lane", data.J3P_LKA_Right_Lane);
        saveJ3Line(rootElement, "J3P_Next_Left_Lane", data.J3P_Next_Left_Lane);
        saveJ3Line(rootElement, "J3P_Next_Right_Lane", data.J3P_Next_Right_Lane);
        saveJ3Line(rootElement, "J3P_Left_Outer_Lane", data.J3P_Left_Outer_Lane);
        saveJ3Line(rootElement, "J3P_Right_Outer_Lane", data.J3P_Right_Outer_Lane);
        saveJ3Line(rootElement, "J3P_Left_Cone_Road_Edge", data.J3P_Left_Cone_Road_Edge);
        saveJ3Line(rootElement, "J3P_Right_Cone_Road_Edge", data.J3P_Right_Cone_Road_Edge);
        saveJ3Line(rootElement, "J3P_Left_Road_Edge", data.J3P_Left_Road_Edge);
        saveJ3Line(rootElement, "J3P_Right_Road_Edge", data.J3P_Right_Road_Edge);

        // freespace数据
        saveJ3Freespace(rootElement, "J3P_Freespace", data.J3P_Freespace);

        return doc.SaveFile(filename.c_str()) == XML_SUCCESS;
    }

    // 主函数：从XML文件加载J3PerceptionData
    bool loadPerceptionDataFromXML(J3PerceptionData &data, const std::string &filename)
    {
        XMLDocument doc;
        if (doc.LoadFile(filename.c_str()) != XML_SUCCESS)
        {
            return false;
        }

        XMLElement *rootElement = doc.FirstChildElement("J3PerceptionData");
        if (!rootElement)
        {
            return false;
        }

        int tmp_i = 0;
        unsigned int tmp_ui = 0;
        rootElement->QueryIntAttribute("data_valid", &tmp_i);
        data.data_valid = tmp_i;
        rootElement->QueryUnsignedAttribute("command", &tmp_ui);
        data.command = tmp_ui;
        rootElement->QueryUnsignedAttribute("vm_muc_timeStamp", &tmp_ui);
        data.vm_muc_timeStamp = tmp_ui;
        rootElement->QueryUnsignedAttribute("J3_image_time", &tmp_ui);
        data.J3_image_time = tmp_ui;
        rootElement->QueryUnsignedAttribute("J3_frame_index", &data.J3_frame_index);
        rootElement->QueryIntAttribute("J3P_obs_num", &data.J3P_obs_num);

        // XMLElement* obsElement = rootElement->FirstChildElement("J3Obs");

        if (!loadJ3ObsArray(rootElement, data.J3P_obs))
        {
            std::cout << "@j3p: [load xml -> Obs data] load error!" << std::endl;
        }
        // for (int i = 0; i < 20; ++i) {
        //     if (obsElement) {
        //         loadJ3Obs(rootElement, "J3Obs", data.J3P_obs[i]);
        //         obsElement = obsElement->NextSiblingElement("J3Obs");
        //     }
        //     else {
        //         data.J3P_obs[i] = J3Obs{};
        //     }
        // }

        loadJ3Line(rootElement, "J3P_LKA_Left_Lane", data.J3P_LKA_Left_Lane);
        loadJ3Line(rootElement, "J3P_LKA_Right_Lane", data.J3P_LKA_Right_Lane);
        loadJ3Line(rootElement, "J3P_Next_Left_Lane", data.J3P_Next_Left_Lane);
        loadJ3Line(rootElement, "J3P_Next_Right_Lane", data.J3P_Next_Right_Lane);
        loadJ3Line(rootElement, "J3P_Left_Outer_Lane", data.J3P_Left_Outer_Lane);
        loadJ3Line(rootElement, "J3P_Right_Outer_Lane", data.J3P_Right_Outer_Lane);
        loadJ3Line(rootElement, "J3P_Left_Cone_Road_Edge", data.J3P_Left_Cone_Road_Edge);
        loadJ3Line(rootElement, "J3P_Right_Cone_Road_Edge", data.J3P_Right_Cone_Road_Edge);
        loadJ3Line(rootElement, "J3P_Left_Road_Edge", data.J3P_Left_Road_Edge);
        loadJ3Line(rootElement, "J3P_Right_Road_Edge", data.J3P_Right_Road_Edge);

        loadJ3Freespace(rootElement, "J3P_Freespace", data.J3P_Freespace);

        return true;
    }

}
