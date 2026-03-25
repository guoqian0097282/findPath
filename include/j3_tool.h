#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>
#include "tinyxml2.h"
#include "j3_perception_data_api.h"

using namespace tinyxml2;

class J3PerceptionDataXml {

public:
    // 主函数：保存J3PerceptionData为XML文件
    static bool savePerceptionDataToXML(const J3PerceptionData& data, const std::string& filename) {

        XMLDocument doc;
        doc.InsertFirstChild(doc.NewDeclaration());
        XMLElement* rootElement = doc.NewElement("J3PerceptionData");
        doc.LinkEndChild(rootElement);

        //基本数据
        rootElement->SetAttribute("data_valid", (int)data.data_valid);
        rootElement->SetAttribute("command", (unsigned int)data.command);
        rootElement->SetAttribute("vm_muc_timeStamp", (unsigned int)data.vm_muc_timeStamp);
        rootElement->SetAttribute("J3_image_time", (unsigned int)data.J3_image_time);
        rootElement->SetAttribute("SPI_image_time", (unsigned int)data.SPI_time_stamp);
        rootElement->SetAttribute("J3_frame_index", data.J3_frame_index);

        //障碍物数据
        rootElement->SetAttribute("J3P_obs_num", data.J3P_obs_num);
        saveJ3ObsArray(rootElement, data.J3P_obs);

        //车道线数据
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

        //freespace数据
        saveJ3Freespace(rootElement, "J3P_Freespace", data.J3P_Freespace);

        return doc.SaveFile(filename.c_str()) == XML_SUCCESS;
    }


    // 主函数：从XML文件加载J3PerceptionData
    static bool loadPerceptionDataFromXML(J3PerceptionData& data, const std::string& filename) {
        XMLDocument doc;
        if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) {
            return false;
        }

        XMLElement* rootElement = doc.FirstChildElement("J3PerceptionData");
        if (!rootElement) {
            return false;
        }

        int tmp_i = 0;
        unsigned int tmp_ui = 0;
        rootElement->QueryIntAttribute("data_valid", &tmp_i); data.data_valid = tmp_i;
        rootElement->QueryUnsignedAttribute("command", &tmp_ui); data.command = tmp_ui;
        rootElement->QueryUnsignedAttribute("vm_muc_timeStamp", &tmp_ui); data.vm_muc_timeStamp = tmp_ui;
        rootElement->QueryUnsignedAttribute("J3_image_time", &tmp_ui); data.J3_image_time = tmp_ui;
        rootElement->QueryUnsignedAttribute("SPI_image_time", &tmp_ui); data.SPI_time_stamp = tmp_ui;
        rootElement->QueryUnsignedAttribute("J3_frame_index", &data.J3_frame_index);
        rootElement->QueryIntAttribute("J3P_obs_num", &data.J3P_obs_num);

        //XMLElement* obsElement = rootElement->FirstChildElement("J3Obs");

        if (!loadJ3ObsArray(rootElement, data.J3P_obs)) {
            std::cout << "@j3p: [load xml -> Obs data] load error!" << std::endl;
        }
        //for (int i = 0; i < 20; ++i) {
        //    if (obsElement) {
        //        loadJ3Obs(rootElement, "J3Obs", data.J3P_obs[i]);
        //        obsElement = obsElement->NextSiblingElement("J3Obs");
        //    }
        //    else {
        //        data.J3P_obs[i] = J3Obs{};
        //    }
        //}

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



private:


    // 辅助函数：将J3Point2f保存为XML元素
    static void saveJ3Point2f(XMLElement* parent, const char* name, const J3Point2f& point) {
        XMLElement* pointElement = parent->GetDocument()->NewElement(name);
        pointElement->SetAttribute("x", point.x);
        pointElement->SetAttribute("y", point.y);
        parent->LinkEndChild(pointElement);
    }

    // 辅助函数：从XML元素加载J3Point2f
    static bool loadJ3Point2f(XMLElement* parent, const char* name, J3Point2f& point) {
        XMLElement* pointElement = parent->FirstChildElement(name);
        if (!pointElement) {
            return false;
        }
        pointElement->QueryFloatAttribute("x", &point.x);
        pointElement->QueryFloatAttribute("y", &point.y);
        return true;
    }

    //=======================================================================================


    //一次保存20个函数
    static void saveJ3ObsArray(XMLElement* parent, const J3Obs(&obsArray)[20]) {
        XMLDocument* doc = parent->GetDocument();
        XMLElement* obstaclesElement = doc->NewElement("Obstacles");

        for (int i = 0; i < 20; ++i) {
            XMLElement* obsElement = doc->NewElement("J3Obs");

            // 保存基础属性
            obsElement->SetAttribute("index", i);

            obsElement->SetAttribute("obs_id", obsArray[i].obs_id);
            obsElement->SetAttribute("obs_conf", (unsigned int)obsArray[i].obs_conf);
            obsElement->SetAttribute("obs_class", (unsigned int)obsArray[i].obs_class);
            obsElement->SetAttribute("obs_pose_type", (unsigned int)obsArray[i].obs_pose_type);
            obsElement->SetAttribute("obs_ttc", (unsigned int)obsArray[i].obs_ttc);

            obsElement->SetAttribute("obs_x", obsArray[i].obs_x);
            obsElement->SetAttribute("obs_y", obsArray[i].obs_y);
            obsElement->SetAttribute("obs_angle", obsArray[i].obs_angle);
            obsElement->SetAttribute("obs_length", obsArray[i].obs_length);
            obsElement->SetAttribute("obs_width", obsArray[i].obs_width);
            obsElement->SetAttribute("obs_height", obsArray[i].obs_height);

            obsElement->SetAttribute("obs_velx", obsArray[i].obs_velx);
            obsElement->SetAttribute("obs_vely", obsArray[i].obs_vely);
            obsElement->SetAttribute("obs_abs_velx", obsArray[i].obs_abs_velx);
            obsElement->SetAttribute("obs_abs_vely", obsArray[i].obs_abs_vely);
            obsElement->SetAttribute("obs_accx", obsArray[i].obs_accx);
            obsElement->SetAttribute("obs_accy", obsArray[i].obs_accy);
            obsElement->SetAttribute("obs_abs_accx", obsArray[i].obs_abs_accx);
            obsElement->SetAttribute("obs_abs_accy", obsArray[i].obs_abs_accy);

            obsElement->SetAttribute("obs_vaild", (unsigned int)obsArray[i].obs_vaild);
            obsElement->SetAttribute("obs_id0", (unsigned int)obsArray[i].obs_id0);

            obstaclesElement->LinkEndChild(obsElement);
        }

        parent->LinkEndChild(obstaclesElement);
    }


    // 从XML加载所有20个J3Obs（内部循环）
    static bool loadJ3ObsArray(XMLElement* parent, J3Obs(&obsArray)[20]) {
        XMLElement* obstaclesElement = parent->FirstChildElement("Obstacles");
        if (!obstaclesElement) return false;

        XMLElement* obsElement = obstaclesElement->FirstChildElement("J3Obs");
        int loadedCount = 0;

        // 严格限制最多加载20个
        while (obsElement && loadedCount < 20) {
            // 读取索引（用于校验）
            int index = -1;
            obsElement->QueryIntAttribute("index", &index);

            if (index != loadedCount) {
                // 处理索引不连续的情况
                break;
            }

            // 加载基础属性
            //obsElement->QueryUnsignedAttribute("obs_id", &obsArray[loadedCount].obs_id);
            unsigned int tmp_ui = 0;

            obsElement->QueryUnsignedAttribute("obs_id", &obsArray[loadedCount].obs_id);
            obsElement->QueryUnsignedAttribute("obs_conf", &tmp_ui); obsArray[loadedCount].obs_conf = tmp_ui;
            obsElement->QueryUnsignedAttribute("obs_pose_type", &tmp_ui); obsArray[loadedCount].obs_pose_type = tmp_ui;;
            obsElement->QueryUnsignedAttribute("obs_class", &tmp_ui); obsArray[loadedCount].obs_class = tmp_ui;
            obsElement->QueryUnsignedAttribute("obs_ttc", &tmp_ui); obsArray[loadedCount].obs_ttc = tmp_ui;

            obsElement->QueryFloatAttribute("obs_x", &obsArray[loadedCount].obs_x);
            obsElement->QueryFloatAttribute("obs_y", &obsArray[loadedCount].obs_y);
            obsElement->QueryFloatAttribute("obs_angle", &obsArray[loadedCount].obs_angle);
            obsElement->QueryFloatAttribute("obs_length", &obsArray[loadedCount].obs_length);
            obsElement->QueryFloatAttribute("obs_width", &obsArray[loadedCount].obs_width);
            obsElement->QueryFloatAttribute("obs_height", &obsArray[loadedCount].obs_height);

            obsElement->QueryFloatAttribute("obs_velx", &obsArray[loadedCount].obs_velx);
            obsElement->QueryFloatAttribute("obs_vely", &obsArray[loadedCount].obs_vely);
            obsElement->QueryFloatAttribute("obs_abs_velx", &obsArray[loadedCount].obs_abs_velx);
            obsElement->QueryFloatAttribute("obs_abs_vely", &obsArray[loadedCount].obs_abs_vely);
            obsElement->QueryFloatAttribute("obs_accx", &obsArray[loadedCount].obs_accx);
            obsElement->QueryFloatAttribute("obs_accy", &obsArray[loadedCount].obs_accy);
            obsElement->QueryFloatAttribute("obs_abs_accx", &obsArray[loadedCount].obs_abs_accx);
            obsElement->QueryFloatAttribute("obs_abs_accy", &obsArray[loadedCount].obs_abs_accy);

            //obsElement->QueryUnsignedAttribute("obs_contour_point_max_num", &obsArray[loadedCount].obs_contour_point_max_num);
            //obsElement->QueryUnsignedAttribute("obs_contour_point_vaild_num", &obsArray[loadedCount].obs_contour_point_vaild_num);

            obsElement->QueryUnsignedAttribute("obs_id0", &tmp_ui); obsArray[loadedCount].obs_id0 = tmp_ui;
            //obsElement->QueryIntAttribute("obs_enum_class", (int*)&obsArray[loadedCount].obs_enum_class);
            //obsElement->QueryUnsignedAttribute("obs_pose_type", &tmp_ui); obsArray[loadedCount].obs_pose_type = tmp_ui;

            // 加载轮廓点（完整加载4个点）
            //XMLElement* pointElement = obsElement->FirstChildElement("contour_point");
            //unsigned int pointIndex = 0;
            //while (pointElement && pointIndex < 4/*obsArray[loadedCount].obs_contour_point_max_num*/) {
            //    pointElement->QueryFloatAttribute("x", &obsArray[loadedCount].contour_point[pointIndex].x);
            //    pointElement->QueryFloatAttribute("y", &obsArray[loadedCount].contour_point[pointIndex].y);
            //    pointElement = pointElement->NextSiblingElement("contour_point");
            //    pointIndex++;
            //}

            obsElement = obsElement->NextSiblingElement("J3Obs");
            loadedCount++;
        }

        // 填充未加载的障碍物为默认值
        for (; loadedCount < 20; ++loadedCount) {
            obsArray[loadedCount] = J3Obs{};
        }

        return true;
    }
    //=======================================================================================

    // 辅助函数：将J3Line保存为XML元素
    static void saveJ3Line(XMLElement* parent, const char* name, const J3Line& line) {
        XMLElement* lineElement = parent->GetDocument()->NewElement(name);
        lineElement->SetAttribute("is_valid", (int)line.is_valid);
        lineElement->SetAttribute("line_id", (unsigned int)line.line_id);
        lineElement->SetAttribute("line_conf", (unsigned int)line.line_conf);
        lineElement->SetAttribute("line_status", (unsigned int)line.line_status);
        //lineElement->SetAttribute("line_enum_status", (int)line.line_enum_status);
        lineElement->SetAttribute("line_class", (unsigned int)line.line_class);
        //lineElement->SetAttribute("line_enum_class", (int)line.line_enum_class);

        lineElement->SetAttribute("line_start", line.line_start);
        lineElement->SetAttribute("line_end", line.line_end);
        lineElement->SetAttribute("line_c0", line.line_c0);
        lineElement->SetAttribute("line_c1", line.line_c1);
        lineElement->SetAttribute("line_c2", line.line_c2);
        lineElement->SetAttribute("line_c3", line.line_c3);
        parent->LinkEndChild(lineElement);
    }

    // 辅助函数：从XML元素加载J3Line
    static bool loadJ3Line(XMLElement* parent, const char* name, J3Line& line) {
        XMLElement* lineElement = parent->FirstChildElement(name);
        if (!lineElement) {
            return false;
        }
        int tmp_i = 0;
        unsigned int tmp_ui = 0;

        lineElement->QueryIntAttribute("is_valid", &tmp_i); line.is_valid = tmp_i;
        lineElement->QueryUnsignedAttribute("line_id", &tmp_ui); line.line_id = tmp_ui;
        lineElement->QueryUnsignedAttribute("line_conf", &tmp_ui); line.line_conf = tmp_ui;
        lineElement->QueryUnsignedAttribute("line_status", &tmp_ui); line.line_status = tmp_ui;
        //lineElement->QueryIntAttribute("line_enum_status", (int*)&line.line_enum_status);
        lineElement->QueryUnsignedAttribute("line_class", &tmp_ui); line.line_class = tmp_ui;
        //lineElement->QueryIntAttribute("line_enum_class", (int*)&line.line_enum_class);

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
    static void saveJ3Freespace(XMLElement* parent, const char* name, const J3Freespace& freespace) {
        XMLDocument* doc = parent->GetDocument();
        XMLElement* freespaceElement = doc->NewElement(name);

        // 创建 contours_points 的父节点
        XMLElement* contoursElement = doc->NewElement("Contours");

        for (unsigned int i = 0; i < 64; ++i) {
            // 每个点单独作为一个节点，包含坐标和类型
            XMLElement* pointElement = doc->NewElement("Point");
            pointElement->SetAttribute("index", i);
            pointElement->SetAttribute("x", freespace.contours_points[i].x);
            pointElement->SetAttribute("y", freespace.contours_points[i].y);
            pointElement->SetAttribute("type", freespace.point_type[i]);  // 注意：这里假设 point_type 是 uint32_t
            contoursElement->LinkEndChild(pointElement);
        }

        freespaceElement->LinkEndChild(contoursElement);
        parent->LinkEndChild(freespaceElement);
    }

    // 辅助函数：从XML元素加载J3Freespace
    static bool loadJ3Freespace(XMLElement* parent, const char* name, J3Freespace& freespace) {
        XMLElement* freespaceElement = parent->FirstChildElement(name);
        if (!freespaceElement) {
            return false;
        }

        // 查找 Contours 节点
        XMLElement* contoursElement = freespaceElement->FirstChildElement("Contours");
        if (!contoursElement) return false;

        // 遍历所有 Point 节点（最多64个）
        unsigned int pointIndex = 0;
        XMLElement* pointElement = contoursElement->FirstChildElement("Point");
        while (pointElement && pointIndex < 64) {
            // 读取坐标
            pointElement->QueryFloatAttribute("x", &freespace.contours_points[pointIndex].x);
            pointElement->QueryFloatAttribute("y", &freespace.contours_points[pointIndex].y);

            // 读取类型（如果属性不存在则默认为0）
            if (pointElement->FindAttribute("type")) {
                pointElement->QueryUnsignedAttribute("type", &freespace.point_type[pointIndex]);
            }
            else {
                freespace.point_type[pointIndex] = 0;
            }

            // 移动到下一个点（忽略index属性）
            pointElement = pointElement->NextSiblingElement("Point");
            pointIndex++;
        }

        return true;
    }



};