#include "run_code.h"

#include <string>
#include <iostream>
#include <vector>
#include <filesystem> // 要求C++17标准

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "j3_perception_data_api.h"

using namespace cv;
using namespace std;
namespace fs = std::filesystem;
void test_data() {
	J3PerceptionData test_data;

	test_data.data_valid = true;
	test_data.J3_image_time = 238;
	test_data.J3_frame_index = 4921;

	//障碍物车，先给一个大巴车的
	test_data.J3P_obs_num = 1;
	test_data.J3P_obs[0].obs_x = 46.2;
	test_data.J3P_obs[0].obs_y = -1.3;
	test_data.J3P_obs[0].obs_angle = 0;

	test_data.J3P_obs[0].obs_length = 8;//大巴车
	test_data.J3P_obs[0].obs_width = 4;
	//test_data.J3P_obs[0].obs_contour_point_max_num = 4;
	//test_data.J3P_obs[0].obs_contour_point_vaild_num = 4;
	//test_data.J3P_obs[0].contour_point[0].x = 54.2;
	//test_data.J3P_obs[0].contour_point[0].y = 0.7;
	//test_data.J3P_obs[0].contour_point[1].x = 54.2;
	//test_data.J3P_obs[0].contour_point[1].y = -3.3;

	//test_data.J3P_obs[0].contour_point[2].x = 46.2;
	//test_data.J3P_obs[0].contour_point[2].y = 0.7;
	//test_data.J3P_obs[0].contour_point[3].x = 46.2;
	//test_data.J3P_obs[0].contour_point[3].y = -3.3;

	//左线
	test_data.J3P_LKA_Left_Lane.is_valid = true;
	test_data.J3P_LKA_Left_Lane.line_id = 1036;
	test_data.J3P_LKA_Left_Lane.line_conf = 91;
	test_data.J3P_LKA_Left_Lane.line_status = 2;
	test_data.J3P_LKA_Left_Lane.line_start = 2.62608;
	test_data.J3P_LKA_Left_Lane.line_end = 2.62608 + 24.42;
	test_data.J3P_LKA_Left_Lane.line_c0 = 2.40966;
	test_data.J3P_LKA_Left_Lane.line_c1 = -0.0381176;
	test_data.J3P_LKA_Left_Lane.line_c2 = -8.92262e-05;
	test_data.J3P_LKA_Left_Lane.line_c3 = -9.68069e-07;

	//右线
	test_data.J3P_LKA_Right_Lane.is_valid = true;
	test_data.J3P_LKA_Right_Lane.line_id = 1028;
	test_data.J3P_LKA_Right_Lane.line_conf = 89;
	test_data.J3P_LKA_Right_Lane.line_status = 2;
	test_data.J3P_LKA_Right_Lane.line_start = 0;
	test_data.J3P_LKA_Right_Lane.line_end = 0 + 40.9;
	test_data.J3P_LKA_Right_Lane.line_c0 = -1.84994;
	test_data.J3P_LKA_Right_Lane.line_c1 = -0.0261327;
	test_data.J3P_LKA_Right_Lane.line_c2 = -4.59907e-05;
	test_data.J3P_LKA_Right_Lane.line_c3 = -1.22425e-06;


	//freespace没有，上位机没有这个数据，先不填了

	//std::string save_xml_file_path = "data/test.xml";
	//j3p_and_file::saveJ3PerceptionToXML(test_data,save_xml_file_path);
	//std::cout << "save end" << std::endl;
	//
	//J3PerceptionData test_data2;
	//j3p_and_file::loadJ3PerceptionFromXML(test_data2, save_xml_file_path);
	//std::string save_xml_file_path2 = "data/test2.xml";
	//j3p_and_file::saveJ3PerceptionToXML(test_data2, save_xml_file_path2);


	//tinyxml2file::J3Serializer j3serializer;
	//j3serializer.Save(test_data, "data/test3.xml");


	std::cout << "start - save" << std::endl;
	tinyxml2_case02::savePerceptionDataToXML(test_data, "data/test3.xml");
	J3PerceptionData txml2_load;
	tinyxml2_case02::loadPerceptionDataFromXML(txml2_load, "data/test3.xml");
	tinyxml2_case02::savePerceptionDataToXML(txml2_load, "data/test4.xml");

}



int read_xml_file_and_draw_map() {

	//注意改成你对应的路径
	string root_path = "/home/gq/guoqian/PathPlanning/J3map/";
	string data_id = "0415-02";
	string xml_file = root_path + data_id + "/photo/";


	//step1 先扫描json文件，获取文件名和文件数量
	cout << "@yang: [step 1]: scan xml folder" << "\n";
	vector<string> all_file_names;
	string mea = xml_file;
	if (!fs::exists(mea)) {
		std::cerr << "目录不存在: " << mea << std::endl;
		return -1;
	}
	else if (!fs::is_directory(mea)) {
		std::cerr << "路径不是目录: " << mea << std::endl;
		return -1;
	}


	// 递归遍历目录（包含子目录）
	for (const auto& entry : fs::recursive_directory_iterator(
		mea,
		fs::directory_options::skip_permission_denied
	)) {
		if (entry.is_regular_file() && entry.path().extension() == ".xml") {
			// 获取不带后缀的文件名
			std::string filename = entry.path().stem().string();
			all_file_names.emplace_back(filename);
			//std::cout << filename << std::endl;
			
		}
	}

	//// 修改遍历代码，存储带路径的对象
	//std::vector<fs::directory_entry> entries;

	//for (const auto& entry : fs::directory_iterator(mea)) {
	//	if (entry.is_regular_file() && entry.path().extension() == ".xml") {
	//		entries.push_back(entry);
	//	}
	//}

	//// 按最后修改时间排序
	//std::sort(entries.begin(), entries.end(),
	//	[](const fs::directory_entry& a, const fs::directory_entry& b) {
	//		return a.last_write_time() < b.last_write_time();
	//	});

	//// 提取文件名
	//for (const auto& entry : entries) {
	//	all_file_names.emplace_back(entry.path().stem().string());
	//}



	cout << all_file_names.size() << " files found." << "\n";



	//step2 读取xml文件，解析数据，并绘制地图
	cout << "@yang: [step 2]: read and draw map" << "\n";
	string save_new_map = root_path + data_id + "/new_map/";
	fs::create_directories(save_new_map);//生成保存地图文件的文件夹
	for (int i = 0; i < all_file_names.size(); i++) {
		string read_xml_path = mea + all_file_names[i] + ".xml";
		string save_map_path = save_new_map + all_file_names[i] +".jpg";
		J3PerceptionData j3p_data;
		bool load_ret = tinyxml2_case02::loadPerceptionDataFromXML(j3p_data, read_xml_path);//加载xml文件为结构体变量
		if (load_ret) {
			Mat new_map;
			unsigned long time;
			//draw_map::draw_front_perception_img(&j3p_data, new_map, time);//把结构体变量画成地图mat，此函数参数使用默认地图参数

			//draw_map::draw_front_perception_bgr(j3p_data, new_map, time);
			std::vector<J3Obs> j3Obs;
			bool opt_ok=true;
			lidarInfo lidarinfo;
			draw_map::draw_front_perception_gray(j3p_data, new_map, time, j3Obs,opt_ok,lidarinfo);

			imwrite(save_map_path, new_map);//保存图
			cout << "i: " << i << " -> " << all_file_names[i] << endl;
			//imshow("new_map", new_map);
			//waitKey(0);
		}
		else {
			cout << "@j3p: load xml file failed: ->" << read_xml_path << endl;
		}

	}

	return 0;
}