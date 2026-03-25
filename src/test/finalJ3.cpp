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

#include <tinyspline_ros/tinysplinecpp.h>
#include "j3_perception_data_api.h"
#include "run_code.h"
#include "bezier.hpp"

#include <filesystem> // 要求C++17标准
#include <j3_tool.h>

namespace fs = std::filesystem;
PathOptimizationNS::State start_state, end_state, car_state, car_state_local;
bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;
std::vector<PathOptimizationNS::State> reference_path_plot;
PathOptimizationNS::ReferencePath reference_path_opt;
std::vector<cv::Point2f> referencePoint;
bool isFirst = true;
bool optok = true;
double targetSpeed = 4.16; // m/s pathPoints[endIndex].v
cv::Mat img_src;
using namespace cv;
using namespace std;

struct savePoseOri
{
    int id;
    double x;
    double y;
    double z;
    double heading;
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

double resolution = 0.1;
int width = 500;
int height = 500;
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
    int num = (endIndex - index) / 10;
    reference_path_plot.clear();
    referencePoint.clear();
    double refX, refY;
    for (size_t i = 0; i < endIndex - index + 1; i = i + num)
    {
        Point2d globalP, pathP;
        globalP.x = pathPoints[index + i][0] + thX;
        globalP.y = pathPoints[index + i][1] + thY;
        // std::cout<<i<<globalP.x<<","<<globalP.y<<std::endl;
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
        // double y = -xs / resolution + 250;
        // double x = -ys / resolution + 250;
        // cv::Point p(x, y);
        // std::cout << "reference_path_plot111111: " << x << "," << y << std::endl;
        // circle(img_src, p, 1, cv::Scalar(0, 0, 0), -1);
    }
    // imshow("im", img_src);
    // waitKey(0);

    double disX = refX - start_state.x;
    double disY = refY - start_state.y;
    for (int i = 0; i < referencePoint.size(); i++)
    {
        PathOptimizationNS::State reference_point;
        reference_point.x = referencePoint[i].x - disX + thX;
        reference_point.y = referencePoint[i].y - disY + thY;
        // reference_point.x = referencePoint[i].x;
        // reference_point.y = referencePoint[i].y;
        reference_path_plot.emplace_back(reference_point);
        reference_rcv = reference_path_plot.size() >= 6;
        // std::cout << "received a reference point" <<referencePoint[i]<< std::endl;
    }
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

double constrainAngle(double angle)
{
    if (angle > M_PI / 2)
    {
        angle -= M_PI / 2;
        return constrainAngle(angle);
    }
    else if (angle < -M_PI / 2)
    {
        angle += M_PI / 2;
        return constrainAngle(angle);
    }
    else
    {
        return angle;
    }
}

void geneatePathPoint(std::vector<PathOptimizationNS::State> resultPath)
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
    // vector<double> curvitys = calculate_curvature_difference(result_path);
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
        double theta = heading[i] + (car_state.heading - start_state.heading);
        std::cout << result_path[i].x << "," << result_path[i].y << "," << theta << std::endl;
    }
}

std::vector<std::vector<int>> convertGridMapToVector(const grid_map::GridMap &gridMap,
                                                     const std::string &layer = "obstacle")
{
    std::vector<std::vector<int>> result;

    // 获取网格尺寸
    const grid_map::Size size = gridMap.getSize();
    const int rows = size(0);
    const int cols = size(1);

    // 调整结果向量大小
    result.resize(rows, std::vector<int>(cols));

    // 遍历网格并填充数据
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            // 获取网格索引
            grid_map::Index index(i, j);

            // 获取该位置的值并转换为整数
            float value = gridMap.at(layer, index);
            result[i][j] = static_cast<int>(value);
        }
    }

    return result;
}

// 优先队列中的节点
struct NodeMap
{
    Point point;
    int cost;
    NodeMap(Point p, int c) : point(p), cost(c) {}
    // 重载<运算符用于优先队列
    bool operator<(const NodeMap &other) const
    {
        return cost > other.cost; // 最小堆
    }
};

// Dijkstra算法实现
vector<Point> dijkstra(const vector<vector<int>> &grid, Point start, Point goal)
{
    // 地图的行数和列数
    int rows = grid.size();
    if (rows == 0)
        return {};
    int cols = grid[0].size();

    // 定义四个移动方向: 上, 右, 下, 左
    const int dx[] = {-1, 0, 1, 0};
    const int dy[] = {0, 1, 0, -1};

    // 初始化距离矩阵
    vector<vector<int>> dist(rows, vector<int>(cols, INT_MAX));
    dist[start.x][start.y] = 0;

    // 初始化优先队列
    priority_queue<NodeMap> pq;
    pq.push(NodeMap(start, 0));

    // 记录路径
    vector<vector<Point>> came_from(rows, vector<Point>(cols, Point(-1, -1)));

    while (!pq.empty())
    {
        NodeMap current = pq.top();
        pq.pop();

        // 如果到达目标点，回溯路径
        if (current.point == goal)
        {
            vector<Point> path;
            Point p = goal;
            while (!(p == start))
            {
                path.push_back(p);
                p = came_from[p.x][p.y];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }

        // 检查当前点是否已经被处理过(可能有更短的路径已经找到)
        if (current.cost > dist[current.point.x][current.point.y])
        {
            continue;
        }

        // 遍历四个方向
        for (int i = 0; i < 4; ++i)
        {
            int nx = current.point.x + dx[i];
            int ny = current.point.y + dy[i];

            // 检查是否在地图范围内且不是障碍物(假设障碍物值为0)
            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && grid[nx][ny] != 0)
            {
                int new_cost = current.cost + grid[nx][ny]; // 假设grid存储移动成本

                // 如果找到更短的路径
                if (new_cost < dist[nx][ny])
                {
                    dist[nx][ny] = new_cost;
                    came_from[nx][ny] = current.point;
                    pq.push(NodeMap(Point(nx, ny), new_cost));
                }
            }
        }
    }

    // 没有找到路径
    return {};
}

// 打印地图和路径
void printGridWithPath(const vector<vector<int>> &grid, const vector<Point> &path)
{
    vector<vector<char>> display = vector<vector<char>>(grid.size(), vector<char>(grid[0].size(), '.'));

    // 标记障碍物
    for (int i = 0; i < grid.size(); ++i)
    {
        for (int j = 0; j < grid[i].size(); ++j)
        {
            if (grid[i][j] == 0)
            {
                display[i][j] = '#';
            }
        }
    }

    // 标记路径
    for (const auto &p : path)
    {
        display[p.x][p.y] = '*';
    }

    // 标记起点和终点
    if (!path.empty())
    {
        display[path.front().x][path.front().y] = 'S';
        display[path.back().x][path.back().y] = 'G';
    }

    // 打印
    for (const auto &row : display)
    {
        for (char c : row)
        {
            cout << c << ' ';
        }
        cout << endl;
    }
}

int main(int argc, char **argv)
{
    bool isTurn = false;
    clock_t time1 = clock();
    // 注意改成你对应的路径
    string root_path = "/home/gq/guoqian/PathPlanning/J3map/";
    string data_id = "0624-01";
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
        // i = 0;
        string read_xml_path = mea + all_file_names[i] + ".xml";
        string save_map_path = save_new_map + all_file_names[i] + ".jpg";
        J3PerceptionData j3p_data;
        bool load_ret = J3PerceptionDataXml::loadPerceptionDataFromXML(j3p_data, read_xml_path); // 加载xml文件为结构体变量
        if (load_ret)
        {

            Mat new_map;
            unsigned long time;
            // draw_map::draw_front_perception_img(&j3p_data, new_map, time);//把结构体变量画成地图mat，此函数参数使用默认地图参数

            // draw_map::draw_front_perception_bgr(j3p_data, new_map, time);
            std::vector<J3Obs> j3Obs;
            lidarInfo lidarinfo;
            {
                lidarinfo.fieldNum = 4;
                lidarinfo.lidarfield[0].pointNum = 4;
                lidarinfo.lidarfield[0].point[0].x = 8.66181;
                lidarinfo.lidarfield[0].point[0].y = 0.00660551;
                lidarinfo.lidarfield[0].point[1].x = 8.62751;
                lidarinfo.lidarfield[0].point[1].y = -0.424402;
                lidarinfo.lidarfield[0].point[2].x = 8.64303;
                lidarinfo.lidarfield[0].point[2].y = -0.250477;
                lidarinfo.lidarfield[0].point[3].x = 8.65386;
                lidarinfo.lidarfield[0].point[3].y = -0.0790366;
                lidarinfo.lidarfield[1].pointNum = 17;
                lidarinfo.lidarfield[1].point[0].x = 7.13148;
                lidarinfo.lidarfield[1].point[0].y = 1.63863;
                lidarinfo.lidarfield[1].point[1].x = 6.69991;
                lidarinfo.lidarfield[1].point[1].y = 1.63936;
                lidarinfo.lidarfield[1].point[2].x = 6.24122;
                lidarinfo.lidarfield[1].point[2].y = 1.67;
                lidarinfo.lidarfield[1].point[3].x = 5.8616;
                lidarinfo.lidarfield[1].point[3].y = 1.67804;
                lidarinfo.lidarfield[1].point[4].x = 5.50418;
                lidarinfo.lidarfield[1].point[4].y = 1.67707;
                lidarinfo.lidarfield[1].point[5].x = 5.20614;
                lidarinfo.lidarfield[1].point[5].y = 1.68809;
                lidarinfo.lidarfield[1].point[6].x = 4.75634;
                lidarinfo.lidarfield[1].point[6].y = 1.62515;
                lidarinfo.lidarfield[1].point[7].x = 4.45113;
                lidarinfo.lidarfield[1].point[7].y = 1.69528;
                lidarinfo.lidarfield[1].point[8].x = 4.43435;
                lidarinfo.lidarfield[1].point[8].y = 1.73254;
                lidarinfo.lidarfield[1].point[9].x = 4.71525;
                lidarinfo.lidarfield[1].point[9].y = 2.05062;
                lidarinfo.lidarfield[1].point[10].x = 4.83827;
                lidarinfo.lidarfield[1].point[10].y = 2.35373;
                lidarinfo.lidarfield[1].point[11].x = 4.73705;
                lidarinfo.lidarfield[1].point[11].y = 2.42125;
                lidarinfo.lidarfield[1].point[12].x = 4.73064;
                lidarinfo.lidarfield[1].point[12].y = 2.52034;
                lidarinfo.lidarfield[1].point[13].x = 4.7374;
                lidarinfo.lidarfield[1].point[13].y = 2.38912;
                lidarinfo.lidarfield[1].point[14].x = 4.7696;
                lidarinfo.lidarfield[1].point[14].y = 2.13778;
                lidarinfo.lidarfield[1].point[15].x = 4.61633;
                lidarinfo.lidarfield[1].point[15].y = 1.89355;
                lidarinfo.lidarfield[1].point[16].x = 4.56995;
                lidarinfo.lidarfield[1].point[16].y = 1.64659;
                lidarinfo.lidarfield[2].pointNum = 6;
                lidarinfo.lidarfield[2].point[0].x = 7.84757;
                lidarinfo.lidarfield[2].point[0].y = -6.8605;
                lidarinfo.lidarfield[2].point[1].x = 8.21475;
                lidarinfo.lidarfield[2].point[1].y = -6.88386;
                lidarinfo.lidarfield[2].point[2].x = 8.60295,
                lidarinfo.lidarfield[2].point[2].y = -6.91033;
                lidarinfo.lidarfield[2].point[3].x = 8.60295;
                lidarinfo.lidarfield[2].point[3].y = -6.91033;
                lidarinfo.lidarfield[2].point[4].x = 8.99419;
                lidarinfo.lidarfield[2].point[4].y = -6.91815;
                lidarinfo.lidarfield[2].point[5].x = 9.47805;
                lidarinfo.lidarfield[2].point[5].y = -7.01715;
                lidarinfo.lidarfield[2].point[6].x = 8.02765;
                lidarinfo.lidarfield[2].point[6].y = -6.87012;
                lidarinfo.lidarfield[3].pointNum = 6;
                lidarinfo.lidarfield[3].point[0].x = 10.0221;
                lidarinfo.lidarfield[3].point[0].y = -1.5099;
                lidarinfo.lidarfield[3].point[1].x = 9.97935;
                lidarinfo.lidarfield[3].point[1].y = -1.04207;
                lidarinfo.lidarfield[3].point[2].x = 9.94685;
                lidarinfo.lidarfield[3].point[2].y = -0.703565;
                lidarinfo.lidarfield[3].point[3].x = 9.95044;
                lidarinfo.lidarfield[3].point[3].y = -0.605761;
                lidarinfo.lidarfield[3].point[4].x = 10.0127;
                lidarinfo.lidarfield[3].point[4].y = -1.2761;
                lidarinfo.lidarfield[3].point[5].x = 9.99754;
                lidarinfo.lidarfield[3].point[5].y = -1.15867;
            }
            // lidar speed judge
            for (int i = 0; i < lidarinfo.fieldNum; i++)
            {
                if (lidarinfo.lidarfield[i].pointNum < 2 || lidarinfo.lidarfield[i].point[0].z < 0)
                {
                    continue;
                }
                for (int j = 0; j < lidarinfo.lidarfield[i].pointNum - 1; j++)
                {
                    if (abs(lidarinfo.lidarfield[i].point[j].y) < 2.0)
                    {
                        // std::cout << "lidarinfo is close to obstacle: " << i << j << lidarinfo.lidarfield[i].point[j].y << std::endl;
                    }
                }
            }
            // j3p_data.J3P_obs[1].obs_x = 46.2;
            // j3p_data.J3P_obs[1].obs_y = -1.3;
            // j3p_data.J3P_obs[1].obs_width = 4;
            // j3p_data.J3P_obs[1].obs_length = 8;
            draw_map::draw_front_perception_gray(j3p_data, new_map, time, j3Obs, optok, lidarinfo);
            // j3p_data.J3P_Freespace.contours_points[0].x = 0;
            // j3p_data.J3P_Freespace.contours_points[0].y = 40;
            // j3p_data.J3P_Freespace.contours_points[1].x = 100;
            // j3p_data.J3P_Freespace.contours_points[1].y = 40;
            // j3p_data.J3P_Freespace.contours_points[2].x = 100;
            // j3p_data.J3P_Freespace.contours_points[2].y = -40;
            // j3p_data.J3P_Freespace.contours_points[3].x = 0;
            // j3p_data.J3P_Freespace.contours_points[3].y = -40;
            // for (int i = 4; i < 64; i++)
            // {
            //     j3p_data.J3P_Freespace.contours_points[i].x = 0;
            //     j3p_data.J3P_Freespace.contours_points[i].y = -40;
            // }
            imwrite("/home/gq/guoqian/PathPlanning/Findpath/build/image.jpg", new_map); // 保存图
            // cout << "i: " << i << " -> " << all_file_names[i] << "obs nums: " << j3Obs.size() << endl;
            // new_map = cv::imread("/home/gq/guoqian/data/2/4/79J3map.jpg");
            // imshow("new_map", new_map);
            // waitKey(0);

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
                ifstream inFile("/home/gq/guoqian/data/2/save_slamGQ_2.txt", ios::in | ios::binary);

                while (inFile.read((char *)savep, sizeof(savePose)))
                {
                    Vec3d p;
                    p[0] = savep->x;
                    p[1] = savep->y;
                    p[2] = savep->theta + 1.57096;
                    pathPoints.push_back(p);
                    num++;
                    // std::cout << p[0]<<","<<p[1]<<","<<savep->theta+1.57 <<","<<savep->kappas<< std::endl;
                }
                delete savep;
            }

            // car current state pathPoint the last point is distance <8m the last path planning
            // double pathDistance = pow((car_state.x - pathPoints[num][0]), 2) + pow((car_state.y - pathPoints[num][1]), 2);
            // if (!isFirst && pathDistance < 65)
            // {
            //     std::cout << "the last path planning" << std::endl;
            // }
            double refery, referx;
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
                    double l = j3Obs[i].obs_length;
                    j3Obs[i].obs_width = l;
                    j3Obs[i].obs_length = w;
                }
                referx = j3Obs[i].obs_x - j3Obs[i].obs_length / 2;
                refery = abs(j3Obs[i].obs_y - j3Obs[i].obs_width / 2);
                if (j3Obs[i].obs_pose_type == 2) // 后中点
                {
                    referx = j3Obs[i].obs_x;
                    refery = abs(j3Obs[i].obs_y - j3Obs[i].obs_width / 2);
                }

                if (refery < 5)
                {
                    if (referx < 15)
                    {
                        targetSpeed = 0.83;
                    }
                }
                std::cout << "J3 Obs:" << j3Obs[i].obs_pose_type << "," << j3Obs[i].obs_x << "," << j3Obs[i].obs_y << "," << j3Obs[i].obs_width << "," << j3Obs[i].obs_length << std::endl;
            }
            cv::Mat image = new_map;
            vector<Vec3d> initPose;
            {
                initPose.push_back(Vec3d(26.1588, 2.78449, 0.385641));
                initPose.push_back(Vec3d(36.1774, 14.6338, 1.62011));
                initPose.push_back(Vec3d(31.7591, 39.0648, 1.76515));
                initPose.push_back(Vec3d(23.309, 67.6721, 1.76166));
                initPose.push_back(Vec3d(19.6372, 92.8514, 2.0377));
                initPose.push_back(Vec3d(1.835, 95.4297, 3.20857));
                initPose.push_back(Vec3d(-24.9407, 94.7851, 3.12162));
                initPose.push_back(Vec3d(-49.38, 94.2119, 3.14361));
                initPose.push_back(Vec3d(-65.5018, 88.3157, 4.3396));
                initPose.push_back(Vec3d(-64.2858, 63.8289, -1.49495));
                initPose.push_back(Vec3d(-59.4203, 5.0, -1.5349));
                initPose.push_back(Vec3d(-45.6349, -4.1439, -0.038));
                initPose.push_back(Vec3d(-17.2748, -2.026, 0.052));
                // test 2
                initPose.push_back(Vec3d(21.0745, 1.29319, 0.180739));
                initPose.push_back(Vec3d(28.0946, 3.7572, 0.465926));
                initPose.push_back(Vec3d(33.8142, 7.87746, 0.799459));
                initPose.push_back(Vec3d(36.1672, 12.7177, 1.41364));
                initPose.push_back(Vec3d(35.43, 20.7448, 1.68993));
                initPose.push_back(Vec3d(33.7558, 29.8651, 1.75572));
                initPose.push_back(Vec3d(31.4271, 40.4939, 1.7737));
                initPose.push_back(Vec3d(28.9024, 51.7013, 1.7477));
                initPose.push_back(Vec3d(26.3854, 62.8855, 1.77684));
                initPose.push_back(Vec3d(24.0518, 73.6597, 1.74682));
                initPose.push_back(Vec3d(21.7665, 84.7855, 1.76061));
                initPose.push_back(Vec3d(19.6372, 92.8514, 2.03777));
                initPose.push_back(Vec3d(15.0327, 96.6699, 2.8298));
                initPose.push_back(Vec3d(7.00684, 95.9084, 3.2773));
                initPose.push_back(Vec3d(-2.93662, 95.1086, 3.17729));
                initPose.push_back(Vec3d(-13.0734, 94.5302, 3.14309));
                initPose.push_back(Vec3d(-23.0506, 94.7596, 3.10417));
                initPose.push_back(Vec3d(-33.5341, 94.4193, 3.18881));
                initPose.push_back(Vec3d(-41.9472, 94.1901, 3.09701));
                initPose.push_back(Vec3d(-51.2404, 94.145, 3.16752));
                initPose.push_back(Vec3d(-58.3734, 93.5101, 3.26229));
                initPose.push_back(Vec3d(-64.4111, 90.3717, 4.04944));
                initPose.push_back(Vec3d(-66.027, 83.2561, -1.58222));
                initPose.push_back(Vec3d(-65.0923, 73.5133, -1.49774));
                initPose.push_back(Vec3d(-63.167, 51.1905, -1.50019));
                initPose.push_back(Vec3d(-62.0255, 39.5304, -1.46546));
                initPose.push_back(Vec3d(-61.002, 27.1586, -1.52567));
                initPose.push_back(Vec3d(-60.1042, 15.6363, -1.52689));
                initPose.push_back(Vec3d(-59.4203, 5.00661, -1.53492));
                initPose.push_back(Vec3d(-57.0254, -1.46051, -0.689307));
                initPose.push_back(Vec3d(-49.2219, -3.71811, -0.167105));
                initPose.push_back(Vec3d(-40.8505, -3.68502, 0.12646));
                initPose.push_back(Vec3d(-30.8939, -2.76402, 0.0245319));
            }

            for (int j = 0; j < initPose.size(); j++) // initPose.size()
            {
                // j = 16;
                car_state.x = initPose[j][0];         // initPose[j][0];       //  Y: T:-1.643736
                car_state.y = initPose[j][1];        // initPose[j][1];       // currentPose[j][1];              //-78.2019;//0.000;//-7.16728;
                car_state.heading = initPose[j][2]; // initPose[j][2]; // currentPose[j][2] + 1.5706; // 4.70266;//-0.000796446;//-1.31224;

                cv::Mat gray, binaryImage;
                // image = imread("/home/gq/guoqian/data/1/687J3map.jpg");
                if (image.channels() == 3)
                {
                    cv::cvtColor(image, gray, COLOR_BGR2GRAY);
                }
                else
                {
                    image.copyTo(gray);
                }
                threshold(gray, binaryImage, 120, 255, 0);
                vector<Point> points;
                double cropX;
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
                    // resize(img_src,img_src,Size(img_src.size().width*2,img_src.size().height*2));
                    // resolution = resolution/2;
                    height = img_src.size().height;
                    width = img_src.size().width;
                    cropX = bb.x;
                }
                // img_src = binaryImage; //(Range(0,500),Range(225,295));
                // imshow("iamge",img_src);
                // waitKey(0);
                // 7. 选取终点
                double maxDis = 0;
                int maxIndex = 32;
                for (int i = 0; i < 64; i++)
                {
                    double x = j3p_data.J3P_Freespace.contours_points[i].x;
                    if (x > maxDis)
                    {
                        maxDis = x;
                        maxIndex = i;
                    }
                }
                double targetFreespaceX = j3p_data.J3P_Freespace.contours_points[maxIndex].x;
                double targetFreespaceY = j3p_data.J3P_Freespace.contours_points[maxIndex].y;
                double targetDistance = (targetFreespaceX) * (targetFreespaceX); // + targetFreespaceY * targetFreespaceY;
                std::cout << "targetDistance: " << targetFreespaceX << std::endl;
                if (targetDistance > 1200)
                {
                    targetDistance = 35 * 35;
                }
                cv::Mat circleImage(img_src.size(), CV_8UC1, Scalar(0));
                double targetPixel = targetFreespaceX / resolution;
                vector<Point2d> centerPoint;
                double downpixel = 50;
                while (centerPoint.size() == 0)
                {
                    std::cout << "freesapce target pixel: " << targetPixel << "," << maxIndex << std::endl;
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
                if (centerPoint.size() > 0)
                {
                    double centerX = (height - centerPoint[0].y) * resolution;
                    if (sqrt(targetDistance) > centerX)
                    {
                        targetDistance = centerX * centerX;
                    }
                }
                clock_t time0 = clock();
                // Initialize grid map from image.
                // img_src = binaryImage;
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

                vector<vector<int>> gdMap = convertGridMapToVector(grid_map);

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

                std::cout << "targetDistance: " << targetDistance << std::endl;
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
                double thX = car_state.x - pathPoints[index][0];
                double thY = car_state.y - pathPoints[index][1];
                double thXL, thYL;
                if (abs(thX) > 0.5 || abs(thY) > 0.5)
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
                endpathPoint.x = pathPoints[endIndex][0];
                endpathPoint.y = pathPoints[endIndex][1];
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
                imagepathPoint.x = abs(endpathPoint_local.x - car_state_local.x) - height / 2 * resolution;
                imagepathPoint.y = endpathPoint_local.y - car_state_local.y;
                pixelpathPoint.x = width / 2 - imagepathPoint.y / resolution;
                pixelpathPoint.y = height / 2 - imagepathPoint.x / resolution;
                double minDis = 1000000;
                double x = (height / 2 - pixelpathPoint.y) * resolution;
                double y = (width / 2 - pixelpathPoint.x) * resolution;
                grid_map::Position pose(x, y);
                double angleCh = abs(pathPoints[endIndex][2] - pathPoints[index][2]);

                if (centerPoint.size() == 0)
                {
                    std::cout << "use mapping path curvity points1" << std::endl;
                    PathOptimizationNS::State state;
                    for (size_t i = index; i < endIndex - 3; i = i + 4)
                    {
                        state.x = thXL + pathPoints[i][0];
                        state.y = thYL + pathPoints[i][1];
                        state.heading = pathPoints[i][2];
                    }
                    continue;
                }
                // else if (angleCh > 0.3 || isTurn)
                // {
                //     std::cout << "use mapping path curvity points2" << std::endl;
                //     PathOptimizationNS::State state;
                //     for (size_t i = index; i < endIndex; i++)
                //     {
                //         state.x = pathPoints[i][0];
                //         state.y = pathPoints[i][1];
                //         state.heading = pathPoints[i][2];
                //     }
                // }

                // find the end point
                if (centerPoint.size() > 1)
                {
                    Point2d centerPointend;
                    for (int i = 0; i < centerPoint.size(); i++)
                    {
                        double distY = pow((centerPoint[i].x - pixelpathPoint.x), 2);
                        std::cout << "dist:" << distY << std::endl;
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
                    std::cout << "poseDistance: " << poseDistance << "poseCenDistance:" << poseCenDistance << "path_dis:" << path_dis << std::endl;

                    if (poseDistance > 0.5)
                    {
                        endCentrPoint = pixelpathPoint;
                    }
                    else if (path_dis < 2.0 && poseCenDistance > 0.5)
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
                    double dist = pow((centerPoint[0].x - pixelpathPoint.x), 2);
                    double distY = sqrt(pow((centerPoint[0].y - pixelpathPoint.y), 2)) * resolution;
                    double path_dis = sqrt(dist) * resolution;
                    std::cout << "centerPoint: " << centerPoint[0].x << "," << centerPoint[0].y << "," << distY << std::endl;
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
                // endCentrPoint = pixelpathPoint;
                std::cout << "carState: " << car_state.x << "," << car_state.y << "," << car_state.heading << std::endl;
                std::cout << "referPoint: " << referPoint << std::endl;
                std::cout << "endpathPoint: " << endpathPoint << std::endl;
                std::cout << "imagepathPoint: " << imagepathPoint << std::endl;
                std::cout << "pixelpathPoint: " << pixelpathPoint << std::endl;
                std::cout << "endCentrPoint: " << endCentrPoint << std::endl;
                // convert endCentrPoint to pixle point
                Point2d endpixelPoint;
                endpixelPoint.x = height / 2 - endCentrPoint.y;
                endpixelPoint.y = width / 2 - endCentrPoint.x;
                // set start state
                start_state.x = (-height / 2) * resolution;
                start_state.y = (width / 2 - (250.0 - cropX)) * resolution;
                // set end state
                end_state.x = endpixelPoint.x * resolution;
                end_state.y = endpixelPoint.y * resolution;
                // car_state.heading = -0.066;
                if (endIndex - index < 2)
                {
                    break;
                }

                int startX = -start_state.y / resolution + width / 2;
                int startY = -start_state.x / resolution + height / 2;
                int goalX = -end_state.y / resolution + width / 2;
                int goalY = -end_state.x / resolution + height / 2;
                Point start(startY - 5, startX);
                Point goal(goalY, goalX);

                vector<Point> path = dijkstra(gdMap, start, goal);
                double delta = 0;
                if (false) //(!path.empty() && path.size() > 55)
                {
                    reference_path_plot.clear();
                    for (int i = 3; i < path.size(); i = i + 10)
                    {
                        delta = delta + 0.01;
                        PathOptimizationNS::State reference_point;
                        reference_point.x = (height / 2 - path[i].x) * resolution;
                        reference_point.y = (width / 2 - path[i].y) * resolution + delta;
                        reference_path_plot.emplace_back(reference_point);
                        reference_rcv = reference_path_plot.size() >= 6;
                    }
                    cout << "Path found:" << endl;
                    // printGridWithPath(gdMap, path);
                    // cout << "\nPath coordinates:" << endl;
                    // for (const auto &p : path)
                    // {
                    //     cout << "(" << p.x << ", " << p.y << ") ";
                    // }
                    // cout << endl;
                }
                else
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
                        printf("1111111111111111\n");
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
                std::cout << "theAngle regulater angle :" << theAngle << "," << theAngle2 << "," << std::endl;
                std::cout << "turn start state: " << start_state.x << "," << start_state.y << "," << start_state.heading << std::endl;
                std::cout << "turn end state: " << end_state.x << "," << end_state.y << "," << end_state.heading << std::endl;

                cv::Mat img_src2(cv::Size(img_src.cols, img_src.rows), CV_8UC3, cv::Scalar(255, 255, 255));
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
                cv::Point p(endCentrPoint.x, endCentrPoint.y);
                circle(img_src, p, 5, cv::Scalar(0, 0, 0), -1);
                // cv::imshow("image", img_src);
                // cv::waitKey(0);
                // Calculate.
                std::vector<PathOptimizationNS::SlState> result_path;
                std::vector<PathOptimizationNS::State> smoothed_reference_path;

                if (reference_rcv)
                {
                    PathOptimizationNS::PathOptimizer path_optimizer(start_state, end_state, grid_map);
                    // calcualte distance from obstacle
                    for (int i = 0; i < reference_path_plot.size(); i++)
                    {
                        // std::cout << reference_path_plot[i].x << "," << reference_path_plot[i].y << std::endl;
                        double distance = path_optimizer.calDisObs(reference_path_plot[i].x, reference_path_plot[i].y);
                        std::cout << "reference path point distance: " << distance << std::endl;
                    }
                    bool opt_ok = path_optimizer.solve(reference_path_plot, &result_path);
                    // std::cout << "plan state: " << opt_ok << std::endl;
                    if (opt_ok)
                    {
                        std::cout << "ok!" << std::endl;
                    }
                    else
                    {
                        continue;
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
                // speed set

                bool downSpeed = false;
                bool addSpeed = true;
                double distanceFromTurn;
                if (distanceFromTurn < 5)
                {
                    targetSpeed = 0.83; // m/s
                }
                double resultLength = result_path.at(result_path.size() - 1).x + 25;
                if (resultLength < 10)
                {
                    targetSpeed = 0.83;
                }

                double angle = -car_state.heading;
                std::cout << "angle: " << angle << std::endl;
                double disX, disY;
                for (size_t i = 0; i < result_path.size() - 1; i = i + 2)
                {
                    const auto path_point = result_path.at(i);
                    disX = (path_point.x - start_state.x) * cos(angle) + (path_point.y - start_state.y) * sin(angle);
                    disY = -(path_point.x - start_state.x) * sin(angle) + (path_point.y - start_state.y) * cos(angle);
                    double x = car_state.x + disX;
                    double y = car_state.y + disY;
                    double heading = path_point.heading + (car_state.heading - start_state.heading);
                    std::cout << "result_path: " << x << "," << y << "," << heading << std::endl;
                }
                std::vector<double> x_set, y_set, s_set, heading;
                for (size_t i = 0; i != result_path.size(); ++i)
                {
                    x_set.push_back(result_path.at(i).x);
                    y_set.push_back(result_path.at(i).y);
                }
                // for (int i = 126; i < 229; i++)
                // {
                //     x_set.push_back(pathPoints[i][0]);
                //     y_set.push_back(pathPoints[i][1]);
                // }
                s_set.push_back(0);
                double tmp_s = 0;
                for (size_t i = 1; i != result_path.size(); ++i)
                {
                    double dis = sqrt(pow(result_path.at(i).x - result_path.at(i - 1).x, 2) + pow(result_path.at(i).y - result_path.at(i - 1).y, 2));
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
                    // std::cout << "result_path heading: " << h << std::endl;
                }
                for (size_t i = 0; i != result_path.size(); ++i)
                {
                    const auto path_point = result_path.at(i);
                    double y = -path_point.x / resolution + height / 2;
                    double x = -path_point.y / resolution + width / 2;
                    cv::Point p(x, y);
                    // std::cout << "result pixle: " << x << "," << y << std::endl;
                    circle(img_src, p, 1, cv::Scalar(0, 0, 0), -1);
                }
                // for (size_t i = 0; i != reference_path_plot.size(); ++i)
                // {
                //     grid_map::Position pose(reference_path_plot[i].x, reference_path_plot[i].y);
                //     // std::cout << "reference path obstacle distance: " << map.getObstacleDistance(pose) << std::endl;
                //     double y = -reference_path_plot[i].x / resolution + height / 2;
                //     double x = -reference_path_plot[i].y / resolution + width / 2;
                //     cv::Point p(x, y);
                //     // std::cout << "reference_path_plot: " << x << "," << y << std::endl;
                //     circle(img_src, p, 3, cv::Scalar(0, 0, 0), -1);
                // }
                isFirst = false;
                clock_t time2 = clock();
                cout << "LocalPathPlanning_time:" << double(time2 - time0) / 1000 << "ms" << std::endl;
                cv::imwrite("img.jpg", img_src);
                cv::imshow("image", img_src);
                cv::waitKey(0);
            }
        }
    }
}