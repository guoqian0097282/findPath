//
//#ifndef J3_DATA_H_
//#define J3_DATA_H_
////------对外接口和内部实现公用结构体定义 start------
//#pragma pack()
//
//#ifdef __cplusplus
//extern "C"
//{
//#endif // __cplusplus
//
///*
//* J3感知结果结构体注意事项：
//* 
//* 1.结构体成员的部分注释参考自协议文档的说明，其含义、范围、res等属性未能详尽测试，有疑问请联系相关人员！！！
//* 
//* 2.res的意思。res：0.0625，意味这该变量只能是0.0625的倍数，已测试了部分数据验证了此结论
//* 
//* 3.置信度说明。j3置信度的并不是常见的百分比。例如障碍物置信的范围是0-100，数值5并不意味着置信度只有5%，注释中给出了协议中对此置信度的说明，更详细的信息则需要额外测试
//*/
//
//
////用于表示点的结构体
//typedef struct {
//    float x;
//    float y;
//}J3Point2f;
//
//
////用于表示障碍物的历史轨迹点
//typedef struct {
//    float x;        //默认单位：米
//    float y;        //默认单位：米
//    //float z;       //默认单位：米
//    float theat;    //默认单位：degree，TODO：转成弧度
//    
//    float v_x;      //默认单位：米/s
//    float v_y;      //默认单位：米/s
//
//    unsigned int relative_time;     //相对时间，单位ms，即这个轨迹点相对于当前障碍物距离的时间
//
//}J3_obs_trajectory;
//
////障碍物OBS的类别枚举
//typedef enum {
//
//    OBS_CLASS_INVALID = 0,    // 未知类型
//    OBS_CLASS_VEHICLE,        // 车
//    OBS_CLASS_PEDESTRIAN,     // 行人
//    OBS_CLASS_CYCLIST,        // 骑行者
//    OBS_CLASS_BICYCLE         // 自行车
//
//} J3_obs_class;
//
//J3_obs_class obs_class_us2enum(unsigned short obs_class) {
//    switch (obs_class) {
//    case 0: return OBS_CLASS_INVALID;
//    case 1: return OBS_CLASS_VEHICLE;
//    case 2: return OBS_CLASS_PEDESTRIAN;
//    case 3: return OBS_CLASS_CYCLIST;
//    case 4: return OBS_CLASS_BICYCLE;
//    default: return OBS_CLASS_INVALID;
//    }
//}
//
////解析后的J3障碍物感知数据，单个障碍物的数据
//typedef struct {
//
//    unsigned short obs_id0;             //(忽略)障碍物的某个id，范围0~255，Obstacles.Obstacle.id。0:默认，1~200是车，201~255是行人。
//
//    unsigned int obs_id;                //障碍物id。范围0~4294967295，Obstacles.Obstacle.id。TODO：测试发现部分障碍物的id是0，与解析的pack不符合，待排查
//
//    unsigned short obs_conf;            //置信度。取值0~100。  根据CNN检测计算出的障碍物存在概率，当连续>6时表示障碍物目标高度一致。 补充：该值和常见的检测结果置信度不同，数值5不能直接理解为5%概率，j3算法内部可能有其他处理，
//    unsigned short obs_class;           //障碍物的类型，范围0~4。类别0=INVALID,1 = VEHICLE,2 = PEDESTRIAN,3 = CYCLIST,4 = BICYCLE。 补充：TODO 类别为0的数据之前我一般是直接忽略，后续会测试一下其坐标、chang、宽是不是有含义
//    J3_obs_class obs_enum_class;        //和obs_class一样，但使用枚举方式
//
//    unsigned short obs_pose_type;       //(reserve) 障碍物坐标类型，大致意思是pose的坐标是车头还是车尾，此数据J3有，但暂时未发送到vm，暂时忽略
//
//    float obs_x;                        //障碍物坐标x。范围-250~250,单位m。res：0.0625。障碍物vcs坐标x
//    float obs_y;                        //障碍物坐标y。范围-250~250,单位m。res：0.0625。障碍物vcs坐标y
//    float obs_angle;                    //障碍物角度。范围-327.68~327.67，单位degree。res：0.01。障碍物vcs坐标角度   TODO:需要改成弧度制
//    //float obs_z;                        //(reserve)障碍物坐标z。协议中并没有此数据，无意义或固定为0；
//
//    float obs_length;                   //障碍物长。范围0~25.6，单位m。障碍物长度。补充：测试发现行人的长和宽都是0
//    float obs_width;                    //障碍物宽。范围0~10.24，单位m。障碍物宽度.
//    float obs_height;                   //障碍物高。范围0~10.24，单位m。障碍物高度.补充：没使用过
//
//    //TODO：下面这些速度、加速度信息，J3有此数据但还没有发到vm，后续可补充协议信息或自己实现
//    float obs_x_rel_vel;                //(reserve)x方向相对自车的速度，单位m/s
//    float obs_y_rel_vel;                //(reserve)y方向相对自车的速度，单位m/s
//    //float obs_z_rel_vel;                //(reserve)z方向相对自车的速度，单位m/s
//
//    float obs_x_abs_vel;                //(reserve)x方向相对地面的速度，单位m/s
//    float obs_y_abs_vel;                //(reserve)y方向相对地面的速度，单位m/s
//    //float obs_z_abs_vel;                //(reserve)z方向相对地面的速度
//
//    float obs_x_rel_acc;                //(reserve)x方向相对自车的加速度，单位m/s^2
//    float obs_y_rel_acc;                //(reserve)y方向相对自车的加速度，单位m/s^2
//    //float obs_z_rel_acc;                //(reserve)z方向相对自车的加速度
//
//    float obs_x_abs_acc;                //(reserve)x方向相对地面的加速度，单位m/s^2
//    float obs_y_abs_acc;                //(reserve)y方向相对地面的加速度，单位m/s^2
//    //float obs_z_abs_acc;                //(reserve)z方向相对地面的加速度
//
//    //障碍物轮廓：目前测试发现的前视障碍物，只有行人和车辆，行人无轮廓，车辆轮廓则是通过坐标和长宽计算得到4个点的轮廓，由于此结构体要兼容C语言，因此暂定轮廓最多只有4个点。TODO：已实现待测试
//    unsigned int obs_contour_point_max_num = 4;   //轮廓的最大点个数
//    unsigned int obs_contour_point_vaild_num = 0;       //轮廓的有效点个数
//    J3Point2f contour_point[4];                         //障碍物轮廓点的坐标，单位m，暂定最多4个点。注意使用时，下标索引不能超过设定的最大长度
//
//    //障碍物的历史跟踪轨迹，TODO:此项信息J3没有，只能自己实现
//    unsigned int trajectory_point_max_num = 30;   //记录障碍物轨迹点的最大个数。暂定最多记录30个
//    unsigned int trajectory_point_vaild_num = 0;              //障碍物轨迹的有效点个数
//    float probability;                      //障碍物轨迹的置信度
//    J3_obs_trajectory trajectory[30];       //障碍物轨迹点的坐标，单位米。暂定最多30个。注意使用时，下标索引不能超过设定的最大长度
//    
//} J3Obs;
//
//
//
////车道线的line_status枚举类型
//typedef enum {
//
//    LINE_STATUS_UNKNOWN = 0,
//    LINE_STATUS_NEW,
//    LINE_STATUS_MEASURED_PARALLEL,
//    LINE_STATUS_PREDICTED_PARALLEL,
//    LINE_STATUS_MEASURED_UNPARALLEL,
//    LINE_STATUS_PREDICTED_UNPARALLEL,
//    LINE_STATUS_NOT_VALID
//
//} J3_line_status;
//
//
//inline J3_line_status line_status_us2enum(unsigned short line_status) {
//    switch (line_status) {
//    case 0: return LINE_STATUS_UNKNOWN;
//    case 1: return LINE_STATUS_NEW;
//    case 2: return LINE_STATUS_MEASURED_PARALLEL;
//    case 3: return LINE_STATUS_PREDICTED_PARALLEL;
//    case 4: return LINE_STATUS_MEASURED_UNPARALLEL;
//    case 5: return LINE_STATUS_PREDICTED_UNPARALLEL;
//    case 6: return LINE_STATUS_NOT_VALID;
//    default: return LINE_STATUS_NOT_VALID;
//    }
//}
//
//
////车道线的line_class枚举类型，详情看文档末尾的《line_class说明》
//typedef enum {
//
//    LINE_CLASS_UNKNOWN = 0,
//    LINE_CLASS_SOLID,
//    LINE_CLASS_ROAD_EDGE,
//    LINE_CLASS_DASHED,
//
//    LINE_CLASS_DOUBLE_LANE_LEFT_DASHED_RIGHT_SOLID,
//    LINE_CLASS_DOUBLE_LANE_LEFT_SOLID_RIGHT_DASHED,
//    LINE_CLASS_DOUBLE_LANE_DOUBLE_DASHED,
//    LINE_CLASS_DOUBLE_LANE_DOUBLE_SOLID,
//
//    LINE_CLASS_RESERVED,
//    LINE_CLASS_LINE_RAMP,
//    LINE_CLASS_SHADED_AREA,
//    LINE_CLASS_DECELERATION_SOLID_LINE,
//
//    LINE_CLASS_DECELERATION_DASHED_LINE,
//    LINE_CLASS_NO13,
//    LINE_CLASS_NO14,
//    LINE_CLASS_INVALID,
//
//    LINE_CLASS_CONE_DEFAULT,
//    LINE_CLASS_CONE_TRAFFIC_CONE,
//    LINE_CLASS_CONE_OTHER_CONE,
//
//    LINE_CLASS_EDGE_DEFAULT,
//    LINE_CLASS_EDGE_CURB,
//    LINE_CLASS_EDGE_OTHER,
//
//    LINE_CLASS_ILLEGAL
//
//} J3_line_class;
//
//
//inline J3_line_class line_class_us2enum(unsigned short line_class) {
//
//    switch (line_class) {
//    case 0: return LINE_CLASS_UNKNOWN;
//    case 1: return LINE_CLASS_SOLID;
//    case 2: return LINE_CLASS_ROAD_EDGE;
//    case 3: return LINE_CLASS_DASHED;
//
//    case 4: return LINE_CLASS_DOUBLE_LANE_LEFT_DASHED_RIGHT_SOLID;
//    case 5: return LINE_CLASS_DOUBLE_LANE_LEFT_SOLID_RIGHT_DASHED;
//    case 6: return LINE_CLASS_DOUBLE_LANE_DOUBLE_DASHED;
//    case 7: return LINE_CLASS_DOUBLE_LANE_DOUBLE_SOLID;
//
//    case 8: return LINE_CLASS_RESERVED;
//    case 9: return LINE_CLASS_LINE_RAMP;
//    case 10: return LINE_CLASS_SHADED_AREA;
//    case 11: return LINE_CLASS_DECELERATION_SOLID_LINE;
//
//    case 12: return LINE_CLASS_DECELERATION_DASHED_LINE;
//    case 13: return LINE_CLASS_NO13;
//    case 14: return LINE_CLASS_NO14;
//    case 15: return LINE_CLASS_INVALID;
//
//    case 16: return LINE_CLASS_CONE_DEFAULT;
//    case 17: return LINE_CLASS_CONE_TRAFFIC_CONE;
//    case 18: return LINE_CLASS_CONE_OTHER_CONE;
//
//    case 19: return LINE_CLASS_EDGE_DEFAULT;
//    case 20: return LINE_CLASS_EDGE_CURB;
//    case 21: return LINE_CLASS_EDGE_OTHER;
//
//    default: return LINE_CLASS_ILLEGAL;
//    }
//}
//
////解析后的J3车道线、锥桶连线、道路边界的感知结果，只表示一条线的结果
//typedef struct {
//    bool is_valid = false;              //非J3原始数据，解析数据时自己判断赋值，方便标记数据。标记数据是否有效。true表示有效，false表示无效。该值本质上是通过id、conf、status自己判断而来
//
//    unsigned short line_id;             //id。范围0~65535。车道线ID，有效值1~65535，0为默认
//    unsigned short line_conf;           //置信度。范围0~100，概率值。车道标记的置信度，根据车道跟踪状态计算，主要来自车道跟踪中心与车道原始解析轮廓的平均拟合误差。如果为90~100，则表示跟踪置信度高
//    
//    unsigned short line_status;         //范围0~7。0 = Unknown, 1 = New, 2 = Measured_Parallel,3 = Predicted_Parallel,4 = Measured_Unparallel,5 = Predicted_Unparallel,6 = Not valid
//    J3_line_status line_enum_status;    //和lane_status一样，但是使用枚举类型
//
//    unsigned short line_class;          //类别，内容太多，具体看文件末尾的《line_class说明》
//    J3_line_class line_enum_class;      //和line_class一样，但是使用枚举类型
//
//    float line_start;                   //范围0~200，单位m。vcs坐标系下车道线纵向距离近点，即x的起点。res = 0.1。
//    float line_end;                     //范围0~200，单位m。vcs坐标系下车道线纵向距离远点，即x的终点。res = 0.1。
//
//    //线的曲线方程系统，曲线方程为y = c0 + c1*x + c2*x*x + c3*x*x*x
//    short line_c0;                      //车道线结果：范围-0.1~10；锥桶连线和道路边界结果：-10~10； 单位m。res = 0.01
//    float line_c1;                      //范围-1~0.99902344，单位rad。res：0.00097656。
//    float line_c2;                      //范围-0.032~-0.032，单位1/meter。res：0.000000977。
//    float line_c3;                      //范围-0.00024414~0.00024414，单位1/meter^2。 res：0.0000000037252902985。
//} J3Line;
//
//
////freespace感知结果
//typedef struct {
//    J3Point2f contours_points[64];      //障碍物在vcs坐标系下的坐标，单位m。x取值范围[-102.4, 102.3]，y取值范围[-51.2, 51.1]。res：0.1；
//    unsigned int point_type[64];        //0=ignore,1 = car,2 = road delimiter,3 = pedestrain,4 = reserved,5 = reserved, 6 = reserved,7 = reserved
//} J3Freespace;
//
//
//
////单帧感知结果包含的所有数据
//typedef struct {
//    bool data_valid = false;                //标记这帧数据是否有效，true表示有效，false表示无效(无效一般是初始未赋值的结构体)
//
//    unsigned char   command;                //还不知道是什么数据
//    unsigned long   vm_muc_timeStamp;       //应该是VM收到can数据时附上的时间戳，TODO：还未测试验证过
//
//    unsigned long J3_image_time;    //J3的图像时间戳
//    unsigned int J3_frame_index;    //J3的图像帧号
//
//
//    //障碍物
//    int J3P_obs_num = 0;    //实际有效的障碍物个数，最大不超过20个
//    J3Obs J3P_obs[20];
//
//    //线、路沿数据。这些线数据是固定10条。    //TODO:后续会测试一下路沿的变量名称是不是对的上。
//    J3Line  J3P_LKA_Left_Lane;              //左侧车道线
//    J3Line  J3P_LKA_Right_Lane;             //右侧车道线
//    J3Line  J3P_Next_Left_Lane;             //左左车道线
//    J3Line  J3P_Next_Right_Lane;            //右右车道线
//    J3Line  J3P_Left_Outer_Lane;            //左分叉线
//    J3Line  J3P_Right_Outer_Lane;           //右分叉线
//    J3Line  J3P_Left_Cone_Road_Edge;        //左侧锥桶连线
//    J3Line  J3P_Right_Cone_Road_Edge;       //右侧锥桶连线
//    J3Line  J3P_Left_Road_Edge;             //左侧道路边界线
//    J3Line  J3P_Right_Road_Edge;            //右侧道路边界线
//
//    //freespace数据
//    J3Freespace J3P_Freespace;
//
//}J3PerceptionData;
//
//
////------对外接口和内部实现公用结构体 end------
//#ifdef __cplusplus
//}
//#endif // __cplusplus
//
//#endif // J3_DATA_H_
//
//
///* 《line_class说明》
//0 = Unknown                 //0~15是line专属
//1 = Solid
//2 = Road Edge
//3 = Dashed
//4 = Double Lane(Left Dashed, Right Solid)
//5 = Double Lane(Left Solid, Right Dashed)
//6 = Double Lane(Double Dashed)
//7 = Double Lane(Double Solid)
//8 = Reserved
//9 = Line Ramp
//10 = ShadedArea
//11 = DecelerationSolidLine
//12 = DecelerationDashedLine
//13 = No13 协议文档没写
//14 = No14 协议文档没写
//15 = Invalid
//
//16 = Cone_Default       //16、17、18是Cone专属
//17 = Cone_Traffic_Cone
//18 = Cone_Other_Cone
//
//19 = Edge_Default       //19、20、21是Edge专属
//20 = Edge_Curb
//21 = Edge_Other
//
//22 = illegal 协议中未说明的其他数值
//*/
