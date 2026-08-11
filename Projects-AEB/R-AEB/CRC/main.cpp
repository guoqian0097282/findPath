#include "crc.h"

int main(void)
{
    // 示例：直接从 VisPer RAEB 获取结果并发送（需要链接 VisPer C ABI）
    // 发送到 CAN ID 0x18FF0000（示例）
    ProcessRAEBAndSend(0x18FF0000);
    return 0;
}


// 从auto result = VisPer_GetResult("RAEB");函数拿到结果，
// 根据/home/gq/guoqian/Projects-AEB/perception_data_from_LH.xlsx表格格式填写数据
// 其中
//     uint32_t    Object_ID; 对应track_info中的第一个元素
//     int32_t     Object_Position_X;对应tracked_cuboids中的第一个元素
//     int32_t     Object_Position_Y;对应tracked_cuboids中的第二个元素
//     int32_t     Object_Velocity_X;对应tracked_cuboids_vel中的第三个元素
//     int32_t     Object_Velocity_Y;对应tracked_cuboids_vel中的第四个元素
//     int32_t     Object_Heading;对应tracked_cuboids中的最后一个元素
//     int32_t     Object_Width;对应tracked_cuboids中的第五个元素
//     int32_t     Object_Length;对应tracked_cuboids中的第四个元素
//     uint8_t     Object_Class;对应tracked_cuboids中的第八个元素
//     uint8_t     Object_Confidence;对应tracked_cuboids中的第七个元素
//     uint8_t     Object_Probability;这个需要重新计算，当track_info中的第三个元素连续帧数超过3帧认为可信度80，连续帧数超过5帧可信度超过90相应的计算。丢失之后可信度0
//     int32_t     Object_Age;对应track_info中的第三个元素
//     uint8_t     Object_Status;对应track_info中的第二个元素
// 并根据已有的CRC计算公式，计算出CRC。
// 封装成一个新的结构体，使用C语言。包含表格中的所有数据。
// 相应的修改CRC文件夹中的代码