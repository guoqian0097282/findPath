#include <stdint.h>
#include <string.h>

// 业务数据结构体（参与CRC计算的全部内容）
typedef __attribute__((packed)) struct
{
    uint32_t    Object_ID;
    int32_t     Object_Position_X;
    int32_t     Object_Position_Y;
    int32_t     Object_Velocity_X;
    int32_t     Object_Velocity_Y;
    int32_t     Object_Heading;
    int32_t     Object_Width;
    int32_t     Object_Length;
    uint8_t     Object_Class;
    uint8_t     Object_Confidence;
    uint8_t     Object_Probability;
    int32_t     Object_Age;
    uint8_t     Object_Status;
} ObjBusinessData_t;

// 完整发送数据包：业务数据 + CRC16校验码
typedef __attribute__((packed)) struct
{
    ObjBusinessData_t data;
    uint16_t crc_check; // CRC小端存储
} ObjCanPacket_t;

#define BUS_DATA_LEN    sizeof(ObjBusinessData_t)  // 41字节
#define PACK_TOTAL_LEN  sizeof(ObjCanPacket_t)      // 43字节

#ifdef __cplusplus
extern "C" {
#endif

// CAN 发送接口（在 CRC.cpp 中实现）
void CAN_SendObjMsg(uint32_t can_id, ObjBusinessData_t *send_data);
// 批量发送接口：一次发送多个ObjBusinessData_t
void CAN_SendObjMsgs(uint32_t can_id, ObjBusinessData_t *send_data, uint8_t count);

// Bridge: 从 VisPer RAEB 结果封装并发送 CAN 报文
// can_id: 发送的 CAN 标识符
void ProcessRAEBAndSend(uint32_t can_id);

#ifdef __cplusplus
}
#endif