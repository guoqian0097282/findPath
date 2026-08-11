#include "crc.h"

int main(void)
{
    // 示例：直接从 VisPer RAEB 获取结果并发送（需要链接 VisPer C ABI）
    // 发送到 CAN ID 0x18FF0000（示例）
    ProcessRAEBAndSend(0x18FF0000);
    return 0;
}
