#include "crc.h"
#include <VisPer_c.h>
#include <cstdint>
#include <cmath>
#include <algorithm>

extern "C" {

// 将 VisPer RAEB 结果映射为 ObjBusinessData_t 并发送
void ProcessRAEBAndSend(uint32_t can_id)
{
    VisPerRaebResult_C res;
    std::memset(&res, 0, sizeof(res));
    VisPer_GetResult_C("RAEB", &res);

    if (res.tracked_cuboids_rows <= 0 || res.track_info_rows <= 0) {
        return;
    }

    int rows = res.tracked_cuboids_rows;
    if (res.track_info_rows < rows) rows = res.track_info_rows;
    if (res.tracked_cuboids_vel_rows < rows) rows = res.tracked_cuboids_vel_rows;

    for (int i = 0; i < rows; ++i) {
        ObjBusinessData_t obj{};

        // Object_ID <- track_info[0]
        obj.Object_ID = static_cast<uint32_t>(res.track_info[i][0]);

        // Positions: tracked_cuboids [cx, cy, cz, l, w, h, conf, cls, theta_abs]
        // scale positions by 100 to preserve two decimals (same as main.cpp example)
        float cx = res.tracked_cuboids[i][0];
        float cy = res.tracked_cuboids[i][1];
        obj.Object_Position_X = static_cast<int32_t>(std::round(cx * 100.0f));
        obj.Object_Position_Y = static_cast<int32_t>(std::round(cy * 100.0f));

        // Velocities: tracked_cuboids_vel [Mode, MotionState, VelX, VelY]
        float velx = res.tracked_cuboids_vel[i][2];
        float vely = res.tracked_cuboids_vel[i][3];
        obj.Object_Velocity_X = static_cast<int32_t>(std::round(velx * 100.0f));
        obj.Object_Velocity_Y = static_cast<int32_t>(std::round(vely * 100.0f));

        // Heading: theta_abs (radians) -> degrees *100
        float theta = res.tracked_cuboids[i][8];
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
        float heading_deg = theta * 180.0f / static_cast<float>(M_PI);
        obj.Object_Heading = static_cast<int32_t>(std::round(heading_deg * 100.0f));

        // Width and Length: tracked_cuboids: l (idx3), w (idx4)
        float length = res.tracked_cuboids[i][3];
        float width = res.tracked_cuboids[i][4];
        obj.Object_Width = static_cast<int32_t>(std::round(width * 100.0f));
        obj.Object_Length = static_cast<int32_t>(std::round(length * 100.0f));

        // Class and confidence
        float cls = res.tracked_cuboids[i][7];
        float conf = res.tracked_cuboids[i][6];
        obj.Object_Class = static_cast<uint8_t>(std::round(cls));
        obj.Object_Confidence = static_cast<uint8_t>(std::clamp(static_cast<int>(std::round(conf * 100.0f)), 0, 100));

        // Age and Status from track_info: [TrackID, TrackState, TrackAge, Idx]
        int32_t age = res.track_info[i][2];
        int32_t state = res.track_info[i][1];
        obj.Object_Age = age;
        obj.Object_Status = static_cast<uint8_t>(state);

        // Object_Probability mapping per chosen policy:
        // age <= 0 -> 0
        // 1-3 -> 60
        // 4-5 -> 80
        // >5 -> 95
        uint8_t prob = 0;
        if (state == 2 || state == 3) {
            // Lost or Removed => probability 0
            prob = 0;
        } else {
            if (age <= 0) prob = 0;
            else if (age <= 3) prob = 60;
            else if (age <= 5) prob = 80;
            else prob = 95;
        }
        obj.Object_Probability = prob;

        // Pack and send
        CAN_SendObjMsg(can_id, &obj);
    }
}

} // extern "C"