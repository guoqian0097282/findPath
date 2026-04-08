#ifndef DATATYPE_H
#define DATATYPE_H

#define ARRAY_LEN (256) // 数组长度
// input image type
typedef struct _FreeSpaceImg
{
    unsigned int uiHeight; // image height
    unsigned int uiWidth;  // image width
    unsigned char *pucImg; // buf, 单通道
} T_FreeSpaceImg;

typedef struct _CarState
{
    double dTheta;     // car(-PI/2 , PI/2)
    double dX;         // X position, 单位m
    double dY;         // Y position, 单位m
    double dTimeStamp; // timestamp
    double dV;  //speed
} T_CarState;

typedef struct _Obs
{
    float obs_x;
    float obs_y;
    float obs_length;
    float obs_width;
    float obs_angle;
}T_Obs;

typedef enum _PathPlanAlgErrorType
{
    PathPlan_NOCENTERPOINT = -9,    // can not find center point
    PathPlan_TEMPFILE_EMPTY = -8,   // temp file is empty
    PathPlan_INIT_FAILED = -7,      // init failed
    PathPlan_SYSTEM_NOT_READY = -6, // system is not ready
    PathPlan_NO_IMAGE_YET = -5,     // image is empty
    PathPlan_PARKING = -4,          // car state is empty
    PathPlan_LASTWRITE = -3,        // false: no path generate; true: write path to temp txt file
    PathPlan_FAILED = -2,           // failed
    PathPlan_SUCCESS = -1,          // success
    PathPlan_OK = 0,
    PathPlan_UPDATE = 1   // update finish
} E_PathPlanAlgErrorType; // Error type

// ##########################################################################################################
#endif // endof define DATATYPE_H
