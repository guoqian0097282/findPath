#ifndef PATH_PLAN_OUT_H
#define PATH_PLAN_OUT_H

// ##########################################################################################################

#ifdef __cplusplus
extern "C"
{
#endif

// ##########################################################################################################
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "DataType.hpp"

    // ##########################################################################################################
    int InitPathPlanning();
    int LocalPathPlanOut(T_FreeSpaceImg tFreeSpaceImg, T_J3ObsData *Obs, T_CarState tCarState,bool needPath, IpcSignal_Road_Det_TrackType *pTrack);

    int GetPathPlanAlgVersion(char pucAlgVersionArray[ARRAY_LEN]); 

#ifdef __cplusplus
};
#endif

// ##########################################################################################################

#endif // end of define ORB_SLAM_OUT_H