#pragma once

#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <libgen.h>
#include <chrono>
#include <math.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "viwo_utils.h"

namespace radar
{
#define USBCAN_0_SOCKETCAN_1 1

#ifndef M_PI
#define M_PI 3.14159265f
#endif

enum class RadarDomain_t : int8_t{
    FUSION = 0,
    FRONT = 1,
    LEFT_FRONT = 2,
    RIGHT_FRONT = 3,
};

/* defined canid */
constexpr static unsigned int MR415_HEADER_CANID_ = 0x500;
constexpr static unsigned int MR415_TARGET_A_CANID_ = 0x503;
constexpr static unsigned int MR415_TARGET_B_CANID_ = 0x504;
constexpr static float MR415_FAKE_TARGET_HEIGHT_ = 1.0f;

constexpr static unsigned int SR439_FL_HEADER_CANID_ = 0x61A;
constexpr static unsigned int SR439_FR_HEADER_CANID_ = 0x62A;
constexpr static unsigned int SR439_RL_HEADER_CANID_ = 0x60A;
constexpr static unsigned int SR439_RR_HEADER_CANID_ = 0x63A;

constexpr static unsigned int SR439_FL_OBJ_GENERAL_CANID_ = 0x61B;
constexpr static unsigned int SR439_FR_OBJ_GENERAL_CANID_ = 0x62B;
constexpr static unsigned int SR439_RL_OBJ_GENERAL_CANID_ = 0x60B;
constexpr static unsigned int SR439_RR_OBJ_GENERAL_CANID_ = 0x63B;

constexpr static unsigned int SR439_FL_OBJ_QUALITY_CANID_ = 0x61C;
constexpr static unsigned int SR439_FR_OBJ_QUALITY_CANID_ = 0x62C;
constexpr static unsigned int SR439_RL_OBJ_QUALITY_CANID_ = 0x60C;
constexpr static unsigned int SR439_RR_OBJ_QUALITY_CANID_ = 0x63C;

constexpr static unsigned int SR439_FL_OBJ_EXTENDED_CANID_ = 0x61D;
constexpr static unsigned int SR439_FR_OBJ_EXTENDED_CANID_ = 0x62D;
constexpr static unsigned int SR439_RL_OBJ_EXTENDED_CANID_ = 0x60D;
constexpr static unsigned int SR439_RR_OBJ_EXTENDED_CANID_ = 0x63D;

typedef struct can_frame CanFrameClassical_t;
typedef std::queue<CanFrameClassical_t> FrameList_t;

struct MR415Header_t{
    const unsigned int canid = MR415_HEADER_CANID_;
    /* No_Obj: start=2, len=6 */
    int no_obj = -1;
    /* TunnelFlag: start=0, len=1 */
    int tunnel_flag = -1;
    /* CIPV_ID: start=16, len=8 */
    int cipv_id = -1;
    /* ACC_CIPV_ID: start=24, len=8 */
    int acc_cipv_id = -1;
    /* AEB_CIPV_ID: start=32, len=8 */
    int aeb_cipv_id = -1;
    /* Func_Status: start=8, len=8 */
    int func_status = -1;
    /* Radar_Frame: start=56, len=16 */
    int radar_frame = -1;
};

struct MR415TargetA_t{
    const unsigned int canid = MR415_TARGET_A_CANID_;
    /* Target1_MsgCnt_A: start=56, len=2 */
    int msgcnt_a = -1;
    /* Target1_ID: start=48, len=8 */
    int id = -1;
    /* Target1_Pos_X: start=12, len=12, Offset=0m, Resolution=0.125m */
    float px = 0.0f;
    /* Target1_Pos_Y: start=16, len=12, Offset=0m, Resolution=0.125m */
    float py = 0.0f;
    /* Target1_Vel_X: start=36, len=12, Offset=-102m/s, Resolution=0.05m/s */
    float vx = 0.0f;
    /* Target1_Vel_Y: start=40, len=12, Offset=-102m/s, Resolution=0.05m/s */
    float vy = 0.0f;
    /* Target1_CIPVFlag: start=59, len=1 */
    int cipv_flag = -1;
    /* Target1_ACC_CIPVFlag: start=63, len=1 */
    int acc_cipv_flag = -1;
    /* Target1_AEB_CIPVFlag: start=62, len=1 */
    int aeb_cipv_flag = -1;
};

struct MR415TargetB_t{
    const unsigned int canid = MR415_TARGET_B_CANID_;
    /* Target1_MsgCnt_B: start=56, len=2 */
    int msgcnt_b = -1;
    /* Target1_ID: start=16, len=8 */
    int id = -1;
    /* Target1_Accel_X: start=12, len=12, Offset=-40m/s^2, Resolution=0.04m/s^2 */
    float ax = 0.0f;
    /* Target1_MeasStat: start=29, len=3 */
    int meas_stat = -1;
    /* Target1_DynProp: start=24, len=3 */
    int dyn_prop = -1;
    /* Target1_ProbOfExist: start=48, len=2 */
    int prob_exist = -1;
    /* Target1_Type: start=58, len=6, 0=unknow, 1=pedestrian, 2=bike, 3=car, 4=truck */
    int type = -1;
};

struct SR439Header_t{
    unsigned int canid = SR439_FL_HEADER_CANID_;
    /* Obj_NofObject: start=0, len=8 */
    int no_obj = -1;
    /* Obj_MeasCounter: start=16, len=16 */
    int counter = -1;
    /* Obj_CIPVID: start=48, len=8 */
    int cipv_id = -1;
};

struct SR439ObjGeneral_t{
    unsigned int canid = SR439_FL_OBJ_GENERAL_CANID_;
    /* OBJ_ID, start=0, len=8 */
    int id = -1;
    /* Obj_DistLong, start=20, len=12, Offset=-60m, Resolution=0.125m */
    float px = 0.0f;
    /* Obj_DistLat, start=24, len=12, Offset=-128m, Resolution=0.125m */
    float py = 0.0f;
    /* Obj_VrelLong, start=44, len=12, Offset=-102m, Resolution=0.05m */
    float vx = 0.0f;
    /* Obj_VrelLat, start=48, len=12, Offset=-102m, Resolution=0.05m */
    float vy = 0.0f;
};

struct SR439ObjExtend_t{
    unsigned int canid = SR439_FL_OBJ_EXTENDED_CANID_;
    /* OBJ_ID, start=0, len=8 */
    int id = -1;
    /* Obj_ArelLon, start=21, len=11, Offset=-40, Resolution=0.04m/s^2 */
    float ax = 0.0f;
    /* Obj_ArelLat, start=26, len=11, Offset=-40, Resolution=0.04m/s^2 */
    float ay = 0.0f;
    /* Obj_Class, start=39, len=3, 0~7 */
    int type = -1;
    /* Obj_Width, start=40, len=5, Offset=0.0, Resolution=0.1 */
    float width = -1.0f;
    /* Obj_OrienAngle, start=45, len=10, Offset=-180, Resolution=0.4 */
    float orien_angle = 0.0f;
    /* Obj_Length, start=48, len=8, Offset=0, Resolution=0.1 */
    float length = -1.0f;
    /* Obj_Height, start=59, len=5, Offset=-10, Resolution=0.5 */
    float height = -1.0f;
};


}
