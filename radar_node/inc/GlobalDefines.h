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
#define USBCAN_0_SOCKETCAN_1 0

#ifndef M_PI
#define M_PI 3.14159265f
#endif

/* defined canid */
constexpr static unsigned int MR415_HEADER_CANID_ = 0x500;
constexpr static unsigned int MR415_TARGET_A_CANID_ = 0x503;
constexpr static unsigned int MR415_TARGET_B_CANID_ = 0x504;

typedef struct can_frame CanFrameClassical_t;
typedef std::queue<CanFrameClassical_t> FrameList_t;

}