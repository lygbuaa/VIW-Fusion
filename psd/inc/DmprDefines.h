#pragma once

#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <libgen.h>
#include <chrono>
#include <math.h>
#include "PreProcessor.h"
#include "viwo_utils.h"

namespace psdonnx
{
#ifndef M_PI
#define M_PI 3.14159265f
#endif

/* world coord fence */
constexpr static float CARLA_WX_FENCE_[2] = {267.0f, 304.0f};
constexpr static float CARLA_WY_FENCE_[2] = {183.0f, 240.0f};

/* model input width & height */
constexpr static int DMPR_W_ = 512;
constexpr static int DMPR_H_ = 512;

/* marking-point score thresh */
constexpr static float DMPR_THRESH_ = 0.8f;
/* totally 16*16 marking-points */
constexpr static size_t MP_ROW_ = 16;
constexpr static size_t MP_COL_ = 16;

/* pixel per meter */
constexpr static float PPM_ = 640.0f/18.0f;
/* length of parklots, in meter */
constexpr static float SLOT_L_M_ = 5.9f;
/* width of parklots, in meter */
constexpr static float SLOT_W_M_ = 3.8f;
/* length of parklots, in pixel */
constexpr static float SLOT_L_ = SLOT_L_M_ * PPM_;
/* width of parklots, in pixel */
constexpr static float SLOT_W_ = SLOT_W_M_ * PPM_;
/* if two point near than, treate as the same */
constexpr static float VERTEX_MATCH_TH_ = SLOT_W_ / 2.0f;
/* if two marking-point direction diff less than, put into the same cluster */
constexpr static float DIRECTION_CLUSTER_TH_ = M_PI / 20.0f;
/* if marking-point direction fall in, fix it */
constexpr static float DIRECTION_COVER_TH_ = M_PI / 4.0f;
/* if two branch direction diff less than, paired them */
constexpr static float BRANCH_PAIR_TH_ = M_PI / 8.0f;
/* iterate marking-points with quads */
constexpr static float QUADS_[9] = {-2*M_PI, -1.5*M_PI, -1*M_PI, -0.5*M_PI, 0.0, 0.5*M_PI, M_PI, 1.5*M_PI, 2*M_PI};

typedef struct{
    std::vector<float> score;
    std::vector<float> shape;
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> cos;
    std::vector<float> sin;
}OutputRaw_t;

typedef struct{
    std::string name;
    float dir = 0.0;
    float cos = 0.0;
    float sin = 0.0;
    bool paired = false;
    int p1x = 0;
    int p1y = 0;
    int p2x = 0;
    int p2y = 0;
    int p3x = 0;
    int p3y = 0;
}MPBranch_t;

typedef struct{
    int idx = -1;
    float score = 0.0f;
    float shape = 0.0f;
    char type = 'T';
    float direction = 0.0;
    float dir_quad = 0.0;
    float x = 0.0;
    float y = 0.0;
    /* world coordinate */
    float wx = 0.0;
    float wy = 0.0;
    int p0x = 0;
    int p0y = 0;
    std::unordered_map<std::string, MPBranch_t> brs;
}MarkingPoint_t;

typedef struct{
    /* image coordinate */
    int ix = 0;
    int iy = 0;
    /* world coordinate */
    float wx = 0.0;
    float wy = 0.0;
}Vertex_t;

typedef struct{
    int idx = -1;
    Vertex_t center;
    Vertex_t p0;
    Vertex_t p1;
    Vertex_t p2;
    Vertex_t p3;
}Parklot_t;

typedef struct{
    int idx = -1;
    std::string img_path;
    int w;
    int h;
    std::vector<Parklot_t> slots;
    std::vector<MarkingPoint_t> mps;
    std::vector<Vertex_t> virt_vertex;
}Detections_t;

}