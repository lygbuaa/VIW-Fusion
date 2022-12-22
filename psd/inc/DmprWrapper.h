#pragma once

#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <libgen.h>
#include <chrono>
#include <ros/ros.h>
#include "tinyjson.hpp"
#include "PreProcessor.h"
#include "OnnxWrapper.h"
#include "viwo_utils.h"
#include "DmprDefines.h"

namespace psdonnx{
class DmprWrapper
{
public:
    std::unique_ptr<OnnxWrapper> onnx_wrapper_;
    OutputRaw_t model_output_raw_;

public:
    DmprWrapper(){
        onnx_wrapper_ = std::unique_ptr<OnnxWrapper>(new OnnxWrapper());
    }
    ~DmprWrapper(){}

    void test(){
        onnx_wrapper_ -> test_dmpr_model();
    }

    bool load_model(const std::string& dmpr_path){
        onnx_wrapper_ -> load_dmpr_model(dmpr_path);
        return true;
    }

    bool run_model(cv::Mat& img, Detections_t& det, bool debug_draw=false, const std::string& path=""){
        HANG_STOPWATCH();
        det.h = img.rows;
        det.w = img.cols;
        onnx_wrapper_ -> run_dmpr_model(img, model_output_raw_);

        retrieve_marking_points(det);
        fix_marking_points_direction(det);
        marking_point_add_branches(det);
        infer_parklots(det);

        if(debug_draw){
            draw_marking_points(img, det);
            draw_parklots(img, det);
        }

        if(path.size()>3){
            cv::imwrite(path, img);
        }
        return true;
    }

    int retrieve_marking_points(Detections_t& det){
        const size_t mp_num = model_output_raw_.score.size();
        assert(mp_num == MP_ROW_*MP_COL_);

        for(size_t i=0; i<MP_ROW_; ++i){
            for(size_t j=0; j<MP_COL_; ++j){
                size_t idx = i*MP_COL_+j;
                const float& score = model_output_raw_.score[idx];
                /* save marking-point */
                if(score > DMPR_THRESH_){
                    MarkingPoint_t mp;
                    mp.idx = idx;
                    mp.score = score;
                    mp.shape = model_output_raw_.shape[idx];
                    mp.type = (mp.shape>0.5f) ? 'L' : 'T';
                    mp.direction = atan2(model_output_raw_.sin[idx], model_output_raw_.cos[idx]);
                    mp.x = (j+model_output_raw_.x[idx]) / (float)MP_COL_;
                    mp.y = (i+model_output_raw_.y[idx]) / (float)MP_ROW_;
                    mp.p0x = int(mp.x*det.w-0.5f);
                    mp.p0y = int(mp.y*det.h-0.5f);
                    det.mps.emplace_back(mp);
                }
            }
        }
        ROS_INFO("retrieve_marking_points count: %d\n", det.mps.size());
        return det.mps.size();
    }

    void marking_point_add_branches(Detections_t& det){
        // HANG_STOPWATCH();
        det.virt_vertex.clear();
        for(MarkingPoint_t& mp : det.mps){
            /* add branch forward */
            MPBranch_t bf;
            bf.name = "forward";
            bf.dir = mp.direction;
            bf.cos = cos(bf.dir);
            bf.sin = sin(bf.dir);
            bf.p1x = mp.p0x + bf.cos*SLOT_L_;
            bf.p1y = mp.p0y + bf.sin*SLOT_L_;
            bf.paired = false;
            mp.brs["branch_forward"] = bf;
            Vertex_t vvtx;
            vvtx.ix = bf.p1x;
            vvtx.iy = bf.p1y;
            det.virt_vertex.push_back(vvtx);

            /* add branch right */
            MPBranch_t br;
            br.name = "right";
            br.dir = round_direction(mp.direction + M_PI/2);
            br.cos = cos(br.dir);
            br.sin = sin(br.dir);
            br.p2x = bf.p1x + br.cos*SLOT_W_;
            br.p2y = bf.p1y + br.sin*SLOT_W_;
            br.p3x = mp.p0x + br.cos*SLOT_W_;
            br.p3y = mp.p0y + br.sin*SLOT_W_;
            br.paired = false;
            mp.brs["branch_right"] = br;

            /* add branch left for 'T' */
            if(mp.type == 'T'){
                MPBranch_t bl;
                bl.name = "left";
                bl.dir = round_direction(mp.direction - M_PI/2);
                bl.cos = cos(bl.dir);
                bl.sin = sin(bl.dir);
                bl.p2x = bf.p1x + bl.cos*SLOT_W_;
                bl.p2y = bf.p1y + bl.sin*SLOT_W_;
                bl.p3x = mp.p0x + bl.cos*SLOT_W_;
                bl.p3y = mp.p0y + bl.sin*SLOT_W_;
                bl.paired = false;
                mp.brs["branch_left"] = bl;
            }
            // fprintf(stderr, "mp[%d] add branches: %d\n", mp.idx, mp.brs.size());
        }
    }

    /* search marking-point in radius: VERTEX_MATCH_TH_ */
    int find_marking_point_vertex(Detections_t& det, const int px, const int py){
        // HANG_STOPWATCH();
        float dist_min = DMPR_W_;
        int idx = -1;
        // for(MarkingPoint_t& mp : det.mps){
        // for(std::vector<MarkingPoint_t>::iterator it=det.mps.begin(); it!=det.mps.end(); ++it){
        for(int i=0; i<det.mps.size(); i++){
            const MarkingPoint_t& mp = det.mps[i];
            float dist = sqrt((px-mp.p0x)*(px-mp.p0x) + (py-mp.p0y)*(py-mp.p0y));
            if(dist < VERTEX_MATCH_TH_ && dist < dist_min){
                dist_min = dist;
                idx = i;
            }
        }
        return idx;
    }

    /* search virtual vertex added by branch_forward */
    int find_virtual_vertex(Detections_t& det, const int px, const int py){
        // HANG_STOPWATCH();
        float dist_min = DMPR_W_;
        int idx = -1;
        // for(Vertex_t& vvt : det.virt_vertex){
        // for(std::vector<Vertex_t>::iterator it=det.virt_vertex.begin(); it!=det.virt_vertex.end(); ++it){
        for(int i=0; i<det.virt_vertex.size(); ++i){
            const Vertex_t& vvt = det.virt_vertex[i];
            float dist = sqrt((px-vvt.ix)*(px-vvt.ix) + (py-vvt.iy)*(py-vvt.iy));
            if(dist < VERTEX_MATCH_TH_ && dist < dist_min){
                dist_min = dist;
                idx = i;
            }
        }
        return idx;
    }

    /* search mp.brs, pair to b */
    bool pair_marking_point_to_branch(MarkingPoint_t& mp, const MPBranch_t& b){
        // HANG_STOPWATCH();
        MPBranch_t& bf = mp.brs["branch_forward"];
        MPBranch_t& br = mp.brs["branch_right"];
        /* try pair branch_forward */
        float rad_diff = fabs(fabs(b.dir - bf.dir) - M_PI);
        // fprintf(stderr, "bf rad_diff: %f\n", rad_diff);
        if(rad_diff < BRANCH_PAIR_TH_){
            if(bf.paired){
                /* branch_forward already paired */
                return false;
            }else{
                /* if branch_forward match, this marking-point is KO */
                bf.paired = true;
                br.paired = true;
                if(mp.type == 'T'){
                    mp.brs["branch_left"].paired = true;
                }
                return true;
            }
        }

        /* try pair branch_right */
        rad_diff = fabs(fabs(b.dir - br.dir) - M_PI);
        // fprintf(stderr, "br rad_diff: %f\n", rad_diff);
        if(rad_diff < BRANCH_PAIR_TH_){
            if(br.paired){
                /* branch_right already paired */
                return false;
            }else{
                /* if branch_right match */
                br.paired = true;
                return true;
            }
        }

        if(mp.type == 'T'){
            MPBranch_t& bl = mp.brs["branch_left"];
            /* try pair branch_left */
            rad_diff = fabs(fabs(b.dir - bl.dir) - M_PI);
            // fprintf(stderr, "bl rad_diff: %f\n", rad_diff);
            if(rad_diff < BRANCH_PAIR_TH_){
                if(bl.paired){
                    /* branch_left already paired */
                    return false;
                }else{
                    /* if branch_left match */
                    bl.paired = true;
                    return true;
                }
            }
        }

        return false;
    }

    bool infer_mp_branch_forward(Detections_t& det, MPBranch_t& bf){
        // HANG_STOPWATCH();
        int idx = find_marking_point_vertex(det, bf.p1x, bf.p1y);
        if(idx >= 0){
            MarkingPoint_t& mp = det.mps[idx];
            bf.p1x = mp.p0x;
            bf.p1y = mp.p0y;
            return pair_marking_point_to_branch(mp, bf);
        }
        return true;
    }

    bool infer_mp_branch_right_left(Detections_t& det, MPBranch_t& brl){
        // HANG_STOPWATCH();
        bool p2_match = true;
        bool p3_match = true;

        int idx = find_marking_point_vertex(det, brl.p2x, brl.p2y);
        if(idx >= 0){
            MarkingPoint_t& mp = det.mps[idx];
            brl.p2x = mp.p0x;
            brl.p2y = mp.p0y;
            p2_match = pair_marking_point_to_branch(mp, brl);
        }else{
            idx = find_virtual_vertex(det, brl.p2x, brl.p2y);
            if(idx >= 0){
                Vertex_t& vvt = det.virt_vertex[idx];
                brl.p2x = vvt.ix;
                brl.p2y = vvt.iy;
            }
        }

        idx = find_marking_point_vertex(det, brl.p3x, brl.p3y);
        if(idx >= 0){
            MarkingPoint_t& mp = det.mps[idx];
            brl.p3x = mp.p0x;
            brl.p3y = mp.p0y;
            p3_match = pair_marking_point_to_branch(mp, brl);
        }

        return p2_match && p3_match;
    }


    void infer_parklots(Detections_t& det){
        // HANG_STOPWATCH();
        int counter = 0;
        det.slots.clear();
        for(MarkingPoint_t& mp : det.mps){
            /* only infer 'T' points */
            if(mp.type != 'T'){
                continue;
            }

            MPBranch_t& bf = mp.brs["branch_forward"];
            if(!bf.paired){
                infer_mp_branch_forward(det, bf);
            }

            MPBranch_t& br = mp.brs["branch_right"];
            if(!br.paired && infer_mp_branch_right_left(det, br)){
                Parklot_t sr;
                sr.idx = counter;
                sr.p0.ix = mp.p0x;
                sr.p0.iy = mp.p0y;
                sr.p1.ix = bf.p1x;
                sr.p1.iy = bf.p1y;
                sr.p2.ix = br.p2x;
                sr.p2.iy = br.p2y;
                sr.p3.ix = br.p3x;
                sr.p3.iy = br.p3y;
                sr.center.ix = (sr.p0.ix+sr.p1.ix+sr.p2.ix+sr.p3.ix) / 4;
                sr.center.iy = (sr.p0.iy+sr.p1.iy+sr.p2.iy+sr.p3.iy) / 4;
                det.slots.push_back(sr);
                counter += 1;
            }

            MPBranch_t& bl = mp.brs["branch_left"];
            if(!bl.paired && infer_mp_branch_right_left(det, bl)){
                Parklot_t sl;
                sl.idx = counter;
                sl.p0.ix = mp.p0x;
                sl.p0.iy = mp.p0y;
                sl.p1.ix = bf.p1x;
                sl.p1.iy = bf.p1y;
                sl.p2.ix = bl.p2x;
                sl.p2.iy = bl.p2y;
                sl.p3.ix = bl.p3x;
                sl.p3.iy = bl.p3y;
                sl.center.ix = (sl.p0.ix+sl.p1.ix+sl.p2.ix+sl.p3.ix) / 4;
                sl.center.iy = (sl.p0.iy+sl.p1.iy+sl.p2.iy+sl.p3.iy) / 4;
                det.slots.push_back(sl);
                counter += 1;
            }
        }
        ROS_INFO("total parklots: %d\n", counter);
    }

    void draw_marking_points(cv::Mat& img, const Detections_t& det){
        for(const MarkingPoint_t& mp : det.mps){
            /* red T, yellow L */
            cv::Scalar color(0, 0, 255);
            if(mp.type == 'L'){
                color = cv::Scalar(0, 255, 255);
            }
            cv::Point center(mp.p0x, mp.p0y);
            // char tmp[32] = {0};
            // sprintf(tmp, "%.0f - %.2f", 57.3*mp.direction, mp.score);
            // cv::putText(img, tmp, center, cv::FONT_HERSHEY_DUPLEX, 0.8, color, 1);
            cv::circle(
                img,
                center,
                8, //radius
                color,
                8   //thickness
            );
        }
    }

    void draw_parklots(cv::Mat& img, const Detections_t& det){
        for(const Parklot_t& slot : det.slots){
            std::vector<cv::Point> vps;
            vps.emplace_back(slot.p0.ix, slot.p0.iy);
            vps.emplace_back(slot.p1.ix, slot.p1.iy);
            vps.emplace_back(slot.p2.ix, slot.p2.iy);
            vps.emplace_back(slot.p3.ix, slot.p3.iy);
            cv::Scalar line_color(0, 255, 0);
            for(int i=0; i<4; i++){
                cv::line(img, vps[i], vps[(i+1)%4], line_color, 3, cv::LINE_8);
            }

            int cx = slot.center.ix;
            cx = (cx<20) ? 20 : cx;
            int cy = slot.center.iy;
            cy = (cy<20) ? 20 : cy;

            cv::Scalar text_color(0, 255, 255);
            char tmp[32] = {0};
            sprintf(tmp, "%d", slot.idx);
            cv::putText(img, tmp, cv::Point(cx, cy), cv::FONT_HERSHEY_DUPLEX, 1.0, text_color, 2);
        }
    }

    float round_direction(const float dir) const {
        float dirx = dir;
        if(dir > M_PI){
            dirx -= 2*M_PI;
        }else if(dir < -1*M_PI){
            dirx += 2*M_PI;
        }
        return dirx;
    }

    bool fix_marking_points_direction(Detections_t& det){
        std::vector<MarkingPoint_t>& mps = det.mps;
        std::sort(mps.begin(), mps.end(), \
            [](const MarkingPoint_t& mp1, const MarkingPoint_t& mp2) -> bool{ \
                return (mp1.direction < mp2.direction); \
            } \
        );

        float avg_direction = 0.0f;

        std::vector<MarkingPoint_t> cluster;
        const size_t mps_num = mps.size();
        for(MarkingPoint_t& mp0 : mps){
            cluster.clear();
            const float dir0 = mp0.direction;
            mp0.dir_quad = dir0;
            cluster.push_back(mp0);

            /* cluster around mp0 */
            for(MarkingPoint_t& mp : mps){
                if(mp.idx == mp0.idx){
                    continue;
                }
                for(int i=0; i<9; ++i){
                    const float quad = QUADS_[i];
                    const float dir_diff = fabs(mp.direction + quad - dir0);
                    if(dir_diff < DIRECTION_CLUSTER_TH_){
                        mp.dir_quad = mp.direction + quad;
                        cluster.push_back(mp);
                        break;
                    }
                }
            }

            /* check cluster result */
            if(cluster.size() < mps_num/2+1){
                /* mp0 is outlier, skip */
                continue;
            }else{
                // fprintf(stderr, "mp0[%d] cluster done, total %d\n", mp0.idx, cluster.size());
                avg_direction = 0.0f;
                for(const MarkingPoint_t& mpx : cluster){
                    avg_direction += mpx.dir_quad;
                }
                avg_direction /= cluster.size();
                /* fix marking-points direction */
                for(MarkingPoint_t& mp : det.mps){
                    float dir_diff_min = M_PI;
                    float quad = 0.0f;
                    for(int i=0; i<9; ++i){
                        const float qd = QUADS_[i];
                        float dir = avg_direction + qd;
                        float dir_diff = fabs(dir - mp.direction);
                        if(dir_diff < dir_diff_min){
                            dir_diff_min = dir_diff;
                            quad = qd;
                        }
                    }
                    if(dir_diff_min < DIRECTION_COVER_TH_){
                        float dir = round_direction(avg_direction + quad);
                        // fprintf(stderr, "fix mp[%d].direction from %.2f to %.2f\n", mp.idx, mp.direction, dir);
                        mp.direction = dir;
                    }
                }
                return true;
            }
        }

        fprintf(stderr, "mps direction cluster failed!");
        return false;
    }

    std::string gen_save_name(const std::string input_img_path){
        std::string save_name = basename(const_cast<char*>(input_img_path.c_str()));
        save_name += ".psd.png";
        return save_name;
    }

    static uint64_t current_micros() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::time_point_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now()).time_since_epoch()).count();
    }

};
}
