#pragma once

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "SocketCanWrapper.h"
#include "UsbCanWrapper.h"
#include "viwo_utils.h"
#include "radar_node/RadarTarget.h"
#include "radar_node/RadarDetection.h"
#include "CvParamLoader.h"

namespace radar{


class RadarMR415
#if USBCAN_0_SOCKETCAN_1 == 0
    :public UsbCanClassical
#elif USBCAN_0_SOCKETCAN_1 == 1
    :public SocketCanClassical
#endif
{
private:
    volatile bool alive_ = false;
    std::unique_ptr<std::thread> pworker_ = nullptr;
    radar_node::RadarDetection g_det_front_;
    radar_node::RadarTarget g_target_front_;
    
    radar_node::RadarDetection g_det_front_left_;
    radar_node::RadarTarget g_target_front_left_;

    radar_node::RadarDetection g_det_front_right_;
    radar_node::RadarTarget g_target_front_right_;

    ros::Publisher pub_detection_;
    ros::Publisher pub_markers_;
    std::shared_ptr<CvParamLoader> cpl_;

public:
    RadarMR415(const std::string can_name)
#if USBCAN_0_SOCKETCAN_1 == 0
    :UsbCanClassical(can_name)
#elif USBCAN_0_SOCKETCAN_1 == 1
    :SocketCanClassical(can_name)
#endif
    {}

    ~RadarMR415(){}

    void init_params(std::shared_ptr<CvParamLoader> cpl){
        cpl_ = cpl;
    }

    /* only publish mr415_detection, sr439_detection will be fused */
    void start_listening(ros::NodeHandle& nh, const std::string det_topic="mr415_detection", const std::string marker_topic="mr415_markers"){
        alive_ = true;
        pub_detection_ = nh.advertise<radar_node::RadarDetection>(det_topic, 100);
        // pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>(marker_topic, 100);
        pworker_ = std::unique_ptr<std::thread> (new std::thread(&RadarMR415::recv_loop, this));
    }

    void stop_listening(){
        alive_ = false;
        pworker_ -> join();
    }

    /*
        input: g_det_front_, g_det_front_left_, g_det_front_right_
        output: g_det_front_
        fusion in the same recv thread, so we don't need mutex.
    */
    void try_fusion(){
        /* just use now() as timestamp_front */
        double ts_f = ros::Time::now().toSec();

        // if(g_det_front_.objs.size() > 0){
        //     ts_f = g_det_front_.header.stamp.toSec();
        // }

        const double ts_fl = g_det_front_left_.header.stamp.toSec();
        const double ts_fr = g_det_front_right_.header.stamp.toSec();

        if(fabs(ts_f - ts_fl) < 0.1f){
            /* fusion latest front && front_left */
            /* traverse evey sr439 target, transform into mr415 coord, then compute iou with mr415 target */
        }

        if(fabs(ts_f - ts_fr) < 0.1f){
            /* fusion latest front && front_right */
        }

    }

    /*
        0x8F >> 4 = 0x08
        0xFF >> 4 = 0x0F
    */
    void process_mr415_header(const CanFrameClassical_t& frame){
        /* publish detection of last cycle, filter empty no_obj in case of 0x503 comes first */
        try_fusion();
        if(g_det_front_.no_obj > 0){
            pub_detection_.publish(g_det_front_);
            ROS_INFO("[fusion] publish mr415_detection, no_obj = %d", g_det_front_.no_obj);
        }

        static uint64_t counter = 0;
        static double start_sec = ros::Time::now().toSec();
        // HANG_STOPWATCH();
        MR415Header_t frame_header;
        assert(frame.can_id == frame_header.canid);
        frame_header.no_obj = (int)(frame.data[0] >> 2);

        /* clear detection */
        g_det_front_.objs.clear();
        std_msgs::Header header;
        header.frame_id = "world";
        header.stamp = ros::Time(ros::Time::now().toSec());

        g_det_front_.header = header;
        g_det_front_.no_obj = frame_header.no_obj;

        ++counter;
        double now_sec = ros::Time::now().toSec();
        float fps = counter / (now_sec - start_sec);
        ROS_INFO("[mr415] header counter: %ld, fps: %.2f", counter, fps);
    }

    void process_mr415_target_a(const CanFrameClassical_t& frame){
        // HANG_STOPWATCH();
        MR415TargetA_t frame_target_a;
        frame_target_a.id = (int)(frame.data[6]);

        unsigned int tmp = ((frame.data[0]&0xff)<<4) + ((frame.data[1]&0xf0)>>4);
        frame_target_a.px = tmp * 0.125f;
        // ROS_INFO("[target_a] id = %d, px = %.2f (%d)", frame_target_a.id, frame_target_a.px, tmp);

        tmp &= 0x0;
        tmp = ((frame.data[1]&0x0f)<<8) + (frame.data[2]&0xff);
        frame_target_a.py = tmp * 0.125f - 128.0f;
        // ROS_INFO("[target_a] id = %d, py = %.2f (%d)", frame_target_a.id, frame_target_a.py, tmp);

        tmp &= 0x0;
        tmp = ((frame.data[3]&0xff)<<4) + ((frame.data[4]&0xf0)>>4);
        frame_target_a.vx = tmp * 0.05f - 102.0f;
        // ROS_INFO("[target_a] id = %d, vx = %.2f (%d)", frame_target_a.id, frame_target_a.vx, tmp);

        tmp &= 0x0;
        tmp = ((frame.data[4]&0x0f)<<8) + (frame.data[5]&0xff);
        frame_target_a.vy = tmp * 0.05f - 102.0f;
        // ROS_INFO("[target_a] id = %d, vy = %.2f (%d)", frame_target_a.id, frame_target_a.vy, tmp);

        cv::Point2f p2f = cpl_->radar_to_image(frame_target_a.px, frame_target_a.py, MR415_FAKE_TARGET_HEIGHT_);

        g_target_front_.id = frame_target_a.id;
        g_target_front_.px = frame_target_a.px;
        g_target_front_.py = frame_target_a.py;
        g_target_front_.vx = frame_target_a.vx;
        g_target_front_.vy = frame_target_a.vy;
        g_target_front_.ix = p2f.x;
        g_target_front_.iy = p2f.y;
        g_target_front_.domain = radar_node::RadarTarget::FRONT;

        /* ignore target_b, in case of message loss */
        if(is_valid_target(g_target_front_)){
            ROS_INFO("[mr415] target, id = %d, type = %d, px = %.2f, py = %.2f, vx = %.2f, vy = %.2f", \
                g_target_front_.id, g_target_front_.type, g_target_front_.px, g_target_front_.py, g_target_front_.vx, g_target_front_.vy);
            g_det_front_.objs.push_back(g_target_front_);
        }
    }

#if 0
    /* target_b is useless */
    void process_mr415_target_b(const CanFrameClassical_t& frame){
        // HANG_STOPWATCH();
        MR415TargetB_t frame_target_b;
        frame_target_b.id = (int)(frame.data[2]);
        /* make sure target_b comes with the same target_a */
        // assert(g_target_front_.id == frame_target_b.id);

        frame_target_b.type = (int)((frame.data[7]&0xfc)>>2);
        // ROS_INFO("[target_b] id = %d, type = %d", frame_target_b.id, frame_target_b.type);

        g_target_front_.type = frame_target_b.type;

        if(is_valid_target(g_target_front_)){
            ROS_INFO("add new target, id = %d, type = %d, px = %.2f, py = %.2f, vx = %.2f, vy = %.2f", \
                g_target_front_.id, g_target_front_.type, g_target_front_.px, g_target_front_.py, g_target_front_.vx, g_target_front_.vy);
            g_det_front_.objs.push_back(g_target_front_);
        }
    }
#endif

    bool is_valid_target(const radar_node::RadarTarget& target){
        /* filter distance */
        if(g_target_front_.px < 0.05f || g_target_front_.px > 15.0f){
            return false;
        }

        for(const radar_node::RadarTarget& obj : g_det_front_.objs){
            /* filter duplicate target */
            if(obj.id == target.id){
                return false;
            }
        }

        return true;
    }

    void process_sr439_header(const RadarDomain_t domain, const CanFrameClassical_t& frame){
        // HANG_STOPWATCH();
        static uint64_t counter = 0;
        static double start_sec = ros::Time::now().toSec();

        SR439Header_t sr439_header;
        sr439_header.no_obj = (int)(frame.data[0]);
        sr439_header.counter = ((frame.data[1]&0x0f)<<8) + (frame.data[2]&0xff);
        ROS_INFO("[sr439] header domain: %d, no_obj: %d, counter: %d", (int)domain, sr439_header.no_obj, sr439_header.counter);

        std_msgs::Header header;
        header.frame_id = "world";
        header.stamp = ros::Time(ros::Time::now().toSec());

        if(domain == RadarDomain_t::LEFT_FRONT){
            g_det_front_left_.objs.clear();
            g_det_front_left_.header = header;
            g_det_front_left_.no_obj = sr439_header.no_obj;

        }else if(domain == RadarDomain_t::RIGHT_FRONT){
            g_det_front_right_.objs.clear();
            g_det_front_right_.header = header;
            g_det_front_right_.no_obj = sr439_header.no_obj;

        }else{
            ROS_ERROR("invalid radar domain!");
            return;
        }

        ++counter;
        double now_sec = ros::Time::now().toSec();
        float fps = counter / (now_sec - start_sec);
        ROS_INFO("[sr439] header counter: %ld, fps: %.2f", counter, fps);
    }

    void process_sr439_obj_general(const RadarDomain_t domain, const CanFrameClassical_t& frame){
        // HANG_STOPWATCH();

        SR439ObjGeneral_t obj_gen;
        obj_gen.id = (int)(frame.data[0]);
        unsigned int tmp = 0x0;
        tmp = ((frame.data[1]&0xff)<<4) + ((frame.data[2]&0xf0)>>4);
        obj_gen.px = tmp*0.125f - 60.0f;
        tmp &= 0x0;
        tmp = ((frame.data[2]&0x0f)<<8) + (frame.data[3]&0xff);
        obj_gen.py = tmp*0.125f - 128.0f;

        ROS_INFO("[sr439] domain: %d, obj id: %d, px: %.2f, py: %.2f", (int)domain, obj_gen.id, obj_gen.px, obj_gen.py);

        if(domain == RadarDomain_t::LEFT_FRONT){
            g_target_front_left_.domain = radar_node::RadarTarget::LEFT_FRONT;
            g_target_front_left_.px = obj_gen.px;
            g_det_front_left_.objs.push_back(g_target_front_left_);
        }else if(domain == RadarDomain_t::RIGHT_FRONT){
            g_target_front_right_.domain = radar_node::RadarTarget::RIGHT_FRONT;
            g_target_front_right_.py = obj_gen.py;
            g_det_front_right_.objs.push_back(g_target_front_right_);
        }else{
            ROS_ERROR("[sr439] invalid radar domain!");
        }
    }

    void recv_loop(){
        ROS_INFO("start listening to MR415");
        FrameList_t frames;

        while(alive_){
            /* read frames */
            // assert(frames.empty());
            if(recv_frame(frames) < 0){
                // ROS_WARN("read frame error!");
                std::chrono::milliseconds dura(50);
                continue;
            }

            while(!frames.empty()){
                CanFrameClassical_t frame = frames.front();
                /* hack 0x505 ~ 0x552, remap canid to 0x503 and 0x504 */
                if(frame.can_id >= 0x505 && frame.can_id <= 0x552){
                    frame.can_id = 0x504 - frame.can_id % 2;
                }

                /* process frame */
                switch(frame.can_id){
                    case MR415_HEADER_CANID_:
                        process_mr415_header(frame);
                        break;
                    case MR415_TARGET_A_CANID_:
                        process_mr415_target_a(frame);
                        break;
                    case MR415_TARGET_B_CANID_:
                        // process_mr415_target_b(frame);
                        break;
                    case SR439_FL_HEADER_CANID_:
                        process_sr439_header(RadarDomain_t::LEFT_FRONT, frame);
                        break;
                    case SR439_FR_HEADER_CANID_:
                    case SR439_RL_HEADER_CANID_:
                    case SR439_RR_HEADER_CANID_:
                        process_sr439_header(RadarDomain_t::RIGHT_FRONT, frame);
                        break;
                    case SR439_FL_OBJ_GENERAL_CANID_:
                        process_sr439_obj_general(RadarDomain_t::LEFT_FRONT, frame);
                        break;
                    case SR439_FR_OBJ_GENERAL_CANID_:
                    case SR439_RL_OBJ_GENERAL_CANID_:
                    case SR439_RR_OBJ_GENERAL_CANID_:
                        process_sr439_obj_general(RadarDomain_t::RIGHT_FRONT, frame);
                        break;
                    default:
                        ROS_INFO("unsupported canid: 0x%03X", frame.can_id);
                }
                frames.pop();
            }

        }
        ROS_INFO("stop listening to MR415");
    }

private:

#if 0
    void publish_markers(){
        constexpr static float EGO_SIZE_X = 1.0f;
        constexpr static float EGO_SIZE_Y = 0.2f;
        constexpr static float EGO_SIZE_Z = 0.2f;
        constexpr static float TARGET_SIZE_X = 0.5f;
        constexpr static float TARGET_SIZE_Y = 0.5f;
        constexpr static float TARGET_SIZE_Z = 1.0f;

        visualization_msgs::MarkerArray markerArray_msg;
        std_msgs::Header header;
        /* map is the default frame_id */
        header.frame_id = "map";
        header.stamp = ros::Time(ros::Time::now().toSec());

        visualization_msgs::Marker marker;
        marker.header = header;
        marker.id = 0;
        // marker.text = std::to_string(obj.id);
        marker.ns = "radar_node";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        /* markers last for 1.2 sec */
        marker.lifetime = ros::Duration(0.0f);

        marker.scale.x = EGO_SIZE_X;
        marker.scale.y = EGO_SIZE_Y;
        marker.scale.z = EGO_SIZE_Z;
        marker.pose.position.x = 0.0f;
        marker.pose.position.y = 0.0f;
        marker.pose.position.z = EGO_SIZE_Z/2.0f;
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;

        markerArray_msg.markers.push_back(marker);

        for(const radar_node::RadarTarget& obj : g_det_front_.objs){
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.id = obj.id;
            // marker.text = std::to_string(obj.id);
            marker.ns = "radar_node";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
            /* markers last for 1.2 sec */
            marker.lifetime = ros::Duration(0.1f);

            marker.scale.x = TARGET_SIZE_X;
            marker.scale.y = TARGET_SIZE_Y;
            marker.scale.z = TARGET_SIZE_Z;
            marker.pose.position.x = obj.px;
            marker.pose.position.y = obj.py;
            marker.pose.position.z = TARGET_SIZE_Z/2.0f;
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;

            markerArray_msg.markers.push_back(marker);
        }
        pub_markers_.publish(markerArray_msg);
    }
#endif

};


}