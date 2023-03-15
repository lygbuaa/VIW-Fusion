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
struct FrameHeader_t{
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

struct FrameTargetA_t{
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

struct FrameTargetB_t{
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
    radar_node::RadarDetection g_det_;
    radar_node::RadarTarget g_target_;
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

    void start_listening(ros::NodeHandle& nh, const std::string det_topic="mr415_detection", const std::string marker_topic="mr415_markers"){
        alive_ = true;
        pub_detection_ = nh.advertise<radar_node::RadarDetection>(det_topic, 100);
        pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>(marker_topic, 100);
        pworker_ = std::unique_ptr<std::thread> (new std::thread(&RadarMR415::recv_loop, this));
    }

    void stop_listening(){
        alive_ = false;
        pworker_ -> join();
    }

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

        for(const radar_node::RadarTarget& obj : g_det_.objs){
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

    /*
        0x8F >> 4 = 0x08
        0xFF >> 4 = 0x0F
    */
    void process_frame_header(const CanFrameClassical_t& frame){
        static uint64_t counter = 0;
        static double start_sec = ros::Time::now().toSec();
        // HANG_STOPWATCH();
        FrameHeader_t frame_header;
        assert(frame.can_id == frame_header.canid);
        frame_header.no_obj = (int)(frame.data[0] >> 2);

        /* publish detection of last cycle, filter empty no_obj in case of 0x503 comes first */
        if(g_det_.no_obj > 0){
            pub_detection_.publish(g_det_);
            ROS_INFO("publish mr415_detection, no_obj = %d", g_det_.no_obj);
            publish_markers();
        }

        /* clear detection */
        g_det_.objs.clear();
        std_msgs::Header header;
        header.frame_id = "world";
        header.stamp = ros::Time(ros::Time::now().toSec());

        g_det_.header = header;
        g_det_.no_obj = frame_header.no_obj;

        ++counter;
        double now_sec = ros::Time::now().toSec();
        float fps = counter / (now_sec - start_sec);
        ROS_INFO("recv 0x500 counter: %ld, fps: %.2f", counter, fps);
    }

    void process_frame_target_a(const CanFrameClassical_t& frame){
        // HANG_STOPWATCH();
        FrameTargetA_t frame_target_a;
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

        cv::Point2f p2f = cpl_->radar_to_image(frame_target_a.px, frame_target_a.py, 1.3f);

        g_target_.id = frame_target_a.id;
        g_target_.px = frame_target_a.px;
        g_target_.py = frame_target_a.py;
        g_target_.vx = frame_target_a.vx;
        g_target_.vy = frame_target_a.vy;
        g_target_.ix = p2f.x;
        g_target_.iy = p2f.y;
    }

    void process_frame_target_b(const CanFrameClassical_t& frame){
        // HANG_STOPWATCH();
        FrameTargetB_t frame_target_b;
        frame_target_b.id = (int)(frame.data[2]);
        /* make sure target_b comes with the same target_a */
        // assert(g_target_.id == frame_target_b.id);

        frame_target_b.type = (int)((frame.data[7]&0xfc)>>2);
        // ROS_INFO("[target_b] id = %d, type = %d", frame_target_b.id, frame_target_b.type);

        g_target_.type = frame_target_b.type;
        ROS_INFO("add new target, id = %d, type = %d, px = %.2f, py = %.2f, vx = %.2f, vy = %.2f", \
            g_target_.id, g_target_.type, g_target_.px, g_target_.py, g_target_.vx, g_target_.vy);
        g_det_.objs.push_back(g_target_);
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
                        process_frame_header(frame);
                        break;
                    case MR415_TARGET_A_CANID_:
                        process_frame_target_a(frame);
                        break;
                    case MR415_TARGET_B_CANID_:
                        process_frame_target_b(frame);
                        break;
                    default:
                        ROS_WARN("unsupported canid: 0x%03X", frame.can_id);
                }
                frames.pop();
            }

        }
        ROS_INFO("stop listening to MR415");
    }

};


}