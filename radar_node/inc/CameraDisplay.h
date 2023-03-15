#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "viwo_utils.h"
#include "CvParamLoader.h"
#include "radar_node/RadarTarget.h"
#include "radar_node/RadarDetection.h"

namespace radar{

class CameraDisplay
{
private:
    std::shared_ptr<CvParamLoader> cpl_;
    ros::Publisher pub_image_front_;
    ros::Subscriber sub_mr415_detection_;
    radar_node::RadarDetection g_det_;
    std::mutex mr415_mutex_;
    cv::Mat img_;
    std::unique_ptr<std::thread> pworker_ = nullptr;
    int w_;
    int h_;

public:
    CameraDisplay(std::shared_ptr<CvParamLoader> cpl){
        cpl_ = cpl;
        w_ = 1280; //cpl_ -> image_width_;
        h_ = 720;  //cpl_ -> image_height_;
    }
    ~CameraDisplay(){}

    void init(ros::NodeHandle& nh){
        pub_image_front_ = nh.advertise<sensor_msgs::Image>("image_front_disp", 10);
        sub_mr415_detection_ = nh.subscribe("/Radar415Node/mr415_detection", 100, &CameraDisplay::mr415_detection_callback, this);
        img_ = cv::Mat(h_, w_, CV_8UC3, cv::Scalar(128, 128, 128));
        pworker_ = std::unique_ptr<std::thread> (new std::thread(&CameraDisplay::test_loop, this));
        pworker_ -> detach();
    }

    void mr415_detection_callback(const radar_node::RadarDetectionConstPtr &det_msg){
        std::unique_lock<std::mutex> lock(mr415_mutex_);
        g_det_.objs = det_msg->objs;
        g_det_.no_obj = det_msg->no_obj;
        ROS_INFO("recv mr415 targets: %d", g_det_.no_obj);
    }

    void test_loop(){
        while(true){
            img_ = cv::Mat(h_, w_, CV_8UC3, cv::Scalar(128, 128, 128));
            // world(radar) coordinate: x-forward, y-left, z-up
            // draw ref points
            for(float x=3.0f; x<3.1f; x+=1.0f){
                for(float y=-5.0f; y<5.1f; y+=1.0f){
                    for(float z=-2.0f; z<2.1f; z+=0.2f){
                        cv::Point2f p2f = cpl_->radar_to_image(x, y, z);
                        cv::Point center(p2f.x * w_, p2f.y * h_);
                        cv::Scalar color(0, 0, 255);
                        cv::circle(
                            img_,
                            center,
                            4, //radius
                            color,
                            4  //thickness
                        );
                    }
                }
            }

            // draw mr415 targets
            if(g_det_.no_obj > 0){
                std::unique_lock<std::mutex> lock(mr415_mutex_);
                char tmpstr[64] = {0};
                sprintf(tmpstr, "radar: %d", g_det_.no_obj);
                cv::Point org(10, 30);
                cv::Scalar yellow(0, 255, 255);
                cv::putText(img_, tmpstr, org, cv::FONT_HERSHEY_SIMPLEX, 1.0, yellow, 2);
                for(const radar_node::RadarTarget& obj : g_det_.objs){
                    cv::Point center(obj.ix * w_, obj.iy * h_);
                    cv::Scalar color(128, 255, 0);
                    cv::circle(
                        img_,
                        center,
                        8, //radius
                        color,
                        8  //thickness
                    );
                    memset(tmpstr, 0, 64);
                    float dist = sqrt(obj.px*obj.px + obj.py*obj.py);
                    sprintf(tmpstr, "%d: %.1fm", obj.id, dist);
                    cv::Point org(center.x-10, center.y+30);
                    cv::Scalar yellow(0, 255, 255);
                    cv::putText(img_, tmpstr, org, cv::FONT_HERSHEY_SIMPLEX, 1.0, yellow, 2);
                }
            }

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time::now();
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", img_).toImageMsg();
            pub_image_front_.publish(img_msg);   
            std::chrono::milliseconds dura(200);
        }
    }


};

}