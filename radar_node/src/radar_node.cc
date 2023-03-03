/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <opencv2/opencv.hpp>

#include "MR415Wrapper.h"


/* global definition */
ros::Publisher g_heartbeat_;

void heartbeat_process(){
    ROS_INFO("heartbeat_process launch");
    int32_t counter = 0;
    while(ros::ok()){
        std_msgs::Int32 msg;
        msg.data = counter;
        g_heartbeat_.publish(msg);
        ROS_INFO("heartbeat counter: %d", counter);
        std::chrono::milliseconds dura(5000);
        std::this_thread::sleep_for(dura);
        counter += 1;
    }
    ROS_INFO("heartbeat_process exit");
}

/* radar_node */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_radar_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    if(argc != 2)
    {
        printf("please intput: rosrun radar_node radar_node_bin [config file] \n");
        return 1;
    }

    std::string config_file_path = argv[1];
    ROS_INFO("config_file_path: %s\n", argv[1]);

    g_heartbeat_ = n.advertise<std_msgs::Int32>("radar_node_heartbeat", 10);
    std::thread heartbeat_thread{heartbeat_process};

    // readParameters(config_file);
    // estimator.setParameter();

    // g_param_loader_ = std::shared_ptr<CvParamLoader> (new CvParamLoader(config_file_path));

    std::unique_ptr<radar::RadarMR415> ptr_radar(new radar::RadarMR415("vcan0"));
    ptr_radar -> start_listening();
    ROS_WARN("waiting for socketcan msg");

    ros::spin();

    ptr_radar -> stop_listening();
    heartbeat_thread.join();
    return 0;
}
