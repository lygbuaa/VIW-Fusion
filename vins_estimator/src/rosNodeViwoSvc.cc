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
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include "PsdWrapper.h"
#include "JsonDataset.h"

Estimator estimator;

std::queue<sensor_msgs::ImuConstPtr> imu_buf_;
std::mutex m_buf_;
std::unique_ptr<IpmComposer> g_ipm_composer_;
std::unique_ptr<psdonnx::PsdWrapper> g_psd_wrapper_;
std::unique_ptr<psdonnx::JsonDataset> g_json_dataset_;

void img_front_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    std::unique_lock<std::mutex> lock(m_buf_);
    g_ipm_composer_ -> PushImage(SvcIndex_t::FRONT, img_msg);
}

void img_left_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    std::unique_lock<std::mutex> lock(m_buf_);
    g_ipm_composer_ -> PushImage(SvcIndex_t::LEFT, img_msg);
    // ROS_INFO("input frame svc left");
}

void img_rear_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    std::unique_lock<std::mutex> lock(m_buf_);
    g_ipm_composer_ -> PushImage(SvcIndex_t::REAR, img_msg);
    // ROS_INFO("input frame svc rear");
}

void img_right_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    std::unique_lock<std::mutex> lock(m_buf_);
    // g_ipm_composer_ -> PushImage(SvcIndex_t::RIGHT, g_ipm_composer_->CompensateImageMsg(img_msg, DELAY_SVC_RIGHT_S_));
    g_ipm_composer_ -> PushImage(SvcIndex_t::RIGHT, img_msg);
    // ROS_INFO("input frame svc right");
}

void wheel_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    double t = odom_msg->header.stamp.toSec();
    double dx = odom_msg->twist.twist.linear.x + ViwoUtils::WheelVelBias(VEL_N_wheel);
    double dy = odom_msg->twist.twist.linear.y + ViwoUtils::WheelVelBias(VEL_N_wheel);
    double dz = odom_msg->twist.twist.linear.z;
    double rx = odom_msg->twist.twist.angular.x;
    double ry = odom_msg->twist.twist.angular.y;
    double rz = odom_msg->twist.twist.angular.z + ViwoUtils::WheelVelBias(GYR_N_wheel);
    Vector3d vel(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputWheel(t, vel, gyr);

#if 1
    static Eigen::Matrix<double,7,1> init_pose;
    static bool init_done = false;
    /* use wheel pose as groundtruth */
    auto tmp_Q = odom_msg->pose.pose.orientation;
    auto tmp_t = odom_msg->pose.pose.position;
    Eigen::Matrix<double,7,1> data;
    data << tmp_Q.w, tmp_Q.x, tmp_Q.y, tmp_Q.z, tmp_t.x, tmp_t.y, tmp_t.z;
    if(!init_done){
        init_pose = data;
        init_done = true;
    }else{
        /* only correct translation initial pose */
        data[4] -= init_pose[4];
        data[5] -= init_pose[5];
        data[6] -= init_pose[6];
    }
    estimator.inputGroundtruth(t, data);
#endif

    return;
}

// extract images with same timestamp from svc topics
void sync_process()
{
    ROS_INFO("svc camera sync process launch");
    //g_json_dataset_ -> init_writer();
    cv::Mat image_front, image_left, image_rear, image_right, image_ipm;
    double last_time = 0.0f;
    SvcPairedImages_t pis;
    while(true)
    {
        std_msgs::Header header;
        double time = -1.0f;
        // m_buf.lock();
        if(!g_ipm_composer_->IsEmpty())
        {
            std::unique_lock<std::mutex> lock(m_buf_);
            SvcIndex_t idx = g_ipm_composer_ -> RemoveOutdateFrame();

            if(idx != SvcIndex_t::VOID){
                ROS_WARN("remove outdated frame and try again!");
                continue;
            }else{
                g_ipm_composer_ -> GetSyncImages(pis);
                time = pis.time;
                // if(time-last_time < 1.0f){
                //     g_ipm_composer_ -> PopImage(SvcIndex_t::ALL);
                //     continue;
                // }
                // g_ipm_composer_ -> Compose(pis, true, OUTPUT_FOLDER);
                g_ipm_composer_ -> Compose(pis, false);
		g_json_dataset_ -> feed(pis.time, pis.img_front, pis.img_left, pis.img_right, pis.img_rear);
                g_ipm_composer_ -> PopImage(SvcIndex_t::ALL);
                // ROS_INFO("get sync images, ts_front: %f", time);
            }
        }
        // m_buf.unlock();
        if(time > 0.0f){
            last_time = time;
            /* send paired images into estimator */
            //estimator.inputImage(time, image0, image1);
            time = -1.0f;
	        // psdonnx::Detections_t det;
	        // std::string psd_save_path = OUTPUT_FOLDER + "/" + std::to_string(pis.time) + "_psd.png";
	        //g_psd_wrapper_ -> run_model(pis.img_ipm, det, true, psd_save_path);
	        // g_psd_wrapper_ -> run_model(pis.img_ipm, det);
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
    ROS_INFO("svc camera sync process exit");
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

/* viwo_svc_node */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "viwo_svc");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    if(argc != 2)
    {
        printf("please intput: rosrun vins viwo_svc_node [config file] \n"
               "for example: rosrun vins viwo_svc_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);
    g_ipm_composer_ = std::unique_ptr<IpmComposer>(new IpmComposer());
    g_ipm_composer_ -> InitIpmPub(n);
    g_ipm_composer_ -> SetHomography(HOMO_SVC_FRONT_, HOMO_SVC_LEFT_, HOMO_SVC_REAR_, HOMO_SVC_RIGHT_);
    // g_psd_wrapper_ = std::unique_ptr<psdonnx::PsdWrapper>(new psdonnx::PsdWrapper());
    // g_psd_wrapper_ -> load_model(PCR_MODEL_PATH, PSD_MODEL_PATH);
    g_json_dataset_ = std::unique_ptr<psdonnx::JsonDataset>(new psdonnx::JsonDataset(OUTPUT_FOLDER));

    ros::Subscriber sub_img_front = n.subscribe(IMAGE_FRONT_TOPIC, 100, img_front_callback);
    ros::Subscriber sub_img_left = n.subscribe(IMAGE_LEFT_TOPIC, 100, img_left_callback);
    ros::Subscriber sub_img_rear = n.subscribe(IMAGE_REAR_TOPIC, 100, img_rear_callback);
    ros::Subscriber sub_img_right = n.subscribe(IMAGE_RIGHT_TOPIC, 100, img_right_callback);

    // ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_wheel = n.subscribe(WHEEL_TOPIC, 2000, wheel_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    std::thread sync_thread{sync_process};
    ros::spin();
    // 如果你的程序写了相关的消息订阅函数，那么程序在执行过程中，除了主程序以外，ROS还会自动在后台按照你规定的格式，接受订阅的消息，但是所接到的消息并不是
    // 立刻就被处理，而是必须要等到ros::spin()或ros::spinOnce()执行的时候才被调用，这就是消息回到函数的原理
    return 0;
}
