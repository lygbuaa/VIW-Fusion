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
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include "PsdWrapper.h"
#include "JsonDataset.h"
#include "CvParamLoader.h"
#include "ipm_composer.h"

IPM_PARAMS_t g_params_;
std::queue<sensor_msgs::ImuConstPtr> imu_buf_;
std::mutex m_buf_;
std::unique_ptr<IpmComposer> g_ipm_composer_;
std::unique_ptr<psdonnx::PsdWrapper> g_psd_wrapper_;
std::unique_ptr<psdonnx::JsonDataset> g_json_dataset_;
std::shared_ptr<CvParamLoader> g_param_loader_;
bool g_exit_ = false;

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
    std::unique_lock<std::mutex> lock(m_buf_);
    g_ipm_composer_ -> PushOdom(odom_msg);
    // double t = odom_msg->header.stamp.toSec();
    // double dx = odom_msg->twist.twist.linear.x;
    // double dy = odom_msg->twist.twist.linear.y;
    // double dz = odom_msg->twist.twist.linear.z;
    // double rx = odom_msg->twist.twist.angular.x;
    // double ry = odom_msg->twist.twist.angular.y;
    // double rz = odom_msg->twist.twist.angular.z;
    // Eigen::Vector3d vel(dx, dy, dz);
    // Eigen::Vector3d gyr(rx, ry, rz);

    /* use wheel pose as groundtruth */
    // auto tmp_Q = odom_msg->pose.pose.orientation;
    // auto tmp_T = odom_msg->pose.pose.position;
    // Eigen::Matrix<double, 7, 1> data;
    // data << tmp_Q.w, tmp_Q.x, tmp_Q.y, tmp_Q.z, tmp_T.x, tmp_T.y, tmp_T.z;
    // Eigen::Quaterniond quat(tmp_Q.w, tmp_Q.x, tmp_Q.y, tmp_Q.z);
    // Eigen::Vector3d loc(tmp_T.x, tmp_T.y, tmp_T.z);
    // g_ipm_composer_ -> PushOdom(quat, loc);

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
    while(!g_exit_)
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
                g_ipm_composer_ -> Compose(pis, true, g_param_loader_->output_path_);
                // g_ipm_composer_ -> Compose(pis, false);
		        // g_json_dataset_ -> feed(pis.time, pis.img_front, pis.img_left, pis.img_right, pis.img_rear);
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
    Eigen::Vector3d acc(dx, dy, dz);
    Eigen::Vector3d gyr(rx, ry, rz);
    // estimator.inputIMU(t, acc, gyr);
    return;
}

/* viwo_svc_node */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "apa_ros_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    if(argc != 2)
    {
        printf("please intput: rosrun apa_node apa_node_bin [config file] \n"
               "for example: rosrun apa apa_node_bin "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    std::string config_file_path = argv[1];
    ROS_INFO("config_file_path: %s\n", argv[1]);

    // readParameters(config_file);
    // estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    // registerPub(n);
    g_param_loader_ = std::shared_ptr<CvParamLoader> (new CvParamLoader(config_file_path));
    g_ipm_composer_ = std::unique_ptr<IpmComposer>(new IpmComposer());
    g_ipm_composer_ -> InitIpmPub(n);
    g_ipm_composer_ -> InitParams(g_param_loader_);
    g_ipm_composer_ -> ReadCarModel(g_param_loader_->car_top_image_path_);
    // g_ipm_composer_ -> SetHomography(HOMO_SVC_FRONT_, HOMO_SVC_LEFT_, HOMO_SVC_REAR_, HOMO_SVC_RIGHT_);
    // g_psd_wrapper_ = std::unique_ptr<psdonnx::PsdWrapper>(new psdonnx::PsdWrapper());
    // g_psd_wrapper_ -> load_model(PCR_MODEL_PATH, PSD_MODEL_PATH);
    g_json_dataset_ = std::unique_ptr<psdonnx::JsonDataset>(new psdonnx::JsonDataset(g_param_loader_->output_path_));

    ros::Subscriber sub_img_front = n.subscribe(g_param_loader_->image_front_topic_, 100, img_front_callback);
    ros::Subscriber sub_img_left = n.subscribe(g_param_loader_->image_left_topic_, 100, img_left_callback);
    ros::Subscriber sub_img_rear = n.subscribe(g_param_loader_->image_rear_topic_, 100, img_rear_callback);
    ros::Subscriber sub_img_right = n.subscribe(g_param_loader_->image_right_topic_, 100, img_right_callback);

    ROS_WARN("waiting for image and imu...");

    // ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_wheel = n.subscribe(g_param_loader_->wheel_topic_, 100, wheel_callback, ros::TransportHints().tcpNoDelay());
    std::thread sync_thread{sync_process};
    ros::spin();
    g_exit_ = true;
    sync_thread.join();
    return 0;
}
