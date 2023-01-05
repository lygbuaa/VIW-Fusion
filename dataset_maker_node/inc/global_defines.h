#pragma once

/* need c++17 support */
#include <any>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

namespace data_maker{

static constexpr float SYNC_THRESHOLD_SEC_ = 0.02f;
static constexpr int BUFFER_MAX_ = 20;

enum class SensorIndex_t : int{
    VOID = -1,
    SVC_FRONT = 0,
    SVC_LEFT = 1,
    SVC_REAR = 2,
    SVC_RIGHT = 3,
    ODOM = 4,
    MAX = 5,
    ALL = 6,
};

using SensorTimestampVector_t = std::vector<std::tuple<SensorIndex_t, double>>;
using ImageQueuePtr_t = std::shared_ptr<std::queue<sensor_msgs::ImageConstPtr>>;
using OdomQueuePtr_t = std::shared_ptr<std::queue<nav_msgs::OdometryConstPtr>>;
using SensorDataMap_t = std::map<SensorIndex_t, std::any>;

struct Odom_Carla_t{
    /* timestamp */
    double t = 0.0f;
    /* pose.pose.position */
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    /* pose.pose.orientation */
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 0.0f;
    /* twist.twist.linear */
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    /* twist.twist.angular */
    float wx = 0.0f;
    float wy = 0.0f;
    float wz = 0.0f;
    /* euler angles, in rad */
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    /* primal timestamp from rosbag */
    double primal_t = -1*DBL_MAX;

    void from_ros_odom(const nav_msgs::OdometryConstPtr &odom_msg){
        /* if image could take timestamp forward, primal_t is useless */
        // t = ros::Time::now().toSec();
        t = odom_msg->header.stamp.toSec();
        primal_t = odom_msg->header.stamp.toSec();
        x = odom_msg->pose.pose.position.x;
        y = odom_msg->pose.pose.position.y;
        z = odom_msg->pose.pose.position.z;
        qx = odom_msg->pose.pose.orientation.x;
        qy = odom_msg->pose.pose.orientation.y;
        qz = odom_msg->pose.pose.orientation.z;
        qw = odom_msg->pose.pose.orientation.w;
        vx = odom_msg->twist.twist.linear.x;
        vy = odom_msg->twist.twist.linear.y;
        vz = odom_msg->twist.twist.linear.z;
        wx = odom_msg->twist.twist.angular.x;
        wy = odom_msg->twist.twist.angular.y;
        wz = odom_msg->twist.twist.angular.z;
        roll = odom_msg->pose.covariance[0];
        pitch = odom_msg->pose.covariance[1];
        yaw = odom_msg->pose.covariance[2];
    }

    void to_ros_odom(nav_msgs::Odometry& odom) const {
        odom.header.stamp = ros::Time(t);
        odom.header.frame_id = "world";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = z;
        odom.pose.pose.orientation.x = qx;
        odom.pose.pose.orientation.y = qy;
        odom.pose.pose.orientation.z = qz;
        odom.pose.pose.orientation.w = qw;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.linear.z = vz;
        odom.twist.twist.angular.x = wx;
        odom.twist.twist.angular.y = wy;
        odom.twist.twist.angular.z = wz;

        odom.pose.covariance[0] = roll;
        odom.pose.covariance[1] = pitch;
        odom.pose.covariance[2] = yaw;
        odom.pose.covariance[3] = primal_t;
    }
};

struct SensorDataBatch_t{
    double t;
    cv::Mat img_svc_front;
    cv::Mat img_svc_left;
    cv::Mat img_svc_rear;
    cv::Mat img_svc_right;
    Odom_Carla_t odom;
};

}