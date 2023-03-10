#pragma once

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>
#include "viwo_utils.h"

namespace radar{

class CvParamLoader
{
public:
    static constexpr double PI_ = 3.141592741;
    static constexpr double DEG2RAD = 180.0 / PI_;

    std::string project_rootdir_;
    std::string can_name_;
    std::string mr415_det_topic_;
    std::string mr415_marker_topic_;

    int image_width_;
    int image_height_;

    cv::Mat k_front_;
    cv::Mat d_kb_;
    cv::Mat t_radar_to_camera_;
    cv::Mat euler_radar_to_camera_;
    cv::Mat rot_r2c_;

private:
    std::string yaml_path_;
    cv::FileStorage fs_;

public:
    CvParamLoader(std::string yaml_path){
        yaml_path_ = yaml_path;
        fs_.open(yaml_path, cv::FileStorage::READ);
        if(!fs_.isOpened()){
            ROS_ERROR("open config file failed: %s\n", yaml_path.c_str());
        }
        assert(fs_.isOpened());
        load_params();
    }

    ~CvParamLoader(){
        if(fs_.isOpened()){
            fs_.release();
        }
    }

    void load_params(){
        fs_["project_rootdir"] >> project_rootdir_;
        fs_["can_name"] >> can_name_;
        fs_["mr415_det_topic"] >> mr415_det_topic_;
        fs_["mr415_marker_topic"] >> mr415_marker_topic_;

        ROS_INFO("project_rootdir: %s", project_rootdir_.c_str());
        ROS_INFO("can_name: %s", can_name_.c_str());
        ROS_INFO("mr415_det_topic: %s", mr415_det_topic_.c_str());
        ROS_INFO("mr415_marker_topic: %s", mr415_marker_topic_.c_str());

        fs_["image_height"] >> image_height_;
        fs_["image_width"] >> image_width_;

        fs_["k_front"] >> k_front_;
        ROS_INFO("k_front_: %s", ViwoUtils::CvMat2Str(k_front_).c_str());

        fs_["d_kb"] >> d_kb_;
        ROS_INFO("d_kb_: %s", ViwoUtils::CvMat2Str(d_kb_).c_str());

        fs_["t_radar_to_camera"] >> t_radar_to_camera_;
        ROS_INFO("t_radar_to_camera_: %s", ViwoUtils::CvMat2Str(t_radar_to_camera_).c_str());

        fs_["euler_radar_to_camera"] >> euler_radar_to_camera_;
        ROS_INFO("euler_radar_to_camera_: %s", ViwoUtils::CvMat2Str(euler_radar_to_camera_).c_str());

        init_radar_param();
    }

    /* radar coordinate: x-front, y-left, z-up */
    void init_radar_param(){
        const double radar_pitch = euler_radar_to_camera_.at<double>(0, 0) / 57.3f;
        const double radar_roll = euler_radar_to_camera_.at<double>(1, 0) / 57.3f;
        const double radar_yaw = euler_radar_to_camera_.at<double>(2, 0) / 57.3f;
        rot_r2c_ = euler_2_rot(radar_pitch, radar_roll, radar_yaw);
        ROS_INFO("r2c extrinsics: %s", ViwoUtils::CvMat2Str(rot_r2c_).c_str());
    }

    /*
        # roll: +X
        q_roll = np.quaternion(np.cos(roll/2), np.sin(roll/2), 0, 0)
        # pitch: +Y
        q_pitch = np.quaternion(np.cos(pitch/2), 0, np.sin(pitch/2), 0)
        # yaw: -Z
        q_yaw = np.quaternion(np.cos(-yaw/2), 0, 0, np.sin(-yaw/2))
        # the rotation sequence of body_to_cam is ZYX, yaw-roll-pitch
        # so the rotation sequence of cam_to_body is XYZ, roll-pitch-yaw
        q =  q_yaw * q_pitch * q_roll
    */
    cv::Mat euler_2_rot(double pitch, double roll, double yaw){
        cv::Mat rot;
        // cv::euler2rot() is unknown rotation-seq, so we use eigen
        // rot = cv::euler2rot(euler);
        Eigen::Quaterniond q = Eigen::AngleAxisd(-1*yaw, Eigen::Vector3d::UnitZ()) * 
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        cv::eigen2cv(q.toRotationMatrix(), rot);
        return rot;
    }    

};

}