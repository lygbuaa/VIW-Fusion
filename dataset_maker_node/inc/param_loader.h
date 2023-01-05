#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "viwo_utils.h"

namespace data_maker{

class ParamLoader
{
public:
    static constexpr double PI_ = 3.141592741;
    static constexpr double DEG2RAD = 180.0 / PI_;

    std::string node_rootdir_;
    std::string output_root_path_;
    std::string output_json_file_;
    std::string output_svc_front_path_;
    std::string output_svc_left_path_;
    std::string output_svc_rear_path_;
    std::string output_svc_right_path_;
    std::string output_odom_path_;

    std::string svc_front_topic_;
    std::string svc_left_topic_;
    std::string svc_rear_topic_;
    std::string svc_right_topic_;
    std::string odom_topic_;

    int cv_png_compression_level_ = 0;
    int write_or_read_ = 0;
    double delay_odom_ = 0.0f;
    double delay_svc_front_ = 0.0f;
    double delay_svc_left_ = 0.0f;
    double delay_svc_rear_ = 0.0f;
    double delay_svc_right_ = 0.0f;

private:
    std::string yaml_path_;
    cv::FileStorage fs_;

public:
    ParamLoader(std::string yaml_path){
        yaml_path_ = yaml_path;
        fs_.open(yaml_path, cv::FileStorage::READ);
        if(!fs_.isOpened()){
            ROS_ERROR("open config file failed: %s\n", yaml_path.c_str());
        }
        assert(fs_.isOpened());
        load_params();
    }

    ~ParamLoader(){
        if(fs_.isOpened()){
            fs_.release();
        }
    }

    void load_params(){
        fs_["node_rootdir"] >> node_rootdir_;
        fs_["svc_front_topic"] >> svc_front_topic_;
        fs_["svc_left_topic"] >> svc_left_topic_;
        fs_["svc_rear_topic"] >> svc_rear_topic_;
        fs_["svc_right_topic"] >> svc_right_topic_;
        fs_["odom_topic"] >> odom_topic_;

        fs_["output_root_path"] >> output_root_path_;
        output_root_path_ = node_rootdir_ + "/" + output_root_path_;
        ViwoUtils::MakeDir(output_root_path_);

        fs_["output_json_file"] >> output_json_file_;
        output_json_file_ = output_root_path_ + "/" + output_json_file_;

        fs_["output_svc_front_path"] >> output_svc_front_path_;
        output_svc_front_path_ = output_root_path_ + "/" + output_svc_front_path_;
        ViwoUtils::MakeDir(output_svc_front_path_);

        fs_["output_svc_left_path"] >> output_svc_left_path_;
        output_svc_left_path_ = output_root_path_ + "/" + output_svc_left_path_;
        ViwoUtils::MakeDir(output_svc_left_path_);

        fs_["output_svc_rear_path"] >> output_svc_rear_path_;
        output_svc_rear_path_ = output_root_path_ + "/" + output_svc_rear_path_;
        ViwoUtils::MakeDir(output_svc_rear_path_);

        fs_["output_svc_right_path"] >> output_svc_right_path_;
        output_svc_right_path_ = output_root_path_ + "/" + output_svc_right_path_;
        ViwoUtils::MakeDir(output_svc_right_path_);

        fs_["output_odom_path"] >> output_odom_path_;
        output_odom_path_ = output_root_path_ + "/" + output_odom_path_;
        ViwoUtils::MakeDir(output_odom_path_);

        fs_["write_or_read"] >> write_or_read_;

        fs_["cv_png_compression_level"] >> cv_png_compression_level_;
        fs_["delay_odom"] >> delay_odom_;
        fs_["delay_svc_front"] >> delay_svc_front_;
        fs_["delay_svc_left"] >> delay_svc_left_;
        fs_["delay_svc_rear"] >> delay_svc_rear_;
        fs_["delay_svc_right"] >> delay_svc_right_;

        ROS_INFO("output_odom_path_: %s, odom_topic_: %s, cv_png_compression_level_: %d, delay_svc_front_: %.3f\n", \
            output_odom_path_.c_str(), odom_topic_.c_str(), cv_png_compression_level_, delay_svc_front_);
    }

};
}