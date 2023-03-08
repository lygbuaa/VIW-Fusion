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
    }

};

}