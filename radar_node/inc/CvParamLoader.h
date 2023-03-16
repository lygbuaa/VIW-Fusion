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
    float pixel_size_um_;
    float lens_efl_mm_;

    cv::Mat k_front_;
    cv::Mat d_kb_;
    cv::Mat t_radar_to_camera_;
    cv::Mat euler_radar_to_camera_;
    cv::Mat m33_rot_r2c_;
    cv::Mat m31_trans_r2c_;
    cv::Mat m33_w2c_;

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
        fs_["pixel_size_um"] >> pixel_size_um_;
        fs_["lens_efl_mm"] >> lens_efl_mm_;
        ROS_INFO("pixel_size_um_: %.2f, lens_efl_mm_: %.2f", pixel_size_um_, lens_efl_mm_);

        fs_["k_front"] >> k_front_;
        ROS_INFO("k_front_: %s", ViwoUtils::CvMat2Str(k_front_).c_str());

        /* distortion kb model */
        fs_["d_kb"] >> d_kb_;
        ROS_INFO("d_kb_: %s", ViwoUtils::CvMat2Str(d_kb_).c_str());

        fs_["t_radar_to_camera"] >> m31_trans_r2c_;
        ROS_INFO("m31_trans_r2c_: %s", ViwoUtils::CvMat2Str(m31_trans_r2c_).c_str());

        fs_["euler_radar_to_camera"] >> euler_radar_to_camera_;
        ROS_INFO("euler_radar_to_camera_: %s", ViwoUtils::CvMat2Str(euler_radar_to_camera_).c_str());

        init_radar_param();
    }

    /* radar coordinate: x-front, y-left, z-up */
    void init_radar_param(){
        const double radar_pitch = euler_radar_to_camera_.at<double>(0, 0) / 57.3f;
        const double radar_roll = euler_radar_to_camera_.at<double>(1, 0) / 57.3f;
        const double radar_yaw = euler_radar_to_camera_.at<double>(2, 0) / 57.3f;
        m33_rot_r2c_ = euler_2_rot(radar_pitch, radar_roll, radar_yaw);
        ROS_INFO("m33_rot_r2c_: %s", ViwoUtils::CvMat2Str(m33_rot_r2c_).c_str());

        /* radar coord to camera coord */
        m33_w2c_ = cv::Mat::zeros(3, 3, CV_64F);
        m33_w2c_.at<double>(0, 1) = -1.0f;
        m33_w2c_.at<double>(1, 2) = -1.0f;
        m33_w2c_.at<double>(2, 0) = 1.0f;
        ROS_INFO("m33_w2c_: %s", ViwoUtils::CvMat2Str(m33_w2c_).c_str());
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

    /* apply kb model: rd = k1*sita + k3*sita**3 */
    cv::Mat distort(const cv::Mat& m31_cp){
        cv::Mat m31_cp_dist = cv::Mat::zeros(3, 1, CV_64F);
        const double& cx0 = m31_cp.at<double>(0, 0);
        const double& cy0 = m31_cp.at<double>(1, 0);
        const double& cz = m31_cp.at<double>(2, 0);
        // assert(cz > 0.0f);
        if(cz < 0.01f){
            ROS_WARN("invalid m31:cp: %s", ViwoUtils::CvMat2Str(m31_cp).c_str());
            return m31_cp_dist;
        }
        const double& k1 = d_kb_.at<double>(0, 0);
        const double& k3 = d_kb_.at<double>(1, 0);
        double r0 = sqrt(cx0*cx0 + cy0*cy0);
        double sita0 = atan2(r0, cz);
        double rd = k1*sita0 + k3*pow(sita0, 3);
        // double sita1 = atan2(rd, lens_efl_mm_);
        double r1 = cz * rd / lens_efl_mm_;
        double cx1 = cx0 * r1 / r0;
        double cy1 = cy0 * r1 / r0;

        /* p_dist still in camera coordinate, normalized */
        m31_cp_dist.at<double>(0, 0) = cx1 / cz;
        m31_cp_dist.at<double>(1, 0) = cy1 / cz;
        m31_cp_dist.at<double>(2, 0) = 1.0f;
        return m31_cp_dist;
    }

    /* convert 3d-world-point to 2d-image-pixel
    1. world(radar) coordinate: x-forward, y-left, z-up
    2. euler angles in right-hand, roll: +X, pitch: +Y, yaw: -Z
    3. camera coordinate: x-right, y-down, z-forward.
     */
    cv::Point2f radar_to_image(const float wx, const float wy, const float wz){
        cv::Point2f p2f(0, 0);
        cv::Mat m31_wp = cv::Mat::zeros(3, 1, CV_64F);
        m31_wp.at<double>(0, 0) = wx;
        m31_wp.at<double>(1, 0) = wy;
        m31_wp.at<double>(2, 0) = wz;

        cv::Mat m31_cp = m33_w2c_ * (m33_rot_r2c_ * m31_wp + m31_trans_r2c_);
        // ROS_INFO("m31_cp: %s", ViwoUtils::CvMat2Str(m31_cp).c_str());
        cv::Mat m31_cp_dist = distort(m31_cp);
        // ROS_INFO("m31_cp_dist: %s", ViwoUtils::CvMat2Str(m31_cp_dist).c_str());
        cv::Mat m31_ip = k_front_ * m31_cp_dist;
        // ROS_INFO("m31_ip: %s", ViwoUtils::CvMat2Str(m31_ip).c_str());

        p2f.x = m31_ip.at<double>(0, 0) / image_width_;
        p2f.y = m31_ip.at<double>(1, 0) / image_height_;
        return p2f;
    }

};

}