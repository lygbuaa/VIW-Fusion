#pragma once

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>
#include "viwo_utils.h"

typedef struct{
    int BEV_H_ = 512;
    int BEV_W_ = 512;
    double BEV_XMAX_ = 16.0f;
    double BEV_YMAX_ = 16.0f;
    double PPM_ = 1.0f;

    cv::Mat HOMO_SVC_FRONT_;
    cv::Mat HOMO_SVC_LEFT_;
    cv::Mat HOMO_SVC_REAR_;
    cv::Mat HOMO_SVC_RIGHT_;

    cv::Mat R0_CAM_TO_BODY_;

    cv::Mat K_SVC_34_;
    cv::Mat T_SVC_FRONT_31_;
    cv::Mat T_SVC_LEFT_31_;
    cv::Mat T_SVC_REAR_31_;
    cv::Mat T_SVC_RIGHT_31_;
    cv::Mat EULER_SVC_FRONT_31_;
    cv::Mat EULER_SVC_LEFT_31_;
    cv::Mat EULER_SVC_REAR_31_;
    cv::Mat EULER_SVC_RIGHT_31_;

    cv::Mat INV_K_BEV_43_;
    cv::Mat E_BEV_C2B_44_;
    cv::Mat E_FRONT_B2C_44_;
    cv::Mat E_LEFT_B2C_44_;
    cv::Mat E_REAR_B2C_44_;
    cv::Mat E_RIGHT_B2C_44_;

    double SVC_FRONT_X0_ = 0.0f;
    double SVC_FRONT_Y0_ = 0.0f;
    double SVC_LEFT_X0_ = 0.0f;
    double SVC_LEFT_Y0_ = 0.0f;
    double SVC_REAR_X0_ = 0.0f;
    double SVC_REAR_Y0_ = 0.0f;
    double SVC_RIGHT_X0_ = 0.0f;
    double SVC_RIGHT_Y0_ = 0.0f;

    int CAR_MODEL_W_ = 0;
    int CAR_MODEL_H_ = 0;
    int CAR_MODEL_X0_ = 0;
    int CAR_MODEL_Y0_ = 0;
}IPM_PARAMS_t;

extern IPM_PARAMS_t g_params_;

class CvParamLoader
{
public:
    static constexpr double PI_ = 3.141592741;
    static constexpr double DEG2RAD = 180.0 / PI_;

    std::string project_rootdir_;
    std::string output_path_;
    std::string dmpr_model_path_;
    std::string psd_model_path_;
    std::string dataset_path_;
    std::string car_top_image_path_;

    std::string image_front_topic_;
    std::string image_left_topic_;
    std::string image_rear_topic_;
    std::string image_right_topic_;

    std::string wheel_topic_;

    double homo_update_rate_ = 1.0f;

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
        fs_["bev_h"] >> g_params_.BEV_H_;
        fs_["bev_w"] >> g_params_.BEV_W_;
        fs_["bev_xmax"] >> g_params_.BEV_XMAX_;
        fs_["bev_ymax"] >> g_params_.BEV_YMAX_;

        g_params_.PPM_ = g_params_.BEV_H_/g_params_.BEV_XMAX_;

        fs_["r0_cam_to_body"] >> g_params_.R0_CAM_TO_BODY_;

        fs_["k_svc_34"] >> g_params_.K_SVC_34_;
        fs_["t_svc_front_31"] >> g_params_.T_SVC_FRONT_31_;
        fs_["t_svc_left_31"] >> g_params_.T_SVC_LEFT_31_;
        fs_["t_svc_rear_31"] >> g_params_.T_SVC_REAR_31_;
        fs_["t_svc_right_31"] >> g_params_.T_SVC_RIGHT_31_;

        fs_["euler_svc_front_31"] >> g_params_.EULER_SVC_FRONT_31_;
        g_params_.EULER_SVC_FRONT_31_ /= DEG2RAD;
        fs_["euler_svc_left_31"] >> g_params_.EULER_SVC_LEFT_31_;
        g_params_.EULER_SVC_LEFT_31_ /= DEG2RAD;
        fs_["euler_svc_rear_31"] >> g_params_.EULER_SVC_REAR_31_;
        g_params_.EULER_SVC_REAR_31_ /= DEG2RAD;
        fs_["euler_svc_right_31"] >> g_params_.EULER_SVC_RIGHT_31_;
        g_params_.EULER_SVC_RIGHT_31_ /= DEG2RAD;

        // fs_["homo_svc_front"] >> g_params_.HOMO_SVC_FRONT_;
        // fs_["homo_svc_left"] >> g_params_.HOMO_SVC_LEFT_;
        // fs_["homo_svc_rear"] >> g_params_.HOMO_SVC_REAR_;
        // fs_["homo_svc_right"] >> g_params_.HOMO_SVC_RIGHT_;

        fs_["project_rootdir"] >> project_rootdir_;
        fs_["output_path"] >> output_path_;
        output_path_ = project_rootdir_ + output_path_;

        fs_["dmpr_model_path"] >> dmpr_model_path_;
        dmpr_model_path_ = project_rootdir_ + dmpr_model_path_;

        fs_["dataset_path"] >> dataset_path_;
        dataset_path_ = project_rootdir_ + dataset_path_;

        fs_["car_top_image_path"] >> car_top_image_path_;
        car_top_image_path_ = project_rootdir_ + car_top_image_path_;

        ROS_INFO("dataset_path_: %s, output_path_: %s, dmpr_model_path_: %s\n", \
            dataset_path_.c_str(), output_path_.c_str(), dmpr_model_path_.c_str());

        fs_["image_front_topic"] >> image_front_topic_;
        fs_["image_left_topic"] >> image_left_topic_;
        fs_["image_rear_topic"] >> image_rear_topic_;
        fs_["image_right_topic"] >> image_right_topic_;
        fs_["wheel_topic"] >> wheel_topic_;
        ROS_INFO("image_front_topic: %s, image_left_topic: %s, image_rear_topic: %s, image_right_topic: %s\n", \
            image_front_topic_.c_str(), image_left_topic_.c_str(), image_rear_topic_.c_str(), image_right_topic_.c_str());

        calc_cam_center();
        init_homo();
        update_homo(0.0, 0.0, 0.0);
    }

    void calc_cam_center(){
        g_params_.SVC_FRONT_X0_ = g_params_.BEV_W_/2.0 + g_params_.PPM_ * g_params_.T_SVC_FRONT_31_.at<double>(1, 0);
        g_params_.SVC_FRONT_Y0_ = g_params_.BEV_H_/2.0 - g_params_.PPM_ * g_params_.T_SVC_FRONT_31_.at<double>(0, 0);
        g_params_.SVC_LEFT_X0_ = g_params_.BEV_W_/2.0 + g_params_.PPM_ * g_params_.T_SVC_LEFT_31_.at<double>(1, 0);
        g_params_.SVC_LEFT_Y0_ = g_params_.BEV_H_/2.0 - g_params_.PPM_ * g_params_.T_SVC_LEFT_31_.at<double>(0, 0);
        g_params_.SVC_REAR_X0_ = g_params_.BEV_W_/2.0 + g_params_.PPM_ * g_params_.T_SVC_REAR_31_.at<double>(1, 0);
        g_params_.SVC_REAR_Y0_ = g_params_.BEV_H_/2.0 - g_params_.PPM_ * g_params_.T_SVC_REAR_31_.at<double>(0, 0);
        g_params_.SVC_RIGHT_X0_ = g_params_.BEV_W_/2.0 + g_params_.PPM_ * g_params_.T_SVC_RIGHT_31_.at<double>(1, 0);
        g_params_.SVC_RIGHT_Y0_ = g_params_.BEV_H_/2.0 - g_params_.PPM_ * g_params_.T_SVC_RIGHT_31_.at<double>(0, 0);

        ROS_INFO("SVC_FRONT_: (%.2f, %.2f), SVC_LEFT_: (%.2f, %.2f), SVC_REAR_: (%.2f, %.2f), SVC_RIGHT_: (%.2f, %.2f)\n", \
            g_params_.SVC_FRONT_X0_, g_params_.SVC_FRONT_Y0_, g_params_.SVC_LEFT_X0_, g_params_.SVC_LEFT_Y0_, \
            g_params_.SVC_REAR_X0_, g_params_.SVC_REAR_Y0_, g_params_.SVC_RIGHT_X0_, g_params_.SVC_RIGHT_Y0_);
    }

    void init_homo(){
        double x0_bev = 0.0;
        double y0_bev = 0.0;
        double z0_bev = g_params_.T_SVC_FRONT_31_.at<double>(2, 0);
        double f_bev = z0_bev * g_params_.PPM_;

        //1. calc INV_K_BEV_43_
        cv::Mat k_bev_33 = cv::Mat::eye(3, 3, CV_64F);
        k_bev_33.at<double>(0, 0) = f_bev;  k_bev_33.at<double>(0, 1) = 0.0;    k_bev_33.at<double>(0, 2) = g_params_.BEV_W_/2.0;
        k_bev_33.at<double>(1, 0) = 0.0;    k_bev_33.at<double>(1, 1) = f_bev;  k_bev_33.at<double>(1, 2) = g_params_.BEV_H_/2.0;
        k_bev_33.at<double>(2, 0) = 0.0;    k_bev_33.at<double>(2, 1) = 0.0;    k_bev_33.at<double>(2, 2) = 1.0;
        k_bev_33 = k_bev_33 / z0_bev;
        cv::Mat inv_k_bev_33 = k_bev_33.inv();
        cv::Mat eye_13 = cv::Mat::zeros(1, 3, CV_64F);
        eye_13.at<double>(0, 2) = 1.0;
        cv::Mat matArray[] = {inv_k_bev_33, eye_13};
        cv::vconcat(matArray, 2, g_params_.INV_K_BEV_43_);
        ROS_INFO("INV_K_BEV_43_: %s", ViwoUtils::CvMat2Str(g_params_.INV_K_BEV_43_).c_str());

        //2. calc E_BEV_C2B_44_
        const double bev_pitch = 90.0/DEG2RAD;
        const double bev_roll = 0.0;
        const double bev_yaw = 0.0;
        cv::Mat bev_rot_33 = euler_2_rot(bev_pitch, bev_roll, bev_yaw) * g_params_.R0_CAM_TO_BODY_;
        // ROS_INFO("bev_rot: %s", ViwoUtils::CvMat2Str(bev_rot).c_str());

        g_params_.E_BEV_C2B_44_ = cv::Mat::eye(4, 4, CV_64F);
        g_params_.E_BEV_C2B_44_.at<double>(0, 0) = bev_rot_33.at<double>(0, 0);
        g_params_.E_BEV_C2B_44_.at<double>(0, 1) = bev_rot_33.at<double>(0, 1);
        g_params_.E_BEV_C2B_44_.at<double>(0, 2) = bev_rot_33.at<double>(0, 2);
        g_params_.E_BEV_C2B_44_.at<double>(0, 3) = x0_bev;
        g_params_.E_BEV_C2B_44_.at<double>(1, 0) = bev_rot_33.at<double>(1, 0);
        g_params_.E_BEV_C2B_44_.at<double>(1, 1) = bev_rot_33.at<double>(1, 1);
        g_params_.E_BEV_C2B_44_.at<double>(1, 2) = bev_rot_33.at<double>(1, 2);
        g_params_.E_BEV_C2B_44_.at<double>(1, 3) = y0_bev;
        g_params_.E_BEV_C2B_44_.at<double>(2, 0) = bev_rot_33.at<double>(2, 0);
        g_params_.E_BEV_C2B_44_.at<double>(2, 1) = bev_rot_33.at<double>(2, 1);
        g_params_.E_BEV_C2B_44_.at<double>(2, 2) = bev_rot_33.at<double>(2, 2);
        g_params_.E_BEV_C2B_44_.at<double>(2, 3) = z0_bev;
        ROS_INFO("E_BEV_C2B_44_: %s", ViwoUtils::CvMat2Str(g_params_.E_BEV_C2B_44_).c_str());

        // cos(20degree)
        homo_update_rate_ = cos(g_params_.EULER_SVC_FRONT_31_.at<double>(0, 0));
    }

    void update_homo(double car_pitch=0.0, double car_roll=0.0, double car_yaw=0.0){
        car_pitch *= homo_update_rate_;
        car_roll *= homo_update_rate_;

        //3. calc E_FRONT_B2C_44_
        const double front_pitch = g_params_.EULER_SVC_FRONT_31_.at<double>(0, 0) + car_pitch;
        const double front_roll = g_params_.EULER_SVC_FRONT_31_.at<double>(1, 0) + car_roll;
        const double front_yaw = g_params_.EULER_SVC_FRONT_31_.at<double>(2, 0);
        cv::Mat front_rot_33 = euler_2_rot(front_pitch, front_roll, front_yaw) * g_params_.R0_CAM_TO_BODY_;
        cv::Mat e_front_c2b_44 = cv::Mat::eye(4, 4, CV_64F);
        e_front_c2b_44.at<double>(0, 0) = front_rot_33.at<double>(0, 0);
        e_front_c2b_44.at<double>(0, 1) = front_rot_33.at<double>(0, 1);
        e_front_c2b_44.at<double>(0, 2) = front_rot_33.at<double>(0, 2);
        e_front_c2b_44.at<double>(0, 3) = g_params_.T_SVC_FRONT_31_.at<double>(0, 0);
        e_front_c2b_44.at<double>(1, 0) = front_rot_33.at<double>(1, 0);
        e_front_c2b_44.at<double>(1, 1) = front_rot_33.at<double>(1, 1);
        e_front_c2b_44.at<double>(1, 2) = front_rot_33.at<double>(1, 2);
        e_front_c2b_44.at<double>(1, 3) = g_params_.T_SVC_FRONT_31_.at<double>(1, 0);
        e_front_c2b_44.at<double>(2, 0) = front_rot_33.at<double>(2, 0);
        e_front_c2b_44.at<double>(2, 1) = front_rot_33.at<double>(2, 1);
        e_front_c2b_44.at<double>(2, 2) = front_rot_33.at<double>(2, 2);
        e_front_c2b_44.at<double>(2, 3) = g_params_.T_SVC_FRONT_31_.at<double>(2, 0);
        g_params_.E_FRONT_B2C_44_ = e_front_c2b_44.inv();
        // ROS_INFO("E_FRONT_B2C_44_: %s", ViwoUtils::CvMat2Str(g_params_.E_FRONT_B2C_44_).c_str());
        //4. calc HOMO_SVC_FRONT_
        cv::Mat h_bev_to_front = g_params_.K_SVC_34_*(g_params_.E_FRONT_B2C_44_*(g_params_.E_BEV_C2B_44_*g_params_.INV_K_BEV_43_));
        g_params_.HOMO_SVC_FRONT_ = h_bev_to_front.inv();

        
        const double left_pitch = g_params_.EULER_SVC_LEFT_31_.at<double>(0, 0) + car_roll;
        const double left_roll = g_params_.EULER_SVC_LEFT_31_.at<double>(1, 0) - car_pitch;
        const double left_yaw = g_params_.EULER_SVC_LEFT_31_.at<double>(2, 0);
        cv::Mat left_rot_33 = euler_2_rot(left_pitch, left_roll, left_yaw) * g_params_.R0_CAM_TO_BODY_;
        cv::Mat e_left_c2b_44 = cv::Mat::eye(4, 4, CV_64F);
        e_left_c2b_44.at<double>(0, 0) = left_rot_33.at<double>(0, 0);
        e_left_c2b_44.at<double>(0, 1) = left_rot_33.at<double>(0, 1);
        e_left_c2b_44.at<double>(0, 2) = left_rot_33.at<double>(0, 2);
        e_left_c2b_44.at<double>(0, 3) = g_params_.T_SVC_LEFT_31_.at<double>(0, 0);
        e_left_c2b_44.at<double>(1, 0) = left_rot_33.at<double>(1, 0);
        e_left_c2b_44.at<double>(1, 1) = left_rot_33.at<double>(1, 1);
        e_left_c2b_44.at<double>(1, 2) = left_rot_33.at<double>(1, 2);
        e_left_c2b_44.at<double>(1, 3) = g_params_.T_SVC_LEFT_31_.at<double>(1, 0);
        e_left_c2b_44.at<double>(2, 0) = left_rot_33.at<double>(2, 0);
        e_left_c2b_44.at<double>(2, 1) = left_rot_33.at<double>(2, 1);
        e_left_c2b_44.at<double>(2, 2) = left_rot_33.at<double>(2, 2);
        e_left_c2b_44.at<double>(2, 3) = g_params_.T_SVC_LEFT_31_.at<double>(2, 0);
        g_params_.E_LEFT_B2C_44_ = e_left_c2b_44.inv();
        // ROS_INFO("E_LEFT_B2C_44_: %s", ViwoUtils::CvMat2Str(g_params_.E_LEFT_B2C_44_).c_str());
        cv::Mat h_bev_to_left = g_params_.K_SVC_34_*(g_params_.E_LEFT_B2C_44_*(g_params_.E_BEV_C2B_44_*g_params_.INV_K_BEV_43_));
        g_params_.HOMO_SVC_LEFT_ = h_bev_to_left.inv();


        const double rear_pitch = g_params_.EULER_SVC_REAR_31_.at<double>(0, 0) - car_pitch;
        const double rear_roll = g_params_.EULER_SVC_REAR_31_.at<double>(1, 0) - car_roll;
        const double rear_yaw = g_params_.EULER_SVC_REAR_31_.at<double>(2, 0);
        cv::Mat rear_rot_33 = euler_2_rot(rear_pitch, rear_roll, rear_yaw) * g_params_.R0_CAM_TO_BODY_;
        cv::Mat e_rear_c2b_44 = cv::Mat::eye(4, 4, CV_64F);
        e_rear_c2b_44.at<double>(0, 0) = rear_rot_33.at<double>(0, 0);
        e_rear_c2b_44.at<double>(0, 1) = rear_rot_33.at<double>(0, 1);
        e_rear_c2b_44.at<double>(0, 2) = rear_rot_33.at<double>(0, 2);
        e_rear_c2b_44.at<double>(0, 3) = g_params_.T_SVC_REAR_31_.at<double>(0, 0);
        e_rear_c2b_44.at<double>(1, 0) = rear_rot_33.at<double>(1, 0);
        e_rear_c2b_44.at<double>(1, 1) = rear_rot_33.at<double>(1, 1);
        e_rear_c2b_44.at<double>(1, 2) = rear_rot_33.at<double>(1, 2);
        e_rear_c2b_44.at<double>(1, 3) = g_params_.T_SVC_REAR_31_.at<double>(1, 0);
        e_rear_c2b_44.at<double>(2, 0) = rear_rot_33.at<double>(2, 0);
        e_rear_c2b_44.at<double>(2, 1) = rear_rot_33.at<double>(2, 1);
        e_rear_c2b_44.at<double>(2, 2) = rear_rot_33.at<double>(2, 2);
        e_rear_c2b_44.at<double>(2, 3) = g_params_.T_SVC_REAR_31_.at<double>(2, 0);
        g_params_.E_REAR_B2C_44_ = e_rear_c2b_44.inv();
        // ROS_INFO("E_REAR_B2C_44_: %s", ViwoUtils::CvMat2Str(g_params_.E_REAR_B2C_44_).c_str());
        cv::Mat h_bev_to_rear = g_params_.K_SVC_34_*(g_params_.E_REAR_B2C_44_*(g_params_.E_BEV_C2B_44_*g_params_.INV_K_BEV_43_));
        g_params_.HOMO_SVC_REAR_ = h_bev_to_rear.inv();


        const double right_pitch = g_params_.EULER_SVC_RIGHT_31_.at<double>(0, 0) - car_roll;
        const double right_roll = g_params_.EULER_SVC_RIGHT_31_.at<double>(1, 0) + car_pitch;
        const double right_yaw = g_params_.EULER_SVC_RIGHT_31_.at<double>(2, 0);
        cv::Mat right_rot_33 = euler_2_rot(right_pitch, right_roll, right_yaw) * g_params_.R0_CAM_TO_BODY_;
        cv::Mat e_right_c2b_44 = cv::Mat::eye(4, 4, CV_64F);
        e_right_c2b_44.at<double>(0, 0) = right_rot_33.at<double>(0, 0);
        e_right_c2b_44.at<double>(0, 1) = right_rot_33.at<double>(0, 1);
        e_right_c2b_44.at<double>(0, 2) = right_rot_33.at<double>(0, 2);
        e_right_c2b_44.at<double>(0, 3) = g_params_.T_SVC_RIGHT_31_.at<double>(0, 0);
        e_right_c2b_44.at<double>(1, 0) = right_rot_33.at<double>(1, 0);
        e_right_c2b_44.at<double>(1, 1) = right_rot_33.at<double>(1, 1);
        e_right_c2b_44.at<double>(1, 2) = right_rot_33.at<double>(1, 2);
        e_right_c2b_44.at<double>(1, 3) = g_params_.T_SVC_RIGHT_31_.at<double>(1, 0);
        e_right_c2b_44.at<double>(2, 0) = right_rot_33.at<double>(2, 0);
        e_right_c2b_44.at<double>(2, 1) = right_rot_33.at<double>(2, 1);
        e_right_c2b_44.at<double>(2, 2) = right_rot_33.at<double>(2, 2);
        e_right_c2b_44.at<double>(2, 3) = g_params_.T_SVC_RIGHT_31_.at<double>(2, 0);
        g_params_.E_RIGHT_B2C_44_ = e_right_c2b_44.inv();
        // ROS_INFO("E_RIGHT_B2C_44_: %s", ViwoUtils::CvMat2Str(g_params_.E_RIGHT_B2C_44_).c_str());
        cv::Mat h_bev_to_right = g_params_.K_SVC_34_*(g_params_.E_RIGHT_B2C_44_*(g_params_.E_BEV_C2B_44_*g_params_.INV_K_BEV_43_));
        g_params_.HOMO_SVC_RIGHT_ = h_bev_to_right.inv();
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