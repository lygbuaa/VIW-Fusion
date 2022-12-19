#pragma once

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

typedef struct{
    int BEV_H_ = 640;
    int BEV_W_ = 640;
    float BEV_XMAX_ = 16.0f;
    float BEV_YMAX_ = 16.0f;
    cv::Mat HOMO_SVC_FRONT_;
    cv::Mat HOMO_SVC_LEFT_;
    cv::Mat HOMO_SVC_REAR_;
    cv::Mat HOMO_SVC_RIGHT_;
    float SVC_FRONT_X0_ = 0.0f;
    float SVC_FRONT_Y0_ = 0.0f;
    float SVC_LEFT_X0_ = 0.0f;
    float SVC_LEFT_Y0_ = 0.0f;
    float SVC_REAR_X0_ = 0.0f;
    float SVC_REAR_Y0_ = 0.0f;
    float SVC_RIGHT_X0_ = 0.0f;
    float SVC_RIGHT_Y0_ = 0.0f;
    int CAR_MODEL_W_ = 0;
    int CAR_MODEL_H_ = 0;
    int CAR_MODEL_X0_ = 0;
    int CAR_MODEL_Y0_ = 0;
}IPM_PARAMS_t;

class CvParamLoader
{
public:
    IPM_PARAMS_t ipm_params_;
    std::string project_rootdir_;
    std::string output_path_;
    std::string pcr_model_path_;
    std::string psd_model_path_;
    std::string dataset_path_;
    std::string car_top_image_path_;

    std::string image_front_topic_;
    std::string image_left_topic_;
    std::string image_rear_topic_;
    std::string image_right_topic_;

private:
    std::string yaml_path_;
    cv::FileStorage fs_;

public:
    CvParamLoader(std::string yaml_path){
        yaml_path_ = yaml_path;
        fs_.open(yaml_path, cv::FileStorage::READ);
        if(!fs_.isOpened()){
            fprintf(stderr, "open config file failed: %s\n", yaml_path.c_str());
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
        fs_["bev_h"] >> ipm_params_.BEV_H_;
        fs_["bev_w"] >> ipm_params_.BEV_W_;
        fs_["bev_xmax"] >> ipm_params_.BEV_XMAX_;
        fs_["bev_ymax"] >> ipm_params_.BEV_YMAX_;

        fs_["svc_front_x0"] >> ipm_params_.SVC_FRONT_X0_;
        fs_["svc_front_y0"] >> ipm_params_.SVC_FRONT_Y0_;
        fs_["svc_left_x0"] >> ipm_params_.SVC_LEFT_X0_;
        fs_["svc_left_y0"] >> ipm_params_.SVC_LEFT_Y0_;
        fs_["svc_rear_x0"] >> ipm_params_.SVC_REAR_X0_;
        fs_["svc_rear_y0"] >> ipm_params_.SVC_REAR_Y0_;
        fs_["svc_right_x0"] >> ipm_params_.SVC_RIGHT_X0_;
        fs_["svc_right_y0"] >> ipm_params_.SVC_RIGHT_Y0_;

        fs_["homo_svc_front"] >> ipm_params_.HOMO_SVC_FRONT_;
        fs_["homo_svc_left"] >> ipm_params_.HOMO_SVC_LEFT_;
        fs_["homo_svc_rear"] >> ipm_params_.HOMO_SVC_REAR_;
        fs_["homo_svc_right"] >> ipm_params_.HOMO_SVC_RIGHT_;

        fs_["project_rootdir"] >> project_rootdir_;
        fs_["output_path"] >> output_path_;
        output_path_ = project_rootdir_ + output_path_;

        fs_["pcr_model_path"] >> pcr_model_path_;
        pcr_model_path_ = project_rootdir_ + pcr_model_path_;

        fs_["psd_model_path"] >> psd_model_path_;
        psd_model_path_ = project_rootdir_ + psd_model_path_;

        fs_["dataset_path"] >> dataset_path_;
        dataset_path_ = project_rootdir_ + dataset_path_;

        fs_["car_top_image_path"] >> car_top_image_path_;
        car_top_image_path_ = project_rootdir_ + car_top_image_path_;

        fprintf(stderr, "dataset_path_: %s, output_path_: %s, pcr_model_path_: %s, psd_model_path_: %s\n", \
            dataset_path_.c_str(), output_path_.c_str(), pcr_model_path_.c_str(), psd_model_path_.c_str());

        fs_["image_front_topic"] >> image_front_topic_;
        fs_["image_left_topic"] >> image_left_topic_;
        fs_["image_rear_topic"] >> image_rear_topic_;
        fs_["image_right_topic"] >> image_right_topic_;
        fprintf(stderr, "image_front_topic: %s, image_left_topic: %s, image_rear_topic: %s, image_right_topic: %s\n", \
            image_front_topic_.c_str(), image_left_topic_.c_str(), image_rear_topic_.c_str(), image_right_topic_.c_str());
    }
};