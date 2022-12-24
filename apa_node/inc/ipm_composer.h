#ifndef IPM_COMPOSER_H
#define IPM_COMPOSER_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include "viwo_utils.h"
#include "CvParamLoader.h"

enum class SvcIndex_t : int{
    VOID = -1,
    FRONT = 0,
    LEFT = 1,
    REAR = 2,
    RIGHT = 3,
    MAX = 4,
    ALL = 5,
};

using SvcTimestampVector_t = std::vector<std::tuple<SvcIndex_t, double>>;
using ImageQueuePtr_t = std::shared_ptr<std::queue<sensor_msgs::ImageConstPtr>>;
using SvcImageBuffer_t = std::map<SvcIndex_t, ImageQueuePtr_t>;
using OdomQueuePtr_t = std::shared_ptr<std::queue<nav_msgs::OdometryConstPtr>>;
// using SvcPairedImages_t = std::map<SvcIndex_t, sensor_msgs::ImageConstPtr>;

typedef struct{
    double time;
    cv::Mat img_front;
    cv::Mat img_left;
    cv::Mat img_rear;
    cv::Mat img_right;
    cv::Mat img_ipm;
    double x=0.0f;
    double y=0.0f;
    double z=0.0f;
    double pitch=0.0f;
    double roll=0.0f;
    double yaw=0.0f;
} SvcPairedImages_t;

class IpmComposer
{
public:
    static constexpr float DT_THRESHOLD_SEC_ = 0.02f;
    static constexpr int BUFFER_MAX_ = 10;
    static constexpr double PI_ = 3.141592741;
    std::shared_ptr<CvParamLoader> cpl_;

private:
    ros::Publisher pub_image_ipm_;
    SvcImageBuffer_t svc_bufs_;
    ImageQueuePtr_t svc_front_buf_ptr_;
    ImageQueuePtr_t svc_left_buf_ptr_;
    ImageQueuePtr_t svc_rear_buf_ptr_;
    ImageQueuePtr_t svc_right_buf_ptr_;
    OdomQueuePtr_t odom_buf_ptr_;
    cv::Mat ipm_mask_svc_front_;
    cv::Mat ipm_mask_svc_left_;
    cv::Mat ipm_mask_svc_rear_;
    cv::Mat ipm_mask_svc_right_;
    cv::Mat car_top_view_;

public:
    IpmComposer() {
        svc_front_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());
        svc_left_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());
        svc_rear_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());
        svc_right_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());
        odom_buf_ptr_ = OdomQueuePtr_t(new std::queue<nav_msgs::OdometryConstPtr>());

        svc_bufs_[SvcIndex_t::FRONT] = svc_front_buf_ptr_;
        svc_bufs_[SvcIndex_t::LEFT] = svc_left_buf_ptr_;
        svc_bufs_[SvcIndex_t::REAR] = svc_rear_buf_ptr_;
        svc_bufs_[SvcIndex_t::RIGHT] = svc_right_buf_ptr_;

        // g_params_.HOMO_SVC_FRONT_ = cv::Mat::eye(3, 3, CV_32F);
        // g_params_.HOMO_SVC_LEFT_ = cv::Mat::eye(3, 3, CV_32F);
        // g_params_.HOMO_SVC_REAR_ = cv::Mat::eye(3, 3, CV_32F);
        // g_params_.HOMO_SVC_RIGHT_ = cv::Mat::eye(3, 3, CV_32F);

        ipm_mask_svc_front_ = cv::Mat::ones(g_params_.BEV_H_, g_params_.BEV_W_, CV_8UC1);
        ipm_mask_svc_left_ = cv::Mat::ones(g_params_.BEV_H_, g_params_.BEV_W_, CV_8UC1);
        ipm_mask_svc_rear_ = cv::Mat::ones(g_params_.BEV_H_, g_params_.BEV_W_, CV_8UC1);
        ipm_mask_svc_right_ = cv::Mat::ones(g_params_.BEV_H_, g_params_.BEV_W_, CV_8UC1);
    }

    ~IpmComposer() {}

    void InitParams(std::shared_ptr<CvParamLoader> cpl){
        cpl_ = cpl;
        ROS_INFO("g_params_.HOMO_SVC_FRONT_: %s\n", ViwoUtils::CvMat2Str(g_params_.HOMO_SVC_FRONT_).c_str());
        ROS_INFO("g_params_.HOMO_SVC_LEFT_: %s\n", ViwoUtils::CvMat2Str(g_params_.HOMO_SVC_LEFT_).c_str());
        ROS_INFO("g_params_.HOMO_SVC_REAR_: %s\n", ViwoUtils::CvMat2Str(g_params_.HOMO_SVC_REAR_).c_str());
        ROS_INFO("g_params_.HOMO_SVC_RIGHT_: %s\n", ViwoUtils::CvMat2Str(g_params_.HOMO_SVC_RIGHT_).c_str());
        GenIpmMasksBow1();
        ReadCarModel(cpl_->car_top_image_path_);
    }

    bool ReadCarModel(std::string car_img_path){
        cv::Mat car_top_view = cv::imread(car_img_path, cv::IMREAD_COLOR);
        float h_w_ration = (float)car_top_view.rows / car_top_view.cols;
        g_params_.CAR_MODEL_W_ = int(g_params_.SVC_RIGHT_X0_ - g_params_.SVC_LEFT_X0_) + 40;
        // g_params_.CAR_MODEL_H_ = int(g_params_.SVC_REAR_Y0_ - g_params_.SVC_FRONT_Y0_) + 2;
        g_params_.CAR_MODEL_H_ = int(g_params_.CAR_MODEL_W_ * h_w_ration);
        car_top_view_ = psdonnx::PreProcessor::resize(car_top_view, g_params_.CAR_MODEL_W_, g_params_.CAR_MODEL_H_);
        g_params_.CAR_MODEL_X0_ = (g_params_.BEV_W_-g_params_.CAR_MODEL_W_)/2 - 1;
        g_params_.CAR_MODEL_Y0_ = (g_params_.BEV_H_-g_params_.CAR_MODEL_H_)/2 - 1;
        ROS_INFO("car model x: %d, y: %d, w: %d, h: %d\n", g_params_.CAR_MODEL_X0_, g_params_.CAR_MODEL_Y0_, g_params_.CAR_MODEL_W_, g_params_.CAR_MODEL_H_);
        return true;
    }

    void InitIpmPub(ros::NodeHandle& n){
        pub_image_ipm_ = n.advertise<sensor_msgs::Image>("image_ipm", 1000);
    }

    ImageQueuePtr_t GetSvcBuffer(SvcIndex_t idx){
        return svc_bufs_[idx];
    }

    bool IsEmpty() const {
        return (svc_front_buf_ptr_->empty() || svc_left_buf_ptr_->empty() || svc_rear_buf_ptr_->empty() || svc_right_buf_ptr_->empty());
    }

    //Eigen::Quaterniond& quat, Eigen::Vector3d& loc
    void PushOdom(const nav_msgs::OdometryConstPtr &odom_msg){
        odom_buf_ptr_ -> push(odom_msg);
        if(odom_buf_ptr_->size() > BUFFER_MAX_*10){
            odom_buf_ptr_ -> pop();
        }
    }

    void PushImage(SvcIndex_t idx, const sensor_msgs::ImageConstPtr& img_msg){
        svc_bufs_[idx] -> push(img_msg);
        if(svc_bufs_[idx]->size() > BUFFER_MAX_){
            svc_bufs_[idx] -> pop();
        }
    }

    void PopImage(SvcIndex_t idx){
        if(idx >= SvcIndex_t::FRONT && idx <=SvcIndex_t::RIGHT){
            svc_bufs_[idx] -> pop();
        }else if(idx == SvcIndex_t::ALL){
            svc_front_buf_ptr_ -> pop();
            svc_left_buf_ptr_ -> pop();
            svc_rear_buf_ptr_ -> pop();
            svc_right_buf_ptr_ -> pop();
        }
    }

    cv::Mat GetImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg){
        cv_bridge::CvImageConstPtr ptr;
        std::string encoding = (img_msg->encoding=="8UC1") ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::RGB8;
        ptr = cv_bridge::toCvCopy(img_msg, encoding);
        cv::Mat img = ptr->image.clone();
        return img;
    }

    /* 
      dt_sec > 0.0f: bring forward with a larger timestamp.
      dt_sec < 0.0f: put off with a smaller timestamp.
     */
    sensor_msgs::ImageConstPtr CompensateImageMsg(const sensor_msgs::ImageConstPtr& img_msg, double dt_sec=0.0f) const {
        sensor_msgs::ImagePtr new_msg(new sensor_msgs::Image(*img_msg));
        double old_ts = img_msg->header.stamp.toSec();
        double new_ts = old_ts + dt_sec;
        new_msg -> header.stamp = ros::Time().fromSec(new_ts);
        return new_msg;
    }

    /* call this function after RemoveOutdateFrame */
    void GetSyncImages(SvcPairedImages_t& paired_images){
        paired_images.time = svc_front_buf_ptr_->front()->header.stamp.toSec();
        paired_images.img_front = GetImageFromMsg(svc_front_buf_ptr_->front());
        paired_images.img_left = GetImageFromMsg(svc_left_buf_ptr_->front());
        paired_images.img_rear = GetImageFromMsg(svc_rear_buf_ptr_->front());
        paired_images.img_right = GetImageFromMsg(svc_right_buf_ptr_->front());

        while(!odom_buf_ptr_->empty()){
            auto odom_msg = odom_buf_ptr_->front();
            double t = odom_msg->header.stamp.toSec();
            if (t < paired_images.time-DT_THRESHOLD_SEC_){
                odom_buf_ptr_->pop();
                continue;
            }else if(t > paired_images.time+DT_THRESHOLD_SEC_){
                ROS_ERROR("impossisble, image time: %f, odom time: %f", paired_images.time, t);
                break;
            }else{
                /* eigen.eulerAngles() don't match tramsform3d.euler2quat(), use carla.rpy directly. */
                // auto tmp_Q = odom_msg->pose.pose.orientation;
                auto tmp_T = odom_msg->pose.pose.position;
                auto cov = odom_msg->pose.covariance;
                // Eigen::Quaterniond quat = Eigen::Quaterniond(tmp_Q.w, tmp_Q.x, tmp_Q.y, tmp_Q.z);
                // 0-z-yaw, 1-x-roll, 2-y-pitch
                // Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 0, 1);
                paired_images.roll = -1*cov[0];//euler[1]; //-1*tmp_Q.x*2;
                // if(paired_images.roll>PI_/2.0){
                //     paired_images.roll -= PI_;
                // }else if(paired_images.roll<-1*PI_/2.0){
                //     paired_images.roll += PI_;
                // }
                paired_images.pitch = cov[1];//euler[2]; //tmp_Q.y*2;
                // if(paired_images.pitch>PI_/2.0){
                //     paired_images.pitch -= PI_;
                // }else if(paired_images.pitch<-1*PI_/2.0){
                //     paired_images.pitch += PI_;
                // }
                paired_images.yaw = cov[2];//euler[0]; //tmp_Q.z*2;

                Eigen::Vector3d loc = Eigen::Vector3d(tmp_T.x, tmp_T.y, tmp_T.z);
                paired_images.x = loc[0];
                paired_images.y = loc[1];
                paired_images.z = loc[2];
                ROS_INFO("sync image time: %f, odom time: %f, cov: %.2f, %.2f, %.2f, %.2f", paired_images.time, t, cov[0], cov[1], cov[2], cov[3]);
                break;
            }
        }
    }

    void Compose(SvcPairedImages_t& pis, bool debug_save=false, std::string path="./"){
        HANG_STOPWATCH();
        const cv::Mat& img_front = pis.img_front;
        const cv::Mat& img_left = pis.img_left;
        const cv::Mat& img_rear = pis.img_rear;
        const cv::Mat& img_right = pis.img_right;
        cv::Mat& ipm = pis.img_ipm;

        cpl_ -> update_homo(pis.pitch, pis.roll, 0.0);

        ipm = cv::Mat::zeros(g_params_.BEV_H_, g_params_.BEV_W_, CV_8UC3);
        cv::Mat tmp(g_params_.BEV_H_, g_params_.BEV_W_, CV_8UC3);
        cv::warpPerspective(img_front, tmp, g_params_.HOMO_SVC_FRONT_, tmp.size(), cv::INTER_NEAREST);
        tmp.setTo(cv::Scalar(0,0,0), ipm_mask_svc_front_);
        ipm += tmp;

        cv::warpPerspective(img_left, tmp, g_params_.HOMO_SVC_LEFT_, tmp.size(), cv::INTER_NEAREST);
        tmp.setTo(cv::Scalar(0,0,0), ipm_mask_svc_left_);
        ipm += tmp;

        cv::warpPerspective(img_rear, tmp, g_params_.HOMO_SVC_REAR_, tmp.size(), cv::INTER_NEAREST);
        tmp.setTo(cv::Scalar(0,0,0), ipm_mask_svc_rear_);
        ipm += tmp;

        cv::warpPerspective(img_right, tmp, g_params_.HOMO_SVC_RIGHT_, tmp.size(), cv::INTER_NEAREST);
        tmp.setTo(cv::Scalar(0,0,0), ipm_mask_svc_right_);
        ipm += tmp;

        char tmpstr[64] = {0};
        sprintf(tmpstr, "rxyz: %.1f, %.2f, %.2f, %.2f", pis.yaw*57.3, pis.x, pis.y, pis.z);
        cv::Point org(10, 20);
        cv::Scalar yellow(0, 255, 255);
        // cv::putText(ipm, tmpstr, org, cv::FONT_HERSHEY_DUPLEX, 0.6, yellow, 1);
        ROS_INFO("carla rxyz: %s", tmpstr);

        // PubIpmImage(ipm, pis.time);

        if(debug_save){
            std::string tstr = std::to_string(pis.time);
            std::string filepath = path + "/" + tstr + "_ipm.png";
            cv::imwrite(filepath.c_str(), ipm);
#if 0
            filepath = path + "/" + tstr + "_front.png";
            cv::imwrite(filepath.c_str(), img_front);

            filepath = path + "/" + tstr + "_left.png";
            cv::imwrite(filepath.c_str(), img_left);

            filepath = path + "/" + tstr + "_rear.png";
            cv::imwrite(filepath.c_str(), img_rear);

            filepath = path + "/" + tstr + "_right.png";
            cv::imwrite(filepath.c_str(), img_right);
#endif
        }
    }

    void AddCarTopview(cv::Mat& ipm_img, cv::Mat& car_top_img){
        cv::Mat ipm_roi = ipm_img(cv::Rect(g_params_.CAR_MODEL_X0_, g_params_.CAR_MODEL_Y0_, g_params_.CAR_MODEL_W_, g_params_.CAR_MODEL_H_));
        cv::Mat mask = cv::Mat::ones(g_params_.CAR_MODEL_H_, g_params_.CAR_MODEL_W_, CV_8UC1);
        car_top_img.copyTo(ipm_roi, mask);
    }

    void PubIpmImage(cv::Mat& img_ipm, const double t){
        AddCarTopview(img_ipm, car_top_view_);
        std_msgs::Header header;
        header.frame_id = "world";
        header.stamp = ros::Time(t);
        sensor_msgs::ImagePtr imgIpmMsg = cv_bridge::CvImage(header, "bgr8", img_ipm).toImageMsg();
        pub_image_ipm_.publish(imgIpmMsg);
    }

    // void SaveIpmImage(const std::string& path, const cv::Mat& img_ipm, const double t){
    //     std::string filepath = path + "/ipm_" + std::to_string(t) + ".png";
    //     cv::imwrite(filepath.c_str(), img_ipm);
    // }

    SvcIndex_t RemoveOutdateFrame(){
        double ts_front = svc_front_buf_ptr_->front()->header.stamp.toSec();
        double ts_left = svc_left_buf_ptr_->front()->header.stamp.toSec();
        double ts_rear = svc_rear_buf_ptr_->front()->header.stamp.toSec();
        double ts_right = svc_right_buf_ptr_->front()->header.stamp.toSec();
        ROS_INFO("svc_front_buf_: %d(%f), svc_left_buf_: %d(%f), svc_rear_buf_: %d(%f), svc_right_buf_: %d(%f)", svc_front_buf_ptr_->size(), ts_front, svc_left_buf_ptr_->size(), ts_left, svc_rear_buf_ptr_->size(), ts_rear, svc_right_buf_ptr_->size(), ts_right);

        SvcTimestampVector_t vts;
        vts.emplace_back(std::tuple<SvcIndex_t, double>(SvcIndex_t::FRONT, ts_front));
        vts.emplace_back(std::tuple<SvcIndex_t, double>(SvcIndex_t::LEFT, ts_left));
        vts.emplace_back(std::tuple<SvcIndex_t, double>(SvcIndex_t::REAR, ts_rear));
        vts.emplace_back(std::tuple<SvcIndex_t, double>(SvcIndex_t::RIGHT, ts_right));

        std::sort(vts.begin(), vts.end(), \
            [](const std::tuple<SvcIndex_t, double>& ts1, const std::tuple<SvcIndex_t, double>& ts2) -> bool{ \
                return (std::get<1>(ts1) < std::get<1>(ts2)); \
            } \
        );

        double ts_min = std::get<1>(vts[0]);
        double ts_max = std::get<1>(vts[3]);
        // ROS_INFO("ts_max: %f, ts_min: %f", ts_max, ts_min);

        /* remove the oldest frame if max_diff exceed DT_THRESHOLD_SEC_ */
        if(ts_max-ts_min < DT_THRESHOLD_SEC_){
            return SvcIndex_t::VOID;
        }else{
            SvcIndex_t idx = std::get<0>(vts[0]);
            svc_bufs_[idx]->pop();
            ROS_WARN("remove outdata frame from buffer-%d", int(idx));
            return idx;
        }
    }

private:
    /* 
      line_left_forward: svc_left_center, (0, R*h)
      line_right_forward: svc_right_center, (w, R*h)
      line_left_rear: svc_left_center, (0, (1-R)*h)
      line_right_rear: svc_right_center, (w, (1-R)*h)
      k = (y2-y1)/(x1-x2)
      b = (x2y1-x1y2)/(x1-x2)
     */
    void GenIpmMasksBow2(){
        //ratio of height start-point, 0.0~0.5
        const float R = 0.27f; 

        /* line_left_forward */
        float k_lf, b_lf;
        GetKBFromTwoPoints(g_params_.SVC_LEFT_X0_, g_params_.SVC_LEFT_Y0_, 0.0f, R*g_params_.BEV_H_, k_lf, b_lf);
        /* line_right_forward */
        float k_rf, b_rf;
        GetKBFromTwoPoints(g_params_.SVC_RIGHT_X0_, g_params_.SVC_RIGHT_Y0_, g_params_.BEV_W_, R*g_params_.BEV_H_, k_rf, b_rf);
        /* line_left_rear */
        float k_lr, b_lr;
        GetKBFromTwoPoints(g_params_.SVC_LEFT_X0_, g_params_.SVC_LEFT_Y0_, 0, (1.0f-R)*g_params_.BEV_H_, k_lr, b_lr);
        /* line_right_rear */
        float k_rr, b_rr;
        GetKBFromTwoPoints(g_params_.SVC_RIGHT_X0_, g_params_.SVC_RIGHT_Y0_, g_params_.BEV_W_, (1.0f-R)*g_params_.BEV_H_, k_rr, b_rr);

        for(int y=0; y<g_params_.BEV_H_; y++){
            for(int x=0; x<g_params_.BEV_W_; x++){
                float line_lf = k_lf*x + y + b_lf;
                float line_rf = k_rf*x + y + b_rf;
                float line_lr = k_lr*x + y + b_lr;
                float line_rr = k_rr*x + y + b_rr;
                /* svc_front area */
                if(line_lf<=0.0f && line_rf<0.0f && y<g_params_.SVC_FRONT_Y0_){
                    ipm_mask_svc_front_.at<uchar>(y, x) = 0;
                }
                /* svc_left area */
                else if(line_lf>0.0f && line_lr<=0.0f){
                    ipm_mask_svc_left_.at<uchar>(y, x) = 0;
                }
                /* svc_rear area */
                else if(line_lr>=0.0f && line_rr>0.0f && y>g_params_.SVC_REAR_Y0_){
                    ipm_mask_svc_rear_.at<uchar>(y, x) = 0;
                }
                /* svc_right area */
                else if(line_rr<0.0f && line_rf>=0.0f){
                    ipm_mask_svc_right_.at<uchar>(y, x) = 0;
                }
            }
        }
        // std::string filepath = "/home/hugoliu/github/catkin_ws/src/VIW-Fusion/output";
        // cv::imwrite((filepath+"/svc_front_mask.png").c_str(), 255*ipm_mask_svc_front_);
        // cv::imwrite((filepath+"/svc_left_mask.png").c_str(), 255*ipm_mask_svc_left_);
        // cv::imwrite((filepath+"/svc_rear_mask.png").c_str(), 255*ipm_mask_svc_rear_);
        // cv::imwrite((filepath+"/svc_right_mask.png").c_str(), 255*ipm_mask_svc_right_);
    }

    /* 
      line_left_forward: svc_left_center, (R*w, 0)
      line_right_forward: svc_right_center, ((1-R)*w, 0)
      line_left_rear: svc_left_center, (R*w, h)
      line_right_rear: svc_right_center, ((1-R)*w, h)
      k = (y2-y1)/(x1-x2)
      b = (x2y1-x1y2)/(x1-x2)
     */
    void GenIpmMasksBow1(){
        //ratio of width start-point, 0.0~0.5, 0.128 for w=640, 0.16 for w=1280
        const float R = 0.16f;

        /* line_left_forward */
        float k_lf, b_lf;
        GetKBFromTwoPoints(g_params_.SVC_LEFT_X0_, g_params_.SVC_LEFT_Y0_, R*g_params_.BEV_W_, 0.0f, k_lf, b_lf);
        /* line_right_forward */
        float k_rf, b_rf;
        GetKBFromTwoPoints(g_params_.SVC_RIGHT_X0_, g_params_.SVC_RIGHT_Y0_, (1.0f-R)*g_params_.BEV_W_, 0.0f, k_rf, b_rf);
        /* line_left_rear */
        float k_lr, b_lr;
        GetKBFromTwoPoints(g_params_.SVC_LEFT_X0_, g_params_.SVC_LEFT_Y0_, R*g_params_.BEV_W_, g_params_.BEV_H_, k_lr, b_lr);
        /* line_right_rear */
        float k_rr, b_rr;
        GetKBFromTwoPoints(g_params_.SVC_RIGHT_X0_, g_params_.SVC_RIGHT_Y0_, (1.0f-R)*g_params_.BEV_W_, g_params_.BEV_H_, k_rr, b_rr);

        for(int y=0; y<g_params_.BEV_H_; y++){
            for(int x=0; x<g_params_.BEV_W_; x++){
                float line_lf = k_lf*x + y + b_lf;
                float line_rf = k_rf*x + y + b_rf;
                float line_lr = k_lr*x + y + b_lr;
                float line_rr = k_rr*x + y + b_rr;
                /* svc_front area */
                if(line_lf<=0.0f && line_rf<0.0f && y<g_params_.SVC_FRONT_Y0_){
                    ipm_mask_svc_front_.at<uchar>(y, x) = 0;
                }
                /* svc_left area */
                else if(line_lf>0.0f && line_lr<=0.0f){
                    ipm_mask_svc_left_.at<uchar>(y, x) = 0;
                }
                /* svc_rear area */
                else if(line_lr>=0.0f && line_rr>0.0f && y>g_params_.SVC_REAR_Y0_){
                    ipm_mask_svc_rear_.at<uchar>(y, x) = 0;
                }
                /* svc_right area */
                else if(line_rr<0.0f && line_rf>=0.0f){
                    ipm_mask_svc_right_.at<uchar>(y, x) = 0;
                }
            }
        }
        // std::string filepath = "/home/hugoliu/github/catkin_ws/src/VIW-Fusion/output";
        // cv::imwrite((filepath+"/svc_front_mask.png").c_str(), 255*ipm_mask_svc_front_);
        // cv::imwrite((filepath+"/svc_left_mask.png").c_str(), 255*ipm_mask_svc_left_);
        // cv::imwrite((filepath+"/svc_rear_mask.png").c_str(), 255*ipm_mask_svc_rear_);
        // cv::imwrite((filepath+"/svc_right_mask.png").c_str(), 255*ipm_mask_svc_right_);
    }

    void GetKBFromTwoPoints(const float x1, const float y1, const float x2, const float y2, float& k, float& b){
        k = (y2-y1)/(x1-x2);
        b = (x2*y1-x1*y2)/(x1-x2);
    }

    /* 
      line1-(0, 0)-(w, h): -1*BEV_H_/BEV_W_*x + y = 0
      line2-(w, 0)-(0, h): BEV_H_/BEV_W_*x + y - BEV_H_ = 0
     */
    void GenIpmMasksOrtho(){
        const float k1 = -1*g_params_.BEV_H_/(float)g_params_.BEV_W_;
        const float b1 = 0.0f;
        const float k2 = g_params_.BEV_H_/(float)g_params_.BEV_W_;
        const float b2 = -1 * g_params_.BEV_H_;

        for(int y=0; y<g_params_.BEV_H_; y++){
            for(int x=0; x<g_params_.BEV_W_; x++){
                float line1 = k1*x + y + b1;
                float line2 = k2*x + y + b2;
                /* svc_front area */
                if(line1<=0.0f && line2<0.0f){
                    ipm_mask_svc_front_.at<uchar>(y, x) = 0;
                }
                /* svc_left area */
                else if(line1>0.0f && line2<=0.0f){
                    ipm_mask_svc_left_.at<uchar>(y, x) = 0;
                }
                /* svc_rear area */
                else if(line1>=0.0f && line2>0.0f){
                    ipm_mask_svc_rear_.at<uchar>(y, x) = 0;
                }
                /* svc_right area */
                else if(line1<0.0f && line2>=0.0f){
                    ipm_mask_svc_right_.at<uchar>(y, x) = 0;
                }

            }
        }
        // std::string filepath = "/home/hugoliu/github/catkin_ws/src/VIW-Fusion/output";
        // cv::imwrite((filepath+"/svc_front_mask.png").c_str(), 255*ipm_mask_svc_front_);
        // cv::imwrite((filepath+"/svc_left_mask.png").c_str(), 255*ipm_mask_svc_left_);
        // cv::imwrite((filepath+"/svc_rear_mask.png").c_str(), 255*ipm_mask_svc_rear_);
        // cv::imwrite((filepath+"/svc_right_mask.png").c_str(), 255*ipm_mask_svc_right_);
    }

    /* only reserve positive half */
    void GenIpmMasksHalf(){
        for(int y=0; y<g_params_.BEV_H_; y++){
            for(int x=0; x<g_params_.BEV_W_; x++){
                if(x > g_params_.BEV_W_/2){
                    ipm_mask_svc_right_.at<uchar>(y, x) = 0;
                }else{
                    ipm_mask_svc_left_.at<uchar>(y, x) = 0;
                }

                if(y > g_params_.BEV_H_/2){
                    ipm_mask_svc_rear_.at<uchar>(y, x) = 0;
                }else{
                    ipm_mask_svc_front_.at<uchar>(y, x) = 0;
                }
            }
        }
        // std::string filepath = "/home/hugoliu/github/catkin_ws/src/VIW-Fusion/output";
        // cv::imwrite((filepath+"/svc_front_mask.png").c_str(), 255*ipm_mask_svc_front_);
        // cv::imwrite((filepath+"/svc_left_mask.png").c_str(), 255*ipm_mask_svc_left_);
        // cv::imwrite((filepath+"/svc_rear_mask.png").c_str(), 255*ipm_mask_svc_rear_);
        // cv::imwrite((filepath+"/svc_right_mask.png").c_str(), 255*ipm_mask_svc_right_);
    }

};

#endif //IPM_COMPOSER_H
