#ifndef IPM_COMPOSER_H
#define IPM_COMPOSER_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "viwo_utils.h"

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
// using SvcPairedImages_t = std::map<SvcIndex_t, sensor_msgs::ImageConstPtr>;

typedef struct{
    double time;
    cv::Mat img_front;
    cv::Mat img_left;
    cv::Mat img_rear;
    cv::Mat img_right;
    cv::Mat img_ipm;
} SvcPairedImages_t;

class IpmComposer
{
public:
    static constexpr float DT_THRESHOLD_SEC_ = 0.02f;
    static constexpr int BEV_H_ = 1280;
    static constexpr int BEV_W_ = 1280;
    static constexpr float BEV_XMAX_ = 32.0;
    static constexpr float BEV_YMAX_ = 32.0;


private:
    ros::Publisher pub_image_ipm_;
    SvcImageBuffer_t svc_bufs_;
    ImageQueuePtr_t svc_front_buf_ptr_;
    ImageQueuePtr_t svc_left_buf_ptr_;
    ImageQueuePtr_t svc_rear_buf_ptr_;
    ImageQueuePtr_t svc_right_buf_ptr_;
    cv::Mat homo_svc_front_;
    cv::Mat homo_svc_left_;
    cv::Mat homo_svc_rear_;
    cv::Mat homo_svc_right_;
    cv::Mat ipm_mask_svc_front_;
    cv::Mat ipm_mask_svc_left_;
    cv::Mat ipm_mask_svc_rear_;
    cv::Mat ipm_mask_svc_right_;

public:
    IpmComposer() {
        svc_front_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());
        svc_left_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());
        svc_rear_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());
        svc_right_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());

        svc_bufs_[SvcIndex_t::FRONT] = svc_front_buf_ptr_;
        svc_bufs_[SvcIndex_t::LEFT] = svc_left_buf_ptr_;
        svc_bufs_[SvcIndex_t::REAR] = svc_rear_buf_ptr_;
        svc_bufs_[SvcIndex_t::RIGHT] = svc_right_buf_ptr_;

        homo_svc_front_ = cv::Mat::eye(3, 3, CV_32F);
        homo_svc_left_ = cv::Mat::eye(3, 3, CV_32F);
        homo_svc_rear_ = cv::Mat::eye(3, 3, CV_32F);
        homo_svc_right_ = cv::Mat::eye(3, 3, CV_32F);

        ipm_mask_svc_front_ = cv::Mat::ones(BEV_H_, BEV_W_, CV_8UC1);
        ipm_mask_svc_left_ = cv::Mat::ones(BEV_H_, BEV_W_, CV_8UC1);
        ipm_mask_svc_rear_ = cv::Mat::ones(BEV_H_, BEV_W_, CV_8UC1);
        ipm_mask_svc_right_ = cv::Mat::ones(BEV_H_, BEV_W_, CV_8UC1);
    }

    ~IpmComposer() {}

    void InitIpmPub(ros::NodeHandle& n){
        pub_image_ipm_ = n.advertise<sensor_msgs::Image>("image_ipm", 1000);
    }

    void SetHomography(cv::Mat& homo_svc_front, cv::Mat& homo_svc_left, cv::Mat& homo_svc_rear, cv::Mat& homo_svc_right){
        homo_svc_front_ = homo_svc_front;
        ROS_INFO("homo_svc_front_: %s", ViwoUtils::CvMat2Str(homo_svc_front_).c_str());
        homo_svc_left_ = homo_svc_left;
        ROS_INFO("homo_svc_left_: %s", ViwoUtils::CvMat2Str(homo_svc_left_).c_str());
        homo_svc_rear_ = homo_svc_rear;
        ROS_INFO("homo_svc_rear_: %s", ViwoUtils::CvMat2Str(homo_svc_rear_).c_str());
        homo_svc_right_ = homo_svc_right;
        ROS_INFO("homo_svc_right_: %s", ViwoUtils::CvMat2Str(homo_svc_right_).c_str());

        GenIpmMasks();
    }

    ImageQueuePtr_t GetSvcBuffer(SvcIndex_t idx){
        return svc_bufs_[idx];
    }

    bool IsEmpty() const {
        return (svc_front_buf_ptr_->empty() || svc_left_buf_ptr_->empty() || svc_rear_buf_ptr_->empty() || svc_right_buf_ptr_->empty());
    }

    void PushImage(SvcIndex_t idx, const sensor_msgs::ImageConstPtr& img_msg){
        svc_bufs_[idx] -> push(img_msg);
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
    }

    void Compose(SvcPairedImages_t& pis, bool debug_save=false, std::string path="./"){
        HANG_STOPWATCH();
        const cv::Mat& img_front = pis.img_front;
        const cv::Mat& img_left = pis.img_left;
        const cv::Mat& img_rear = pis.img_rear;
        const cv::Mat& img_right = pis.img_right;
        cv::Mat& ipm = pis.img_ipm;

        ipm = cv::Mat::zeros(BEV_H_, BEV_W_, CV_8UC3);
        cv::Mat tmp(BEV_H_, BEV_W_, CV_8UC3);
        cv::warpPerspective(img_front, tmp, homo_svc_front_, tmp.size(), cv::INTER_NEAREST);
        tmp.setTo(cv::Scalar(0,0,0), ipm_mask_svc_front_);
        ipm += tmp;

        cv::warpPerspective(img_left, tmp, homo_svc_left_, tmp.size(), cv::INTER_NEAREST);
        tmp.setTo(cv::Scalar(0,0,0), ipm_mask_svc_left_);
        ipm += tmp;

        cv::warpPerspective(img_rear, tmp, homo_svc_rear_, tmp.size(), cv::INTER_NEAREST);
        tmp.setTo(cv::Scalar(0,0,0), ipm_mask_svc_rear_);
        ipm += tmp;

        cv::warpPerspective(img_right, tmp, homo_svc_right_, tmp.size(), cv::INTER_NEAREST);
        tmp.setTo(cv::Scalar(0,0,0), ipm_mask_svc_right_);
        ipm += tmp;

        PubIpmImage(ipm, pis.time);

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

    void PubIpmImage(const cv::Mat& img_ipm, const double t){
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
      line1-(0, 0)-(w, h): -1*BEV_H_/BEV_W_*x + y = 0
      line2-(w, 0)-(0, h): BEV_H_/BEV_W_*x + y - BEV_H_ = 0
     */
    void GenIpmMasks(){
        const float k1 = -1*BEV_H_/(float)BEV_W_;
        const float b1 = 0.0f;
        const float k2 = BEV_H_/(float)BEV_W_;
        const float b2 = -1 * BEV_H_;

        for(int y=0; y<BEV_H_; y++){
            for(int x=0; x<BEV_W_; x++){
                float line1 = k1*x + y + b1;
                float line2 = k2*x + y + b2;
                /* svc_front area */
                if(line1<0.0f && line2<0.0f){
                    // cv::Vec3b& val = ipm_mask_svc_front_.at<cv::Vec3b>(y, x);
                    // val[0] = val[1] = val[2] = 0;
                    ipm_mask_svc_front_.at<uchar>(y, x) = 0;
                }
                /* svc_left area */
                else if(line1>0.0f && line2<0.0f){
                    // cv::Vec3b& val = ipm_mask_svc_left_.at<cv::Vec3b>(y, x);
                    // val[0] = val[1] = val[2] = 0;
                    ipm_mask_svc_left_.at<uchar>(y, x) = 0;
                }
                /* svc_rear area */
                else if(line1>0.0f && line2>0.0f){
                    // cv::Vec3b& val = ipm_mask_svc_rear_.at<cv::Vec3b>(y, x);
                    // val[0] = val[1] = val[2] = 0;
                    ipm_mask_svc_rear_.at<uchar>(y, x) = 0;
                }
                /* svc_right area */
                else if(line1<0.0f && line2>0.0f){
                    // cv::Vec3b& val = ipm_mask_svc_right_.at<cv::Vec3b>(y, x);
                    // val[0] = val[1] = val[2] = 0;
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

};

#endif IPM_COMPOSER_H