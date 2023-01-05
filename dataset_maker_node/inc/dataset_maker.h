#pragma once

#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include "viwo_utils.h"
#include "param_loader.h"
#include "json_dataset.h"
#include "global_defines.h"

namespace data_maker{

class DatasetMaker
{
public:
    ros::NodeHandle rnh_;
    std::shared_ptr<data_maker::ParamLoader> pl_;

    ImageQueuePtr_t svc_front_buf_ptr_;
    ImageQueuePtr_t svc_left_buf_ptr_;
    ImageQueuePtr_t svc_rear_buf_ptr_;
    ImageQueuePtr_t svc_right_buf_ptr_;
    OdomQueuePtr_t odom_buf_ptr_;
    SensorDataMap_t sensor_data_map_;

    ros::Subscriber sub_svc_front_;
    ros::Subscriber sub_svc_left_;
    ros::Subscriber sub_svc_rear_;
    ros::Subscriber sub_svc_right_;
    ros::Subscriber sub_odom_ego_;

    std::mutex sensor_mutex_;
    bool exit_ = false;
    std::unique_ptr<std::thread> sync_thread_;
    std::unique_ptr<JsonDataset> json_dataset_;

public:
    DatasetMaker(){
        svc_front_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());
        svc_left_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());
        svc_rear_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());
        svc_right_buf_ptr_ = ImageQueuePtr_t(new std::queue<sensor_msgs::ImageConstPtr>());
        odom_buf_ptr_ = OdomQueuePtr_t(new std::queue<nav_msgs::OdometryConstPtr>());

        sensor_data_map_[SensorIndex_t::SVC_FRONT] = svc_front_buf_ptr_;
        sensor_data_map_[SensorIndex_t::SVC_LEFT] = svc_left_buf_ptr_;
        sensor_data_map_[SensorIndex_t::SVC_REAR] = svc_rear_buf_ptr_;
        sensor_data_map_[SensorIndex_t::SVC_RIGHT] = svc_right_buf_ptr_;
        sensor_data_map_[SensorIndex_t::ODOM] = odom_buf_ptr_;
    }

    ~DatasetMaker(){
    }

    bool IsEmpty() const {
        return (svc_front_buf_ptr_->empty() || svc_left_buf_ptr_->empty() || svc_rear_buf_ptr_->empty() || svc_right_buf_ptr_->empty() || odom_buf_ptr_->empty());
    }

    void load_params(ros::NodeHandle& nh, std::shared_ptr<data_maker::ParamLoader> pl){
        rnh_ = nh;
        pl_ = pl;
    }

    void start_work(){
        exit_ = false;
        json_dataset_ = std::unique_ptr<JsonDataset>(new JsonDataset());
        json_dataset_->load_param(pl_);
        sync_thread_ = std::unique_ptr<std::thread>(new std::thread(&DatasetMaker::sync_work, this));
        listen_topics();
    }

    void finish_work(){
        exit_ = true;
        sync_thread_ -> join();
        json_dataset_ -> close_reader();
        json_dataset_ -> close_writer();
    }

    void listen_topics(){
        sub_svc_front_ = rnh_.subscribe(pl_->svc_front_topic_, 100, &DatasetMaker::svc_front_callback, this);
        sub_svc_left_ = rnh_.subscribe(pl_->svc_left_topic_, 100, &DatasetMaker::svc_left_callback, this);
        sub_svc_rear_ = rnh_.subscribe(pl_->svc_rear_topic_, 100, &DatasetMaker::svc_rear_callback,this);
        sub_svc_right_ = rnh_.subscribe(pl_->svc_right_topic_, 100, &DatasetMaker::svc_right_callback, this);
        sub_odom_ego_ = rnh_.subscribe(pl_->odom_topic_, 100, &DatasetMaker::odom_callback, this);
    }

    void remove_outdate_data(SensorIndex_t idx){
        if(idx >= SensorIndex_t::SVC_FRONT && idx <= SensorIndex_t::SVC_RIGHT){
            pop_sensor_data<sensor_msgs::ImageConstPtr>(idx);
        }else if(idx == SensorIndex_t::ODOM){
            pop_sensor_data<nav_msgs::OdometryConstPtr>(idx);
        }
    }

    /* call this function after RemoveOutdateFrame */
    void GetSyncSensorBatch(SensorDataBatch_t& batch){
        std::unique_lock<std::mutex> lock(sensor_mutex_);
        batch.t = svc_front_buf_ptr_->front()->header.stamp.toSec();
        batch.img_svc_front = GetImageFromMsg(svc_front_buf_ptr_->front());
        batch.img_svc_left = GetImageFromMsg(svc_left_buf_ptr_->front());
        batch.img_svc_rear = GetImageFromMsg(svc_rear_buf_ptr_->front());
        batch.img_svc_right = GetImageFromMsg(svc_right_buf_ptr_->front());
        batch.odom.from_ros_odom(odom_buf_ptr_->front());
    }

    void test(){
        // json_dataset_ = std::unique_ptr<JsonDataset>(new JsonDataset(pl_));
        json_dataset_ -> test_writer();
        usleep(1000*50);
        json_dataset_ -> test_reader();
    }

private:
    void sync_work(){
        ROS_INFO("sync thread start");
        SensorDataBatch_t batch;
        while(ros::ok() && !exit_){
            if(IsEmpty()){
                std::chrono::milliseconds dura(20);
                std::this_thread::sleep_for(dura);
                ROS_DEBUG_THROTTLE(3, "sensor buffer empty!");
                continue;
            }

            SensorIndex_t idx = find_outdate_sensor();
            if(idx == SensorIndex_t::VOID){
                /* sync done, save data to json file */
                GetSyncSensorBatch(batch);
                json_dataset_ -> feed(batch);
                
            }else{
                remove_outdate_data(idx);
                continue;
            }

        }
        ROS_INFO("sync thread stop");
    }

    template <typename T>
    void push_sensor_data(SensorIndex_t idx, std::any data_ptr){
        std::shared_ptr<std::queue<T>> buffer = std::any_cast<std::shared_ptr<std::queue<T>>>(sensor_data_map_[idx]);
        T data = std::any_cast<T>(data_ptr);
        std::unique_lock<std::mutex> lock(sensor_mutex_);
        buffer -> push(data);
        if(buffer->size() > BUFFER_MAX_){
            buffer -> pop();
            // ROS_INFO("buffer [%d] overflow!", (int)idx);
        }
    }

    template <typename T>
    void pop_sensor_data(SensorIndex_t idx){
        std::shared_ptr<std::queue<T>> buffer = std::any_cast<std::shared_ptr<std::queue<T>>>(sensor_data_map_[idx]);
        std::unique_lock<std::mutex> lock(sensor_mutex_);
        if(!buffer->empty()){
            buffer -> pop();
        }
    }

    void svc_front_callback(const sensor_msgs::ImageConstPtr& img_msg){
        static uint32_t counter = 0;
        push_sensor_data<sensor_msgs::ImageConstPtr>(SensorIndex_t::SVC_FRONT, img_msg);
        counter ++;
        // ROS_INFO("svc_front_callback counter: %d", counter);
    }

    void svc_left_callback(const sensor_msgs::ImageConstPtr& img_msg){
        push_sensor_data<sensor_msgs::ImageConstPtr>(SensorIndex_t::SVC_LEFT, img_msg);
    }

    void svc_rear_callback(const sensor_msgs::ImageConstPtr& img_msg){
        push_sensor_data<sensor_msgs::ImageConstPtr>(SensorIndex_t::SVC_REAR, img_msg);
    }

    void svc_right_callback(const sensor_msgs::ImageConstPtr& img_msg){
        push_sensor_data<sensor_msgs::ImageConstPtr>(SensorIndex_t::SVC_RIGHT, img_msg);
    }

    void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg){
        static uint32_t counter = 0;
        push_sensor_data<nav_msgs::OdometryConstPtr>(SensorIndex_t::ODOM, odom_msg);
        counter ++;
        ROS_DEBUG_THROTTLE(3, "odom_callback counter: %d", counter);
    }

    cv::Mat GetImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg){
        cv_bridge::CvImageConstPtr ptr;
        std::string encoding = (img_msg->encoding=="8UC1") ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::RGB8;
        ptr = cv_bridge::toCvCopy(img_msg, encoding);
        cv::Mat img = ptr->image.clone();
        return img;
    }

    SensorIndex_t find_outdate_sensor(){
        SensorTimestampVector_t vts;
        {
            std::unique_lock<std::mutex> lock(sensor_mutex_);
            for(int idx=0; idx<=3; idx++){
                SensorIndex_t s_idx = static_cast<SensorIndex_t>(idx);
                ImageQueuePtr_t buffer = std::any_cast<ImageQueuePtr_t>(sensor_data_map_[s_idx]);
                double ts_svc = buffer->front()->header.stamp.toSec();
                vts.emplace_back(std::tuple<SensorIndex_t, double>(s_idx, ts_svc));
                ROS_DEBUG_THROTTLE(3, "retrieve sensor[%d] timestamp: %.2f", idx, ts_svc);
            }
            OdomQueuePtr_t buffer = std::any_cast<OdomQueuePtr_t>(sensor_data_map_[SensorIndex_t::ODOM]);
            double ts_odom = buffer->front()->header.stamp.toSec();
            vts.emplace_back(std::tuple<SensorIndex_t, double>(SensorIndex_t::ODOM, ts_odom));
            ROS_DEBUG_THROTTLE(3, "retrieve sensor[%d] timestamp: %.2f", (int)(SensorIndex_t::ODOM), ts_odom);
        }

        std::sort(vts.begin(), vts.end(), \
            [](const std::tuple<SensorIndex_t, double>& ts1, const std::tuple<SensorIndex_t, double>& ts2) -> bool{ \
                return (std::get<1>(ts1) < std::get<1>(ts2)); \
            } \
        );

        const size_t last_idx = vts.size() - 1;
        double ts_min = std::get<1>(vts[0]);
        double ts_max = std::get<1>(vts[last_idx]);
        // ROS_INFO("ts_max: %f, ts_min: %f", ts_max, ts_min);

        /* remove the oldest frame if max_diff exceed SYNC_THRESHOLD_SEC_ */
        if(ts_max-ts_min < SYNC_THRESHOLD_SEC_){
            return SensorIndex_t::VOID;
        }else{
            SensorIndex_t idx = std::get<0>(vts[0]);
            ROS_WARN("find outdata sensor: %d", int(idx));
            return idx;
        }
    }

};

}