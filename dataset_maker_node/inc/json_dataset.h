#pragma once

#include <ros/ros.h>
#include "viwo_utils.h"
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <libgen.h>
#include <chrono>
#include <unistd.h>
#include "tinyjson.hpp"
#include "global_defines.h"

namespace data_maker{
class JsonDataset{
public:
    std::shared_ptr<data_maker::ParamLoader> pl_;
    std::ofstream json_ofs_;
    std::ifstream json_ifs_;
    std::vector<int> png_compression_params_;

public:
    /* must set the path, where to save images and json file */
    JsonDataset(){}

    ~JsonDataset(){
        close_writer();
        close_reader();
    }

    void load_param(std::shared_ptr<data_maker::ParamLoader> pl){
        pl_ = pl;
        png_compression_params_.push_back(cv::IMWRITE_PNG_COMPRESSION);
        png_compression_params_.push_back(pl_->cv_png_compression_level_);
        if(pl_->write_or_read_==0){
            init_writer();
            ROS_INFO("init writer mode");
        }else if(pl_->write_or_read_==1){
            init_reader();
            ROS_INFO("init reader mode");
        }else{
            ROS_FATAL("invalid mode: %d", pl_->write_or_read_);
        }
    }

    void init_writer(){
        json_ofs_.open(pl_->output_json_file_.c_str(), std::ofstream::out);
        ROS_WARN("%s ofstream open.\n", pl_->output_json_file_.c_str());
    }

    void init_reader(){
        json_ifs_.open(pl_->output_json_file_.c_str(), std::ifstream::in);
        ROS_WARN("%s ifstream open.\n", pl_->output_json_file_.c_str());
    }

    void close_writer(){
        if(json_ofs_.is_open()){
            json_ofs_.close();
            fprintf(stderr, "%s ofstream closed.\n", pl_->output_json_file_.c_str());
        }
    }

    void close_reader(){
        if(json_ifs_.is_open()){
            json_ifs_.close();
            fprintf(stderr, "%s ifstream closed.\n", pl_->output_json_file_.c_str());
        }
    }

    void save_odom(const Odom_Carla_t& odom, const std::string& path){
        tiny::TinyJson obj;
        obj["t"].Set(odom.t);
        obj["x"].Set(odom.x);
        obj["y"].Set(odom.y);
        obj["z"].Set(odom.z);
        obj["qx"].Set(odom.qx);
        obj["qy"].Set(odom.qy);
        obj["qz"].Set(odom.qz);
        obj["qw"].Set(odom.qw);
        obj["vx"].Set(odom.vx);
        obj["vy"].Set(odom.vy);
        obj["vz"].Set(odom.vz);
        obj["wx"].Set(odom.wx);
        obj["pitch"].Set(odom.pitch);
        obj["roll"].Set(odom.roll);
        obj["yaw"].Set(odom.yaw);
        obj["primal_t"].Set(odom.primal_t);
        std::string str = obj.WriteJson();
        std::ofstream odom_ofs;
        odom_ofs.open(path.c_str(), std::ofstream::out);
        odom_ofs << str << std::endl;
        odom_ofs.close();
    }

    Odom_Carla_t load_odom(const std::string& path){
        std::ifstream odom_ifs;
        odom_ifs.open(path.c_str(), std::ifstream::in);
        std::string line;
        std::getline(odom_ifs, line);
        tiny::TinyJson obj;
        obj.ReadJson(line);
        Odom_Carla_t odom;
        odom.t = obj.Get<double>("t");
        odom.x = obj.Get<float>("x");
        odom.y = obj.Get<float>("y");
        odom.z = obj.Get<float>("z");
        odom.qx = obj.Get<float>("qx");
        odom.qy = obj.Get<float>("qy");
        odom.qz = obj.Get<float>("qz");
        odom.qw = obj.Get<float>("qw");
        odom.vx = obj.Get<float>("vx");
        odom.vy = obj.Get<float>("vy");
        odom.vz = obj.Get<float>("vz");
        odom.pitch = obj.Get<float>("pitch");
        odom.roll = obj.Get<float>("roll");
        odom.yaw = obj.Get<float>("yaw");
        odom.primal_t = obj.Get<double>("primal_t");
        odom_ifs.close();
        return odom;
    }

    bool feed(const SensorDataBatch_t& batch){
        const std::string tstr = std::to_string(batch.t);

        const std::string svc_front = pl_->output_svc_front_path_ + "/" + tstr + "_front.png";
        cv::imwrite(svc_front.c_str(), batch.img_svc_front, png_compression_params_);

        const std::string svc_left = pl_->output_svc_left_path_ + "/" + tstr + "_left.png";
        cv::imwrite(svc_left.c_str(), batch.img_svc_left, png_compression_params_);

        const std::string svc_rear = pl_->output_svc_rear_path_ + "/" + tstr + "_rear.png";
        cv::imwrite(svc_rear.c_str(), batch.img_svc_rear, png_compression_params_);

        const std::string svc_right = pl_->output_svc_right_path_ + "/" + tstr + "_right.png";
        cv::imwrite(svc_right.c_str(), batch.img_svc_right, png_compression_params_);

        const std::string odom_path = pl_->output_odom_path_ + "/" + tstr + "_odom.json";
        save_odom(batch.odom, odom_path);

        tiny::TinyJson obj;
        obj["t"].Set(batch.t);
        obj["svc_front"].Set(svc_front);
        obj["svc_left"].Set(svc_left);
        obj["svc_rear"].Set(svc_rear);
        obj["svc_right"].Set(svc_right);
        obj["odom"].Set(odom_path);
        std::string str = obj.WriteJson();
        json_ofs_ << str << std::endl << std::flush;
        ROS_DEBUG_THROTTLE(1, "save json string: %s\n", str.c_str());

        return true;
    }

    bool load(SensorDataBatch_t& batch){
        std::string line;
        if(std::getline(json_ifs_, line)){
            /* parse json */
            tiny::TinyJson obj;
            obj.ReadJson(line);
            batch.t = obj.Get<double>("t");
            std::string svc_front = obj.Get<std::string>("svc_front");
            std::string svc_left = obj.Get<std::string>("svc_left");
            std::string svc_right = obj.Get<std::string>("svc_right");
            std::string svc_rear = obj.Get<std::string>("svc_rear");
            std::string odom_path = obj.Get<std::string>("odom");

            ROS_DEBUG_THROTTLE(1, "t: %f, svc_front: %s, svc_left: %s, svc_right: %s, svc_rear: %s\n", \
                    batch.t, svc_front.c_str(), svc_left.c_str(), svc_right.c_str(), svc_rear.c_str());
            
            batch.img_svc_front = cv::imread(svc_front, cv::IMREAD_UNCHANGED);
            batch.img_svc_left = cv::imread(svc_left, cv::IMREAD_UNCHANGED);
            batch.img_svc_rear = cv::imread(svc_right, cv::IMREAD_UNCHANGED);
            batch.img_svc_right = cv::imread(svc_rear, cv::IMREAD_UNCHANGED);
            batch.odom = load_odom(odom_path);

            return true;
        }
        return false;
    }

    void test_writer(){
        close_writer();
        close_reader();
        usleep(10000);
        init_writer();
        cv::Mat img = cv::Mat::ones(1080, 1920, CV_8UC3);
        SensorDataBatch_t batch;
        batch.t = 0.01f;
        batch.img_svc_front = img*64;
        batch.img_svc_left = img*128;
        batch.img_svc_rear = img*255;
        batch.img_svc_right = img;
        while(batch.t < 10.0f){
            feed(batch);
            batch.t += 1.0f;
        }
        close_writer();
        fprintf(stderr, "test writter done\n");
    }

    void test_reader(){
        close_writer();
        close_reader();
        usleep(10000);
        init_reader();
        SensorDataBatch_t batch;
        while(load(batch)){
            fprintf(stderr, "load dataset t: %f, h: %d, w: %d\n", batch.t, batch.img_svc_front.rows, batch.img_svc_front.cols);
        }
        close_reader();
        fprintf(stderr, "test reader done\n");
    }

};

}