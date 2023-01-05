#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <ros/ros.h>
#include "dataset_maker.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dataset_maker_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    if(argc != 2)
    {
        printf("please intput: rosrun dataset_maker_node dataset_maker_node_bin [config file] \n"
               "for example: rosrun dataset_maker_node dataset_maker_node_bin "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    std::string config_file_path = argv[1];
    ROS_INFO("config_file_path: %s\n", argv[1]);
    std::shared_ptr<data_maker::ParamLoader> pl_ptr(new data_maker::ParamLoader(argv[1]));
    std::unique_ptr<data_maker::DatasetMaker> dataset_maker_ptr(new data_maker::DatasetMaker());
    dataset_maker_ptr -> load_params(n, pl_ptr);
    dataset_maker_ptr -> start_work();
    // dataset_maker_ptr -> test();

    ros::spin();
    dataset_maker_ptr -> finish_work();
    return 0;
}
