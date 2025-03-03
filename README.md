# debug topics
/vins_estimator/camera_pose
/vins_estimator/camera_pose_visual
/vins_estimator/extrinsic
/vins_estimator/groundtruth
/vins_estimator/image_track
/vins_estimator/imu_propagate
/vins_estimator/key_poses
/vins_estimator/keyframe_point
/vins_estimator/keyframe_pose
/vins_estimator/margin_cloud
/vins_estimator/odometry
/vins_estimator/path
/vins_estimator/point_cloud
/vins_estimator/pure_wheel_propagate
/vins_estimator/wheel_preintegration
/vins_estimator/wheel_propagate

# control topics
1. restart vins_estimator, `rostopic pub /vins_restart std_msgs/Bool true -1`
2. enable/disable imu usage, `rostopic pub /vins_imu_switch std_msgs/Bool true -1` 
3. enable/disable stereo usage, `rostopic pub /vins_cam_switch std_msgs/Bool true -1`

# use carla data-bag
1. `roslaunch vins vins_rviz.launch` # launch rviz
2. `rosrun vins viwo_node config/carla/avp_town04_01.yaml` # launch viwo
3. `rosbag play data/avp_town04_20220919.bag -r 0.1` # "-r 0.1" will slow-down publish rate by 0.1

# run realsense_stereo_imu_config_ridgeback
1. package name(defined in package.xml): vins, process name(defined in ridgeback_viwo.launch): viwo_node
2. source devel/setup.bash
3. `roslaunch vins ridgeback_viwo.launch` viwo_node will die, so run rviz and viwo_node seperately
4. `roslaunch vins vins_rviz.launch &`
5. `rosrun vins viwo_node config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml &`
6. `rosbag play data/ridgeback_dark.bag` can't play bag in the background
7. debug topics published in visualization.cpp::registerPub(), as for "/vins_estimator/image_track", FeatureTracker::drawTrack() // red-circle stands for good-tracking feature, blue-circle stands for bad-tracking feature, green-arrow stands for feature movement.

# get env ready
1. docker pull ros:melodic
2. docker run --gpus all -it -v `pwd`:`pwd` -w `pwd` --ipc=host  -v /etc/localtime:/etc/localtime:ro -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE --network host ros:melodic /bin/bash
3. install ceres solver
4. install sophus
5. install other libraries:
apt install ros-melodic-image-transport
apt install ros-melodic-tf
apt install ros-melodic-cv-bridge
apt install libopencv-dev


# VIW-Fusion
## An visual-inertial-wheel fusion odometry
VIW-Fusion is an optimization-based viusla-inertial-wheel fusion odometry, which is developed as a part of my master thesis. First of all, I need to thank HKUST Aerial Robotics Group led by Prof. Shaojie Shen for their outstanding work [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion). VIW-Fusion is developed based on VINS-Fusion.
**Features:**
- multiple sensors support (stereo cameras+IMU+wheel / mono camera+IMU+wheel)
- wheel enhanced visual-inertial initialization
- online spatial calibration (transformation between camera, IMU and wheel)
- online wheel intrinsic calibration
- online temporal calibration (time offset between camera, IMU and wheel)
- plane constraint

### Performance in the scene with challenge light
We tested Mono VIWO in scenes with drastic changes in light, and the parameters between different scenes remained unchanged. The video is below, if you can't access youtube, please try [bilibili](https://www.bilibili.com/video/BV1zg411N75H?share_source=copy_web):
[![visual-inertial-wheel fusion odometry](https://res.cloudinary.com/marcomontalbano/image/upload/v1638587991/video_to_markdown/images/youtube--HXNaLTJWea4-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/HXNaLTJWea4 "visual-inertial-wheel fusion odometry")
We compare the VIWO with Mono VIO. The trajectory estimated by Mono VIO is completely different from the real trajectory, while the trajectory estimated by Mono VIWO is in good agreement with the real trajectory.

[//]: # (<img src = "/home/td/slam/vins_fusion_ws/src/VINS-Fusion/support_files/image/mono_compare.png" width = 54% height = 30% />)

<img src = "https://github.com/TouchDeeper/VIW-Fusion/blob/master/support_files/image/mono_compare.png" width = 54% height = 30% />

We align the trajectory with the entrance door and exit door to further compare VIWO and Stereo VIO, the trajectory estimated by Mono VIWO is also more reasonable.

[//]: # (<img src = "/home/td/slam/vins_fusion_ws/src/VINS-Fusion/support_files/image/performance_compare_light.png" width = 54% height = 30% />)
[//]: # (<img src = "/home/td/slam/vins_fusion_ws/src/VINS-Fusion/support_files/image/performance_compare_dark.png" width = 54% height = 30% />)

<img src = "https://github.com/TouchDeeper/VIW-Fusion/blob/master/support_files/image/performance_compare_light.png" width = 54% height = 30% />
<img src = "https://github.com/TouchDeeper/VIW-Fusion/blob/master/support_files/image/performance_compare_dark.png" width = 54% height = 30% />

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

Version: 1.14.0

### 1.3 **Sophus**
```asm
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout a0fe89a323e20c42d3cecb590937eb7a06b8343a
mkdir build && cd ./build
cmake ..
make -j4
sudo make install
```

## 2. Build VIW-Fusion
Clone the repository and catkin_make:
```asm
cd ~/catkin_ws/src
git clone https://github.com/TouchDeeper/VIW-Fusion.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3. Example

Download dataset [here](https://drive.google.com/drive/folders/1m2msbo3DRGhtINtDE47v-1blyJc0RK0E?usp=sharing).
```asm
roslaunch vins vins_rviz.launch
rosrun vins viwo_node ~/catkin_ws/src/VIW-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml 
(optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VIW-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml
rosbag play YOUR_DATASET_FOLDER/ridgeback_dark.bag
```

## 4. Citation
If you use this package for your research, a footnote with the link to this repository is appreciated: `github.com/TouchDeeper/VIW-Fusion`, or for citation with BibTeX:
```
@misc{ztd2021viwo,
  title={VIW-Fusion: visual-inertial-wheel fusion odometry.},
  author={Tingda Zhuang},
  howpublished={\url{https://github.com/TouchDeeper/VIW-Fusion}},
  year={2021}
}
```

**-------------------- separation line----------------------------**

# VINS-Fusion
## An optimization-based multi-sensor state estimator

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/vins_logo.png" width = 55% height = 55% div align=left />
<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti.png" width = 34% height = 34% div align=center />

VINS-Fusion is an optimization-based multi-sensor state estimator, which achieves accurate self-localization for autonomous applications (drones, cars, and AR/VR). VINS-Fusion is an extension of [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono), which supports multiple visual-inertial sensor types (mono camera + IMU, stereo cameras + IMU, even stereo cameras only). We also show a toy example of fusing VINS with GPS. 
**Features:**
- multiple sensors support (stereo cameras / mono camera+IMU / stereo cameras+IMU)
- online spatial calibration (transformation between camera and IMU)
- online temporal calibration (time offset between camera and IMU)
- visual loop closure

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti_rank.png" width = 80% height = 80% />

We are the **top** open-sourced stereo algorithm on [KITTI Odometry Benchmark](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) (12.Jan.2019).

**Authors:** [Tong Qin](http://www.qintonguav.com), Shaozu Cao, Jie Pan, [Peiliang Li](https://peiliangli.github.io/), and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [Aerial Robotics Group](http://uav.ust.hk/), [HKUST](https://www.ust.hk/)

**Videos:**

<a href="https://www.youtube.com/embed/1qye82aW7nI" target="_blank"><img src="http://img.youtube.com/vi/1qye82aW7nI/0.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>


**Related Paper:** (paper is not exactly same with code)

* **Online Temporal Calibration for Monocular Visual-Inertial Systems**, Tong Qin, Shaojie Shen, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS, 2018), **best student paper award** [pdf](https://ieeexplore.ieee.org/abstract/document/8593603)

* **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**, Tong Qin, Peiliang Li, Shaojie Shen, IEEE Transactions on Robotics [pdf](https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert) 


*If you use VINS-Fusion for your academic research, please cite our related papers. [bib](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/paper_bib.txt)*

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).


## 2. Build VINS-Fusion
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3. EuRoC Example
Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) to YOUR_DATASET_FOLDER. Take MH_01 for example, you can run VINS-Fusion with three sensor types (monocular camera + IMU, stereo cameras + IMU and stereo cameras). 
Open four terminals, run vins odometry, visual loop closure(optional), rviz and play the bag file respectively. 
Green path is VIO odometry; red path is odometry under visual loop closure.

### 3.1 Monocualr camera + IMU

```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

### 3.2 Stereo cameras + IMU

```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

### 3.3 Stereo cameras

```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml 
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/euroc.gif" width = 430 height = 240 />


## 4. KITTI Example
### 4.1 KITTI Odometry (Stereo)
Download [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to YOUR_DATASET_FOLDER. Take sequences 00 for example,
Open two terminals, run vins and rviz respectively. 
(We evaluated odometry on KITTI benchmark without loop closure funtion)
```
    roslaunch vins vins_rviz.launch
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml
    rosrun vins kitti_odom_test ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/ 
```
### 4.2 KITTI GPS Fusion (Stereo + GPS)
Download [KITTI raw dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) to YOUR_DATASET_FOLDER. Take [2011_10_03_drive_0027_synced](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip) for example.
Open three terminals, run vins, global fusion and rviz respectively. 
Green path is VIO odometry; blue path is odometry under GPS global fusion.
```
    roslaunch vins vins_rviz.launch
    rosrun vins kitti_gps_test ~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml YOUR_DATASET_FOLDER/2011_10_03_drive_0027_sync/ 
    rosrun global_fusion global_fusion_node
```

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti.gif" width = 430 height = 240 />

## 5. VINS-Fusion on car demonstration
Download [car bag](https://drive.google.com/open?id=10t9H1u8pMGDOI6Q2w2uezEq5Ib-Z8tLz) to YOUR_DATASET_FOLDER.
Open four terminals, run vins odometry, visual loop closure(optional), rviz and play the bag file respectively. 
Green path is VIO odometry; red path is odometry under visual loop closure.
```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/vi_car/vi_car.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/vi_car/vi_car.yaml 
    rosbag play YOUR_DATASET_FOLDER/car.bag
```

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/car_gif.gif" width = 430 height = 240  />


## 6. Run with your devices 
VIO is not only a software algorithm, it heavily relies on hardware quality. For beginners, we recommend you to run VIO with professional equipment, which contains global shutter cameras and hardware synchronization.

### 6.1 Configuration file
Write a config file for your device. You can take config files of EuRoC and KITTI as the example. 

### 6.2 Camera calibration
VINS-Fusion support several camera models (pinhole, mei, equidistant). You can use [camera model](https://github.com/hengli/camodocal) to calibrate your cameras. We put some example data under /camera_models/calibrationdata to tell you how to calibrate.
```
cd ~/catkin_ws/src/VINS-Fusion/camera_models/camera_calib_example/
rosrun camera_models Calibrations -w 12 -h 8 -s 80 -i calibrationdata --camera-model pinhole
```

## 7. Docker Support
To further facilitate the building process, we add docker in our code. Docker environment is like a sandbox, thus makes our code environment-independent. To run with docker, first make sure [ros](http://wiki.ros.org/ROS/Installation) and [docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) are installed on your machine. Then add your account to `docker` group by `sudo usermod -aG docker $YOUR_USER_NAME`. **Relaunch the terminal or logout and re-login if you get `Permission denied` error**, type:
```
cd ~/catkin_ws/src/VINS-Fusion/docker
make build
```
Note that the docker building process may take a while depends on your network and machine. After VINS-Fusion successfully built, you can run vins estimator with script `run.sh`.
Script `run.sh` can take several flags and arguments. Flag `-k` means KITTI, `-l` represents loop fusion, and `-g` stands for global fusion. You can get the usage details by `./run.sh -h`. Here are some examples with this script:
```
# Euroc Monocualr camera + IMU
./run.sh ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml

# Euroc Stereo cameras + IMU with loop fusion
./run.sh -l ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml

# KITTI Odometry (Stereo)
./run.sh -k ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/

# KITTI Odometry (Stereo) with loop fusion
./run.sh -kl ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/

#  KITTI GPS Fusion (Stereo + GPS)
./run.sh -kg ~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml YOUR_DATASET_FOLDER/2011_10_03_drive_0027_sync/

```
In Euroc cases, you need open another terminal and play your bag file. If you need modify the code, simply re-run `./run.sh` with proper auguments after your changes.


## 8. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, a generic [camera model](https://github.com/hengli/camodocal) and [GeographicLib](https://geographiclib.sourceforge.io/).

## 9. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong Qin <qintonguavATgmail.com>.

For commercial inquiries, please contact Shaojie Shen <eeshaojieATust.hk>.
