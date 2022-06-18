# ROS_BASALT

This is a ROS wrapper for the [Basalt project](https://gitlab.com/VladyslavUsenko/basalt). It's state-of-the-art visual inertial odometry with open source code release.

This wrapper includes the newest code release from the [Basalt project](https://gitlab.com/VladyslavUsenko/basalt) including:

* **Square Root Marginalization for Sliding-Window Bundle Adjustment**, N. Demmel, D. Schubert, C. Sommer, D. Cremers, V. Usenko, In 2021 International Conference on Computer Vision (ICCV), [[arXiv:2109.02182]](https://arxiv.org/abs/2109.02182)

---
## Attention

Please refer to the [Basalt project](https://gitlab.com/VladyslavUsenko/basalt)  for details of code implementation and relevant documentations and publications.

This wrapper is made specifically to combine Basalt VIO with Intel REALSENSE T265 while publishing to ROS. Therefore, only a portion of the scripts in the original project are included(performance testing on datasets, testing modules and etc are not included)

---
## Tested environment

ROS Noetic on Ubuntu 20.04.4 LTS with Intel REALSENSE T265

---
## Installation

1. Clone this package to your catkin workspace src folder

2. Install the libraries in the thirdparty folder if you don't already have them in your enviroment:
    ```
    cd to_the_package_not_yet_installed
    ```
    ```
    mkdir build 
    ```
    ```
    cd build
    ```
    ```
    cmake ..
    ```
    ```
    sudo make install
    ```
3. Install librealsense following this [link](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

4. Build the ros package 
    ```
    catkin_make --pkg ros_basalt
    ```

5. Launch
    ```
    roslaunch ros_basalt rs_t265_vio.launch
    ```
    a. Raw IMU readings are published to `rs_t265_imu`. Note that pose estimation by T265 itself is disabled

    b. Camera images are published to `rs_t265/cam0` and `rs_t265/cam1`

    c. Pose estimation by Basalt VIO is published to `basalt/pose`

    d. Linear velocity estimation by Basalt VIO is published to `basalt/velocity`
