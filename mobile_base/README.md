# renov_robot

navigation stack of renovation robot

## 1 Author

name : glh

mail : georgeglh@link.cuhk.edu.hk

## 2 Third-party pkgs

### 2.1 LiDAR driver

reference document : <http://wiki.ros.org/sick_tim>

clone from git : 

```shell
$ git clone https://github.com/uos/sick_tim.git
```

### 2.2 open source SLAM

reference document : <http://wiki.ros.org/gmapping>

clone from git (openslam_gmapping) : 

```shell
$ git clone https://github.com/ros-perception/openslam_gmapping.git
```

clone from git(gmapping) : 

```shell
$ git clone https://github.com/ros-perception/slam_gmapping.git
```
or :
```shell
$ sudo apt-get install ros-kinetic-<distro>-gmapping
```

### 2.3 robot_pose_ekf

reference document : <http://wiki.ros.org/robot_pose_ekf>

get pkg: 
```shell
$ sudo apt-get install ros-<distro>-robot-pose-ekf
```

### 2.4 serial port communication

get pkg: 
```shell
$ sudo apt-get install ros-<distro>-serial
```

### 2.5 imu_tools

reference document : <http://wiki.ros.org/imu_tools>

clone from git :
```shell
$ git clone https://github.com/ccny-ros-pkg/imu_tools.git
```

## 3 Pkgs && Nodes

### 3.1 sensor_startup

sensor drivers for applying or testing sensors
  - For motor driver with CANOpen protocol based on CAN analyst, reinstall the driver of CH340

#### 3.1.1 Node : motor_core

  - subscribe :
  - publish :
  - tf :


### 3.2 mobile_base_slam

launch slam node for base navigation

### 3.3 mobile_base_navigation

navigation scheme including environment perception

### 3.4 mobile_base_description

external paramters describing links of robot

### 3.5 mobile_base_controller

path tracking scheme

### 3.6 mobile_base_teleop

key-borad tele-operation of mobile base