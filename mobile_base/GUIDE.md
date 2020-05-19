# The guide for mobile base operation

## 1 SLAM
### 1.1 Start the sensors
Set up authority of serial port
```terminal
$ sudo chmod 777 /dev/ttyUSB*
```
Startup all sensors
```terminal
$ roslaunch sensor_startup sick_sensor_bringup.launch
```

This launch includes:

A) Driver of imu
```terminal
$ roslaunch sensor_startup imu_bringup.launch
```
B) Driver of LiDAR Sick Tim571
```terminal
$ roslaunch sick_tim sick_tim571_2050101.launch
```
C) Static transformation of sensor external parameters
```
$ roslaunch mobile_base_description static_transformation.launch
```

### 1.2 Apply SLAM
Source the setup.bash
```terminal
$ source <absolute-path-of-the-setup-bash-file-of-cartographer-ros>
```
or simply (please check the alias setting in ~/.bashrc before using)
```terminal
$ source_cart
```
Start SLAM
```terminal
$ roslaunch cartographer_ros cartographer_demo_rplidar.launch
```
Save the map with name \<file-name\>
```terminal
$ rosservice call /finish_trajectory 0
$ rosservice call /write_state "filename: '<path>/<file-name>.pbstream'"
```

### 1.3 Apply localization
Start pure-localization node 
(remember to modify the path of pbstream file in this launch file with new name first)
```terminal
$ roslaunch cartographer_ros cartographer_demo_rplidar_localization_sick571.launch
```
Control the base manually to get the correct pose before launching the navigation node 

## 2 Motor 
Startup the driver
```terminal
$ roslaunch sensor_startup kinco_motor_bringup.launch
```
Start the tele-operation node
```terminal
$ roslaunch mobile_base_teleop teleop.launch
```
Disable the motor drivers and switch off CAN-Analyst
```terminal
$ rosrun sensor_startup stop_motor
```

## 3 Navigation
```terminal
$ roslaunch mobile_base_navigation diff_base_navigation.launch
```
