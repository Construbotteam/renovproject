## Notification
Hi,This is lz repo. Including a sample omnidirection path tracking controller,zhouligong CAN analysis driver,a 8 motors driver,aubo5 or aubo10 manipulator driver,a sample end-effector sector planning alogrithm,a 3-DOF climbing robot driver with 485 and a plc driver node.

**For getting more details,Please see the documentation folder.**

The painting_robot_demo folder is the main folder,you can just clone to your ROS pkg src folder to use them.
The config foler contains the common plc parameter and mobile platform parameters for initial all node
The lib folder contains zhouligong CAN analysis Dynamic-link library.
The robot folder will contain the robot program
The scripts folder is also another main foler. The 3dof_flex_bar_homing folder is used for homing all robot except mobile platform.
The 3dof_platform_driver folder and aubo_manipulator_driver contains 3DOF dirver and aubo manipulator driver respectively. 
The coverage_path_planning folder is *Empty* right now, 
The list_all_parameters folder contains a noder to show all ROS parameter server's parameters.Because we maintain a huge parameter server' parameters form to control all those thing.
The mobile_platform_driver and painting_opreating_planning use for control mobile and painting tools.
The plc_package folder is used for PLC board control.
The usb_port_fixed_node folder is a node for choosing USB port automaticall of Ubuntu 18.0.4(Recommended)/16.0.4

## Catkin_Make
To build all code:
```
cd your ros workspace
catkin_make
```

Please note,if you wanna use aubo driver with your own computer,you must use the shell script named initial_aubo_driver.sh
```
cd painting_robot_demo/scripts/aubo_manipulator_driver/libpyauboi5-v1.2.2.x64
sudo sh initial_aubo_driver.sh
```
otherwise it will not work. And before you control the Robot,you must know all parameters for this platform such as

```
    <!--set /plc_pkg_para/top_limit_switch_status 1-on -->
    <param name="read_buffer_bytes_num" value="6" />
    <!--set /plc_pkg_para/top_limit_switch_status 1-on -->
    <param name="top_limit_switch_status" value="0" />
    <!--set /plc_pkg_para/mid_limit_switch_status 1-on -->
    <param name="mid_limit_switch_status" value="0" />
    <!--set /plc_pkg_para/bottom_limit_switch_status 1 on-->
    <param name="bottom_limit_switch_status" value="0" />
    <param name="climb_distance_tracking_over" value="0" />
    <!-- anticlockwise abs encode increase -->
    <param name="rotation_abs_encode" value="0" />
    <param name="rotation_distance_tracking_over" value="0" />


```
We have a complex system with more than 50 parameters,of course we need to optimazting this. Ha ha.
## Run simulator
To run the simulator:
1. you need to change another repo name painting_robot_description
```
roslaunch *****
```

## Dependencies:
- Ros Melodic - http://wiki.ros.org/melodic
- Pointcloud - http://pointclouds.org/ 
- `Zlg Can`
- `Aubo driver`



