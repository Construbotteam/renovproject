## 联合仿真：roslaunch paintingrobot_description bringup_moveit.launch 
## 修改gazebo的环境：/paintingrobot_description/launch/gazebo_world.launch
## 只在rviz中显示模型：roslaunch paintingrobot_description display.launch 
## Gazebo仿真： roslaunch paintingrobot_description bringup_gazebo.launch
## Gmapping SLAM建图: roslaunch paintingrobot_slam paintingrobot_slam.launch
## 保存地图： rosrun map_server map_saver -f <文件保存路径>/<文件name>
## Navigation： roslaunch paintingrobot_navigation paintingrobot_navigation.launch

## 安装项目依赖项：
cd ~/${workspace}/src

rosdep install -y --from-paths . --ignore-src --rosdistro melodic

## 
修改/添加 传感器、执行器：paintingrobot_description/urdf/paintingrobot_witharm.xacro
修改机器人urdf：paintingrobot_description/urdf/base/
修改传感器参数：paintingrobot_description/urdf/sensors/

在rviz中可以显示相应传感器数据（可以在传感器xacro文件修改对应topic命名空间）：
激光雷达：
/scan
kinect相机：

/kinect/depth/camera_info

/kinect/depth/image_raw

/kinect/depth/points

/kinect/parameter_descriptions

/kinect/parameter_updates

/kinect/rgb/camera_info

/kinect/rgb/image_raw

/kinect/rgb/image_raw/compressed

/kinect/rgb/image_raw/compressed/parameter_descriptions

/kinect/rgb/image_raw/compressed/parameter_updates

/kinect/rgb/image_raw/compressedDepth

/kinect/rgb/image_raw/compressedDepth/parameter_descriptions

/kinect/rgb/image_raw/compressedDepth/parameter_updates

/kinect/rgb/image_raw/theora

/kinect/rgb/image_raw/theora/parameter_descriptions

/kinect/rgb/image_raw/theora/parameter_updates

zed双目：

/zed/depth/camera_info

/zed/depth/image_raw

/zed/depth/points

/zed/parameter_descriptions

/zed/parameter_updates

/zed/rgb/camera_info

/zed/rgb/image_raw

/zed/rgb/image_raw/compressed

/zed/rgb/image_raw/compressed/parameter_descriptions

/zed/rgb/image_raw/compressed/parameter_updates

/zed/rgb/image_raw/compressedDepth

/zed/rgb/image_raw/compressedDepth/parameter_descriptions

/zed/rgb/image_raw/compressedDepth/parameter_updates

/zed/rgb/image_raw/theora

/zed/rgb/image_raw/theora/parameter_descriptions

/zed/rgb/image_raw/theora/parameter_updates
