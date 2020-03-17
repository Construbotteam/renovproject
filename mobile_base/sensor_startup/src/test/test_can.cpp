#include "ros/ros.h"
#include "sensor_startup/controlcan.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_can");
    ros::NodeHandle nh;

    uint device_type = 3;
    uint devide_index = 0;

    int flag = VCI_OpenDevice(device_type, devide_index, 0);
    if (flag != 1) {
        ROS_WARN("open failure");
    }

    ros::Duration(0.5).sleep();

    return 0;
}

