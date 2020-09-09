#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "load_pcdmap");
    ros::NodeHandle nh;
    //建一个点云指针,类型是有颜色的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    //导入点云

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/sean/cloud0908_voxel0.02.pcd", *Ptr) == -1) {
        ROS_ERROR("检查/home/sean/cloud0908_voxel0.02.pcd文件\n");
        return (-1);
    }
    //发布点云
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcd", 1);
    sensor_msgs::PointCloud2 cloud_ros;
    pcl::toROSMsg(*Ptr, cloud_ros);  //点云转到ROS格式

    cloud_ros.header.frame_id = "odom";
    std::cout << "导入成功,点云发布在/pcd话题" << std::endl;
    while (ros::ok()) {
        pubCloud.publish(cloud_ros);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}