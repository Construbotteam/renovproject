/**
 * @author [Li.Xiang]
 * @email [lix0419@outlook.com]
 * @create date 2020-09-08
 * @modify date 2020-09-08
 * @desc [description]
 */
#include <pcl/point_cloud.h>  //pcl
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>                  //ROS
#include <sensor_msgs/PointCloud2.h>  //sensor
#include <tf/tf.h>                    //tf
#include <tf/transform_listener.h>

#include <memory>  //STL
#include <string>

sensor_msgs::PointCloud2 cloud1, cloud2, cloud3;      //原始点云
sensor_msgs::PointCloud2 cloud2_in1, cloud3_in1;      //转换到相机1的点云
sensor_msgs::PointCloud2 cloud_1plus2, merged_cloud;  //合并后的点云

void merge_pointcloud(const ros::Publisher& cloud_pub) {
    while (ros::ok()) {  //避免程序一直被占用
        //相机之间的坐标变换
        tf::TransformListener tf12;
        tf::TransformListener tf13;

        boost::shared_ptr<sensor_msgs::PointCloud2 const> Ptr;
        //处理1号相机点云
        Ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect_1/depth/points", ros::Duration(1.0));
        if (!Ptr) {
            ROS_INFO("no pointcloud data received from kinect1,try again");
            return;
        } else {
            cloud1 = *Ptr;
        }

        //处理2号相机点云
        Ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect_2/depth/points", ros::Duration(1.0));
        if (!Ptr) {
            ROS_INFO("no pointcloud data received from kinect2");
            return;
        } else {
            cloud2 = *Ptr;
            tf12.waitForTransform("kinect_1_frame_optical", "kinect_2_frame_optical", ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud("kinect_1_frame_optical", cloud2, cloud2_in1, tf12);
            //1+2点云
            pcl::concatenatePointCloud(cloud1, cloud2_in1, cloud_1plus2);
        }

        // 处理3号相机点云
        Ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect_3/depth/points", ros::Duration(1.0));
        if (!Ptr) {
            ROS_INFO("no pointcloud data received from kinect3");
            return;
        } else {
            cloud3 = *Ptr;
            tf13.waitForTransform("kinect_1_frame_optical", "kinect_3_frame_optical", ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud("kinect_1_frame_optical", cloud3, cloud3_in1, tf13);
            //12+3点云
            pcl::concatenatePointCloud(cloud_1plus2, cloud3_in1, merged_cloud);
        }
        cloud_pub.publish(merged_cloud);
        ROS_INFO("Successful merger three point cloud data to topic /merged_cloud");
    }
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_pointcloud_node");
    ros::NodeHandle nh;
    ros::Publisher cloud_pub;
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("merged_cloud", 100);
    ros::Rate loop_rate(ros::Duration(1.0));
    merge_pointcloud(cloud_pub);  //用waitForMessage就不用写回调了
}