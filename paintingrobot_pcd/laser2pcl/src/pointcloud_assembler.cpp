/*
 * cloud_assembler.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: dimitri prosser
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

using namespace pcl;

namespace cloud_assembler
{

    typedef PointCloud<PointXYZ> PointCloud2;

    class CloudAssembler
    {

    public:
        CloudAssembler();
        void cloudCallback(const sensor_msgs::PointCloud2 &cloud);

    private:
        ros::NodeHandle node_;

        ros::ServiceServer pause_srv_;

        ros::Publisher output_pub_;
        ros::Subscriber cloud_sub_;

        tf::TransformListener tf_;

        PointCloud2 assembled_cloud_;
        int buffer_length_;
        std::vector<sensor_msgs::PointCloud2> cloud_buffer_;
        bool assemblerPaused_;

        void addToBuffer(sensor_msgs::PointCloud2 cloud);
        void assembleCloud();
        bool pauseSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    };

    CloudAssembler::CloudAssembler()
    {
        ros::NodeHandle private_nh("~");

        private_nh.param("buffer_length", buffer_length_, 50);

        output_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/assembled_cloud", 100);

        pause_srv_ = node_.advertiseService("/pause_assembler", &CloudAssembler::pauseSrv, this);

        cloud_sub_ = node_.subscribe("/input_cloud", 100, &CloudAssembler::cloudCallback, this);

        PointCloud2 clear;
        assembled_cloud_ = clear;

        assemblerPaused_ = false;
    }

    void CloudAssembler::cloudCallback(const sensor_msgs::PointCloud2 &cloud)
    {
        addToBuffer(cloud);
        assembleCloud();

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(assembled_cloud_, cloud_msg);

        cloud_msg.header.frame_id = cloud.header.frame_id;
        cloud_msg.header.stamp = ros::Time::now();

        output_pub_.publish(cloud_msg);
    }

    void CloudAssembler::assembleCloud()
    {
        ROS_DEBUG("Assembling.");

        unsigned int i;

        if (assemblerPaused_)
        {
            ROS_INFO("assemblerPaused_ is true");
        }
        if (!assemblerPaused_)
        {
            ROS_DEBUG("assemblerPaused_ is false");
        }

        std::string fixed_frame = cloud_buffer_[0].header.frame_id;

        PointCloud2 new_cloud;
        new_cloud.header.frame_id = fixed_frame;
        new_cloud.header.stamp = ros::Time::now();

        for (i = 0; i < cloud_buffer_.size(); i++)
        {
            PointCloud2 temp_cloud;
            pcl::fromROSMsg(cloud_buffer_[i], temp_cloud);
            temp_cloud.header.frame_id = fixed_frame;
            new_cloud += temp_cloud;
        }

        // If it's paused, don't overwrite the stored cloud with a new one, just keep publishing the same cloud
        if (!assemblerPaused_)
        {
            assembled_cloud_ = new_cloud;
        }
        else if (assemblerPaused_)
        {
            ROS_DEBUG("The Assembler will continue to publish the same cloud.");
        }
    }

    bool CloudAssembler::pauseSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
    {
        ROS_DEBUG("In service call: %s", assemblerPaused_ ? "true" : "false");

        if (!assemblerPaused_)
        {
            ROS_DEBUG("Now paused.");
            assemblerPaused_ = true;
        }
        else if (assemblerPaused_)
        {
            assemblerPaused_ = false;
            ROS_DEBUG("Unpaused.");
        }

        return true;
    }

    void CloudAssembler::addToBuffer(sensor_msgs::PointCloud2 cloud)
    {
        ROS_DEBUG("Adding cloud to buffer. Current buffer length is %d", cloud_buffer_.size());

        if (cloud_buffer_.size() >= (unsigned int)buffer_length_)
        {
            cloud_buffer_.erase(cloud_buffer_.begin());
        }

        cloud_buffer_.push_back(cloud);
    }

}; // namespace cloud_assembler

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_assembler");
    cloud_assembler::CloudAssembler cloud_assembler;

    ros::spin();

    return 0;
}