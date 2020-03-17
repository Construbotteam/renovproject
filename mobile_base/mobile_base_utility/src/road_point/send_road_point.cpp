#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "tf/tf.h"

struct StatusPose {
  geometry_msgs::PoseStamped pose;
  bool status;
};

typedef std::vector<StatusPose> StatusPoseVec;
typedef std::vector<StatusPoseVec> StatusPathVec;

typedef std::vector<geometry_msgs::PoseStamped> PoseStampedVec;

void getRoadPoints(const std::string& addr, PoseStampedVec& road_poses,
                   std::vector<bool>& poses_status, int* poses_num) {
  YAML::Node config = YAML::LoadFile(addr);

  int point_quantity = config["point_quantity"].as<int>();
  *poses_num = point_quantity;
  road_poses.resize(point_quantity);
  poses_status.resize(point_quantity);

  for (size_t i = 0; i < point_quantity; i++) {
    road_poses[i].header.frame_id = "map";
    road_poses[i].pose.position.x = config["x"][i].as<double>();
    road_poses[i].pose.position.y = config["y"][i].as<double>();
    poses_status[i] = config["poses_status"][i].as<bool>();

    double yaw = config["yaw"][i].as<double>();

    std::cout << "x: " << road_poses[i].pose.position.x
              << "  y: " << road_poses[i].pose.position.y << "  yaw: " << yaw
              << "  status: " << poses_status[i] << std::endl;
    road_poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "send_road_point");
  ros::NodeHandle nh;
  ros::NodeHandle nh_pirvate("~");

  bool if_reach_goal_pose;
  int pub_goal_freq;
  int path_index = 0;
  std::string path_index_param_name;
  std::string yaml_addr;
  nh_pirvate.param("path_index_param_name", path_index_param_name,
                   std::string("path_index"));
  nh_pirvate.param("pub_goal_freq", pub_goal_freq, 10);
  nh_pirvate.param("yaml_addr", yaml_addr, std::string(""));

  ros::Publisher goal_pub =
      nh.advertise<geometry_msgs::PoseStamped>("demo_goal", 10);

  while (true) {
    if (!nh.hasParam(path_index_param_name)) {
      ROS_WARN("no param : %s exists!", path_index_param_name.c_str());
      ros::Duration(1.0).sleep();
      continue;
    } else {
      break;
    }
  }

  PoseStampedVec road_poses;
  int poses_num;
  std::vector<bool> poses_status;
  getRoadPoints(yaml_addr, road_poses, poses_status, &poses_num);

  /********************/
  StatusPathVec road_path;
  road_path.clear();
  StatusPoseVec status_pose_vec_tmp;
  for (size_t i = 0; i < road_poses.size(); i++) {
    StatusPose status_pose_tmp;
    status_pose_tmp.pose = road_poses[i];
    status_pose_tmp.status = poses_status[i];
    status_pose_vec_tmp.push_back(status_pose_tmp);

    if (poses_status[i]) {
      road_path.push_back(status_pose_vec_tmp);
      status_pose_vec_tmp.clear();
    }
  }
  /********************/

  std::vector<size_t> real_poses_index_vec;
  real_poses_index_vec.clear();
  int real_poses_count = 0;
  for (size_t i = 0; i < road_poses.size(); i++) {
    if (poses_status[i]) {
      real_poses_index_vec.push_back(i);
      real_poses_count++;
    }
  }
  ROS_INFO("We have totally %d road points", (int)road_poses.size());
  ROS_INFO("We have totally %d REAL road points", real_poses_count);

  size_t index_tmp = 0;

  size_t intermediate_index_list[road_path.size()];
  for (size_t i = 0; i < road_path.size(); i++) {
    intermediate_index_list[i] = 0;
  }

  bool get_first_index = false;

  ros::Rate r(pub_goal_freq);

  if (!ros::param::has("if_reach_goal_pose")) {
    ros::param::set("if_reach_goal_pose", false);
  }

  while (ros::ok()) {
    nh.param(path_index_param_name, path_index, 0);
    if (path_index > real_poses_count || path_index <= 0) {
      ROS_WARN("index exceeds, index dropped");
      ros::Duration(0.5).sleep();
      continue;
    } else if (path_index > 0) {
      if (1 == road_path[path_index - 1].size()) {
        goal_pub.publish(road_path[path_index - 1][0].pose);
	ros::Duration(0.5).sleep();
	ros::param::set("search_port/inter_mobile_way_point_judge_status", 1);
      } else if (road_path[path_index - 1].size() > 1) {
        ros::param::get("if_reach_goal_pose", if_reach_goal_pose);

        if (intermediate_index_list[path_index - 1] >
            road_path[path_index - 1].size() - 1) {
          intermediate_index_list[path_index - 1] =
              road_path[path_index - 1].size() - 1;
        }

        goal_pub.publish(
            road_path[path_index - 1][intermediate_index_list[path_index - 1]]
                .pose);
      
        if (road_path[path_index - 1][intermediate_index_list[path_index - 1]].status) {
	  ros::Duration(0.5).sleep();
	  ros::param::set("search_port/inter_mobile_way_point_judge_status", 1);
	} else {
	  ros::Duration(0.5).sleep();
	  ros::param::set("search_port/inter_mobile_way_point_judge_status", 0);
        }

        if (if_reach_goal_pose) {
          intermediate_index_list[path_index - 1]++;
          ros::param::set("if_reach_goal_pose", false);
          if_reach_goal_pose = false;
        }
      }
      // if (!ros::param::has("if_reach_goal_pose")) {
      //   ros::param::set("if_reach_goal_pose", true);
      // }
      // ros::param::get("if_reach_goal_pose", if_reach_goal_pose);
      // if (if_reach_goal_pose) {
      //   if (1 == path_index) {
      //     if (real_poses_index_vec[path_index - 1] > 0) {
      //       if (index_tmp != real_poses_index_vec[path_index - 1]) {
      //         goal_pub.publish(road_poses[index_tmp]);
      //         index_tmp++;
      //         ros::param::set("if_reach_goal_pose", false);
      //         continue;
      //       } else {
      //         index_tmp = 0;
      //       }
      //     }
      //     goal_pub.publish(road_poses[real_poses_index_vec[path_index - 1]]);
      //     ros::param::set("if_reach_goal_pose", false);
      //   } else if (path_index > 1) {
      //     if (real_poses_index_vec[path_index - 1] -
      //             real_poses_index_vec[path_index - 2] !=
      //         1) {
      //       if (!get_first_index) {
      //         index_tmp = real_poses_index_vec[path_index - 2] + 1;
      //         get_first_index = true;
      //       }
      //       if (index_tmp != real_poses_index_vec[path_index - 1]) {
      //         goal_pub.publish(road_poses[index_tmp]);
      //         index_tmp++;
      //         ros::param::set("if_reach_goal_pose", false);
      //         continue;
      //       } else {
      //         get_first_index = false;
      //         index_tmp = 0;
      //       }
      //     }
      //     goal_pub.publish(road_poses[real_poses_index_vec[path_index - 1]]);
      //     ros::param::set("if_reach_goal_pose", false);
      //   }
      // }

      // goal_pub.publish(road_poses[real_poses_index_vec[path_index - 1]]);
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
