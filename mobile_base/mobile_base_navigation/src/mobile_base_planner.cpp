#include "base_local_planner/trajectory_planner_ros.h"
#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "global_planner/planner_core.h"
#include "mobile_base_navigation/path_smoother.h"
#include "nav_core/base_global_planner.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/Float64.h"

#include <fstream>

namespace mobile_base {

class MobileBasePlannerROS {
 public:
  MobileBasePlannerROS(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                       tf2_ros::Buffer& bf);
  virtual ~MobileBasePlannerROS();
  void ParamInit(ros::NodeHandle& nh_private);
  void GetGoalCallback(const geometry_msgs::PoseStamped& goal_pose);
  void PlanCallback(const ros::TimerEvent&);
  bool StraightPlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    nav_msgs::Path* path);

 private:
  bool if_use_local_planner_;
  bool if_get_goal_;
  bool if_reach_goal_;
  bool send_reach_goal_flag_;
  double plan_frequency_;
  double angular_arrival_tolerance_;

  std::string path_pub_topic_;
  std::string cmd_vel_pub_topic_;
  std::string goal_sub_topic_;
  std::string path_frame_id_;
  std::string map_frame_id_;
  std::string base_frame_id_;

  std::string global_costmap_namespace_;
  std::string local_costmap_namespace_;
  std::string global_planner_namespace_;
  std::string local_planner_namespace_;
  std::string path_smoother_namespace_;

  ros::Publisher path_pub_;
  ros::Publisher smooth_path_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber goal_sub_;
  ros::Timer plan_timer_;

  ros::Publisher yaw_err_pub_;

  tf2_ros::Buffer& bf_;
  costmap_2d::Costmap2DROS* global_costmap_ros_ptr_;
  costmap_2d::Costmap2DROS* local_costmap_ros_ptr_;

  global_planner::GlobalPlanner base_global_planner_;
  base_local_planner::TrajectoryPlannerROS base_local_planner_;
  PathSmoother path_smoother_;

  geometry_msgs::PoseStamped goal_pose_;
};

MobileBasePlannerROS::MobileBasePlannerROS(ros::NodeHandle& nh,
                                           ros::NodeHandle& nh_private,
                                           tf2_ros::Buffer& bf)
    : bf_(bf) {
  ParamInit(nh_private);
  if_get_goal_ = false;
  if_reach_goal_ = false;
  send_reach_goal_flag_ = false;

  path_smoother_.Initialize(path_smoother_namespace_);
  // init global costmap and global_planner
  global_costmap_ros_ptr_ =
      new costmap_2d::Costmap2DROS(global_costmap_namespace_, bf_);
  global_costmap_ros_ptr_->pause();
  base_global_planner_.initialize(global_planner_namespace_,
                                  global_costmap_ros_ptr_);
  global_costmap_ros_ptr_->start();

  // init local costmap and local_planner
  if (if_use_local_planner_) {
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_pub_topic_, 10);
    local_costmap_ros_ptr_ =
        new costmap_2d::Costmap2DROS(local_costmap_namespace_, bf_);
    local_costmap_ros_ptr_->pause();
    base_local_planner_.initialize(local_planner_namespace_, &bf_,
                                   local_costmap_ros_ptr_);
    local_costmap_ros_ptr_->start();
  }

  // create publisher, subscriber and timer
  path_pub_ = nh.advertise<nav_msgs::Path>(path_pub_topic_, 10);
  yaw_err_pub_ = nh.advertise<std_msgs::Float64>("yaw_err", 10);
  smooth_path_pub_ = nh.advertise<nav_msgs::Path>("smooth_path", 10);
  goal_sub_ = nh.subscribe(goal_sub_topic_, 10,
                           &MobileBasePlannerROS::GetGoalCallback, this);
  plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_frequency_),
                               &MobileBasePlannerROS::PlanCallback, this);
}

MobileBasePlannerROS::~MobileBasePlannerROS() {
  if (global_costmap_ros_ptr_) {
    global_costmap_ros_ptr_->stop();
    delete global_costmap_ros_ptr_;
  }
  if (if_use_local_planner_ || local_costmap_ros_ptr_) {
    local_costmap_ros_ptr_->stop();
    delete local_costmap_ros_ptr_;
  }
}

void MobileBasePlannerROS::ParamInit(ros::NodeHandle& nh_private) {
  nh_private.param("if_use_local_planner", if_use_local_planner_, false);

  nh_private.param("path_pub_topic", path_pub_topic_,
                   std::string("mobile_base_path"));
  nh_private.param("goal_sub_topic", goal_sub_topic_,
                   std::string("mobile_base_goal"));
  nh_private.param("cmd_vel_pub_topic", cmd_vel_pub_topic_,
                   std::string("cmd_vel"));
  nh_private.param("path_frame_id", path_frame_id_, std::string("map"));
  nh_private.param("map_frame_id", map_frame_id_, std::string("map"));
  nh_private.param("base_frame_id", base_frame_id_, std::string("base_link"));

  nh_private.param("global_costmap_namespace", global_costmap_namespace_,
                   std::string(""));
  nh_private.param("local_costmap_namespace", local_costmap_namespace_,
                   std::string(""));
  nh_private.param("global_planner_namespace", global_planner_namespace_,
                   std::string(""));
  nh_private.param("local_planner_namespace", local_planner_namespace_,
                   std::string(""));
  nh_private.param("path_smoother_namespace", path_smoother_namespace_,
                   std::string(""));
  nh_private.param("plan_frequency", plan_frequency_, 0.5);
  nh_private.param("angular_arrival_tolerance", angular_arrival_tolerance_,
                   0.07);
}

void MobileBasePlannerROS::GetGoalCallback(
    const geometry_msgs::PoseStamped& goal_pose) {
  goal_pose_ = goal_pose;
  if_get_goal_ = true;
}

void MobileBasePlannerROS::PlanCallback(const ros::TimerEvent&) {
  if (!if_get_goal_) {
    //ROS_WARN("Please provide a goal point!!!");
    return;
  }

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped tfs;

  int tmp_count = 0;
  while (true) {
    if (tf_buffer.canTransform(map_frame_id_, base_frame_id_, ros::Time(0),
                               ros::Duration(0.2))) {
      try {
        tfs = tf_buffer.lookupTransform(map_frame_id_, base_frame_id_,
                                        ros::Time(0), ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("error in mobile planner -> %s", ex.what());
        ros::Duration(1.0).sleep();
        return;
      }
      break;
    } else {
      tmp_count++;
      ROS_WARN("mobile planner error -> no transform, read again : %d",
               tmp_count);
      if (tmp_count > 10) {
        ROS_WARN("mobile planner error -> no transform, read failure");
        exit(0);
      }
      ros::Duration(0.2).sleep();
      continue;
    }
  }

  geometry_msgs::PoseStamped start;
  start.header = goal_pose_.header;
  start.pose.position.x = tfs.transform.translation.x;
  start.pose.position.y = tfs.transform.translation.y;
  start.pose.position.z = tfs.transform.translation.z;

  start.pose.orientation.x = tfs.transform.rotation.x;
  start.pose.orientation.y = tfs.transform.rotation.y;
  start.pose.orientation.z = tfs.transform.rotation.z;
  start.pose.orientation.w = tfs.transform.rotation.w;

  double cur_yaw = tf::getYaw(start.pose.orientation);
  double goal_yaw = tf::getYaw(goal_pose_.pose.orientation);

  std_msgs::Float64 yaw_err_tmp;
  yaw_err_tmp.data = cur_yaw - goal_yaw;
  //yaw_err_pub_.publish(yaw_err_tmp);

  double start_goal_dis =
      sqrt(pow(start.pose.position.x - goal_pose_.pose.position.x, 2) +
           pow(start.pose.position.y - goal_pose_.pose.position.y, 2));
  double limit = 2.0 * global_costmap_ros_ptr_->getCostmap()->getResolution();
  if (start_goal_dis < limit) {
    /*ROS_WARN(
        "start point and goal point are too close whose distance is : %.4f, "
        "greater than limit: %.4f, no "
        "path output",
        start_goal_dis, limit);
*/
    int tmp_status;
    while (true) {
      if (ros::param::has("search_port/inter_mobile_way_point_judge_status")) {
        break;
      } else {
        ros::Duration(0.2).sleep();
      }
    }
    ros::param::get("search_port/inter_mobile_way_point_judge_status", tmp_status);

    if (fabs(cur_yaw - goal_yaw) < angular_arrival_tolerance_ && send_reach_goal_flag_ && tmp_status == 0) {
      ROS_INFO("arrive the pose");
      ros::param::set("search_port/mobile_way_point_judge_status", tmp_status);
      ros::Duration(1.0).sleep();
      ros::param::set("if_reach_goal_pose", true);
      send_reach_goal_flag_ = false;
      return;
    }

    ros::param::set("search_port/mobile_way_point_judge_status", tmp_status);
    return;
  }
  ros::param::set("search_port/mobile_way_point_judge_status", 0);
  ros::param::set("if_reach_goal_pose", false);
  send_reach_goal_flag_ = true;

  /********************************/
  //
  nav_msgs::Path straight_path;
  if (StraightPlan(start, goal_pose_, &straight_path)) {


    std_msgs::Float64 first_angle;
    first_angle.data = tf::getYaw(straight_path.poses[0].pose.orientation);
    //  attention !!!! this is a trick !!!!
    yaw_err_pub_.publish(first_angle);



    std::ofstream out("/home/curi/Desktop/first_direc.txt", std::ios::app);
    double first_yaw = tf::getYaw(straight_path.poses[0].pose.orientation);
    out << first_yaw  << std::endl;
    out.close();
    path_pub_.publish(straight_path);
    return;
  }

  
  /********************************/

  std::vector<geometry_msgs::PoseStamped> pose_vec;
  if (!base_global_planner_.makePlan(start, goal_pose_, pose_vec)) {
    ROS_WARN("plan failure by base global planner");
    return;
  }

  /********************/
  //
  double x_media, y_media;
  x_media = pose_vec[1].pose.position.x;
  y_media = pose_vec[1].pose.position.y;
  
  double custom_angle;
  custom_angle = atan2(y_media - pose_vec[0].pose.position.y, x_media - pose_vec[0].pose.position.x);
  pose_vec[0].pose.orientation = tf::createQuaternionMsgFromYaw(custom_angle);
   

  /********************/

  ROS_INFO("START : [x: %.4f, y: %.4f, z: %.4f]", start.pose.position.x,
           start.pose.position.y, start.pose.position.z);
  ROS_INFO("GOAL  : [x: %.4f, y: %.4f, z: %.4f]", goal_pose_.pose.position.x,
           goal_pose_.pose.position.y, goal_pose_.pose.position.z);

  if (if_use_local_planner_) {
    geometry_msgs::Twist command_velo;

    base_local_planner_.setPlan(pose_vec);
    base_local_planner_.computeVelocityCommands(command_velo);
    cmd_vel_pub_.publish(command_velo);
  }

  nav_msgs::Path path;
  path.header.frame_id = path_frame_id_;
  path.header.stamp = pose_vec[0].header.stamp;
  for (size_t i = 0; i < pose_vec.size(); i++) {
    geometry_msgs::PoseStamped pose_stamped_tmp;
    pose_stamped_tmp.header = pose_vec[i].header;
    pose_stamped_tmp.pose = pose_vec[i].pose;

    path.poses.push_back(pose_stamped_tmp);
  }
  /*
    nav_msgs::Path final_path;
    if (!path_smoother_.UpdatePath(start, path, &final_path)) {
      ROS_ERROR("Get smooth plan failure, no path output");
      return ;
    }
    */
  path_pub_.publish(path);
  // smooth_path_pub_.publish(final_path);
}

bool MobileBasePlannerROS::StraightPlan(const geometry_msgs::PoseStamped& start,
                                        const geometry_msgs::PoseStamped& goal,
                                        nav_msgs::Path* path) {
  double x1, x2, y1, y2;
  x1 = start.pose.position.x;
  y1 = start.pose.position.y;
  x2 = goal.pose.position.x;
  y2 = goal.pose.position.y;

  path->header.frame_id = map_frame_id_;
  path->header.stamp = ros::Time::now();
  path->poses.clear();
  path->poses.push_back(start);

  double angle = atan2(y2 - y1, x2 - x1);
  double dis_increment = 0.05;
  int max_point_num = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)) / dis_increment;
  while (true) {
    double x_tmp =
        path->poses.back().pose.position.x + cos(angle) * dis_increment;
    double y_tmp =
        path->poses.back().pose.position.y + sin(angle) * dis_increment;

    geometry_msgs::PoseStamped path_point;
    path_point.header = start.header;
    path_point.pose.position.x = x_tmp;
    path_point.pose.position.y = y_tmp;
    // path_point.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

    path->poses.push_back(path_point);

    double rest_dis = sqrt(pow(x2 - x_tmp, 2) + pow(y2 - y_tmp, 2));
    if (rest_dis < dis_increment) {
      path->poses.push_back(goal);
      break;
    }
    if (path->poses.size() > max_point_num * 2) {
      ROS_WARN("path point num exceeds");
      return false;
    }
  }
  for (size_t i = 0; i < path->poses.size() - 1; i++) {
    path->poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  }
  /*
  double goal_yaw_tmp  = tf::getYaw(goal.pose.orientation);
  for (size_t i = 0; i < path->poses.size() - 1; i++) {
    path->poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw_tmp);
  }
  */
/*
  costmap_2d::Costmap2D* costmap_tmp = global_costmap_ros_ptr_->getCostmap();
  for (size_t i = 0; i < path->poses.size(); i++) {
    double wx = path->poses[i].pose.position.x;
    double wy = path->poses[i].pose.position.y;
    unsigned int mx, my;
    unsigned int cost_val;
    if (costmap_tmp->worldToMap(wx, wy, mx, my)) {
      cost_val = costmap_tmp->getCost(mx, my);
      if (0 != cost_val) {
        ROS_WARN(
            "a point on the straight path is located in obstacles, get "
            "straight path failure");
        return false;
      }
    } else {
      ROS_WARN("unable to get cost value!!");
      return false;
    }
  }
  */
  return true;
}

};  // namespace mobile_base

int main(int argc, char** argv) {
  ros::init(argc, argv, "mobile_base_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  mobile_base::MobileBasePlannerROS planner(nh, nh_private, buffer);

  ros::spin();

  return 0;
}
