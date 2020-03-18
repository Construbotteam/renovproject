#include "base_local_planner/trajectory_planner_ros.h"
#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "global_planner/planner_core.h"
#include "mobile_base_navigation/diff_planner.h"
#include "nav_core/base_global_planner.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf2_ros/transform_listener.h"
#include "mobile_base_navigation/broke_line.h"

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
  bool if_use_local_planner_, if_get_goal_, if_reach_goal_;
  bool valid_traj_;
  double plan_frequency_;
  double angular_arrival_tolerance_;

  std::string path_pub_topic_, cmd_vel_pub_topic_, goal_sub_topic_;
  std::string path_frame_id_, map_frame_id_, base_frame_id_;

  std::string global_costmap_namespace_, local_costmap_namespace_;
  std::string global_planner_namespace_, diff_planner_namespace_,
      local_planner_namespace_;

  ros::Publisher path_pub_, smooth_path_pub_, cmd_vel_pub_;
  ros::Publisher new_path_pub_;
  ros::Subscriber goal_sub_;
  ros::Timer plan_timer_;

  tf2_ros::Buffer& bf_;
  costmap_2d::Costmap2DROS* global_costmap_ros_ptr_;
  costmap_2d::Costmap2DROS* local_costmap_ros_ptr_;

  global_planner::GlobalPlanner base_global_planner_;
  base_local_planner::TrajectoryPlannerROS base_local_planner_;
  mobile_base::DiffPlanner diff_local_planner_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  BrokeLine breaker_;

  geometry_msgs::PoseStamped goal_pose_;
  nav_msgs::Path tmp_path_;
  ros::Time way_block_time_;

  std::ofstream out_;
};

MobileBasePlannerROS::MobileBasePlannerROS(ros::NodeHandle& nh,
                                           ros::NodeHandle& nh_private,
                                           tf2_ros::Buffer& bf)
    : bf_(bf), if_get_goal_(false), if_reach_goal_(false), valid_traj_(true) {
  ParamInit(nh_private);
  way_block_time_ = ros::Time::now();
  global_plan_.clear();
  out_.open("/home/glh/Desktop/valid.txt", std::ios::app);
  out_.clear();
  // init global costmap and global_planner
  global_costmap_ros_ptr_ =
      new costmap_2d::Costmap2DROS(global_costmap_namespace_, bf_);
  global_costmap_ros_ptr_->pause();
  base_global_planner_.initialize(global_planner_namespace_,
                                  global_costmap_ros_ptr_);
  global_costmap_ros_ptr_->start();

  // init local costmap and local_planner
  // if (if_use_local_planner_) {
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_pub_topic_, 10);
  local_costmap_ros_ptr_ =
      new costmap_2d::Costmap2DROS(local_costmap_namespace_, bf_);
  local_costmap_ros_ptr_->pause();
  diff_local_planner_.initialize(diff_planner_namespace_, &bf_,
                                 local_costmap_ros_ptr_);
  base_local_planner_.initialize(local_planner_namespace_, &bf_,
                                 local_costmap_ros_ptr_);
  local_costmap_ros_ptr_->start();
  // }

  breaker_.initialize(global_costmap_ros_ptr_, nh);

  // create publisher, subscriber and timer
  path_pub_ = nh.advertise<nav_msgs::Path>(path_pub_topic_, 10);
  new_path_pub_ = nh.advertise<nav_msgs::Path>("new_path", 10);
  // smooth_path_pub_ = nh.advertise<nav_msgs::Path>("smooth_path", 10);
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

  out_.close();
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
                   std::string("global_costmap"));
  nh_private.param("local_costmap_namespace", local_costmap_namespace_,
                   std::string("local_costmap"));
  nh_private.param("global_planner_namespace", global_planner_namespace_,
                   std::string("global_base_planner"));
  nh_private.param("diff_planner_namespace", diff_planner_namespace_,
                   std::string("diff_local_planner"));
  nh_private.param("local_planner_namespace", local_planner_namespace_,
                   std::string("base_local_planner"));
  nh_private.param("plan_frequency", plan_frequency_, 0.5);
  ROS_INFO("plan freq : %.3f", plan_frequency_);
  nh_private.param("angular_arrival_tolerance", angular_arrival_tolerance_,
                   0.07);
}

void MobileBasePlannerROS::GetGoalCallback(
    const geometry_msgs::PoseStamped& goal_pose) {
  goal_pose_ = goal_pose;
  if_get_goal_ = true;
}

void MobileBasePlannerROS::PlanCallback(const ros::TimerEvent&) {
  // if (!if_get_goal_) {
  //   // ROS_WARN("Please provide a goal point!!!");
  //   return;
  // }

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped tfs;

  // ROS_INFO("get robot pose: %.4f", ros::Time::now().toSec());
  geometry_msgs::PoseStamped start;
  if (!global_costmap_ros_ptr_->getRobotPose(start)) {
    ROS_WARN("get robot pose failure");
    return;
  }

  double cur_yaw = tf::getYaw(start.pose.orientation);
  double goal_yaw = tf::getYaw(goal_pose_.pose.orientation);

  std::vector<geometry_msgs::PoseStamped> new_path;

  bool plan_again;
  if (!if_reach_goal_) {
    plan_again = fabs(ros::Time::now().toSec() - way_block_time_.toSec()) > 1.0;
  } else {
    plan_again = false;
  }

  if (if_get_goal_ || plan_again) {
    global_plan_.clear();
    if (!base_global_planner_.makePlan(start, goal_pose_, global_plan_) ||
        !breaker_.breakPath(global_plan_, new_path)) {
      if_get_goal_ = false;
      ROS_WARN(
          "plan failure by base global planner, please offer a goal again");
      return;
    } else if (global_plan_.size() < 2 ||
               !diff_local_planner_.setPlan(new_path) ||
               !base_local_planner_.setPlan(new_path)) {
      //  !diff_local_planner_.setPlan(global_plan_)) {
      ROS_WARN("length of global plan is %d or set plan failure",
               (int)global_plan_.size());
      return;
    } else {
      diff_local_planner_.enableFirstRotation();
      ROS_INFO("make global plan and set it local planner");
    }
    if_get_goal_ = false;
  } else if (global_plan_.size() < 2) {
    return;
  }

  geometry_msgs::Twist command_velo;
  if_reach_goal_ = diff_local_planner_.isGoalReached();
  if (!if_reach_goal_) {
    // ROS_INFO("goal not reached");
  } else {
    command_velo.angular.x = command_velo.angular.y = command_velo.angular.z =
        0.0;
    command_velo.linear.x = command_velo.linear.y = command_velo.linear.z = 0.0;
    cmd_vel_pub_.publish(command_velo);
    geometry_msgs::PoseStamped final_pose;
    global_costmap_ros_ptr_->getRobotPose(final_pose);
    double position_err =
        hypot(final_pose.pose.position.x - goal_pose_.pose.position.x,
              final_pose.pose.position.y - goal_pose_.pose.position.y);
    double angular_err = tf::getYaw(goal_pose_.pose.orientation) -
                         tf::getYaw(final_pose.pose.orientation);
    ROS_INFO("goal reached, position err : %.4f,  angular err : %.4f",
             position_err, angular_err / M_PI * 180.0);
    return;
  }

  if (!diff_local_planner_.computeVelocityCommands(command_velo)) {
    ROS_WARN("compute velo failure");
    return;
  }
  double v_check, vth_check;
  // v_check = command_velo.linear.x;
  
  v_check =
      std::max(fabs(command_velo.linear.x), diff_local_planner_.getMaxV());
  vth_check = sign(command_velo.angular.z) *
              std::max(fabs(command_velo.angular.z),
                       diff_local_planner_.getMaxThetaV());

  valid_traj_ = base_local_planner_.checkTrajectory(v_check, 0.0, vth_check);
  if (valid_traj_) {
    way_block_time_ = ros::Time::now();
  }
  diff_local_planner_.getCollision(!valid_traj_);

  cmd_vel_pub_.publish(command_velo);
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
