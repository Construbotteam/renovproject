#include "mobile_base_controller/MoveBase.h"

using mobile::MoveBase;
using mobile::Pose2dVec;



MoveBase::MoveBase() {
  
  n_private = ros::NodeHandle("mobile");
  path.clear();
  get_path = false;
  ParamInit();
  Setup();

}

void MoveBase::PathCallback(const nav_msgs::Path& path_msg) {

  Pose2d path_point;
  for (size_t i = 0; i < path_msg.poses.size(); i++) {
    Eigen::Quaterniond q(path_msg.poses[i].pose.orientation.w,
                         path_msg.poses[i].pose.orientation.x,
                         path_msg.poses[i].pose.orientation.y,
                         path_msg.poses[i].pose.orientation.z);
    double yaw = q.toRotationMatrix().eulerAngles(0,1,2)[2];
    path_point.x = path_msg.poses[i].pose.position.x;
    path_point.y = path_msg.poses[i].pose.position.y;
    path_point.yaw = yaw;
    path.push_back(path_point);
  }
  get_path = true;

}

void MoveBase::PoseCallback(const ros::TimerEvent&) {

  tf::TransformListener tl;
  // frameA2frameB refers to pose of frameA w.r.t. frameB
  tf::StampedTransform base2map;
  ros::Time stamp = ros::Time(0);
  try {
    tl.waitForTransform(map_frame_id, 
                        base_frame_id, 
                        stamp, 
                        ros::Duration(0.5));
    tl.lookupTransform(map_frame_id,
                       base_frame_id,
                       stamp,
                       base2map);      
  } catch (const tf::TransformException& ex) {
    ROS_ERROR("Get error in node move base : %s", ex.what());
    ros::Duration(0.2).sleep();
  }
  
  // computation starts only when path is got
  if (!get_path) {
    ROS_WARN("wait for path msg!!");
    return;
  }

  double roll, pitch, yaw;
  tf::Matrix3x3 mat(base2map.getRotation());
  mat.getEulerYPR(yaw, pitch, roll);
  Pose2d global_pose(base2map.getOrigin().x(),
                     base2map.getOrigin().y(),
                     yaw);
                     
  if (use_hoffman_control) {  
    Hoffman(global_pose);
  }
  
}

void MoveBase::ParamInit() {  

  if (!n_private.getParam("map_frame_id", map_frame_id)) {
    map_frame_id = "map";
  }
  if (!n_private.getParam("base_frame_id", base_frame_id)) {
    base_frame_id = "base_link";
  }
  if (!n_private.getParam("preset_vel", preset_vel)) {
    preset_vel = 0.2;
  }
  if (!n_private.getParam("use_hoffman_control", use_hoffman_control)) {
    use_hoffman_control = true;
  }  // default control law


}

void MoveBase::Setup() {
  
  cmd_pub = n_private.advertise<sensor_msgs::JointState>("cmd_base_joint", 100);
}

void MoveBase::Hoffman(const Pose2d& pose) {

  int index = ClosestPoint(pose);
  double error = sqrt(pow((pose.x - path[index].x), 2) + 
                      pow((pose.y - path[index].y), 2));
  double ex, ey, ecos_d, esin_d;
  ex = path[index].x - pose.x;
  ey = path[index].y - pose.y;
  ecos_d = cos(path[index].yaw);
  esin_d = sin(path[index].yaw);
  
  bool e_sign = std::signbit(ex * esin_d - ey * ecos_d);
  if (e_sign) {
    error = -1.0 * error;
  }

  std::vector<double> delta;
  delta.push_back(path[index].yaw - pose.yaw + atan(error));
  double tmp = 1 / tan(delta[0]) - 2 * vehicle.Width() / vehicle.Length();
  delta.push_back(atan(1 / tmp));
  delta.push_back(-delta[0]);
  delta.push_back(-delta[1]);

  sensor_msgs::JointState cmd;
  cmd.header.frame_id = base_frame_id;
  cmd.header.stamp = ros::Time::now();

  for (size_t i = 0; i < delta.size(); i++) {
    cmd.position.push_back(delta[i]);
  }
  
  cmd_pub.publish(cmd);  
} 

int MoveBase::ClosestPoint(const Pose2d &pose) {
  
  int index = 0;
  double dis_min = 1000000.0;
  for (size_t i = 0; i < path.size(); i++) {
    double dis = sqrt(pow((pose.x - path[i].x), 2) + pow((pose.y - path[i].y), 2));
    if (dis < dis_min) {
      index = i;
    }    
  }

  return index;
  
}