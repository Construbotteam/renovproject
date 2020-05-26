#include <eigen3/Eigen/Core>
#include <iostream>

#include "ros/ros.h"

double wheel_x[4];
double wheel_y[4];

double w_car_x[4];
double w_car_y[4];

void getOdom(const double& dt, const Eigen::VectorXd& w_pos,
             const Eigen::VectorXd& w_vel, const Eigen::Vector3d& pose) {
  Eigen::VectorXd b_vec(2 * w_pos.size());
  Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(w_pos.size() * 2, w_pos.size());

  for (size_t i = 0; i < w_pos.size(); i++) {
    Eigen::Vector4d mat_vec_tmp;
    mat_vec_tmp << 1, 0, -w_car_y[i], w_car_x[i];
    mat.block(2 * i, 2 * i, 0, 3) = mat_vec_tmp.transpose();
    mat_vec_tmp << 0, 1, w_car_x[i], w_car_y[i];
    mat.block(2 * i + 1, 2 * i + 1, 0, 3) = mat_vec_tmp.transpose();
    b_vec(i * 2) = wheel_x[i] + w_vel(i) * cos(w_pos[i] + pose(2)) * dt;
    b_vec(i * 2 + 1) = wheel_y[i] + w_vel(i) * sin(w_pos[i] + pose(2)) * dt;
  }

  Eigen::Vector4d solution =
      (mat.transpose() * mat).inverse() * mat.transpose() * b_vec;
}
