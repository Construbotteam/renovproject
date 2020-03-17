#ifndef MODEL_H
#define MODEL_H

#include <string>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include "ros/ros.h"

namespace mobile {

class Model {
 public:
  Model();
  Model(std::string file_address);
  ~Model() {}
  void ReadFile(std::string file_address);
  double Width() const;
  double Length() const;
 
 private:
  double length;
  double width;
  double height;
  double wheel_radius;
  double collision_radius;

  double linear_velo_limit;
  double angular_velo_limit;
};  // class Model

}  // namespace mobile






#endif
