#include "mobile_base_controller/Model.h"

using mobile::Model;

Model::Model() {

  std::cout << "default constructor initialization of class [Model]" << std::endl;  

}


Model::Model(std::string file_address) {
  
  ReadFile(file_address);

}

void Model::ReadFile(std::string file_address) {

  YAML::Node config = YAML::LoadFile("file_address");
  
  // get geometry parameters
  length = config["length"].as<double>();
  width = config["width"].as<double>();
  height = config["height"].as<double>();
  wheel_radius = config["wheel_radius"].as<double>();
  collision_radius = config["collision_radius"].as<double>();
  
  // get kinematics paramters
  linear_velo_limit = config["linear_velo_limit"].as<double>();
  angular_velo_limit = config["angular_velo_limit"].as<double>();
}

double Model::Width() const {

  return width;

}

double Model::Length() const {

  return length;

}

