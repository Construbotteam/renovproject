#include <string>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

void callback(const nav_msgs::OccupancyGrid& map_msg) {
  std::cout << "width is : " << map_msg.info.width << std::endl;
  std::cout << "height is : " << map_msg.info.height << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_map");
  ros::NodeHandle nh;

  std::string test_map_topic_name;
  if (!ros::param::get("test_map_topic_name", test_map_topic_name)) {
    test_map_topic_name = "map";
  }
  std::cout << test_map_topic_name << std::endl;

  ros::Subscriber sub = nh.subscribe(test_map_topic_name, 10, callback);
  ros::spin();

  return 0;

}
