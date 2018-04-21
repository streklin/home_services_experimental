#include <ros/ros.h>
#include <string>
#include "rover_platform/blackboardQuery.h"
#include "Blackboard.cpp"

using namespace std;
using json = nlohmann::json;

Blackboard g_blackboard;

bool blackboard(rover_platform::blackboardQuery::Request &req, rover_platform::blackboardQuery::Response &res) {
  res.response = g_blackboard.queryCurrentLocation();
  g_blackboard.addVertex(res.response);
  ROS_INFO("RESPONSE RECEIVED: %s", req.query.c_str());
  return true;
}

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "blackboard_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("blackboard", blackboard);
  ROS_INFO("BLACKBOARD READY");
  ros::spin();

  return 0;
}
