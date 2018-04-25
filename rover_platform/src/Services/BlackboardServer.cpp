#include <ros/ros.h>
#include <string>
#include <iostream>
#include "rover_platform/blackboardQuery.h"
#include "Blackboard.cpp"

using namespace std;
using json = nlohmann::json;

Blackboard g_blackboard;

bool blackboard(rover_platform::blackboardQuery::Request &req, rover_platform::blackboardQuery::Response &res) {
  cout << "BLACKBOARD: " << req.query << endl;
  res.response = g_blackboard.performQuery(req.query);
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
