#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <algorithm>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Bool.h"
#include "../Algorithms/Navigation.cpp"
#include "std_msgs/String.h"

Navigation* g_navigation;

void vertexLabelListener(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Received MSG: %s", msg->data.c_str());
  g_navigation->setCurrentLabel(msg->data);

  string label = g_navigation->getCurrentLabel();
  cout << "CURRENT LABEL: " << label << endl;
}


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;


  /*
  ros::Subscriber sub = nh.subscribe("/rtabmap/proj_map", 1, exploreListener);
  ros::Subscriber stateSub = nh.subscribe("/setExploreState", 1, toggleListener);
  ros::Subscriber toggleGoto = nh.subscribe("/toggleGoto", 1, toggleGotoListener);
  ros::Subscriber labelSub = nh.subscribe("/setVertexLabel", 1, vertexLabelListener);
  ros::Subscriber gotoSub = nh.subscribe("/gotoVertex", 1, gotoVertexListener);
  */
  ros::ServiceClient client = nh.serviceClient<rover_platform::blackboardQuery>("blackboard");
  ros::Subscriber labelSub = nh.subscribe("/setVertexLabel", 1, vertexLabelListener);

  tf::TransformListener listener;
  ros::Rate rate(10);

  float epsilon = 0.2;
  if (nh.getParam("epsilon", epsilon)) {
    ROS_INFO("Set Epsilon: %f", epsilon);
  }

  g_navigation = new Navigation(epsilon);
  g_navigation->setServiceClient(client);

  ROS_INFO("Navigation Node Started");

  while(ros::ok()) {
    // capture current transform information

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
