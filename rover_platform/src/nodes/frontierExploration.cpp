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
#include "../Algorithms/SubsumptionNode.cpp"

Navigation* g_navigation;
SubsumptionNode* g_subsumptionNode;

void vertexLabelListener(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Received MSG: %s", msg->data.c_str());
  g_navigation->setCurrentLabel(msg->data);

  string label = g_navigation->getCurrentLabel();
  cout << "CURRENT LABEL: " << label << endl;
}

void mapListener(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  
  g_navigation->nextStep(msg);
}

void activateExploreListener(const std_msgs::Bool::ConstPtr& msg) {
  ROS_INFO("activateExploreMode");

  if (msg->data) {
      ROS_INFO("ACTIVATE");
      g_subsumptionNode->activate();
  } else {
      ROS_INFO("DEACTIVATE");
      g_subsumptionNode->deActivate();
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<rover_platform::blackboardQuery>("blackboard");
  ros::Subscriber labelSub = nh.subscribe("/setVertexLabel", 1, vertexLabelListener);
  ros::Subscriber sub = nh.subscribe("/rtabmap/proj_map", 1, mapListener);

  ros::Subscriber activateExploreMode = nh.subscribe("/activate_explore", 1, activateExploreListener);

  tf::TransformListener listener;
  ros::Rate rate(10);

  float epsilon = 0.2;
  if (nh.getParam("epsilon", epsilon)) {
    ROS_INFO("Set Epsilon: %f", epsilon);
  }

  g_navigation = new Navigation(epsilon);
  g_navigation->setServiceClient(client);

  g_subsumptionNode = new SubsumptionNode("navigation", 1);
  g_subsumptionNode->start(nh);

  ROS_INFO("Navigation Node Started");

  while(ros::ok()) {

    if (g_subsumptionNode->isActive()) {
      g_navigation->activate();
    } else {
      g_navigation->deactivate();
    }

    // capture current transform information
    tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/map","/base_footprint",ros::Time(0), ros::Duration(5.0) );
            listener.lookupTransform("/map","/base_footprint",ros::Time(0), transform);
            float x = transform.getOrigin().x();
            float y = transform.getOrigin().y();

            g_navigation->setRobotPosition(x,y);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Nope! %s", ex.what());
        }


    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
