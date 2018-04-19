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
#include "Graph/Explorer.cpp"
#include "std_msgs/String.h"


Explorer explorer(0.0);

void exploreListener(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  explorer.nextStep(msg);
}

void toggleListener(const std_msgs::Bool::ConstPtr& exploreActive) {
  if(exploreActive->data) {
    explorer.activate();
  } else {
    explorer.deactivate();
    explorer.clearCurrentPath();
  }
}

void vertexLabelListener(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Received MSG: %s", msg->data.c_str());
  explorer.setCurrentLabel(msg->data);
}

void gotoVertexListener(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Received MSG: %s", msg->data.c_str());
  explorer.gotoPlace(msg->data);
}

void toggleGotoListener(const std_msgs::Bool::ConstPtr& msg) {
   ROS_INFO("Updating GOTO Vertex state");
   explorer.setGotoState(msg->data);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/rtabmap/proj_map", 1, exploreListener);
  ros::Subscriber stateSub = nh.subscribe("/setExploreState", 1, toggleListener);
  ros::Subscriber toggleGoto = nh.subscribe("/toggleGoto", 1, toggleGotoListener);
  ros::Subscriber labelSub = nh.subscribe("/setVertexLabel", 1, vertexLabelListener);
  ros::Subscriber gotoSub = nh.subscribe("/gotoVertex", 1, gotoVertexListener);
  tf::TransformListener listener;
  ros::Rate rate(10);

  float epsilon = 0.2;
  if (nh.getParam("epsilon", epsilon)) {
    ROS_INFO("Set Epsilon: %f", epsilon);
    explorer.setEpsilon(epsilon);
  }

  ROS_INFO("Exploration Started!");

  while(ros::ok()) {

    tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/map","/base_footprint",ros::Time(0), ros::Duration(5.0) );
            listener.lookupTransform("/map","/base_footprint",ros::Time(0), transform);
            float x = transform.getOrigin().x();
            float y = transform.getOrigin().y();

            explorer.setRobotPosition(x,y);
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
