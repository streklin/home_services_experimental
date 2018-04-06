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

Explorer explorer(0.5);

void exploreListener(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  explorer.nextStep(msg);
}

void toggleListener(const std_msgs::Bool::ConstPtr& exploreActive) {
  if(exploreActive->data) {
    explorer.activate();
  } else {
    explorer.deactivate();
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/map", 1, exploreListener);
  ros::Subscriber stateSub = nh.subscribe("/setExploreState", 1, toggleListener);
  tf::TransformListener listener;
  ros::Rate rate(10);

  ROS_INFO("Exploration Started!");

  while(ros::ok()) {

    tf::StampedTransform transform;
        try
        {
            //ROS_INFO("Attempting to read pose...");
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
