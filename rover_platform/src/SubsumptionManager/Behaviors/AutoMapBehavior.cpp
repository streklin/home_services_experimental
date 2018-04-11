#ifndef _INCL_AUTOMAPBEHAVIOR
#define _INCL_AUTOMAPBEHAVIOR

#include <ros/ros.h>
#include <string>
#include "std_msgs/Bool.h"
#include "Behaviour.cpp"

class AutoMapBehavior : public Behavior {
private:
  ros::Publisher setExploreStatePublisher;

  void disableExplorer();
  void enableExplorer();
public:
  AutoMapBehavior(ros::NodeHandle nh, string name, int id);

  void activate();
  void deactivate();
};

AutoMapBehavior::AutoMapBehavior(ros::NodeHandle nh, string name, int id)  {
  this->name = name;
  this->id = id;
  this->isActive = false;
  this->setExploreStatePublisher = nh.advertise<std_msgs::Bool>("setExploreState", 1000);
}

void AutoMapBehavior::disableExplorer() {
  std_msgs::Bool msg;
  msg.data = false;
  this->setExploreStatePublisher.publish(msg);
}

void AutoMapBehavior::enableExplorer() {
  std_msgs::Bool msg;
  msg.data = true;
  this->setExploreStatePublisher.publish(msg);
}

void AutoMapBehavior::activate() {
  ROS_INFO("ACTIVATING Auto Map!");
  this->enableExplorer();
}

void AutoMapBehavior::deactivate() {
  ROS_INFO("DEACTIVATING Auto Map!");
  this->disableExplorer();
}

#endif
