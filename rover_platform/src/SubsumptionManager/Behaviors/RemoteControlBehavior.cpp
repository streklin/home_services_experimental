#ifndef _INCL_REMOTECONTROLBEHAVIOR
#define _INCL_REMOTECONTROLBEHAVIOR

#include <ros/ros.h>
#include <string>
#include "std_msgs/Bool.h"
#include "Behaviour.cpp"

class RemoteControlBehavior : public Behavior {
private:

  ros::Publisher setUserRemoteControlStatePublisher;

  void disableRemoteControl();
  void enableRemoteControl();
public:
  RemoteControlBehavior(ros::NodeHandle nh, string name, int id);

  void activate();
  void deactivate();
};

RemoteControlBehavior::RemoteControlBehavior(ros::NodeHandle nh, string name, int id)  {
  this->name = name;
  this->id = id;
  this->isActive = false;
  this->setUserRemoteControlStatePublisher = nh.advertise<std_msgs::Bool>("setRemoteControlState", 1000);
}

void RemoteControlBehavior::disableRemoteControl() {
  std_msgs::Bool msg;
  msg.data = false;
  this->setUserRemoteControlStatePublisher.publish(msg);
}

void RemoteControlBehavior::enableRemoteControl() {
  std_msgs::Bool msg;
  msg.data = true;
  this->setUserRemoteControlStatePublisher.publish(msg);
}

void RemoteControlBehavior::activate() {
  ROS_INFO("ACTIVATING REMOTE CONTROL");
  this->enableRemoteControl();
}

void RemoteControlBehavior::deactivate() {
  ROS_INFO("DEACTIVATING REMOTE CONTROL");
  this->disableRemoteControl();
}

#endif
