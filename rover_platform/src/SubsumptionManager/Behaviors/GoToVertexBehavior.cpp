#ifndef _INCL_GOTOVERTEXBEHAVIOR
#define _INCL_GOTOVERTEXBEHAVIOR

#include <ros/ros.h>
#include <string>
#include "std_msgs/Bool.h"
#include "Behaviour.cpp"

class GoToVertexBehavior : public Behavior {
private:
  ros::Publisher setExploreStatePublisher;
  ros::Publisher setGoToVertexStatePublisher;

  void disableExplorer();
  void enableGoToVertex();
  void disableGoToVertex();

public:
  GoToVertexBehavior(ros::NodeHandle nh, string name, int id);

  void activate();
  void deactivate();
};

GoToVertexBehavior::GoToVertexBehavior(ros::NodeHandle nh, string name, int id)  {
  this->name = name;
  this->id = id;
  this->isActive = false;
  this->setExploreStatePublisher = nh.advertise<std_msgs::Bool>("setExploreState", 1000);
  this->setGoToVertexStatePublisher = nh.advertise<std_msgs::Bool>("toggleGoto", 1000);
}

void GoToVertexBehavior::disableExplorer() {
  std_msgs::Bool msg;
  msg.data = false;
  this->setExploreStatePublisher.publish(msg);
}

void GoToVertexBehavior::enableGoToVertex() {
  std_msgs::Bool msg;
  msg.data = true;
  this->setGoToVertexStatePublisher.publish(msg);
}

void GoToVertexBehavior::disableGoToVertex() {
  std_msgs::Bool msg;
  msg.data = false;
  this->setGoToVertexStatePublisher.publish(msg);
}

void GoToVertexBehavior::activate() {
  ROS_INFO("ACTIVATING REMOTE CONTROL");
  this->disableExplorer();
  this->enableGoToVertex();

}

void GoToVertexBehavior::deactivate() {
  ROS_INFO("DEACTIVATING REMOTE CONTROL");
  this->disableGoToVertex();
}

#endif
