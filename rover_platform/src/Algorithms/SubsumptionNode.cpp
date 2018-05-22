#ifndef _INCL_SUBSUMPTION_NODE_
#define _INCL_SUBSUMPTION_NODE_

#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include "std_msgs/String.h"

#define RELEASE_NODE "subsumption_release"
#define SUPRESS_NODE "subsumption_supress"

using namespace std;

// from stack overflow: https://stackoverflow.com/questions/236129/the-most-elegant-way-to-iterate-the-words-of-a-string
vector<string> split(const string &text, char sep) {
  vector<string> tokens;
  size_t start = 0, end = 0;
  while ((end = text.find(sep, start)) != std::string::npos) {
    tokens.push_back(text.substr(start, end - start));
    start = end + 1;
  }
  tokens.push_back(text.substr(start));
  return tokens;
}

class SubsumptionNode {
private:
  string branchName;
  string messageData;
  int level;
  bool active;
  bool supressed;
  ros::Subscriber supressionSub;
  ros::Subscriber releaseSub;
  ros::Publisher supressPub;
  ros::Publisher releasePub;

  void supress(const std_msgs::String::ConstPtr& msg);
  void release(const std_msgs::String::ConstPtr& msg);

public:
  SubsumptionNode(string branchName, int level);

  bool isActive();
  void activate();
  void deActivate();
  void start(ros::NodeHandle nh);
};

SubsumptionNode::SubsumptionNode(string branchName, int level) {
  this->branchName = branchName;
  this->level = level;
  this->active = false;
  this->supressed = false;

  string levelStr = std::to_string(this->level);
  string msgData = this->branchName + "-" + levelStr;

  this->messageData = msgData;
}

void SubsumptionNode::start(ros::NodeHandle nh) {

  this->supressionSub = nh.subscribe(SUPRESS_NODE, 1, &SubsumptionNode::supress, this);
  this->releaseSub = nh.subscribe(RELEASE_NODE, 1, &SubsumptionNode::release, this);

  this->supressPub = nh.advertise<std_msgs::String>(SUPRESS_NODE, 1);
  this->releasePub = nh.advertise<std_msgs::String>(RELEASE_NODE, 1);
}

bool SubsumptionNode::isActive() {
  return this->active && !this->supressed;
}

void SubsumptionNode::activate() {
  this->active = true;
  if (this->supressed) return;

  std_msgs::String msg;
  msg.data = this->messageData;

  this->supressPub.publish(msg);
}

void SubsumptionNode::deActivate() {
  this->active = false;
  if (this->supressed) return;

  std_msgs::String msg;
  msg.data = this->messageData;

  this->releasePub.publish(msg);

}

void SubsumptionNode::supress(const std_msgs::String::ConstPtr& msg) {
  vector<string> pair = split(msg->data, '-');

  if (pair[0] != this->branchName) return;

  int level = atoi(pair[1].c_str());
  if (level <= this->level) return;

  this->supressed = true;
}

void SubsumptionNode::release(const std_msgs::String::ConstPtr& msg) {
  vector<string> pair = split(msg->data, '-');

  if (pair[0] != this->branchName) return;


  int level = atoi(pair[1].c_str());
  if (level != this->level + 1) return;


  this->supressed = false;

  std_msgs::String outMsg;
  outMsg.data = this->messageData;

  if (this->active) {
    this->supressPub.publish(outMsg);
    return;
  }

  this->releasePub.publish(outMsg);
}



#endif
