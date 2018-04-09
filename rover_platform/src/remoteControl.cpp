#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

float forwardVelocity = 0.0;
float angularVelocity = 0.0;

bool g_isAutoMapActive = false;

void forwardTwist() {
  forwardVelocity += 0.2;
  if (forwardVelocity > 1.0) forwardVelocity = 1.0;
}

void backwardTwist() {
  forwardVelocity -= 0.2;
  if (forwardVelocity < -1.0) forwardVelocity = -1.0;
}

void rotateRight() {
  angularVelocity -= 0.1;
  if (angularVelocity < -1.0) angularVelocity = -1.0;
}

void rotateLeft() {
  angularVelocity += 0.1;
  if (angularVelocity > 1.0) angularVelocity = 1.0;
}

void stop() {
  forwardVelocity = 0.0;
  angularVelocity = 0.0;
}

void readCommand(const std_msgs::String::ConstPtr& msg) {
  geometry_msgs::Twist t;

  if (msg->data == "F") {
    forwardTwist();
  }

  if (msg->data == "B") {
    backwardTwist();
  }

  if (msg->data == "L") {
    rotateLeft();
  }

  if (msg->data == "R") {
    rotateRight();
  }

  if (msg->data == "S") {
    stop();
  }


}

void toggleListener(const std_msgs::Bool::ConstPtr& exploreActive) {
  g_isAutoMapActive = exploreActive->data;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "remoteControl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/remoteControl", 1, readCommand);
  ros::Subscriber stateSub = nh.subscribe("/setExploreState", 1, toggleListener);
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Rate rate(10);

  while(ros::ok()) {

    // only respond to remote control when auto map is disabled, otherwise
    // the two cmd_vels will intefere with each other causing problems for the
    // movebase functionality.
    if (!g_isAutoMapActive) {
      geometry_msgs::Twist msg;
      msg.linear.x = forwardVelocity;
      msg.angular.z = angularVelocity;
      pub.publish(msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
