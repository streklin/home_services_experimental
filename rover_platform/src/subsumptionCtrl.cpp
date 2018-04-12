#include <ros/ros.h>
#include "std_msgs/String.h"
#include "SubsumptionManager/Behaviors/Behaviour.cpp"
#include "SubsumptionManager/Behaviors/RemoteControlBehavior.cpp"
#include "SubsumptionManager/Behaviors/AutoMapBehavior.cpp"
#include "SubsumptionManager/Behaviors/GoToVertexBehavior.cpp"
#include "SubsumptionManager/SubsumptionManager.cpp"

SubsumptionManager* g_subsumptionManager = NULL;

SubsumptionManager* setupSubsumptionManager(ros::NodeHandle nh) {
  SubsumptionManager* subsumption = new SubsumptionManager();

  RemoteControlBehavior* remoteControlBehavior = new RemoteControlBehavior(nh, "remote_control_behavior", 0);
  AutoMapBehavior* autoMapBehavior = new AutoMapBehavior(nh, "automap_behavior", 1);
  GoToVertexBehavior* goToVertexBehavior = new GoToVertexBehavior(nh, "gotovertex_behavior", 2);

  subsumption->registerBehavior("", remoteControlBehavior);
  subsumption->registerBehavior("remote_control_behavior", autoMapBehavior);
  subsumption->registerBehavior("remote_control_behavior", goToVertexBehavior);

  return subsumption;
}

void activateBehavior(const std_msgs::String::ConstPtr& msg) {
  if (g_subsumptionManager == NULL) return;

  ROS_INFO("ACTIVATING %s", msg->data.c_str());
  g_subsumptionManager->activateBehavior(msg->data);
}

void deActivateBehavior(const std_msgs::String::ConstPtr& msg) {
  if (g_subsumptionManager == NULL) return;

  ROS_INFO("DEACTIVATING %s", msg->data.c_str());
  g_subsumptionManager->deActivateBehavior(msg->data);
}

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "subsumptionCtrl");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/activateBehavior", 10, activateBehavior);
  ros::Subscriber stateSub = nh.subscribe("/deActivateBehavior", 10, deActivateBehavior);

  ROS_INFO("Configuring SubsumptionManager");
  g_subsumptionManager = setupSubsumptionManager(nh);

  ros::Rate rate(10);

  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
