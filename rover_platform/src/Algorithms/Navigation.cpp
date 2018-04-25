#ifndef _INCL_NAVIGATION_
#define _INCL_NAVIGATION_

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
#include <string>
#include "../Graph/Vertex.cpp"
#include "nav_msgs/OccupancyGrid.h"
#include "../utilities/json.hpp"
#include "rover_platform/blackboardQuery.h"

using namespace std;
using namespace cv;
using json = nlohmann::json;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigation {
private:
  Vertex* current;
  vector<Vertex*> path;
  float epsilon;
  float robotX;
  float robotY;
  int robotMapX;
  int robotMapY;
  bool isActive;
  bool isGotoActive;

  ros::ServiceClient client;

  Vertex* getClosestVertexInBall(float x, float y, float epsilon);
  void setCurrentVertex(Vertex* v);
  void updateGraph(const nav_msgs::OccupancyGrid::ConstPtr& msg);

public:
  Navigation(float epsilon);
  void setServiceClient(ros::ServiceClient client);
  void nextStep(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void setRobotPosition(float x, float y);
  void activate();
  void deactivate();
  void setGotoState(bool newState);
  void setCurrentLabel(string label);
  string getCurrentLabel();
  void gotoPlace(string label);
  void clearCurrentPath();
  void setEpsilon(float epsilon);
};

/*
void gotoPlace(string label);
*/

Navigation::Navigation(float epsilon) {
  this->epsilon = epsilon;
  this->current = NULL;
  this->robotX = 0.0;
  this->robotY = 0.0;
  this->isActive = false;
  this->isGotoActive = false;
}

void Navigation::setServiceClient(ros::ServiceClient client) {
  this->client = client;
}

void Navigation::setRobotPosition(float x, float y) {
  this->robotX = x;
  this->robotY = y;
}

void Navigation::activate() {
  this->isActive = true;
}

void Navigation::deactivate() {
  this->isActive = false;
}

void Navigation::setGotoState(bool newState) {

  if (newState) {
    ROS_INFO("ACTIVATE GOTO STATE");
  } else {
    ROS_INFO("DEACTIVATE GOTO STATE");
  }

  this->isGotoActive = newState;

  if (!newState) {
    ROS_INFO("CLEARING PATH");
    this->clearCurrentPath();
  }
}

void Navigation::setEpsilon(float epsilon) {
  this->epsilon = epsilon;
}

void Navigation::clearCurrentPath() {
  this->path.clear();
}

void Navigation::setCurrentLabel(string label) {
  json command = {
    {"Command", "setCurrentLabel"},
    {"Vertex", {
      {"label", label}
    }}
  };

  rover_platform::blackboardQuery srv;
  srv.request.query = command.dump();
  this->client.call(srv);
}

string Navigation::getCurrentLabel() {
  json command = {
    {"Command", "queryCurrentLocation"}
  };

  rover_platform::blackboardQuery srv;
  srv.request.query = command.dump();
  this->client.call(srv);

  auto response = json::parse(srv.response.response);
  return response["Vertex"]["label"];
}

Vertex* Navigation::getClosestVertexInBall(float x, float y, float epsilon) {

  json command = {
    {"Command", "getClosestVertexInBall"},
    {"Ball", {
      {"x", x},
      {"y", y},
      {"epsilon", epsilon}
    }}
  };

  rover_platform::blackboardQuery srv;
  srv.request.query = command.dump();
  this->client.call(srv);

  json response = json::parse(srv.response.response);
  if (response.find("Vertex")) {
    return Vertex::createVertexFromJson(response);
  }

  return NULL;

}

void Navigation::setCurrentVertex(Vertex* v) {
  this->current = v;

  json command = {
    {"Command", "setCurrentLocation"},
    {"Location", {
      {"index", v->getIndex()}
    }}
  };

  rover_platform::blackboardQuery srv;
  srv.request.query = command.dump();
  this->client.call(srv);
}

void Navigation::updateGraph(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

  // get the closest vertex in the ball
  ROS_INFO("Query Blackboard for closest Vertex");
  Vertex* v = this->getClosestVertexInBall(this->robotX, this->robotY, this->epsilon);

  // if there exists a vertex in the ball
  //  set it as the current location
  //  return
  if (v != NULL) {
    ROS_INFO("Vertex was found, updating blackboard with new location.");
    this->setCurrentLocation(v);
    return;
  }

  // create a new vertex
  // add the vertex to the graph
  // add an edge between the current vertex and the new vertex
  // set the current vertex to the new vertex
  ROS_INFO("Vertex not found, creating new location on the blackboard.");
  

}

void Navigation::nextStep(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

}

#endif
