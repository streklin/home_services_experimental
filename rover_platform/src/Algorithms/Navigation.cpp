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
  Vertex* getClosestVertex(float x, float y);
  void setCurrentVertex(Vertex* v);
  void updateGraph(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  Vertex* addVertex(float x, float y);
  void addEdge(int v1, int v2);
  void followPath();
  void findPath(Vertex* v);
  Point getRobotOccMapPosition(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  vector<Point> getBoundaryPoints(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  bool isFrontierPoint(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr& msg);
  Point chooseClosestBoundaryPoint(vector<Point> boundary);
  Vertex* convertToMapFrame(Point p, const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void travelTo(Vertex* v);
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


Navigation::Navigation(float epsilon) {
  this->epsilon = epsilon;
  this->current = NULL;
  this->robotX = 0.0;
  this->robotY = 0.0;
  this->isActive = false;
  this->isGotoActive = true;
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

  // TODO: Look into a better way
  try {
    json vertexJson = response.at("Vertex");
    return Vertex::createVertexFromJson(response);
  }
  catch(json::out_of_range& e) {
    return NULL;
  }

}

Vertex* Navigation::getClosestVertex(float x, float y) {
  json command = {
    {"Command", "getClosestVertex"},
    {"Location", {
      {"x", x},
      {"y", y}
    }}
  };

  rover_platform::blackboardQuery srv;
  srv.request.query = command.dump();
  this->client.call(srv);

  json response = json::parse(srv.response.response);

  try {
    json vertexJson = response.at("Vertex");
    return Vertex::createVertexFromJson(response);
  }
  catch(json::out_of_range& e) {
    return NULL;
  }
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

Vertex* Navigation::addVertex(float x, float y) {
  json command = {
    {"Command", "addVertex"},
    {"Vertex", {
      {"x", x},
      {"y", y}
    }}
  };

  rover_platform::blackboardQuery srv;
  srv.request.query = command.dump();
  this->client.call(srv);

  json response = json::parse(srv.response.response);
  return Vertex::createVertexFromJson(response);
}

void Navigation::addEdge(int v1, int v2) {
  json command = {
    {"Command", "addEdge"},
    {"Edge", {
      {"v1", v1},
      {"v2", v2}
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
    this->setCurrentVertex(v);
    return;
  }

  // create a new vertex
  // add the vertex to the graph
  // add an edge between the current vertex and the new vertex
  // set the current vertex to the new vertex
  ROS_INFO("Vertex not found, creating new location on the blackboard.");
  v = this->addVertex(this->robotX, this->robotY);

  if (this->current != NULL) {
    ROS_INFO("Add a new edge");
    this->addEdge(v->getIndex(), this->current->getIndex());
  }

  ROS_INFO("Setting new vertex as teh current location on blackboard");
  this->setCurrentVertex(v);

}

void Navigation::followPath() {
  Vertex* last = this->path.back();
  this->path.pop_back();
  this->travelTo(last);
}

void Navigation::findPath(Vertex* v) {

  cout << "Navigation::findPath()" << endl;

  this->clearCurrentPath();

  json command = {
    {"Command", "findPath"},
    {"Vertex", {
      {"x", v->getX()},
      {"y", v->getY()},
      {"index", v->getIndex()}
    }}
  };

  rover_platform::blackboardQuery srv;
  srv.request.query = command.dump();
  this->client.call(srv);

  cout << "Navigation::read path from json" << endl;

  json response = json::parse(srv.response.response);

  for (int i = 0; i < response.size(); i++) {
    string vertexJson = response[i];
    json j = json::parse(vertexJson);

    Vertex* v = Vertex::createVertexFromJson(j);
    this->path.push_back(v);
  }

}

Point Navigation::getRobotOccMapPosition(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  float resolution = msg->info.resolution;
  int width = msg->info.width;
  int height = msg->info.height;

  // map origin (real world)
  float mapOriginX = msg->info.origin.position.x;
  float mapOriginY = msg->info.origin.position.y;

  // robot position (real world)
  float robotWorldMapX = this->robotX - mapOriginX;
  float robotWorldMapY = this->robotY - mapOriginY;

  // map position
  int x = (int)(robotWorldMapX / resolution);
  int y = (int)(robotWorldMapY / resolution);

  this->robotMapX = x;
  this->robotMapY = y;

  return Point(x,y);
}

bool Navigation::isFrontierPoint(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  int threshold = 50;

  int width = msg->info.width;
  int height = msg->info.height;

  int startX = x - 2;
  if (startX < 0) startX = 0;

  int startY = y - 2;
  if (startY < 0) startY = 0;

  int endX = x + 2;
  if (endX > width) endX = width;

  int endY = y + 2;
  if (endY > height) endY = height;

  bool hasObstaclePixel = false;
  bool hasUnknownPixel = false;
  bool hasFreePixel = false;

  int unKnownPixelCount = 0;

  for (int i = startX; i <= endX; i++) {

    for(int j = startY; j <= endY; j++) {

      int occupancyProb = (int)msg->data[i + width * j];
      if (occupancyProb < 0) {
        unKnownPixelCount++;
      } else if(occupancyProb < threshold) {
        hasFreePixel = true;
      } else {
        hasObstaclePixel = true;
      }

    }

  }

  if (unKnownPixelCount > 18) hasUnknownPixel = true;

  if (hasFreePixel && hasUnknownPixel && !hasObstaclePixel) {
    return true;
  }

  return false;
}

vector<Point> Navigation::getBoundaryPoints(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  vector<Point> boundary;

  // calculate the nhd width with respect to the map
  int width = msg->info.width;
  int height = msg->info.height;

  //filter out obstacle pixels
  for(int x = 0; x < width; x++) {
    for(int y = 0; y < height; y++) {

      if(this->isFrontierPoint(x,y,msg)) {
        Point b(x,y);
        boundary.push_back(b);
      }
    }
  }

  return boundary;
}

Point Navigation::chooseClosestBoundaryPoint(vector<Point> boundary) {
  int size = boundary.size();
  Point p(0,0);
  float distance = 100000000;
  float d = 0;


  for(int i = 0; i < size; i++) {
    Point bp = boundary[i];

    // convert back to full frame
    int kpX = bp.x;
    int kpY = bp.y;

    d = pow(this->robotMapX - kpX, 2) + pow(this->robotMapY - kpY, 2);

    //ROS_INFO("Distance: %f", d);
    if (d < distance && d > 400.0) {
      distance = d;
      Point n(kpX, kpY);
      p = n;
    }
  }

  return p;
}

Vertex* Navigation::convertToMapFrame(Point p, const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  float resolution = msg->info.resolution;
  int width = msg->info.width;
  int height = msg->info.height;

  // map origin (real world)
  float mapOriginX = msg->info.origin.position.x;
  float mapOriginY = msg->info.origin.position.y;

  float x = p.x * resolution + mapOriginX;
  float y = p.y * resolution + mapOriginY;

  Vertex* v = new Vertex(x, y, 0);
  return v;
}

void Navigation::travelTo(Vertex* v) {
  MoveBaseClient ac("move_base", true);

  // Wait for the action server to come up
  ROS_INFO("Waiting for the move_base action server");
  ac.waitForServer(ros::Duration(5));

  ROS_INFO("Connected to move_base server");

  move_base_msgs::MoveBaseGoal goal;

  // Send goal pose
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = v->getX();
  goal.target_pose.pose.position.y = v->getY();

  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.cancelAllGoals();
  ac.sendGoal(goal);
  ac.waitForResult(ros::Duration(20.0));

  actionlib::SimpleClientGoalState state = ac.getState();

  if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Excellent! Your robot has reached the goal position.");
  } else {
    ROS_INFO("The robot failed to reach the goal position");
  }
}

void Navigation::nextStep(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  ROS_INFO("************************Next Step*****************************");
  this->updateGraph(msg);

  ROS_INFO("I am in: %s", this->current->getLabel().c_str());

  if (!this->isActive && !this->isGotoActive) {
    return;
  }


  ROS_INFO("PATH SIZE: %d", (int)this->path.size());
  if (this->path.size() > 0) {
    ROS_INFO("Follow the path.");
    followPath();
    return;
  }

  if (!this->isActive) {
    ROS_INFO("Automated exploration disabled");
    return;
  }

  ROS_INFO("Get Robot Position on the OccupancyGrid.");
  Point robotOcc = this->getRobotOccMapPosition(msg);

  ROS_INFO("Get the Boundary.");
  vector<Point> frontier = this->getBoundaryPoints(msg);

  if (frontier.size() == 0) {
    ROS_INFO("No Boundary Points where found.");
    return;
  }

  // choose the closest point on the boundary
  ROS_INFO("Get the closest Boundary Point from the OccupancyGrid");
  Point closestBoundaryPoint = this->chooseClosestBoundaryPoint(frontier);

  // convert it to the robots frame
  ROS_INFO("Convert boundary point to the Robot's reference frame");
  Vertex* mapVertex = this->convertToMapFrame(closestBoundaryPoint, msg);

  // find the closest graph vertex to that point
  ROS_INFO("Get closest graph vertex to the boundary point");
  Vertex* graphVertex = this->getClosestVertex(mapVertex->getX(), mapVertex->getY());

  // is that distance with the epsilon ball
  ROS_INFO("Is it within epsilon");
  float distance = sqrt( pow(this->robotX - graphVertex->getX(), 2) + pow(this->robotY - graphVertex->getY(), 2) );
  if (distance < this->epsilon) {
    ROS_INFO("Travel to key point");
    this->travelTo(mapVertex);
  } else {
    ROS_INFO("Plot a path");
    ROS_INFO("Going to x: %f y: %f", graphVertex->getX(), graphVertex->getY());
    this->findPath(graphVertex);
  }
}

void Navigation::gotoPlace(string label) {
  // look up the target vertex Json
  json command = {
    {"Command", "getVertexByLabel"},
    {"Vertex", {
      {"label", label}
    }}
  };

  // if it does not exist, exist
  rover_platform::blackboardQuery srv;
  srv.request.query = command.dump();
  this->client.call(srv);

  json response = json::parse(srv.response.response);

  try {
    json vertexJson = response.at("Vertex");
    Vertex* goal = Vertex::createVertexFromJson(response);
    this->findPath(goal);
  }
  catch(json::out_of_range& e) {
    return;
  }
}

#endif
