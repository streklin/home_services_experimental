#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <string>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/OccupancyGrid.h"
#include "shared/Robot.cpp"

using namespace cv;
using namespace std;

/*
Class Definitions
*/

class Vertex {
private:
  float x_;
  float y_;
  bool explored_;
  string label_;
public:
  Vertex();
  Vertex(float x, float y);
  bool isNear(float x, float y, float epsilon);
  void setLabel(string l);
  string getLabel();
}

/*
Represents a 2D Robot for use with the Rover_Platform.
*/
class Robot {
private:
  float worldX_;
  float worldY_;
  int mapX_;
  int mapY_;
public:
  Robot();
  void setPosition(float worldX, float worldY, const nav_msgs::OccupancyGrid::ConstPtr& msg);
  float getWorldX();
  float getWorldY();
  int getMapX();
  int getMapY();
}

class TopGraph {
private:
  static const int MAXIMUM = 100;
  bool adjacencyMatrix_[MAXIMUM][MAXIMUM];
  vector<Vertex> vertexList_;
public:
  TopGraph();
  void addEdge(int sourceId, int targetId);
  void addVertex(Vertex v);
}

class Explorer {
private:
  Vertex current_;
  TopGraph graph_;
  Robot r_;
  float epsilon_;
public:
  Explorer();
  Vertex getNextNavPoint(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  bool isMapExplored();
}


/*
Implementations
*/

///////// Vertex /////////////
Vertex::Vertex() {
  this->x_ = 0.0;
  this->y_ = 0.0;
  this->explored_ = false;
  this->label_ = "";
}

Vertex::Vertex(float x, float y) {
  this->x_ = x;
  this->y_ = y;
  this->explored_ = false;
  this->label_ = "";
}

bool Vertex::isNear(float x, float y, float epsilon) {
  float distance = sqrt( pow(this->x_ - x, 2) + pow(this->y_ - y, 2)  );
  return distance < epsilon;
}

void Vertex::setLabel(string l) {
  this->label_ = l;
}

string Vertex::getLabel() {
  return this->label_;
}

/////////// Robot ////////////
Robot::Robot() {
  this->worldX_ = 0;
  this->worldY_ = 0;
  this->mapX_ = 0;
  this->mapY_ = 0;
}

void Robot::setPosition(float worldX, float worldY, const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  this->worldX_ = worldX;
  this->worldY_ = worldY;

  float resolution = msg->info.resolution;
  int width = msg->info.width;
  int height = msg->info.height;

  // map origin (real world)
  float mapOriginX = msg->info.origin.position.x;
  float mapOriginY = msg->info.origin.position.y;

  // robot position (real world)
  float robotWorldMapX = worldX - mapOriginX;
  float robotWorldMapY = worldY - mapOriginY;

  // map position
  int x = (int)(robotWorldMapX / resolution);
  int y = (int)(robotWorldMapY / resolution);

  this->mapX_ = x;
  this->mapY_ = y;
}

float Robot::getWorldX() {
  return this->worldX_;
}

float Robot::getWorldY() {
  return this->worldY_;
}

int Robot::getMapX() {
  return this->mapX_;
}

int Robot::getMapY() {
  return this->mapY_;
}

/////////////////// TopGraph //////////////////
/*
class TopGraph {
private:
  static const int MAXIMUM = 100;
  bool adjacencyMatrix_[MAXIMUM][MAXIMUM];
  vector<Vertex> vertexList_;
public:
  TopGraph();
  void addEdge(int sourceId, int targetId);
  void addVertex(Vertex v);
}
*/
TopGraph::TopGraph() {}

void TopGraph::addEdge(int sourceId, int targetId) {
  int vertexCount = this->vertexList_.size();

  if (sourceId < 0 || sourceId >= vertexCount || sourceId >= this->MAXIMUM) {
    throw "Source Vertex Id is out of range";
  }

  if (targetId < 0 || targetId >= vertexCount || targetID >= this->MAXIMUM) {
    throw "Target Vertex Id is out of range";
  }



}
