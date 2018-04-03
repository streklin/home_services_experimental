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
#include "nav_msgs/OccupancyGrid.h"
#include "Vertex.cpp"
#include "Graph.cpp"

using namespace std;
using namespace cv;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Explorer {
private:
  Graph g;
  Vertex* current;
  vector<Vertex*> history;
  float epsilon;
  float robotX;
  float robotY;
  int nhdX;
  int nhdY;

  void updateGraph(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  Point getRobotOccMapPosition(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  Mat getNhd(Point p, const nav_msgs::OccupancyGrid::ConstPtr& msg);
  vector<KeyPoint> extractFrontierKeyPoints(Mat nhd);
  void backtrack();
  Point chooseRandomKP(vector<KeyPoint> frontier);
  Point chooseClosestKP(vector<KeyPoint> frontier);
  Vertex* convertToMapFrame(Point p, const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void travelTo(Vertex* v);
  bool isFrontierPoint(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr& msg);

public:
  Explorer(float epsilon);
  void nextStep(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void setRobotPosition(float x, float y);
};

Explorer::Explorer(float epsilon) {
  this->epsilon = epsilon;
  this->current = NULL;
  this->robotX = 0.0;
  this->robotY = 0.0;
  this->nhdX = 0;
  this->nhdY = 0;
}

void Explorer::updateGraph(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

  ROS_INFO("Get Closest Vertex");

  Vertex* v = this->g.getClosestVertexInBall(this->robotX, this->robotY, this->epsilon);

  ROS_INFO("IS IT NULL?");

  if (v != NULL) {
    this->current = v;
    this->history.push_back(v);
    return;
  }

  ROS_INFO("Create Vertex");
  int index = this->g.getSize();
  v = new Vertex(this->robotX, this->robotY, index);

  ROS_INFO("Add Vertex");

  this->g.addVertex(v);

  ROS_INFO("Add Edge (if applicable)");

  if(this->current != NULL) {
    ROS_INFO("Current Index: %d", this->current->getIndex());
    this->g.addEdge(v->getIndex(), this->current->getIndex());
  }

  ROS_INFO("Update history");

  this->current = v;
  this->history.push_back(v);
}

Point Explorer::getRobotOccMapPosition(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
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

  return Point(x,y);
}

bool Explorer::isFrontierPoint(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr& msg) {

  int threshold = 50;

  int width = msg->info.width;
  int height = msg->info.height;

  int startX = x - 1;
  if (startX < 0) startX = 0;

  int startY = y - 1;
  if (startY < 0) startY = 0;

  int endX = x + 1;
  if (endX > width) endX = width;

  int endY = y + 1;
  if (endY > height) endY = height;

  bool hasObstaclePixel = false;
  bool hasUnknownPixel = false;
  bool hasFreePixel = false;

  for (int i = startX; i <= endX; i++) {

    for(int j = startY; j <= endY; j++) {

      int occupancyProb = (int)msg->data[i + width * j];
      if (occupancyProb < 0) {
        hasUnknownPixel = true;
      } else if(occupancyProb < threshold) {
        hasFreePixel = true;
      } else {
        hasObstaclePixel = true;
      }

    }

  }

  if (hasFreePixel && hasUnknownPixel && !hasObstaclePixel) {
    return true;
  }

  return false;
}

Mat Explorer::getNhd(Point p, const nav_msgs::OccupancyGrid::ConstPtr& msg) {

  // calculate the nhd width with respect to the map
  int width = msg->info.width;
  int height = msg->info.height;
  float resolution = msg->info.resolution;
  int windowSize = (int)(this->epsilon / resolution);

  ROS_INFO("WINDOW SIZE: %d", windowSize);

  // extract a square part of this map centered around p with the above calculated width / height
  int x = p.x;
  int y = p.y;

  int startX = x - windowSize;
  if (startX < 0) startX = 0;

  int startY = y - windowSize;
  if (startY < 0) startY = 0;

  int endX = x + windowSize;
  if (endX > width) endX = width;

  int endY = y + windowSize;
  if (endY > height) endY = height;

  this->nhdX = startX;
  this->nhdY = startY;

  Mat result(windowSize * 2, windowSize * 2, CV_8UC3, Scalar(255,255,255));

  int imageX = 0;
  int imageY = 0;

  for(int i = startX; i < endX; i++) {
    imageY = 0;
    for(int j = startY; j < endY; j++) {

      Vec3b color = result.at<Vec3b>(Point(imageX,imageY));

      // is this point part of the frontier?
      bool isFrontierPoint = this->isFrontierPoint(i, j, msg);

      if (isFrontierPoint) {
        color[0] = 0;
        color[1] = 0;
        color[2] = 0;
      }

      result.at<Vec3b>(Point(imageX,imageY)) = color;

      imageY++;
    }
    imageX++;
  }

  return result;
}

vector<KeyPoint> Explorer::extractFrontierKeyPoints(Mat nhd) {
  SimpleBlobDetector::Params params;

  // Change thresholds
  params.minThreshold = 10;
  params.maxThreshold = 200;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 10;

  // Filter by Circularity
  params.filterByCircularity = false;
  params.minCircularity = 0;

  // Filter by Convexity
  params.filterByConvexity = false;
  params.minConvexity = 0.87;

  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.01;

  // Set up the detector with default parameters.
  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

  // Detect blobs.
  vector<KeyPoint> keypoints;
  detector->detect( nhd, keypoints);

  return keypoints;
}

void Explorer::backtrack() {
  this->history.pop_back();

  if (this->history.size() == 0) {
    ROS_INFO("NOWHERE TO GO");
    return;
  }

  Vertex* v = this->history.back();
  this->travelTo(v);
}

Point Explorer::chooseClosestKP(vector<KeyPoint> frontier) {
  int size = frontier.size();
  Point p(0,0);

  float distance = 1000000;

  for(int i = 0; i < size; i++) {
    KeyPoint kp = frontier[i];

    float d = pow(this->robotX - kp.pt.x, 2) + pow(this->robotY - kp.pt.y, 2);
    if (d < distance) {
      d = distance;
      Point n(kp.pt.x, kp.pt.y);
      p = n;
    }
  }

  return p;
}

Point Explorer::chooseRandomKP(vector<KeyPoint> frontier) {
  int size = frontier.size();

  int index = rand() % size;

  int x = frontier[index].pt.x;
  int y = frontier[index].pt.y;

  return Point(x,y);
}

Vertex* Explorer::convertToMapFrame(Point p, const nav_msgs::OccupancyGrid::ConstPtr& msg) {

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

void Explorer::travelTo(Vertex* v) {
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
  ac.sendGoal(goal);
  ac.waitForResult(ros::Duration(40.0));

  actionlib::SimpleClientGoalState state = ac.getState();

  if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Excellent! Your robot has reached the goal position.");
  } else {
    ROS_INFO("The robot failed to reach the goal position");
  }
}

void Explorer::nextStep(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  ROS_INFO("Next Step");

  // update the graph
  ROS_INFO("Update Graph");
  this->updateGraph(msg);

  // are we at an explored vertex?
  if (this->current->isExplored()) {
    ROS_INFO("Been here before");
    this->backtrack();
    return;
  }

  // get the nhd of the robots current position from the occupancy OccupancyGrid
  ROS_INFO("get the nhd of the robots current position from the occupancy OccupancyGrid");
  Point robotOcc = this->getRobotOccMapPosition(msg);
  Mat nhd = this->getNhd(robotOcc, msg);

  // get the keypoints from the frontier
  ROS_INFO("get the keypoints from the frontier");
  vector<KeyPoint> frontier = extractFrontierKeyPoints(nhd);

  // if this list is empty
  if (frontier.size() == 0) {

    // mark current vertex isExplored
    ROS_INFO("mark current vertex isExplored");
    this->current->markExplored();
    ROS_INFO("backtrack");
    this->backtrack();

  } else {

    // Choose a random keypoint from the frontier
    ROS_INFO("Choose a random keypoint from the frontier");
    Point kp = this->chooseRandomKP(frontier);

    // convert back to full frame
    kp.x += this->nhdX;
    kp.y += this->nhdY;

    // Convert to map co-ordinates
    ROS_INFO("Convert to map co-ordinates");
    Vertex* v = this->convertToMapFrame(kp, msg);

    // Send as goal
    ROS_INFO("travel to");
    this->travelTo(v);
  }

}

void Explorer::setRobotPosition(float x, float y) {
  this->robotX = x;
  this->robotY = y;
}

Explorer explorer(5.0);

void listener(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  explorer.nextStep(msg);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/rtabmap/proj_map", 1, listener);
  tf::TransformListener listener;
  ros::Rate rate(10);

  ROS_INFO("Exploration Started!");

  while(ros::ok()) {

    tf::StampedTransform transform;
        try
        {
            //ROS_INFO("Attempting to read pose...");
            listener.lookupTransform("/map","/robot_footprint",ros::Time(0), transform);
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
