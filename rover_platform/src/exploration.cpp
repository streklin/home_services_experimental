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
#include "nav_msgs/OccupancyGrid.h"
#include "Graph/Vertex.cpp"
#include "Graph/Graph.cpp"

using namespace std;
using namespace cv;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Explorer {
private:
  Graph g;
  Vertex* current;
  vector<Vertex*> path;
  float epsilon;
  float robotX;
  float robotY;
  int robotMapX;
  int robotMapY;

  void updateGraph(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  Point getRobotOccMapPosition(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  Mat getNhd(Point p, const nav_msgs::OccupancyGrid::ConstPtr& msg);
  vector<KeyPoint> extractFrontierKeyPoints(Mat nhd);
  Point chooseRandomKP(vector<KeyPoint> frontier);
  Point chooseClosestKP(vector<KeyPoint> frontier);
  Vertex* convertToMapFrame(Point p, const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void travelTo(Vertex* v);
  bool isFrontierPoint(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void findPath(Vertex* v);
  vector<Point> getBoundaryPoints(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  Point chooseClosestBoundaryPoint(vector<Point> boundary);

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
}

void Explorer::updateGraph(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

  ROS_INFO("Get Closest Vertex");

  Vertex* v = this->g.getClosestVertexInBall(this->robotX, this->robotY, this->epsilon);

  ROS_INFO("IS IT NULL?");

  if (v != NULL) {
    ROS_INFO("SET CURRENT VERTEX");
    ROS_INFO("Robot At: x: %f y:%f", this->robotX, this->robotY);
    ROS_INFO("Current Vertex: x: %f y:%f id: %d", v->getX(), v->getY(), v->getIndex());
    this->current = v;
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

  this->current = v;
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

  this->robotMapX = x;
  this->robotMapY = y;

  return Point(x,y);
}

bool Explorer::isFrontierPoint(int x, int y, const nav_msgs::OccupancyGrid::ConstPtr& msg) {

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

  Mat edges(height, width, CV_8UC3, Scalar(255,255,255));

  //filter out obstacle pixels
  for(int x = 0; x < width; x++) {
    for(int y = 0; y < height; y++) {
      Vec3b color = edges.at<Vec3b>(Point(x,y));

      if(this->isFrontierPoint(x,y,msg)) {
        color[0] = 0;
        color[1] = 0;
        color[2] = 0;
      }

      edges.at<Vec3b>(Point(x,y)) = color;

    }
  }

  return edges;
}

vector<Point> Explorer::getBoundaryPoints(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
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

vector<KeyPoint> Explorer::extractFrontierKeyPoints(Mat nhd) {
  SimpleBlobDetector::Params params;

  // Change thresholds
  params.minThreshold = 1;
  params.maxThreshold = 200;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 2;

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


Point Explorer::chooseClosestKP(vector<KeyPoint> frontier) {
  int size = frontier.size();
  Point p(0,0);
  int keyPointSize = 0;

  float distance = 1000000;

  for(int i = 0; i < size; i++) {
    KeyPoint kp = frontier[i];

    //if (kp.size < 5) continue; // ignore smaller keypoints

    // convert back to full frame
    int kpX = kp.pt.x;
    int kpY = kp.pt.y;

    float d = pow(this->robotMapX - kpX, 2) + pow(this->robotMapY - kpY, 2);
    if (d < distance) {
      distance = d;
      Point n(kpX, kpY);
      p = n;
      keyPointSize = kp.size;
    }
  }

  ROS_INFO("CHOOSEN KEY POINT OF SIZE: %d", size);

  return p;
}

Point Explorer::chooseClosestBoundaryPoint(vector<Point> boundary) {

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

  ROS_INFO("Distance: %f", distance);

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
  ac.cancelAllGoals();
  ac.sendGoal(goal);
  ac.waitForResult(ros::Duration(60.0));

  actionlib::SimpleClientGoalState state = ac.getState();

  if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Excellent! Your robot has reached the goal position.");
  } else {
    ROS_INFO("The robot failed to reach the goal position");
  }
}

void Explorer::findPath(Vertex* v) {
  ROS_INFO("find path");

  this->path.clear();

  vector<Vertex*> stack;
  vector<Vertex*> visited;

  // mark it as the start of the path
  this->current->setParent(-1);
  stack.push_back(this->current);

  bool goalReached = false;
  while(stack.size() > 0 && !goalReached) {

    Vertex* next = stack.back();
    stack.pop_back();

    visited.push_back(next);

    if (v->getIndex() == next->getIndex()) {
      goalReached = true;
      continue;
    }

    vector<Vertex*> frontier = this->g.getConnectedVertices(next->getIndex());

    for(int i = 0; i < frontier.size(); i++) {
      Vertex* fv = frontier[i];

      bool found = false;
      for(int j = 0; j < visited.size(); j++) {
        Vertex* vv = visited[j];

        if (fv->getIndex() == vv->getIndex()) {
          found = true;
          break;
        }
      }

      if (!found) {
        fv->setParent(next->getIndex());
        stack.push_back(fv);
      }
    }
  }

  // extract the path
  Vertex* dest = visited.back();
  vector<Vertex*> path;

  while(dest->getParent() > 0) {
    path.push_back(dest);
    dest = this->g.getVertexByIndex(dest->getParent());
  }

  //reverse(path.begin(), path.end());
  this->path = path;
}

void Explorer::nextStep(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  ROS_INFO("Next Step");

  ROS_INFO("Update Graph");
  this->updateGraph(msg);

  if (this->path.size() > 0) {
    ROS_INFO("Follow the path.");
    Vertex* last = this->path.back();
    this->path.pop_back();
    this->travelTo(last);
    return;
  }

  ROS_INFO("get the nhd of the robots current position from the occupancy OccupancyGrid");
  Point robotOcc = this->getRobotOccMapPosition(msg);
  //Mat nhd = this->getNhd(robotOcc, msg);
  //vector<KeyPoint> frontier = extractFrontierKeyPoints(nhd);

  ROS_INFO("Get the Boundary");
  vector<Point> frontier = this->getBoundaryPoints(msg);

  if (frontier.size() == 0) {
    ROS_INFO("Nothing to explore.");
    return;
  }

  ROS_INFO("Choose KeyPoint");
  Point closestKp = this->chooseClosestBoundaryPoint(frontier);

  ROS_INFO("Convert Key Point to map frame.");
  Vertex* mapVertex = this->convertToMapFrame(closestKp, msg);

  ROS_INFO("Get closest graph vertex to key point.");
  Vertex* graphVertex = this->g.getClosestVertex(mapVertex->getX(), mapVertex->getY());

  ROS_INFO("Is within epsilon?");
  float distance = sqrt( pow(this->robotX - graphVertex->getX(), 2) + pow(this->robotY - graphVertex->getY(), 2) );

  if (distance < this->epsilon) {
    ROS_INFO("Travel to key point");
    this->travelTo(mapVertex);
  } else {
    ROS_INFO("Plot a path");
    this->findPath(mapVertex);
  }
}

void Explorer::setRobotPosition(float x, float y) {
  this->robotX = x;
  this->robotY = y;
}

Explorer explorer(1.0);

void listener(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  explorer.nextStep(msg);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/map", 1, listener);
  tf::TransformListener listener;
  ros::Rate rate(10);

  ROS_INFO("Exploration Started!");

  while(ros::ok()) {

    tf::StampedTransform transform;
        try
        {
            //ROS_INFO("Attempting to read pose...");
            listener.lookupTransform("/map","/base_footprint",ros::Time(0), transform);
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
