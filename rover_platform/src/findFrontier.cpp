#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include<iostream>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/OccupancyGrid.h"

using namespace std;
using namespace cv;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float g_robotX = 0;
float g_robotY = 0;
bool transformLoaded = false;

int g_abortCount = 0;

// list of keypoints that have been deemed unreachable
vector<KeyPoint> g_deadKeyPoints;

Point getRobotOccMapCell(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  float resolution = msg->info.resolution;
  int width = msg->info.width;
  int height = msg->info.height;

  // map origin (real world)
  float mapOriginX = msg->info.origin.position.x;
  float mapOriginY = msg->info.origin.position.y;

  // robot position (real world)
  float robotWorldMapX = g_robotX - mapOriginX;
  float robotWorldMapY = g_robotY - mapOriginY;

  // map position
  int x = (int)(robotWorldMapX / resolution);
  int y = (int)(robotWorldMapY / resolution);

  return Point(x,y);
}

vector<KeyPoint> findFrontierKeyPoints(Mat edges) {
  SimpleBlobDetector::Params params;

  // Change thresholds
  params.minThreshold = 10;
  params.maxThreshold = 200;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 30;

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
  detector->detect( edges, keypoints);

  return keypoints;
}

Mat extractFrontierEdges(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  int width = msg->info.width;
  int height = msg->info.height;

  Mat edges(height, width, CV_8UC3, Scalar(255,255,255));
  int threshold = 50; // adjustable parameter, anything above 40 is consider occupied and not part of the frontier
  int filterSize = 1;

  //filter out obstacle pixels
  for(int x = 0; x < width; x++) {
    for(int y = 0; y < height; y++) {
      Vec3b color = edges.at<Vec3b>(Point(x,y));

      int startX = x - filterSize;
      if (startX < 0) startX = 0;

      int startY = y - filterSize;
      if (startY < 0) startY = 0;

      int endX = x + filterSize;
      if (endX > width) endX = width;

      int endY = y + filterSize;
      if (endY > height) endY = height;

      bool hasObstaclePixel = false;
      bool hasUnknownPixel = false;
      bool hasFreePixel = false;

      for(int i = startX; i < endX; i++) {
        for(int j = startY; j < endY; j++) {
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
        color[0] = 0;
        color[1] = 0;
        color[2] = 0;
      }

      edges.at<Vec3b>(Point(x,y)) = color;

    }
  }

  return edges;
}

KeyPoint findClosestKeyPoint(vector<KeyPoint> keypoints, const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  float navX = 0;
  float navY = 0;
  float distance = 1000000;
  int deadThreshold = 10;

  // get the robot co-ofrdinates
  Point r = getRobotOccMapCell(msg);
  KeyPoint result;

  // find the closest key point
  for(int k = 0; k < keypoints.size(); k++) {
    KeyPoint pt = keypoints[k];
    bool isDead = false;

    // is this keypoint "dead"
    for(int j = 0; j < g_deadKeyPoints.size(); j++) {
      KeyPoint comp = g_deadKeyPoints[j];
      if (abs(comp.pt.x - pt.pt.x) < deadThreshold && abs(comp.pt.y - pt.pt.y) < deadThreshold) isDead = true;
    }

    if (isDead) continue;

    float d = abs(pt.pt.x - r.x) + abs(pt.pt.y - r.y);

    if (d < distance) {
      result = pt;
      distance = d;
    }
  }

  return result;
}

void goToLocation(KeyPoint target, const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  float resolution = msg->info.resolution;
  int width = msg->info.width;
  int height = msg->info.height;

  // map origin (real world)
  float mapOriginX = msg->info.origin.position.x;
  float mapOriginY = msg->info.origin.position.y;

  float navX = target.pt.x * resolution + mapOriginX;
  float navY = target.pt.y * resolution + mapOriginY;

  MoveBaseClient ac("move_base", true);

  // Wait for the action server to come up
  ROS_INFO("Waiting for the move_base action server");
  ac.waitForServer(ros::Duration(5));

  ROS_INFO("Connected to move_base server");

  move_base_msgs::MoveBaseGoal goal;

  // Send goal pose
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = navX;
  goal.target_pose.pose.position.y = navY;

  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult(ros::Duration(20.0));

  actionlib::SimpleClientGoalState state = ac.getState();

  if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Excellent! Your robot has reached the goal position.");
    g_abortCount = 0;
  } else {
    ROS_INFO("The robot failed to reach the goal position");
    g_abortCount++;
  }

  ROS_INFO("Action finisedgeshed: %s",state.toString().c_str());

  // mark the key point dead, we don't want to explore it twice
  if (g_abortCount < 5) {
    g_deadKeyPoints.push_back(target);
  } else {
    g_deadKeyPoints.clear();
    g_abortCount = 0;
    ROS_INFO("CLEARING THE DEAD");
  }

}


void visualizeFrontier(Mat edges, KeyPoint target) {
  int x = target.pt.x;
  int y = target.pt.y;

  int startX = x-1;
  int startY = y-1;

  if (startX < 0) startX = 0;
  if (startY < 0) startY = 0;

  for(int i=startX;i<x+2;i++) {
    for(int j=startY;j<y+2;j++) {
      Vec3b color = edges.at<Vec3b>(Point(i,j));
      color[0] = 0;
      color[1] = 255;
      color[2] = 0;

      edges.at<Vec3b>(Point(i,j)) = color;
    }
  }

  Mat flippedImg;
  flip(edges, flippedImg, 0);

  namedWindow("OccupancyGrid", CV_WINDOW_NORMAL);
  imshow("OccupancyGrid", flippedImg);

  waitKey(1);
}

// extracts the boundary between the explored and the unexplored map_type
// used to determine next navigation goal
void extractFrontier(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  if (!transformLoaded) return;

  Mat edges = extractFrontierEdges(msg);
  vector<KeyPoint> keypoints = findFrontierKeyPoints(edges);
  KeyPoint closest = findClosestKeyPoint(keypoints, msg);
  //visualizeFrontier(edges, closest);
  goToLocation(closest, msg);
}


int main(int argc, char* argv[]) {

  ros::init(argc, argv, "findFrontier");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/rtabmap/proj_map", 1, extractFrontier);
  ros::Rate rate(1);

  ROS_INFO("Find Frontier Started!");

  tf::TransformListener listener;

  while(ros::ok()) {

    tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/map","/robot_footprint",ros::Time(0), transform);

            g_robotX = transform.getOrigin().x();
            g_robotY = transform.getOrigin().y();

            transformLoaded = true;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Nope! %s", ex.what());
            transformLoaded = false;
        }

    ros::spinOnce();
    rate.sleep();
  }

  destroyWindow("edges");

  return 0;
}
