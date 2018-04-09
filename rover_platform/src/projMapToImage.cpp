#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <string>
#include <tf/transform_listener.h>
#include "nav_msgs/OccupancyGrid.h"

using namespace std;
using namespace cv;

float g_robotX = 0.0f;
float g_robotY = 0.0f;

string g_imageFile = "";

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

void convertMapToImage(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  int width = msg->info.width;
  int height = msg->info.height;

  Mat displayImg(height, width, CV_8UC3, Scalar(0,0,0));
  Point robotMapCell = getRobotOccMapCell(msg);

  for(int x = 0; x < width; x++) {
    for(int y = 0; y < height; y++) {
      Vec3b color = displayImg.at<Vec3b>(Point(x,y));

      int occupancyProb = (int)msg->data[x + width * y];

      if (abs(x - robotMapCell.x) <= 5 && abs(y - robotMapCell.y) <= 5) {

        color[0] = 0;
        color[1] = 0;
        color[2] = 255;

      } else if (occupancyProb < 0) {

          color[0] = 0;
          color[1] = 165;
          color[2] = 255;

      } else {

        double ratio = 1.0 - (double)occupancyProb / 100.0;
        int cValue = (int)(ratio * 255);
        color[0] = cValue;
        color[1] = cValue;
        color[2] = cValue;

      }

      displayImg.at<Vec3b>(Point(x,y)) = color;
    }
  }

  Mat flippedImg;
  flip(displayImg, flippedImg, 0);

  namedWindow("OccupancyGrid", CV_WINDOW_NORMAL);
  imshow("OccupancyGrid", flippedImg);
  imwrite(g_imageFile, flippedImg);
  waitKey(1);
}

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "projMapToImage");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/map", 1, convertMapToImage);
  tf::TransformListener listener;
  ros::Rate rate(10);

  if (nh.getParam("lastImageLoc", g_imageFile)) {
    ROS_INFO("Last Image Location: %s", g_imageFile.c_str());
  } else {
    ROS_ERROR("Last Image Location not set");
  }

  ROS_INFO("projMapToImage Started!");

  while(ros::ok()) {

    tf::StampedTransform transform;
        try
        {
            //ROS_INFO("Attempting to read pose...");
            listener.lookupTransform("/map","/base_footprint",ros::Time(0), transform);
            g_robotX = transform.getOrigin().x();
            g_robotY = transform.getOrigin().y();
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Nope! %s", ex.what());
        }

    ros::spinOnce();
    rate.sleep();
  }

  destroyWindow("OccupancyGrid");

  return 0;
}
