#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include<string>
#include"imageProcessing/rosOpenCVUtilities.cpp"

using namespace cv;
using namespace std;

string g_imageFile = "";
static const std::string OPENCV_WINDOW = "Image window";

void imageProcessingPipeline(const sensor_msgs::ImageConstPtr& msg) {
   Mat image = convertROSToOpenCv(msg);
   Mat outImage;

   resize(image, outImage, Size(), 0.20, 0.20);

   imshow(OPENCV_WINDOW, outImage);

   if (g_imageFile != "")
    imwrite(g_imageFile, outImage);

   waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "opencv_image_pipeline");
    ros::NodeHandle nh;

    if (nh.getParam("lastCamImageLoc", g_imageFile)) {
      ROS_INFO("Last Image Location: %s", g_imageFile.c_str());
    } else {
      ROS_ERROR("Last Image Location not set");
    }

    namedWindow(OPENCV_WINDOW);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageProcessingPipeline);

    ros::Rate rate(5);
    while(ros::ok()) {
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}
