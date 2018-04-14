/*
Used on the physical robot to capture the video input from the jetson tx2's on board
camera.
*/

#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include"imageProcessing/rosOpenCVUtilities.cpp"

using namespace cv;

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_video_capture");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher publisher = it.advertise("/camera/image_raw", 1);

    ros::Rate loop_rate(10);

    VideoCapture cap;

    if (!cap.open(0)) {
        ROS_ERROR("Error: Unable to communicate with Web Cam.");
    }

    while(ros::ok()) {

        Mat frame;
        cap >> frame;

        if( frame.empty() ) {
            ROS_ERROR("Error: Video feed disconnected.");
        }

        sensor_msgs::Image ros_img = convertToROSImage(frame);

        publisher.publish(ros_img);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
