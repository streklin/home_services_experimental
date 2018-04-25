
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>

using namespace cv;

Mat convertROSToOpenCv(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        throw "Error: Unable to convert ROS Image.";
    }

    return cv_ptr->image;
}

sensor_msgs::Image convertToROSImage(Mat img) {
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;

    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(img_msg);

    return img_msg;
}
