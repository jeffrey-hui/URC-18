//
// Created by matthew on 11/22/17.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

image_transport::Publisher *image_out;
image_transport::Subscriber *image_in;

double angle;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat src = cv_bridge::toCvShare(msg)->image;

    cv::Point2f center(src.cols/2.0, src.rows/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::Rect bbox = cv::RotatedRect(center,src.size(), angle).boundingRect();
    rot.at<double>(0,2) += bbox.width/2.0 - center.x;
    rot.at<double>(1,2) += bbox.height/2.0 - center.y;

    cv::Mat dst;
    cv::warpAffine(src, dst, rot, bbox.size());
    image_out->publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", dst).toImageMsg());
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "image_rotate");

    ros::NodeHandle nh("~");
    if (!nh.getParam("angle", angle)) {
        ROS_FATAL_STREAM("Failed to degrees of rotation, please set parameter angle!");
        return 1;
    }

    image_transport::ImageTransport it = image_transport::ImageTransport(nh);
    image_transport::Subscriber it_sub_b = it.subscribe("image_in", 10, imageCallback);
    image_in = &it_sub_b;

    image_transport::Publisher it_pub_b = it.advertise("image_out", 10);
    image_out = &it_pub_b;

    ROS_INFO_STREAM("Started image rotater at " << angle << " degrees.");

    while (ros::ok()) {
        ros::spinOnce();
    }

    delete image_out;
    delete image_in;

    return 0;

}