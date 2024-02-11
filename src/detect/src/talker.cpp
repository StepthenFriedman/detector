#include <iostream>

#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "../include/cfg.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  cv::VideoCapture cap(INPUT_PATH);
  if (!cap.isOpened()) {
    return 1;
  }
  cv::Mat frame;
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;
  rclcpp::WallRate loop_rate(30);
  while (rclcpp::ok()) {
    cap >> frame;
    if (!frame.empty()) {
      msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
      pub.publish(msg);
    }else{
      cap = cv::VideoCapture(INPUT_PATH);
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}