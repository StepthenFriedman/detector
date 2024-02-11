#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/imgproc.hpp>
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "std_msgs/msg/string.hpp"
#include "../include/detector.hpp"
#include "../include/cfg.hpp"
detect::Detector detector;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
std_msgs::msg::String message;

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img){
  detector.run(cv_bridge::toCvShare(img, "bgr8")->image,message);
  publisher_->publish(message);
}

int main(int argc, char ** argv)
{
  detector = detect::Detector();
  rclcpp::init(argc, argv);
  message=std_msgs::msg::String();
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  publisher_ = node->create_publisher<std_msgs::msg::String>("msg", 10);
  rclcpp::spin(node);

  return 0;
}