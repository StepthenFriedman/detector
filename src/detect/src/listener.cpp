#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
  public:
    Subscriber()
    : Node("msg_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "msg", 10, std::bind(&Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "%s\n", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}