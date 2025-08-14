#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class StartFlagListener : public rclcpp::Node {
public:
  StartFlagListener()
  : rclcpp::Node("start_flag_listener"), started_(false)
  {
    // Default topic: /start_acousto_control (override with --ros-args -p start_topic:=...)
    this->declare_parameter<std::string>("start_topic", "/start_acousto_control");
    const auto topic = this->get_parameter("start_topic").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    sub_ = this->create_subscription<std_msgs::msg::Bool>(
      topic, qos,
      std::bind(&StartFlagListener::onFlag, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Listening on %s", topic.c_str());
  }

private:
  void onFlag(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !started_) {
      started_ = true;
      RCLCPP_INFO(get_logger(), "Start flag TRUE → initiating process...");
      initiate_process();
    } else if (!msg->data && started_) {
      started_ = false;
      RCLCPP_INFO(get_logger(), "Flag FALSE → reset, ready again.");
    }
  }

  void initiate_process()
  {
    // TODO: your real logic here.
    // If long-running, move to a thread or use an action server.
    RCLCPP_INFO(get_logger(), "[AcoustoControl] process started.");
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  bool started_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StartFlagListener>());
  rclcpp::shutdown();
  return 0;
}
