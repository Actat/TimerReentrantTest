#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

class TestNode : public rclcpp::Node {
public:
  TestNode() : Node("test_node") {
    group_ = nullptr;
    group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    publisher_ = this->create_publisher<std_msgs::msg::Header>("~/topic", 10);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    subscription_          = this->create_subscription<std_msgs::msg::Header>(
        "~/topic", 10,
        std::bind(&TestNode::topic_cb, this, std::placeholders::_1), options);
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&TestNode::timer_cb, this));
  }

private:
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr group_;
  void timer_cb() {
    auto const t0 = this->get_clock()->now();
    auto message  = std_msgs::msg::Header();
    message.stamp = t0;
    publisher_->publish(message);
  }
  void topic_cb(const std_msgs::msg::Header::SharedPtr msg) {
    auto const t0 = this->get_clock()->now();
    rclcpp::sleep_for(std::chrono::milliseconds(1500));
    auto const t1 = this->get_clock()->now();
    RCLCPP_INFO(
        this->get_logger(),
        "\ntimer_cb\t%ld\t%ld\ntopic_cb\t%ld\t%ld\nafter sleep\t%ld\t%ld",
        msg->stamp.sec, msg->stamp.nanosec, (long int)(t0.nanoseconds() * 1e-9),
        t0.nanoseconds() % (long int)(1e9), (long int)(t1.nanoseconds() * 1e-9),
        t1.nanoseconds() % (long int)(1e9));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(), "num of threads: %d",
              executor.get_number_of_threads());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
