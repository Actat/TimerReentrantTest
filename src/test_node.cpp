#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

class TestNode : public rclcpp::Node {
public:
  TestNode() : Node("test_node") {
    group_ = nullptr;
    group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&TestNode::cb, this), group_);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr group_;
  void cb() {
    auto const t0 = this->get_clock()->now();
    rclcpp::sleep_for(std::chrono::milliseconds(1500));
    auto const t1 = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "\nt0\t%ld\t%ld\nt1\t%ld\t%ld",
                (long int)(t0.nanoseconds() * 1e-9),
                t0.nanoseconds() % (long int)(1e9),
                (long int)(t1.nanoseconds() * 1e-9),
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
