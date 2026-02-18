#include <chrono>
#include <cmath> // For sin()
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class SignalGenerator : public rclcpp::Node
{
public:
  SignalGenerator()
  : Node("signal_generator"), start_time_(this->now())
  {
    signal_pub_ = this->create_publisher<std_msgs::msg::Float32>("signal", 10);
    time_pub_ = this->create_publisher<std_msgs::msg::Float32>("time", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&SignalGenerator::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto current_time = this->now();
    double t = (current_time - start_time_).seconds();
    float y = std::sin(t);

    auto signal_msg = std_msgs::msg::Float32();
    signal_msg.data = y;

    auto time_msg = std_msgs::msg::Float32();
    time_msg.data = static_cast<float>(t);

    RCLCPP_INFO(this->get_logger(), "Time: %.2f | Signal: %.2f", t, y);
    signal_pub_->publish(signal_msg);
    time_pub_->publish(time_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr signal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub_;
  rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignalGenerator>());
  rclcpp::shutdown();
  return 0;
}