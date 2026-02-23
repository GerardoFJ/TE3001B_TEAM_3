#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class ProcessNode : public rclcpp::Node
{
public:
  ProcessNode()
  : Node("process"), latest_signal_(0.0f), latest_time_(0.0f), has_signal_(false), has_time_(false)
  {
    signal_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/signal", 10, std::bind(&ProcessNode::signal_callback, this, std::placeholders::_1));
    
    time_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/time", 10, std::bind(&ProcessNode::time_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/proc_signal", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&ProcessNode::process_callback, this));
  }

private:
  void signal_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_signal_ = msg->data;
    has_signal_ = true;
  }

  void time_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_time_ = msg->data;
    has_time_ = true;
  }

  void process_callback()
  {
    if (!has_signal_ || !has_time_) return;

    const float alpha = 1.1f;       
    const float phase_shift = 1.57f; 

    float processed_value = (0.5f * std::sin(latest_time_ + phase_shift)) + alpha;

    auto message = std_msgs::msg::Float32();
    message.data = processed_value;
    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Processed Signal: %.4f", processed_value);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr signal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr time_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  float latest_signal_;
  float latest_time_;
  bool has_signal_;
  bool has_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProcessNode>());
  rclcpp::shutdown();
  return 0;
}