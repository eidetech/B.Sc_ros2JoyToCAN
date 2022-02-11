#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "canbus.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Tranceiver : public rclcpp::Node
{
  public:
    Tranceiver()
    : Node("tranceiver"), can(new CANbus)
    {
      // Subscriber
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Tranceiver::topic_callback, this, _1));

      RCLCPP_INFO(this->get_logger(), "Subscribed to joy topic");
    
      // Publisher
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ps4", 10);

      RCLCPP_INFO(this->get_logger(), "Publishing to ps4 topic");
    }

    ~Tranceiver()
    {
        delete can;
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr input) const
    {
    std_msgs::msg::Float32MultiArray output;
    float can_ps4_output[8];
    output.data.resize(8);
    output.data[0] = input->axes[0];
    output.data[1] = input->axes[1];
    output.data[2] = input->axes[3];
    output.data[3] = input->axes[4];
    output.data[4] = input->buttons[0];
    output.data[5] = input->buttons[1];
    output.data[6] = input->buttons[2];
    output.data[7] = input->buttons[3];

    can_ps4_output[0] = input->axes[0];
    can_ps4_output[1] = input->axes[1];
    can_ps4_output[2] = input->axes[3];
    can_ps4_output[3] = input->axes[4];
    can_ps4_output[4] = input->buttons[0];
    can_ps4_output[5] = input->buttons[1];
    can_ps4_output[6] = input->buttons[2];
    can_ps4_output[7] = input->buttons[3];

    //RCLCPP_INFO(this->get_logger(), "\n%f", can_ps4_output[0]);

    // ROS publisher
    publisher_->publish(output);

    // CAN publisher
    can->send_data(can_ps4_output);
    }
    CANbus* can;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tranceiver>());
  rclcpp::shutdown();
  return 0;
}