#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "canbus.h"
#include "kinematicsCalculations.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Kinematics : public rclcpp::Node
{
  public:
    Kinematics()
    : Node("kinematics"), can(new CANbus)
    {
      // Subscriber
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Kinematics::topic_callback, this, _1));

      RCLCPP_INFO(this->get_logger(), "Subscribed to joy topic");
    
      // Publisher
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_sp", 10);

      RCLCPP_INFO(this->get_logger(), "Publishing to motor_sp topic");
    }

    ~Kinematics()
    {
        delete can;
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr input)
    {
    if (input->axes[0] > 0.1)
    {
        kinematicsCalc.px++;
    }else if (input->axes[0] < -0.1)
    {
        kinematicsCalc.px--;
    }

    if (input->axes[1] > 0.1)
    {
        kinematicsCalc.pz++;
    }else if (input->axes[1] < -0.1)
    {
        kinematicsCalc.pz--;
    }
    
    kinematicsCalc.calculate();
    std_msgs::msg::Float32MultiArray output;
    output.data.resize(2);
    output.data[0] = kinematicsCalc.setpointL;
    output.data[1] = kinematicsCalc.setpointR;

    RCLCPP_INFO(this->get_logger(), "\n setpointL: %f", kinematicsCalc.setpointL);

    // ROS publisher
    publisher_->publish(output);

    // CAN publisher - commented out for now, but keeping it in case it becomes needed
    //can->send_data(can_ps4_output);
    }
    CANbus* can;
    KinematicsCalculations kinematicsCalc;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Kinematics>());
  rclcpp::shutdown();
  return 0;
}