#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "canbus.h"
#include "kinematicsCalculations.h"
#include "quintic.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Kinematics : public rclcpp::Node
{
	public:
		Kinematics(): Node("kinematics"), can(new CANbus)
		{
		// Subscriber
		subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
		"joy", 10, std::bind(&Kinematics::topic_callback, this, _1));

		RCLCPP_INFO(this->get_logger(), "Subscribed to joy topic");

		// Publisher
		publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_sp", 10);
		timer_ = this->create_wall_timer(
		10ms, std::bind(&Kinematics::timer_callback, this));

		RCLCPP_INFO(this->get_logger(), "Publishing to motor_sp topic");
		}

		~Kinematics()
		{
		delete can; 
		}

		std_msgs::msg::Float32MultiArray motorVel;

		Quintic qX;
		Quintic qZ;
		float t = 0.;

	private:
		void timer_callback()
		{
		// Resize motorVel array to 3 elements (this could be refactored to not happen every callback)
		motorVel.data.resize(3);

		// Calculate quintic trajectory for x and z
		this->qX.calcQuinticTraj(2,4,t,0,2,0,6,0,7);
		this->qZ.calcQuinticTraj(2,4,t,0,2,0,6,0,7);

		motorVel.data[0] = this->qX.getVel();
		motorVel.data[1] = this->qZ.getVel();
		motorVel.data[2] = this->t;

		// ROS publisher
		publisher_->publish(motorVel);

		// Increase time variable with 10ms (0.01s)
		this->t += 0.01;
		}

		void topic_callback(const sensor_msgs::msg::Joy::SharedPtr input)
		{
			if(input->buttons[10] == 1.0)
			{
				kinematicsCalc.parked = true;
			}

			if(input->buttons[10] && input->buttons[4])
			{
				kinematicsCalc.parked = false;
			}

			if(!kinematicsCalc.parked)
			{
				if(input->buttons[9] && input->buttons[5])
				{
					kinematicsCalc.pathMode = true;
				}

				if (input->axes[0] < -0.1 && kinematicsCalc.px < kinematicsCalc.c)
				{
					kinematicsCalc.px = kinematicsCalc.px + 10;
				}else if (input->axes[0] > 0.1 && kinematicsCalc.px > 0)
				{
					kinematicsCalc.px = kinematicsCalc.px - 10; 
				}

				if (input->axes[1] < -0.1)
				{
					kinematicsCalc.pz = kinematicsCalc.pz + 10;
				}else if (input->axes[1] > 0.1 && kinematicsCalc.pz > 0)
				{   
					kinematicsCalc.pz = kinematicsCalc.pz - 10;
				}

				if(input->buttons[0] == 1.0)
				{
					kinematicsCalc.px = 0.0;
					kinematicsCalc.pz = 0.0;
				}else if (input->buttons[1] == 1.0)
				{
					kinematicsCalc.px = 1000;
					kinematicsCalc.pz = 500;
				}

				kinematicsCalc.calculate();
				std_msgs::msg::Float32MultiArray output;
				output.data.resize(4);
				output.data[0] = kinematicsCalc.setpointL;
				output.data[1] = kinematicsCalc.setpointR;
				output.data[2] = kinematicsCalc.px;
				output.data[3] = kinematicsCalc.pz;

				//RCLCPP_INFO(this->get_logger(), "\n setpointL: %f", kinematicsCalc.setpointL);

				// ROS publisher
				publisher_->publish(output);

				// CAN publisher - commented out for now, but keeping it in case it becomes needed
				//can->send_data(can_ps4_output);

			}else{
				RCLCPP_INFO(this->get_logger(), "\n System parked.");
			}
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