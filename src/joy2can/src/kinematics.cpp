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
#include "ik.h"
#include "trajectoryPlanner.h"

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

		// Quintic objects for calculating x and z position, velocity and acceleration
		Quintic qX;
		Quintic qZ;

		// Inverse kinematics object for converting [x,z] coordinates to [q1,q2] positions and velocities.
		IK ik;

		// Trajectory planner object
		TrajectoryPlanner trajPlan;

		float t = 0.;
		bool printed = false;

		float t0 = 0;
		float t1 = 0;
		float t_sum = 0;
		float x0 = 0;
		float x1 = 0;
		float z0 = 0;
		float z1 = 0;
		float v0_x = 0;
		float v1_x = 0;
		float v0_z = 0;
		float v1_z = 0;
		float a0_x = 0;
		float a1_x = 0;
		float a0_z = 0;
		float a1_z = 0;
		
		int idx = 0;

	private:
		void timer_callback()
		{
		// Resize motorVel array to 3 elements (this could be refactored to not happen every callback)
		motorVel.data.resize(3);

		// Set offsets from origo to pulleys
		ik.setOffsets(0, 2.0, 2.0, 2.0);

		if(!printed)
		{
		trajPlan.plan();
		trajPlan.printPoints();
		trajPlan.calcCartesianPosVelAcc();
		printed = !printed;
		}

		if(idx <= 8)
		{
			if (t <= trajPlan.posVelAccTime(idx, 14))
			{
			t0 = trajPlan.posVelAccTime(idx,12);
			t1 = trajPlan.posVelAccTime(idx,13);

			x0 = trajPlan.posVelAccTime(idx,0);
			x1 = trajPlan.posVelAccTime(idx,1);

			z0 = trajPlan.posVelAccTime(idx,2);
			z1 = trajPlan.posVelAccTime(idx,3);

			v0_x = trajPlan.posVelAccTime(idx,4);
			v1_x = trajPlan.posVelAccTime(idx,5);

			v0_z = trajPlan.posVelAccTime(idx,6);
			v1_z = trajPlan.posVelAccTime(idx,7);

			a0_x = trajPlan.posVelAccTime(idx,8);
			a1_x = trajPlan.posVelAccTime(idx,9);

			a0_z = trajPlan.posVelAccTime(idx,10);
			a1_z = trajPlan.posVelAccTime(idx,11);

			// Calculate quintic trajectory for x and z
			this->qX.calcQuinticTraj(t0,t1,t,x0,x1,v0_x,v1_x,a0_x,a1_x);
			this->qZ.calcQuinticTraj(t0,t1,t,z0,z1,v0_z,v1_z,a0_z,a1_z);

			// this->qX.calcQuinticTraj(0,2,t,0,3,0,6,0,4);

			std::cout << "x: " << qX.getPos() << std::endl;

			// // Calculate the inverse kinematics based on the quintic trajectory
			this->ik.calc(qX.getPos(), qZ.getPos(), qX.getVel(), qZ.getVel());

			// // Assign the calculated data to the ROS message
			motorVel.data[0] = qX.getPos();
			motorVel.data[1] = qZ.getPos();
			motorVel.data[2] = this->t;

			// //std::cout << "M0: " << motorVel.data[0] << ", M1: " << motorVel.data[1] << ", t: " << motorVel.data[2] << std::endl;

			// ROS publisher
			publisher_->publish(motorVel);

			// Increase time variable with 10ms (0.01s)
			this->t += 0.01;

			if(t >= trajPlan.posVelAccTime(idx, 14))
			{
				idx++;
				std::cout << "idx: " << idx << std::endl;
			}
		}
		}
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