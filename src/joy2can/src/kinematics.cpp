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

		// ROS message to send motor velocities
		std_msgs::msg::Float32MultiArray motorVel;

		// Quintic objects for calculating x and z position, velocity and acceleration
		Quintic qX;
		Quintic qZ;

		// Inverse kinematics object for converting [x,z] coordinates to [q1,q2] positions and velocities.
		IK ik;

		// Trajectory planner object
		TrajectoryPlanner trajPlan;

		// Time variable for keeping track of total time in path
		float t = 0.;

		/* Time variable for keeping track of current time during a path sequence (used for quintic calculations),
			since the quintic class calculates each path with t0 = 0, and t1 = the time it takes to run that path. */
		float t_quintic = 0.;

		// Bool used to print vectors and matrices once at the first callback (for debugging)
		bool printed = false;

        bool run = false;

        int mode = 0;

		// Variables used for quintic trajectory planning
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
        float sprayStatus = 0;
		
		// Index variable to keep track of which path sequence is running
		int idx = 0;

	private:
		void timer_callback()
		{
		// Resize motorVel array to 3 elements (this could be refactored to not happen every callback)
		motorVel.data.resize(6);

		// Set offsets from origo to pulleys
		ik.setOffsets(0, trajPlan.get_outer_frame_height(), trajPlan.get_outer_frame_width(), trajPlan.get_outer_frame_height());

		if(!printed)
		{
		trajPlan.plan();
		trajPlan.printPoints();
		trajPlan.calcCartesianPosVelAcc();
		printed = !printed;
		}

		
		if(idx <= trajPlan.pt_len-1 && run) // TODO: This number will have to be updated when the path length changes. Should be dynamic.
		{
			if (t <= trajPlan.posVelAccTime(idx, 14)) // Checks if the current time is less than the total time at the index of the current path sequence
			{
			// Assign local variables from posVelAccTime matrix
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

            sprayStatus = trajPlan.posVelAccTime(idx, 15);

			// Calculate quintic trajectory for x and z
			this->qX.calcQuinticTraj(t0,t1,t_quintic,x0,x1,v0_x,v1_x,a0_x,a1_x);
			this->qZ.calcQuinticTraj(t0,t1,t_quintic,z0,z1,v0_z,v1_z,a0_z,a1_z);

			// Calculate the inverse kinematics based on the quintic trajectory
			this->ik.calc(qX.getPos(), qZ.getPos(), qX.getVel(), qZ.getVel());

			// Assign the calculated data to the ROS message
			motorVel.data[0] = ik.getAngVel_q1()/(2*PI)*10; // TODO: Get rid of conversion from rad/s to rev/s and gear ratio calculation here. Should be in inverse kinematics.
			motorVel.data[1] = ik.getAngVel_q2()/(2*PI)*10; // TODO: Get rid of conversion from rad/s to rev/s and gear ratio calculation here. Should be in inverse kinematics.
            motorVel.data[2] = ik.getAngPos_q1()/(2*PI)*10;
            motorVel.data[3] = ik.getAngPos_q2()/(2*PI)*10;
            motorVel.data[4] = mode;
			motorVel.data[5] = this->t;

			// Terminal feedback
			std::cout << "x: " << motorVel.data[2] << ", z: " << motorVel.data[3] << ", t: " << motorVel.data[4] << std::endl;

			// ROS publisher
			publisher_->publish(motorVel);

			// Increase time variables with 10ms (0.01s)
			this->t += 0.01;
			this->t_quintic += 0.01;

			// If the current time is greater than the total time up until current sequence, then increase index and reset t_quintic
			if(t >= trajPlan.posVelAccTime(idx, 14))
			{
				idx++;
				t_quintic = 0;

				// Terminal feedback to know when each sequence starts
				std::cout << "idx: " << idx << std::endl;
			}
			}
		}else{
            //RCLCPP_INFO(this->get_logger(), "\n System parked.");
            motorVel.data[0] = 0; // TODO: Get rid of conversion from rad/s to rev/s and gear ratio calculation here. Should be in inverse kinematics.
			motorVel.data[1] = 0; // TODO: Get rid of conversion from rad/s to rev/s and gear ratio calculation here. Should be in inverse kinematics.
            motorVel.data[2] = ik.getAngPos_q1()/(2*PI)*10;
            motorVel.data[3] = ik.getAngPos_q2()/(2*PI)*10;
            motorVel.data[4] = mode;
			motorVel.data[5] = this->t;

            // ROS publisher
            publisher_->publish(motorVel);
        }
		}

		void topic_callback(const sensor_msgs::msg::Joy::SharedPtr input)
		{
			if(input->buttons[10] == 1.0)
			{
				kinematicsCalc.parked = true;
                idx = 0;
                run = true;
                mode = 0;
                t = 0;
                t_quintic = 0;
			}

			if(input->buttons[10] && input->buttons[4])
			{
				kinematicsCalc.parked = false;
                run = true;
			}

            if(idx >= 8 && input->buttons[4] && input->buttons[1])
            {
            mode = 1;
			motorVel.data[0] = 0; // TODO: Get rid of conversion from rad/s to rev/s and gear ratio calculation here. Should be in inverse kinematics.
			motorVel.data[1] = 0; // TODO: Get rid of conversion from rad/s to rev/s and gear ratio calculation here. Should be in inverse kinematics.
            motorVel.data[2] = 0;
            motorVel.data[3] = 0;
            motorVel.data[4] = mode;
			motorVel.data[5] = this->t;

            // ROS publisher
            publisher_->publish(motorVel);
            }
				// CAN publisher
				//can->send_data(can_ps4_output);
                can->send_spray_status(sprayStatus);


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