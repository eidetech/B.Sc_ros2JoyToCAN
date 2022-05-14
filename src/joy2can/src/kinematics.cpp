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
#include "quintic.h"
#include "ik.h"
#include "trajectoryPlanner.h"
#include "Eigen/Dense"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Kinematics : public rclcpp::Node
{
	public:
		Kinematics(): Node("kinematics"), can(new CANbus)
		{

		// Joy subscriber
		subscription_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
		"joy", 10, std::bind(&Kinematics::joy_callback, this, _1));

		RCLCPP_INFO(this->get_logger(), "Subscribed to joy topic");

        // Web subscriber
		subscription_web_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
		"web_data", 10, std::bind(&Kinematics::web_callback, this, _1));

		RCLCPP_INFO(this->get_logger(), "Subscribed to web_data topic");

		// Publisher
		publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_sp", 10);
		timer_ = this->create_wall_timer(
		10ms, std::bind(&Kinematics::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Publishing to motor_sp topic");

        // IMU publisher
		publisher_imu_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("imu", 10);
		timer_imu_ = this->create_wall_timer(
		0.1ms, std::bind(&Kinematics::timer_callback_imu, this));

		}

		~Kinematics()
		{
		delete can; 
		}

		// ROS message to send motor velocities
		std_msgs::msg::Float32MultiArray motorVel;

		// ROS message to send IMU data
		std_msgs::msg::Float32MultiArray imuData;

		// Quintic objects for calculating x and z position, velocity and acceleration
		Quintic qX;
		Quintic qZ;

		// Inverse kinematics object for converting [x,z] coordinates to [q1,q2] positions and velocities.
		IK ik;

		// Trajectory planner object
		TrajectoryPlanner trajPlan;

		// Time variable for keeping track of total time in path
		float t = 0.;

		// Time variable for keeping track of current time during a path sequence (used for quintic calculations),
		// since the quintic class calculates each path with t0 = 0, and t1 = the time it takes to run that path.
		float t_quintic = 0.;

        float t_total = 0.;

		// Bool used to print vectors and matrices once at the first callback (for debugging)
		bool printed = false;

        bool run = false;

        bool reset_initial_q_pos = true;

        // Velocity/position control mode
        // mode = 0 ==> velocity control
        // mode = 1 ==> position control
        int mode = 1; // Start system in position control in order to home robot if the robot is not at home

		// Variables used for quintic trajectory planning
		float t0 = 0.;
		float t1 = 0.;
		float t_sum = 0.;
		float x0 = 0.;
		float x1 = 0.;
		float z0 = 0.;
		float z1 = 0.;
		float v0_x = 0.;
		float v1_x = 0.;
		float v0_z = 0.;
		float v1_z = 0.;
		float a0_x = 0.;
		float a1_x = 0.;
		float a0_z = 0.;
		float a1_z = 0.;
        float sprayStatus = 2.;
		
		// Index variable to keep track of which path sequence is running
		int idx = 0;

        // IMU pitch angle
        float pitch = 0;
        float roll = 0;
        float yaw = 0;
        float pitch_sp_readback = 0;

        float pitch_from_web_hmi = 0;
        int idle_speed_from_web_hmi = 0;
        int counterforce_speed_from_web_hmi = 0;

        int idle_speed_to_web_hmi = 0;
        int counterforce_speed_to_web_hmi = 0;

        float kp_out = 0;
        float ki_out = 0;
        float kd_out = 0;

	private:
        void timer_callback_imu(){
        
        can->receive_can();


        // Receive CAN messages from IMU and propeller speeds.
        can->read_IMU_data();
        pitch = can->pitch;
        roll = can->roll;
        yaw = can->yaw;
        pitch_sp_readback = can->pitch_sp_readback;

        can->receive_propeller();
        idle_speed_to_web_hmi = can->rx_idle_speed;
        counterforce_speed_to_web_hmi = can->rx_counterforce_speed;

        can->receive_pid();

        // Send ROS message with IMU data so that it can be logged with rqt:
        imuData.data.resize(9);
        imuData.data[0] = pitch;
        imuData.data[1] = roll;
        imuData.data[2] = yaw;
        imuData.data[3] = pitch_sp_readback;
        imuData.data[4] = can->rx_idle_speed;
        imuData.data[5] = can->rx_counterforce_speed;
        imuData.data[6] = can->rx_kp;
        imuData.data[7] = can->rx_ki;
        imuData.data[8] = can->rx_kd;
        // ROS publisher
		publisher_imu_->publish(imuData);
        
        // CAN senders
        can->send_pitch_sp(pitch_from_web_hmi);
        can->send_propeller(idle_speed_from_web_hmi, counterforce_speed_from_web_hmi);
        can->send_pid(kp_out, ki_out, kd_out);
        }

		void timer_callback()
		{
		// Resize motorVel array to 3 elements (this could be refactored to not happen every callback)
		motorVel.data.resize(9);

		// Set offsets from origo to pulleys
		// ik.setOffsets(0, trajPlan.get_outer_frame_height(), trajPlan.get_outer_frame_width(), trajPlan.get_outer_frame_height());

		// if(!printed)
		// {
		// trajPlan.plan();
		// trajPlan.printPoints();
		// trajPlan.calcCartesianPosVelAcc();
		// printed = !printed;
		// }
		
		if(idx <= trajPlan.N-1 && run)
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

            t_total = trajPlan.posVelAccTime(trajPlan.N-1,14);

            sprayStatus = trajPlan.posVelAccTime(idx, 15);

            mode = 0; // 0 = velocity control

			// Calculate quintic trajectory for x and z
			this->qX.calcQuinticTraj(t0,t1,t_quintic,x0,x1,v0_x,v1_x,a0_x,a1_x);
			this->qZ.calcQuinticTraj(t0,t1,t_quintic,z0,z1,v0_z,v1_z,a0_z,a1_z);

            // Angle offset compensation with rotation matrix
            float theta = (-0.1)*PI/180;
            Matrix2f rot;
            Vector2f vec;
            
            rot << cos(theta), sin(theta), -sin(theta), cos(theta);
            vec << qX.getPos(), qZ.getPos();

            vec = rot*vec;

            Vector2f vec_vel;
            
            vec_vel << qX.getVel(), qZ.getVel();

            vec_vel = rot*vec_vel;

			// Calculate the inverse kinematics based on the quintic trajectory
			this->ik.calc(qX.getPos(), qZ.getPos(), qX.getVel(), qZ.getVel());

            if(reset_initial_q_pos)
            {
            // Assign the calculated data to the ROS message
			motorVel.data[0] = ik.getAngVel_q1();
			motorVel.data[1] = ik.getAngVel_q2();
            motorVel.data[2] = ik.getAngPos_q1();
            motorVel.data[3] = ik.getAngPos_q2();
            // Velocity/position control mode
            // mode = 0 ==> velocity control
            // mode = 1 ==> position control
            motorVel.data[4] = mode;
			motorVel.data[5] = this->t;
            motorVel.data[6] = this->t_total;
            motorVel.data[7] = 1.0; // Reset signal, 1 = True
            reset_initial_q_pos = false; 
            }else{
			// Assign the calculated data to the ROS message
			motorVel.data[0] = ik.getAngVel_q1();
			motorVel.data[1] = ik.getAngVel_q2();
            motorVel.data[2] = ik.getAngPos_q1();
            motorVel.data[3] = ik.getAngPos_q2();
            // Velocity/position control mode
            // mode = 0 ==> velocity control
            // mode = 1 ==> position control
            motorVel.data[4] = mode;
			motorVel.data[5] = this->t;
            motorVel.data[6] = this->t_total;
            motorVel.data[7] = 0.0; // Reset signal, 0 = False
            motorVel.data[8] = pitch; // Pitch angle from IMU to web HMI
            }
			// Terminal feedback
			std::cout << "x: " << qX.getPos() << ", z: " << qZ.getPos() << ", t: " << motorVel.data[5] << std::endl;

			// ROS publisher
			publisher_->publish(motorVel);

            can->send_spray_status(sprayStatus); // Sends CAN messages to MCU controlling spray gun

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
		}else{ // System is not in run mode
            motorVel.data[0] = 0;
			motorVel.data[1] = 0;
            motorVel.data[2] = ik.getAngPos_q1();
            motorVel.data[3] = ik.getAngPos_q2();
            motorVel.data[4] = mode;
			motorVel.data[5] = this->t;
            motorVel.data[7] = 0.0; // Reset signal, 0 = False
            motorVel.data[8] = pitch; // Pitch angle from IMU to web HMI

            // ROS publisher
            publisher_->publish(motorVel);

            // CAN publisher
            sprayStatus = 2; // Spray gun off
            can->send_spray_status(sprayStatus); // Sends CAN messages to MCU controlling spray gun
        }
		}

		void joy_callback(const sensor_msgs::msg::Joy::SharedPtr input)
		{
			// if(input->buttons[10] == 1.0)
			// {
            //     ik.setOffsets(0, trajPlan.get_outer_frame_height(), trajPlan.get_outer_frame_width(), trajPlan.get_outer_frame_height());
            //     trajPlan.reset();
            //     trajPlan.plan();
            //     trajPlan.calcCartesianPosVelAcc();

            //     run = true;
            //     mode = 0;
            //     idx = 0;
            //     t = 0;
            //     t_quintic = 0;
			// }

			if(input->buttons[10] && input->buttons[4])
			{
                ik.setOffsets(0, trajPlan.get_outer_frame_height(), trajPlan.get_outer_frame_width(), trajPlan.get_outer_frame_height());
                trajPlan.reset();
                trajPlan.plan();
                trajPlan.calcCartesianPosVelAcc();

                run = true;
                reset_initial_q_pos = true;
                mode = 0;
                idx = 0;
                t = 0;
                t_quintic = 0;
			}

            if(idx >= 8 && input->buttons[4] && input->buttons[1])
            {
            mode = 1;
			motorVel.data[0] = 0;
			motorVel.data[1] = 0;
            motorVel.data[2] = 0;
            motorVel.data[3] = 0;
            motorVel.data[4] = mode;
			motorVel.data[5] = this->t;
            motorVel.data[7] = 0.0;

            // ROS publisher
            publisher_->publish(motorVel);
            }
            // CAN publisher
            //can->send_data(can_ps4_output);



		}

        void web_callback(const std_msgs::msg::Float32MultiArray::SharedPtr web_data)
		{
            std::cout << "Incoming data from web." << std::endl;

            // Update all job parameters with data streamed from the web HMI
            trajPlan.set_outer_frame_width(web_data->data[0]);
            trajPlan.set_outer_frame_height(web_data->data[1]);
            trajPlan.set_wall_width(web_data->data[2]);
            trajPlan.set_wall_height(web_data->data[3]);
            trajPlan.set_x_offset(web_data->data[4]);
            trajPlan.set_z_offset(web_data->data[5]);
            trajPlan.calc_vStep(web_data->data[6]);
            pitch_from_web_hmi = (int)(web_data->data[8]);
            idle_speed_from_web_hmi = (int)(web_data->data[9]);
            counterforce_speed_from_web_hmi = (int)(web_data->data[10]);
            kp_out = web_data->data[11];
            ki_out = web_data->data[12];
            kd_out = web_data->data[13];

            std::cout << web_data->data[10] << std::endl;
            
            // Debug check
            // std::cout << "web_data->data[0] = " << web_data->data[0] << ", trajPlan.getget_outer_frame_width() = " << trajPlan.get_outer_frame_width() << std::endl;
            if ((int)web_data->data[7] == 2) // Start job
            {
                ik.setOffsets(0, trajPlan.get_outer_frame_height(), trajPlan.get_outer_frame_width(), trajPlan.get_outer_frame_height());
                trajPlan.reset();
                trajPlan.plan();
                trajPlan.calcCartesianPosVelAcc();

                reset_initial_q_pos = true;
                mode = 0.;
                idx = 0;
                t = 0.;
                t_quintic = 0.;
                run = true;

            }else if ((int)web_data->data[7] == 1) // Pause job
            {
                run = false;
                // CAN publisher
                sprayStatus = 2; // Stop spray gun
                can->send_spray_status(sprayStatus); // Sends CAN messages to MCU controlling spray gun
            }else{ // Stop job
                run = false;
                idx = 0;
                t = 0;
                t_quintic = 0;
                mode = 1.0;
                // CAN publisher
                sprayStatus = 2; // Stop spray gun
                can->send_spray_status(sprayStatus); // Sends CAN messages to MCU controlling spray gun
            }
            
            
        }
		CANbus* can;
		rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer_imu_;
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_imu_;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_joy_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_web_;
	};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Kinematics>());
	rclcpp::shutdown();
	return 0;
}