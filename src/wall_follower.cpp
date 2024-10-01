// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim
//
// Modified by Claude Sammut for COMP3431
// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>


using namespace std::chrono_literals;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	for (int i = 0; i < 12; i++)
		scan_data_[i] = 0.0;

	robot_pose_ = 0.0;
	near_start = false;

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/

#define START_RANGE	0.2

void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	static bool first = true;
	static bool start_moving = true;

	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	robot_pose_ = yaw;

	double current_x =  msg->pose.pose.position.x;
	double current_y =  msg->pose.pose.position.y;
	if (first)
	{
		start_x = current_x;
		start_y = current_y;
		first = false;
	}
	else if (start_moving)
	{
		if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE)
			start_moving = false;
	}
	else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE)
	{
		fprintf(stderr, "Near start!!\n");
		near_start = true;
		first = true;
		start_moving = true;
	}
}

#define BEAM_WIDTH 10

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

	double closest = msg->range_max;
	for (int angle = 360-BEAM_WIDTH; angle < 360; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	for (int angle = 0; angle < BEAM_WIDTH; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	scan_data_[0] = closest;

	for (int i = 1; i < 12; i++)
	{
		closest = msg->range_max;
		for (int angle = scan_angle[i]-BEAM_WIDTH; angle < scan_angle[i]+BEAM_WIDTH; angle++)
			if (msg->ranges.at(angle) < closest)
				closest = msg->ranges.at(angle);
		scan_data_[i] = closest;
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** return number of the paramaters larger than threshold
********************************************************************************/
int compareDoubles(double a, double b, double threshold) {

    if (a < threshold and b < threshold) {
        return 0;
    }

    else if ((a >= threshold && b < threshold) || (a < threshold && b >= threshold)) {
        return 1;
    }
    else if (a >= threshold && b >= threshold) {
        return 2;
    }

    return -1; 
}

/********************************************************************************
** Update functions
********************************************************************************/

bool pl_near;


void WallFollower::update_callback()
{
	const char* direction_names[12] = {
    "FRONT",
    "FRONT_LEFT",
    "LEFT_FRONT",
    "LEFT",
    "LEFT_BACK",
    "BACK_LEFT",
    "BACK",
    "BACK_RIGHT",
    "RIGHT_BACK",
    "RIGHT",
    "RIGHT_FRONT",
    "FRONT_RIGHT"
};
	// 
    constexpr double safe_distance = 0.1;    // saft stop distance
    constexpr double follow_distance = 0.3;  // ideal wall-following distance
	constexpr double warning_distance = 0.6;  // warnning distance
    constexpr double max_linear_speed = 0.1; // max linear speed
	// actually +-1.82
    constexpr double max_angular_speed = 1.5; // max angular speed: absolute value

    // sensor data: four main direction
    double front_distance = scan_data_[FRONT];
    double left_distance = scan_data_[LEFT];
    double right_distance = scan_data_[RIGHT];
    double back_distance = scan_data_[BACK];

    // Urgent Stop
    for (int i = 0; i < 12; ++i) {
        if (scan_data_[i] < safe_distance) {
            RCLCPP_WARN(this->get_logger(), "Obstacle too close in direction %s! Stopping.", direction_names[i]);
            update_cmd_vel(0.0, 0.0); 
            return;
        }
    }

    // Stop Due to close to the start point
    if (near_start) {
        RCLCPP_INFO(this->get_logger(), "near start point. Stopping.");
        update_cmd_vel(0.0, 0.0);
        return;
    }

    double linear_speed = max_linear_speed*0.5;;
    double angular_speed = 0.0;

	//remember: 10ms for once update_cmd_vel, 1s 100 times for calling the speed changed

   /*******************************************
	 * Left-hand rule: prioritize left wall
	 *******************************************/
	if (compareDoubles(left_distance, scan_data_[LEFT_FRONT], follow_distance) > 1) {
		
		// The left wall is too far, turn left to get closer
		RCLCPP_INFO(this->get_logger(), "Apply Left-hand rule: turning left to follow the left wall.");
		angular_speed = max_angular_speed * 0.2; // Turn left
		linear_speed = max_linear_speed *0.4;
		
		//  Update velocities based on the computed linear and angular speeds
		update_cmd_vel(linear_speed, angular_speed);
		RCLCPP_INFO(this->get_logger(), "Updated linear speed: %f, angular speed: %f", linear_speed, angular_speed);
	} else if (compareDoubles(left_distance, scan_data_[LEFT_FRONT], follow_distance) < 1) {
		
		// The left wall is too close, turn right to move away
		RCLCPP_INFO(this->get_logger(), "Apply Left-hand rule: too close, turning right");
		angular_speed = -max_angular_speed * 0.3; // slightly Turn right
		linear_speed = max_linear_speed *0.2;
		
		//  Update velocities based on the computed linear and angular speeds
		update_cmd_vel(linear_speed, angular_speed);
		RCLCPP_INFO(this->get_logger(), "Updated linear speed: %f, angular speed: %f", linear_speed, angular_speed);
	}

    /*******************************************
	 * Handling front obstacles
	 *******************************************/
	if (front_distance < warning_distance) {
		
		// There is an obstacle ahead, turn right to avoid it
		RCLCPP_INFO(this->get_logger(), "Obstacle ahead, turning right.");
		angular_speed = -max_angular_speed; // Turn right
		linear_speed = max_linear_speed * 0.4;    // Slow down
		
		//  Update velocities based on the computed linear and angular speeds
		update_cmd_vel(linear_speed, angular_speed);
		RCLCPP_INFO(this->get_logger(), "Updated linear speed: %f, angular speed: %f", linear_speed, angular_speed);
	} else {
		
		// No obstacle ahead, continue moving forward
		RCLCPP_INFO(this->get_logger(), "Path ahead is clear, moving forward.");
		angular_speed = 0.0; // Move straight
		linear_speed = max_linear_speed * 0.2;
		
		//  Update velocities based on the computed linear and angular speeds
		update_cmd_vel(linear_speed, angular_speed);
		RCLCPP_INFO(this->get_logger(), "Updated linear speed: %f, angular speed: %f", linear_speed, angular_speed);
	}

	/*******************************************
	 * Handling right side for safety
	 *******************************************/
	if (right_distance < safe_distance) {
		
		// Right side too close to an obstacle, turn left to avoid it
		RCLCPP_WARN(this->get_logger(), "Obstacle too close on the right! Single detection Turning left.");
		angular_speed = max_angular_speed * 0.5; // Turn left
		
		//  Update velocities based on the computed linear and angular speeds
		update_cmd_vel(linear_speed, angular_speed);
		RCLCPP_INFO(this->get_logger(), "Updated linear speed: %f, angular speed: %f", linear_speed, angular_speed);
	}
    /*******************************************
     * 后方处理逻辑
     *******************************************/
    if (back_distance < follow_distance) {
		
		// 后方有障碍物，不能后退
		RCLCPP_WARN(this->get_logger(), "Obstacle too close behind! Adjusting position to avoid being stuck.");
		
		// 停止任何后退行为
		if(linear_speed < 0){
			linear_speed = 0.0;  // 停止线速度，避免后退
		}
		
		// 检查左右侧的空间，决定转向方向
		if (left_distance > right_distance) {
			
			// 左侧空间较大，尝试向左转
			RCLCPP_INFO(this->get_logger(), "Turning left to avoid obstacle behind.");
			angular_speed = max_angular_speed * 0.5;  // 左转，避免撞上后方障碍物
			
			//  Update velocities based on the computed linear and angular speeds
			update_cmd_vel(linear_speed, angular_speed);
			RCLCPP_INFO(this->get_logger(), "Updated linear speed: %f, angular speed: %f", linear_speed, angular_speed);
		} else {
			
			// 右侧空间较大，尝试向右转
			RCLCPP_INFO(this->get_logger(), "Turning right to avoid obstacle behind.");
			angular_speed = -max_angular_speed * 0.5;  // 右转，避免撞上后方障碍物

			//  Update velocities based on the computed linear and angular speeds
			update_cmd_vel(linear_speed, angular_speed);
			RCLCPP_INFO(this->get_logger(), "Updated linear speed: %f, angular speed: %f", linear_speed, angular_speed);
		}
	}
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();

	return 0;
}
