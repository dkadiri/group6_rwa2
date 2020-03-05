/**
 *  MIT License
 *
 *  Copyright (c) 2018 Saimouli Katragadda, Saurav Kumar
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/**
 *@file       Node.cpp
 *@author     Saurav Kumar
 *@copyright  MIT License
 *@brief      CollisionDetector class declaration
 *            Declares functions to publish distance from the obstacle
 *            and collision flag
 */

#include <algorithm>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <osrf_gear/LogicalCameraImage.h>

#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3
#include <../include/group6_rwa2/node.h>
#include <group6_rwa2/break_beam.h>
#include <../include/group6_rwa2/depth_camera.h>
#include <../include/group6_rwa2/orders.h>
#include <../include/group6_rwa2/proximity_sensor.h>
#include <../include/group6_rwa2/laser_profiler.h>




Node::Node() {

	// Subscribe to the '/ariac/current_score' topic.
	ros::Subscriber current_score_subscriber = node.subscribe(
			"/ariac/current_score", 10, Node::current_score_callback);

	// Subscribe to the '/ariac/competition_state' topic.
	ros::Subscriber competition_state_subscriber = node.subscribe(
			"/ariac/competition_state", 10, competition_state_callback);

	// Subscribe to the '/ariac/joint_states' topic.
	ros::Subscriber arm_1_joint_state_subscriber = node.subscribe(
			"/ariac/arm1/joint_states", 10,
			&MyCompetitionClass::arm_1_joint_state_callback, &comp_class);

	ros::Subscriber arm_2_joint_state_subscriber = node.subscribe(
			"/ariac/arm2/joint_states", 10,
			&MyCompetitionClass::arm_2_joint_state_callback, &comp_class);
			
	// Subscribe to the '/ariac/proximity_sensor_1' topic.
	ros::Subscriber proximity_sensor_subscriber = node.subscribe(
			"/ariac/proximity_sensor_1", 10,
			&ProximitySensor::proximityCallBack, &proximitySensor);
	// %EndTag(SUB_FUNC)%

	// Subscribe to the '/ariac/break_beam_1_change' topic.
	ros::Subscriber break_beam_subscriber = node.subscribe(
			"/ariac/break_beam_1_change", 10,
			&BreakBeam::break_beam_callback, &breakBeam);

	// Subscribe to the '/ariac/logical_camera_1' topic.
	ros::Subscriber logical_camera1_subscriber = node.subscribe(
			"/ariac/logical_camera_1", 10,
			&LogicalCamera::logical_camera_callback, &comp_class);

	// Subscribe to the '/ariac/logical_camera_2' topic.
	ros::Subscriber logical_camera2_subscriber = node.subscribe(
			"/ariac/logical_camera_2", 10,
			&MyCompetitionClass::logical_camera_callback, &comp_class);

	// Subscribe to the '/ariac/logical_camera_3' topic.
	ros::Subscriber logical_camera3_subscriber = node.subscribe(
			"/ariac/logical_camera_3", 10,
			&MyCompetitionClass::logical_camera_callback, &comp_class);

	// Subscribe to the '/ariac/logical_camera_4' topic.
	ros::Subscriber logical_camera4_subscriber = node.subscribe(
			"/ariac/logical_camera_4", 10,
			&MyCompetitionClass::logical_camera_callback, &comp_class);

	// Subscribe to the '/ariac/laser_profiler_1' topic.
	ros::Subscriber laser_profiler_subscriber = node.subscribe(
			"/ariac/laser_profiler_1", 10,
			&LaserProfiler::laserCallBack , &laserProfiler);

	ros::Subscriber depth_camera_subscriber = node.subscribe(
			"/ariac/depth_camera_1", 10,
			&MyCompetitionClass::depth_camera_callback, &comp_class);
}

void Node::current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
	if (msg->data != current_score_) {
		ROS_INFO_STREAM("Score: " << msg->data);
	}
	current_score_ = msg->data;
}

/// Called when a new message is received.
void Node::competition_state_callback(const std_msgs::String::ConstPtr & msg) {
	if (msg->data == "done" && competition_state_ != "done") {
		ROS_INFO("Competition ended.");
	}
	competition_state_ = msg->data;
}

/// Called when a new Order message is received.


// %Tag(CB_CLASS)%
/// Called when a new JointState message is received.
void Node::arm_1_joint_state_callback(
		const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
	ROS_INFO_STREAM_THROTTLE(10,
			"Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
	// ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
	arm_1_current_joint_states_ = *joint_state_msg;
	if (!arm_1_has_been_zeroed_) {
		arm_1_has_been_zeroed_ = true;
		ROS_INFO("Sending arm to zero joint positions...");
		send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
	}
}

void Node::arm_2_joint_state_callback(
		const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
	ROS_INFO_STREAM_THROTTLE(10,
			"Joint States arm 2 (throttled to 0.1 Hz):\n" << *joint_state_msg);
	// ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
	arm_2_current_joint_states_ = *joint_state_msg;
	if (!arm_2_has_been_zeroed_) {
		arm_2_has_been_zeroed_ = true;
		ROS_INFO("Sending arm 2 to zero joint positions...");
		send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
	}
}
// %EndTag(CB_CLASS)%

// %Tag(ARM_ZERO)%
/// Create a JointTrajectory with all positions set to zero, and command the arm.
void Node::send_arm_to_zero_state(ros::Publisher & joint_trajectory_publisher) {
	// Create a message to send.
	trajectory_msgs::JointTrajectory msg;

	// Fill the names of the joints to be controlled.
	// Note that the vacuum_gripper_joint is not controllable.
	msg.joint_names.clear();
	msg.joint_names.push_back("shoulder_pan_joint");
	msg.joint_names.push_back("shoulder_lift_joint");
	msg.joint_names.push_back("elbow_joint");
	msg.joint_names.push_back("wrist_1_joint");
	msg.joint_names.push_back("wrist_2_joint");
	msg.joint_names.push_back("wrist_3_joint");
	msg.joint_names.push_back("linear_arm_actuator_joint");
	// Create one point in the trajectory.
	msg.points.resize(1);
	// Resize the vector to the same length as the joint names.
	// Values are initialized to 0.
	msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
	// How long to take getting to the point (floating point seconds).
	msg.points[0].time_from_start = ros::Duration(0.001);
	ROS_INFO_STREAM("Sending command:\n" << msg);
	joint_trajectory_publisher.publish(msg);
}
// %EndTag(ARM_ZERO)%

/// Called when a new LogicalCameraImage message is received.
void Node::depth_camera_callback(const sensor_msgs::PointCloud::ConstPtr & msg) {
	ROS_INFO_STREAM_THROTTLE(10,
			"Frame ID of depth camera is " << msg->header.frame_id);
}

/**
 * @brief takes in TransformStamped by reference and sets the translation and rotation fields from the pose
 * sent in by user.
 * @param geometry_msgs::TransformStamped transformStamped - transform of type TransformStamped from geometry_msgs.
 * @param geometry_msgs::Pose pose.
 * @return None.
 */
// void set_pose(geometry_msgs::TransformStamped &transformStamped, const geometry_msgs::Pose pose) {
// 	transformStamped.transform.translation.x = pose.position.x;
// 	transformStamped.transform.translation.y = pose.position.y;
// 	transformStamped.transform.translation.z = pose.position.z;
// 	transformStamped.transform.rotation.x = pose.orientation.x;
// 	transformStamped.transform.rotation.y = pose.orientation.y;
// 	transformStamped.transform.rotation.z = pose.orientation.z;
// 	transformStamped.transform.rotation.w = pose.orientation.w;
// }


void Node::logical_camera_callback(
		const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
	ROS_INFO_STREAM_THROTTLE(10,
			"Logical camera: '" << image_msg->models.size() << "' objects.");
	// auto sensor_pose = image_msg->pose;
	// auto current_time = ros::Time::now();
	// static tf2_ros::TransformBroadcaster br_w_s;
	// geometry_msgs::TransformStamped transformStamped1;
	// static tf2_ros::TransformBroadcaster br_s_c;
	// geometry_msgs::TransformStamped transformStamped2;
	// tf2_ros::Buffer tfBuffer;
	// tf2_ros::TransformListener tfListener(tfBuffer);
	// //		ros::Duration timeout(2.0);
	// geometry_msgs::TransformStamped transformStamped3;

	// transformStamped1.header.stamp = current_time;
	// transformStamped1.header.frame_id = "world";
	// transformStamped1.child_frame_id = "logical_sensor";

	// transformStamped2.header.stamp = current_time;
	// transformStamped2.header.frame_id = "logical_sensor";
	// transformStamped2.child_frame_id = "logical_sensor_child";

	// set_pose(transformStamped1, sensor_pose);
	// br_w_s.sendTransform(transformStamped1);
	// ros::Duration(0.2).sleep();
	// for(auto it =image_msg->models.begin(); it!=image_msg->models.end();++it) {
	// 	set_pose(transformStamped2, it->pose);
	// 	br_s_c.sendTransform(transformStamped2);
	// 	ros::Duration(0.2).sleep();
	// 	auto type = it->type;
	// 	try{
	// 		transformStamped3 = tfBuffer.lookupTransform("world", "logical_sensor_child",
	// 				ros::Time(0));
	// 	}
	// 	catch (tf2::TransformException &ex) {
	// 		ROS_WARN("exception");
	// 		ROS_WARN("%s",ex.what());
	// 		ros::Duration(1.0).sleep();
	// 	}
	// 	ROS_INFO("%s in world frame: [%f,%f,%f] [%f,%f,%f,%f]",type.c_str(),transformStamped3.transform.translation.x,
	// 			transformStamped3.transform.translation.y,
	// 			transformStamped3.transform.translation.z,
	// 			transformStamped3.transform.rotation.x,
	// 			transformStamped3.transform.rotation.y,
	// 			transformStamped3.transform.rotation.z,
	// 			transformStamped3.transform.rotation.w);
	// }

}

/// Called when a new Proximity message is received.
void Node::break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
	if (msg->object_detected) {  // If there is an object in proximity.
		ROS_INFO("Break beam triggered.");
	}
}


};

void Node::proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
	if ((msg->max_range - msg->range) > 0.01) {  // If there is an object in proximity.
		ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
	}
}

void Node::laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
	size_t number_of_valid_ranges = std::count_if(
			msg->ranges.begin(), msg->ranges.end(), [](const float f) {return std::isfinite(f);});
	if (number_of_valid_ranges > 0) {
		ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
	}
}
