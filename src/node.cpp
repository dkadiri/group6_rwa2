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





Node::Node() {
			
	// Subscribe to the '/ariac/proximity_sensor_1' topic.
	ros::Subscriber proximity_sensor_subscriber = node.subscribe(
			"/ariac/proximity_sensor_1", 10,
			&ProximitySensor::proximityCallBack, &proximity_sensor);
	// %EndTag(SUB_FUNC)%

	// Subscribe to the '/ariac/break_beam_1_change' topic.
	ros::Subscriber break_beam_subscriber = node.subscribe(
			"/ariac/break_beam_1_change", 10,
			&BreakBeam::breakBeamCallback, &break_beam);

	// Subscribe to the '/ariac/logical_camera_1' topic.
	ros::Subscriber logical_camera1_subscriber = node.subscribe(
			"/ariac/logical_camera_1", 10,
			&LogicalCamera::logicalCameraCallBack, &logicalCamera);

//	// Subscribe to the '/ariac/logical_camera_2' topic.
//	ros::Subscriber logical_camera2_subscriber = node.subscribe(
//			"/ariac/logical_camera_2", 10,
//			&MyCompetitionClass::logical_camera_callback, &comp_class);
//
//	// Subscribe to the '/ariac/logical_camera_3' topic.
//	ros::Subscriber logical_camera3_subscriber = node.subscribe(
//			"/ariac/logical_camera_3", 10,
//			&MyCompetitionClass::logical_camera_callback, &comp_class);
//
//	// Subscribe to the '/ariac/logical_camera_4' topic.
//	ros::Subscriber logical_camera4_subscriber = node.subscribe(
//			"/ariac/logical_camera_4", 10,
//			&MyCompetitionClass::logical_camera_callback, &comp_class);

	// Subscribe to the '/ariac/laser_profiler_1' topic.
	ros::Subscriber laser_profiler_subscriber = node.subscribe(
			"/ariac/laser_profiler_1", 10,
			&LaserProfiler::laserCallBack , &laserProfiler);

    ros::Subscriber orders_subscriber = node.subscribe(
            "/ariac/orders", 10,
            &Orders::orderCallback, &orders);

//	ros::Subscriber depth_camera_subscriber = node.subscribe(
//			"/ariac/depth_camera_1", 10,
//			&MyCompetitionClass::depth_camera_callback, &comp_class);

//    // Subscribe to the '/ariac/current_score' topic.
//    ros::Subscriber current_score_subscriber = node.subscribe(
//            "/ariac/current_score", 10, Node::current_score_callback);
//
//    // Subscribe to the '/ariac/competition_state' topic.
//    ros::Subscriber competition_state_subscriber = node.subscribe(
//            "/ariac/competition_state", 10, competition_state_callback);
//
//    // Subscribe to the '/ariac/joint_states' topic.
//    ros::Subscriber arm_1_joint_state_subscriber = node.subscribe(
//            "/ariac/arm1/joint_states", 10,
//            &MyCompetitionClass::arm_1_joint_state_callback, &comp_class);
//
//    ros::Subscriber arm_2_joint_state_subscriber = node.subscribe(
//            "/ariac/arm2/joint_states", 10,
//            &MyCompetitionClass::arm_2_joint_state_callback, &comp_class);
}

