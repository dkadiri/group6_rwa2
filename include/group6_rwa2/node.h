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
 *@file       Node.h
 *@author     Saurav Kumar
 *@copyright  MIT License
 *@brief      Node Handeler class declaration
 *            Declares functions to publish distance from the obstacle
 *            and collision flag
 */

#ifndef INCLUDE__GROPU6_NODE_H_
#define INCLUDE__GROPU6_NODE_H_

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
#include <../include/group6_rwa2/break_beam.h>
#include <../include/group6_rwa2/depth_camera.h>
#include <../include/group6_rwa2/orders.h>
#include <../include/group6_rwa2/proximity_sensor.h>
#include <../include/group6_rwa2/laser_profiler.h>

class Node {

    Node();
    ~Node();

private:
	ros::NodeHandle node;
    // creating class objects
    ProximitySensor proximitySensor;
    Orders orders;
    BreakBeam breakBeam;
    LaserProfiler laserProfiler;
	ros::Subscriber current_score_subscriber;
	ros::Subscriber competition_state_subscriber;
	
	ros::Subscriber arm_1_joint_state_subscriber;
	ros::Subscriber arm_2_joint_state_subscriber;

	
	ros::Subscriber proximity_sensor_subscriber;
	ros::Subscriber break_beam_subscriber;
	ros::Subscriber logical_camera1_subscriber;
	ros::Subscriber logical_camera2_subscriber;
	ros::Subscriber logical_camera3_subscriber;
	ros::Subscriber logical_camera4_subscriber;
	ros::Subscriber laser_profiler_subscriber;
	ros::Subscriber depth_camera_subscriber;
public:

};
#endif  // INCLUDE__GROPU6_NODE_H_
