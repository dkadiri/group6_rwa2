//
// Created by zeid on 2/27/20.
//

#ifndef GROUP6_RWA3_ROBOT_CONTROLLER_H
#define GROUP6_RWA3_ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <stdarg.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <initializer_list>

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

class RobotController {
private:
	ros::NodeHandle robot_controller_nh_;
	moveit::planning_interface::MoveGroupInterface::Options robot_controller_options;
	ros::ServiceClient gripper_client_;
	ros::NodeHandle gripper_nh_;
	ros::Subscriber gripper_subscriber_;

	tf::TransformListener robot_tf_listener_;
	tf::StampedTransform robot_tf_transform_;
	tf::TransformListener agv_tf_listener_;
	tf::StampedTransform agv_tf_transform_;

	geometry_msgs::Pose target_pose_;

	moveit::planning_interface::MoveGroupInterface robot_move_group_;
	moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;

	osrf_gear::VacuumGripperControl gripper_service_;
	osrf_gear::VacuumGripperState gripper_status_;

	std::string object;
	bool plan_success_;
	std::vector<double> home_joint_pose_;
	geometry_msgs::Pose home_cart_pose_;
	geometry_msgs::Quaternion fixed_orientation_;
	geometry_msgs::Pose agv_position_;
	std::vector<double> end_position_;
//	geometry_msgs::Pose end_pose_;
	double offset_;
	double roll_def_,pitch_def_,yaw_def_;
	tf::Quaternion q;
	int counter_;
	bool gripper_state_, drop_flag_;

public:
	RobotController(std::string);
	~RobotController();
	bool Planner();
	void Execute();
	void GoToTarget(std::initializer_list<geometry_msgs::Pose>);
	void GoToTarget(const geometry_msgs::Pose& );
	void SendRobotHome();
	bool DropPart(geometry_msgs::Pose );
	void GripperToggle(const bool& );
	void GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr& );
	void GripperStateCheck(geometry_msgs::Pose );
	bool PickPart(geometry_msgs::Pose& );
	bool isPartAttached();
	void GoToEnd();
	void GoToPose(const std::vector<double> &);
	geometry_msgs::Pose getHomeCartPose();
//	void GotoTarget(const geometry_msgs::Pose&);
//	geometry_msgs::Pose convertToArmBaseFrame();
//	geometry_msgs::Pose convertToArmBaseFrame( const  geometry_msgs::TransformStamped& );


};
#endif //GROUP6_RWA3_ROBOT_CONTROLLER_H
