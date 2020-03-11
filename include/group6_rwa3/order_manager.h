
#ifndef GROUP6_RWA3_ORDER_MANAGER
#define GROUP6_RWA3_ORDER_MANAGER

#include <list>
#include <map>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>


#include <../group6_rwa3/robot_controller.h>

// class AriacSensorManager;
//class RobotController;

class AriacOrderManager {

private:
	ros::NodeHandle* order_manager_nh_;
	ros::Subscriber order_subscriber_;
	std::vector<osrf_gear::Order> received_orders_;
	std::vector<std::string> product_type;
	ros::Subscriber wayPoint_subscriber;
	// AriacSensorManager camera_;
	RobotController arm1_;
	//    RobotController arm2_;
	tf::TransformListener part_tf_listener_;
	std::pair<std::string,geometry_msgs::Pose> product_type_pose_;
	std::string object;
	std::map<std::string, std::vector<std::string>> product_frame_list_;
	osrf_gear::Order order_;

public:
	AriacOrderManager(ros::NodeHandle *);
	~AriacOrderManager();
	void OrderCallback(const osrf_gear::Order::ConstPtr&);
	void ExecuteOrder();
	std::string GetProductFrame(std::string);
	std::map<std::string, std::list<std::pair<std::string,geometry_msgs::Pose>>> GetOrder();
	bool PickAndPlace(std::pair<std::string,geometry_msgs::Pose>,int );
	std::vector<std::string> getProductType();
	void setProductType();
	void SubmitAGV(int);
	ros::NodeHandle* getnode();

	void pathplanningCallback(const geometry_msgs::PoseStamped&);
};
#endif //GROUP6_RWA3_ORDER_MANAGER

