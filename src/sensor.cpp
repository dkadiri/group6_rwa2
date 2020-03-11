//
// Created by zeid on 2/27/20.
//
#include "../include/group6_rwa2/sensor.h"

AriacSensorManager::AriacSensorManager(AriacOrderManager* obj): orderManager(obj)
{
	sensor_nh_ = orderManager->getnode();
	ROS_INFO_STREAM(">>>>> Subscribing to logical sensors");

	camera_4_subscriber_ = sensor_nh_->subscribe("/ariac/logical_camera_4", 10 , &AriacSensorManager::logicalCamera4Callback, this);
	//	breakbeam_subscriber = sensor_nh_->subscribe("/ariac/break_beam_1", 10,	&AriacSensorManager::breakBeamCallback,this);
	//	tracking_part = new osrf_gear::Model();
	tracking_part= nullptr;
}

AriacSensorManager::~AriacSensorManager() {}


void AriacSensorManager::setPose( const geometry_msgs::Pose src, geometry_msgs::Pose & dstn){
	dstn.position.x = src.position.x;
	dstn.position.y = src.position.y;
	dstn.position.z = src.position.z;
	dstn.orientation.x = src.orientation.x;
	dstn.orientation.y = src.orientation.y;
	dstn.orientation.z = src.orientation.z;
	dstn.orientation.w = src.orientation.w;
}
void AriacSensorManager::setPose(const geometry_msgs::Pose pose, geometry_msgs::TransformStamped &transformStamped ){
	transformStamped.transform.translation.x = pose.position.x;
	transformStamped.transform.translation.y = pose.position.y;
	transformStamped.transform.translation.z = pose.position.z;
	transformStamped.transform.rotation.x = pose.orientation.x;
	transformStamped.transform.rotation.y = pose.orientation.y;
	transformStamped.transform.rotation.z = pose.orientation.z;
	transformStamped.transform.rotation.w = pose.orientation.w;
}

void AriacSensorManager::setTransform () {
	setPose( tracking_part->pose, transformStamped2);
	br_s_c.sendTransform(transformStamped2);
	ros::Duration(0.01).sleep();
	auto type = tracking_part->type;
	try{
		transformStamped3 = tfBuffer.lookupTransform("world", "logical_sensor_child",
				ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("exception");
		ROS_WARN("%s",ex.what());
		ros::Duration(0.01).sleep();
	}

	ROS_INFO("%s in world frame: [%f,%f,%f] [%f,%f,%f,%f]",type.c_str(),transformStamped3.transform.translation.x,
			transformStamped3.transform.translation.y,
			transformStamped3.transform.translation.z,
			transformStamped3.transform.rotation.x,
			transformStamped3.transform.rotation.y,
			transformStamped3.transform.rotation.z,
			transformStamped3.transform.rotation.w);
}

void AriacSensorManager::logicalCamera4Callback(
		const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
	// order type same as camera type
	ROS_INFO_STREAM_THROTTLE(2,
			"Logical camera: '" << image_msg->models.size() << "' objects.");
	auto sensor_pose = image_msg->pose;
	auto current_time = ros::Time::now();
	tf2_ros::TransformListener tfListener(tfBuffer);


	transformStamped1.header.stamp = current_time;
	transformStamped1.header.frame_id = "world";
	transformStamped1.child_frame_id = "logical_sensor";

	transformStamped2.header.stamp = current_time;
	transformStamped2.header.frame_id = "logical_sensor";
	transformStamped2.child_frame_id = "logical_sensor_child";

	setPose(sensor_pose,transformStamped1);
	br_w_s.sendTransform(transformStamped1);
	ros::Duration(0.01).sleep();
	auto order = orderManager->getProductType();
	for(auto it =image_msg->models.begin(); it!=image_msg->models.end();++it) {
		//		ROS_INFO_STREAM("debug : " << *it << std::endl);


		for (auto o_it = order.begin();o_it != order.end(); ++o_it) {
			//			ROS_INFO_STREAM("debug : " << *o_it << std::endl);
			if (tracking_part == nullptr && it->type.compare(*o_it)) {
				tracking_part = new osrf_gear::Model();
				tracking_part->type = it->type;
				tracking_part->pose = it->pose;
				setTransform();
//				ROS_INFO_STREAM("tracking part id: " << tracking_part->type << std::endl);
			}

		}

		if (it->type.compare(tracking_part->type) == 0 && it-> pose.position.z > tracking_part->pose.position.z ) {
			setPose(it->pose, tracking_part->pose);
			setTransform();
//			ROS_INFO_STREAM("Tracking type: " << tracking_part->type << std::endl);
//			ROS_INFO_STREAM("Tracking pose: " << tracking_part->pose << std::endl);
		}
	}
}


bool AriacSensorManager::isObjectDetected() {
	return object_detected;
}



void AriacSensorManager::breakBeamCallback(const osrf_gear::Proximity::ConstPtr & msg) {

	if (msg->object_detected) {  // If there is an object in proximity.
		ROS_INFO("Break beam triggered.");
		object_detected = true;
	}
	else{
		object_detected = false;
	}
}





