//
// Created by zeid on 2/27/20.
//
#include "../include/group6_rwa2/sensor.h"

AriacSensorManager::AriacSensorManager()
{
	ROS_INFO_STREAM(">>>>> Subscribing to logical sensors");

    camera_4_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_4", 10,
                                                &AriacSensorManager::logicalCamera4Callback, this);

}

AriacSensorManager::~AriacSensorManager() {}

void AriacSensorManager::set_pose(geometry_msgs::TransformStamped &transformStamped, const geometry_msgs::Pose pose){
    transformStamped.transform.translation.x = pose.position.x;
    transformStamped.transform.translation.y = pose.position.y;
    transformStamped.transform.translation.z = pose.position.z;
    transformStamped.transform.rotation.x = pose.orientation.x;
    transformStamped.transform.rotation.y = pose.orientation.y;
    transformStamped.transform.rotation.z = pose.orientation.z;
    transformStamped.transform.rotation.w = pose.orientation.w;
}


void AriacSensorManager::logicalCamera4Callback(
        const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    // order type same as camera type
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera: '" << image_msg->models.size() << "' objects.");
    auto sensor_pose = image_msg->pose;
    auto current_time = ros::Time::now();
    tf2_ros::TransformListener tfListener(tfBuffer);
    static tf2_ros::TransformBroadcaster br_w_s;
    static tf2_ros::TransformBroadcaster br_s_c;

    transformStamped1.header.stamp = current_time;
    transformStamped1.header.frame_id = "world";
    transformStamped1.child_frame_id = "logical_sensor";

    transformStamped2.header.stamp = current_time;
    transformStamped2.header.frame_id = "logical_sensor";
    transformStamped2.child_frame_id = "logical_sensor_child";

    set_pose(transformStamped1, sensor_pose);
    br_w_s.sendTransform(transformStamped1);
    ros::Duration(0.2).sleep();
        if(!image_msg->models.empty()){
            for(auto it =image_msg->models.begin(); it!=image_msg->models.end();++it) {
                if(orderManger.getProductType().front() == it->type){
                    set_pose(transformStamped2, it->pose);
                    br_s_c.sendTransform(transformStamped2);
                    ros::Duration(0.2).sleep();
                    auto type = it->type;
                    try{
                        transformStamped3 = tfBuffer.lookupTransform("world", "logical_sensor_child",
                                                                     ros::Time(0));
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN("exception");
                        ROS_WARN("%s",ex.what());
                        ros::Duration(1.0).sleep();
                    }
                    ROS_INFO("%s in world frame: [%f,%f,%f] [%f,%f,%f,%f]",type.c_str(),transformStamped3.transform.translation.x,
                             transformStamped3.transform.translation.y,
                             transformStamped3.transform.translation.z,
                             transformStamped3.transform.rotation.x,
                             transformStamped3.transform.rotation.y,
                             transformStamped3.transform.rotation.z,
                             transformStamped3.transform.rotation.w);
                }

            }


    }

}








void AriacSensorManager::breakBeamCallback(const osrf_gear::Proximity::ConstPtr & msg) {

//	if (msg->object_detected) {  // If there is an object in proximity.
//		ROS_INFO("Break beam triggered.");
//	}
}





