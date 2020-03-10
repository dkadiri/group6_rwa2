
#ifndef GROUP6_RWA2_SENSOR
#define GROUP6_RWA2_SENSOR
#include <list>
#include <map>
#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Proximity.h>

#include <ariac_part_manager.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "order_manager.h"


class AriacOrderManger;

class AriacSensorManager {
public:
    AriacSensorManager();
    ~AriacSensorManager();

    void set_pose(geometry_msgs::TransformStamped &transformStamped, const geometry_msgs::Pose pose);
    void logicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void breakBeamCallback(const osrf_gear::Proximity::ConstPtr &);
    bool isObjectDetected();


private:
    ros::NodeHandle sensor_nh_;
    AriacOrderManager orderManager;

    ros::Subscriber camera_4_subscriber_;
    ros::Subscriber breakbeam_subscriber;
    geometry_msgs::TransformStamped transformStamped1;
    geometry_msgs::TransformStamped transformStamped2;
    geometry_msgs::TransformStamped transformStamped3;
    tf2_ros::Buffer tfBuffer;



    tf::TransformListener camera_tf_listener_;
    tf::StampedTransform camera_tf_transform_;

    osrf_gear::LogicalCameraImage current_parts_4_;
    bool object_detected = false;
    std::map<std::string, std::vector<geometry_msgs::Pose>> part_list_;

    std::map<std::string, std::vector<std::string>> product_frame_list_;

};

#endif //GROUP6_RWA2_SENSOR
