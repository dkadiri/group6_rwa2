
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

class AriacSensorManager {
public:
    AriacSensorManager();
    ~AriacSensorManager();
    void logicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void logicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void logicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void logicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void breakBeamCallback(const osrf_gear::Proximity::ConstPtr &);


    geometry_msgs::Pose GetPartPose(const std::string& src_frame,
                                    const std::string& target_frame);
    std::map<std::string, std::vector<std::string>> get_product_frame_list(){
        return product_frame_list_;
    }
    //void ScanParts(int cam_number);
    void BuildProductFrames(int);

private:
    ros::NodeHandle sensor_nh_;
    ros::Subscriber camera_1_subscriber_;
    ros::Subscriber camera_2_subscriber_;
    ros::Subscriber camera_3_subscriber_;
    ros::Subscriber camera_4_subscriber_;


    tf::TransformListener camera_tf_listener_;
    tf::StampedTransform camera_tf_transform_;

    osrf_gear::LogicalCameraImage current_parts_1_;
    osrf_gear::LogicalCameraImage current_parts_2_;
    osrf_gear::LogicalCameraImage current_parts_3_;
    osrf_gear::LogicalCameraImage current_parts_4_;
    std::map<std::string, std::vector<geometry_msgs::Pose>> part_list_;
    std::vector<AriacPartManager> camera1_part_list,camera2_part_list,camera3_part_list;

    //std::map<std::string, std::list<std::string>> parts_list_;
    std::map<std::string, std::vector<std::string>> product_frame_list_;

    bool init_, cam_1_, cam_2_,cam_3_;
    int camera1_frame_counter_, camera2_frame_counter_, camera3_frame_counter_;
};

#endif //GROUP6_RWA2_SENSOR
