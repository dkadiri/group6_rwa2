//
// Created by zeid on 3/1/20.
//

#include "ariac_part_manager.h"

AriacPartManager::AriacPartManager():part_type_{}, part_frame_{}{};
AriacPartManager::~AriacPartManager(){};

void AriacPartManager::set_part_type(std::string part_type){
    part_type_ = part_type;
};

void AriacPartManager::set_part_frame(int part_frame){
    part_frame_ = part_frame;
};

void AriacPartManager::set_part_pose(geometry_msgs::Pose part_pose){
    part_pose_ = part_pose;
};

const std::string get_part_type(){
    return part_type_;
};
const int get_part_frame(){
    return part_frame_;
};
const geometry_msgs::Pose get_part_pose(){
    return part_pose_ ;
};