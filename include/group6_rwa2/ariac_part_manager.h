//
// Created by zeid on 3/1/20.
//

#pragma once

#include <string>
#include <geometry_msgs/Pose.h>

class AriacPartManager {
public:
    AriacPartManager();
    ~AriacPartManager();

    void set_part_type(std::string part_type);

    void set_part_frame(int part_frame);

    void set_part_pose(geometry_msgs::Pose part_pose);

    const std::string get_part_type();

    const int get_part_frame();

    const geometry_msgs::Pose get_part_pose();

    private:
    std::string part_type_;
    geometry_msgs::Pose part_pose_;
    int part_frame_;
};

