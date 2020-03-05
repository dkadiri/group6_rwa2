#include <../include/group6_rwa2/orders.h>
#include <ros/ros.h>


Orders::Orders() {
}

Orders::~Orders() {

}


void Orders::orderCallback(const osrf_gear::Order::ConstPtr & order_msg) {
	ROS_INFO_STREAM("Received order:\n" << *order_msg);
	received_orders_.push_back(*order_msg);
}

void Orders::readOrder() {


}

std::vector<osrf_gear::Order> Orders::getOrder(){
    return received_orders_;
}



