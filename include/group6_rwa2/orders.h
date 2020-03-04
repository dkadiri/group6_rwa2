#include <ros/ros.h>
#include <osrf_gear/Order.h>


class Orders{
private :
	std::vector<osrf_gear::Order> received_orders_;

public:
	Orders();
	~Orders();
};
