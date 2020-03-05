#include <ros/ros.h>
#include <osrf_gear/Order.h>


class Orders{
private :
	std::vector<osrf_gear::Order> received_orders_;

public:
	Orders();
	~Orders();
    void orderCallback(const osrf_gear::Order::ConstPtr & order_msg);
    void readOrder();
    std::vector<osrf_gear::Order> getOrder();
};
