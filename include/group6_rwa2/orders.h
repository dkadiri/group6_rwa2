#include <ros/ros.h>
#include <osrf_gear/Order.h>


class Orders{
private :
	std::vector<osrf_gear::Order> received_orders_;

public:
	Orders();
	~Orders();
    void order_callback(const osrf_gear::Order::ConstPtr & order_msg);
private:
};
