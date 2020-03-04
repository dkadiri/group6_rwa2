/*
 * Proximity.h
 *
 *  Created on: Mar 3, 2020
 *      Author: dinesh
 */

#include <sensor_msgs/Range.h>

class ProximitySensor {
public:
	ProximitySensor();
	~ProximitySensor();
	void proximityCallBack(const sensor_msgs::Range::ConstPtr & msg);
};
