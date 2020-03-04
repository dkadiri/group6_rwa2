/*
 * LogicalCamera.h
 *
 *  Created on: Mar 3, 2020
 *      Author: preyash
 */

#include <osrf_gear/LogicalCameraImage.h>

class LogicalCamera {
public:
	LogicalCamera();
	virtual ~LogicalCamera();
	void logical_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);

}
