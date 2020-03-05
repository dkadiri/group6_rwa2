#include <osrf_gear/LogicalCameraImage.h>

class LogicalCamera {
public:
    LogicalCamera();
    ~LogicalCamera();
    void logicalCameraCallBack(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
};