#include <sensor_msgs/LaserScan.h>


class LaserProfiler {

public:
    LaserProfiler();
    ~LaserProfiler();

    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & msg);

};

