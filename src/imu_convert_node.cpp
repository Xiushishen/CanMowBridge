#include "imu_convert.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_convert_node");
    imuConvert imu_convert;
    ros::spin();
    return 0;
}
