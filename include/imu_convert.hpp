#ifndef CAN_H_CONVERTER_
#define CAN_H_CONVERTER_

#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <iostream>
#include <cmath>
#include <thread>
#include <string>

#include "geometry_msgs/Twist.h"
#include <geometry_msgs/Pose.h>
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>


class imuConvert {
public:
    imuConvert();
    ~imuConvert();

    void receiveImuData();

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_imu_;

    struct ifreq ifr;
    struct sockaddr_can can_addr;
    struct can_frame frame;

    int ret;
    bool received_new_can {false};
    std::thread receive_imu_thread;
    int sockfd;

};

#endif // CAN_H_CONVERTER_