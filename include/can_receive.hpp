#ifndef CAN_H_RECEIVER_
#define CAN_H_RECEIVER_

#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <array>
#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>
#include <string>

#include "geometry_msgs/Twist.h"
#include <geometry_msgs/Pose.h>
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define PI 3.1415926

class canReceive {
public:
    canReceive();
    explicit canReceive(std::string &config_path);
    ~canReceive();

    void receiveCANData();
    void stop();

    /**
     * test functions
     */

private:
    void initMarker() {
        marker_.header.frame_id = "odom";
        marker_.header.stamp = ros::Time::now();
        marker_.ns = "optimus";
        marker_.id = 0;
        marker_.type = visualization_msgs::Marker::POINTS;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.pose.orientation.w = 1.0;
        marker_.scale.x = 0.05;
        marker_.scale.y = 0.05;
        marker_.scale.z = 0.05;
        marker_.color.r = 0.0;
        marker_.color.g = 0.0;
        marker_.color.b = 1.0;
        marker_.color.a = 1.0;
    }

    int16_t decodeRPM(const uint8_t &lowbyte, const uint8_t &highbyte);
    float decodeRPMToSpeed(const uint8_t &lowbyte, const uint8_t &highbyte);
    float rpmToSpeed(const int16_t rpm);
    // float rpmToSpeed(const int16_t rpm);

    ros::NodeHandle nh_;
    ros::Time time_last_integration_left = ros::Time(0);
    ros::Time time_last_integration_right = ros::Time(0);
    ros::Duration dt_left_;
    ros::Duration dt_right_;
    ros::Publisher pub_odom_;
    ros::Publisher pub_marker_;

    struct ifreq ifr;
    struct sockaddr_can can_addr;
    struct can_frame frame;

    bool confirmed_left = true;
    bool confirmed_right = true;
    std::string config_file_path_;
    float distance_left_accumulate = 0;
    float distance_right_accumulate = 0;
    float ds_left = 0;
    float ds_right = 0;
    int sockfd;
    int ret;
    double position_x;
    double position_y;
    float wheel_base;
    float wheel_diameter;
    float rpm_motor_to_wheel;
    float total_distance = 0;
    std::thread receive_thread;
    float yaw;
    float prev_left_speed = 0;
    float prev_right_speed = 0;
    bool received_new_can = false;
    bool publish_tf = false;

    std::atomic<bool> keep_running_ {true};

    int16_t prev_left_rpm = 0;
    int16_t prev_right_rpm = 0;

    visualization_msgs::Marker marker_;
    tf::TransformBroadcaster tf_broadcaster_;
    
};

#endif // CAN_H_RECEIVER_
