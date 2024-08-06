#ifndef CAN_H_SENDER_
#define CAN_H_SENDER_

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

#include <atomic>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#define RPM_MOTOR_TO_WHEEL 21.1
#define SEC_IN_MIN 60
#define EPS 0.0001
#define PI 3.1415926
#define M_PER_SEC_TO_MPH 2.23694
#define MAX_LINEAR_SPEED_ON_CAN 4.0 // 2.0
#define MAX_ANGULAR_SPEED_ON_CAN 3.1415
#define RPM_TO_CAN_SCALE_FACTOR_10MPH_MAX 9.9223333333 //32767/3000 32767/4800

class canSend {
public:
    canSend();
    explicit canSend(std::string &config_path);
    ~canSend();

    void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void publishCmd(double linear_vel, double angular_vel);
    void publishCANCmd(double linear_vel, double angular_vel);
    float speedToRPM(const float vel);
    bool confirmationRespondCAN();
    void listenForCANMessages();
    void startListenerThread();
    void stopListener();

private:
    void sendConfirmationCAN();

    ros::Subscriber sub_;
    ros::NodeHandle nh_;

    struct ifreq ifr;
    struct sockaddr_can can_addr;
    struct can_frame frame100;
    struct can_frame frame101;
    
    int sockfd = -1;
    int ret;
    int flag_L = 0, flag_A = 0;
    int flagLeft = 0, flagRight = 0;
    std::string twist_topic;
    float wheel_base;
    float wheel_diameter;

    std::thread listenThread;
    std::atomic<bool> flag {false};
    std::atomic<bool> runListener {true};
    
};

#endif // CAN_H_SENDER_
