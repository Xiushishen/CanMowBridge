#include "can_send.hpp"

#include <poll.h>

using namespace std;

canSend::canSend() : nh_("~") {
    nh_.param<float>("wheel_base", wheel_base, 1.04);
    nh_.param<string>("twistTopic", twist_topic, "/cmd_vel");
    nh_.param<float>("wheel_diameter", wheel_diameter, 0.6096);

    ifr = {0};
    can_addr = {0};
    frame100 = {0};
    
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (0 > sockfd) {
        perror("socket error");
        exit(EXIT_FAILURE);
    }
    strcpy(ifr.ifr_name, "can0");
    ioctl(sockfd, SIOCGIFINDEX, &ifr);
    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = ifr.ifr_ifindex;

    ret = bind(sockfd, (struct sockaddr *)&can_addr, sizeof(can_addr)); 
    if (0 > ret) {
        perror("bind error");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    /* 设置过滤规则：不接受任何报文、仅发送数据 */
    setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    sendConfirmationCAN();
    startListenerThread();

    sub_ = nh_.subscribe(twist_topic, 100, &canSend::velocityCallback, this);
}

void canSend::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    publishCANCmd(msg->linear.x, msg->angular.z);
    // publishCmd(msg->linear.x, msg->angular.z);
}

float canSend::speedToRPM(const float vel) {
    auto rpm_wheel = (abs(vel) / (PI * wheel_diameter)) * SEC_IN_MIN;
    auto rmp_motor = float(RPM_MOTOR_TO_WHEEL * rpm_wheel);
    return rmp_motor;
}

void canSend::sendConfirmationCAN() {
    ROS_INFO("Orin is connected to ETO port without errors.");

    frame100.can_id = 0x100;
    frame100.can_dlc = 8;
    frame100.data[0] = uint8_t(0);
    frame100.data[1] = uint8_t(0);
    frame100.data[2] = uint8_t(0); 
    frame100.data[3] = uint8_t(0);
    frame100.data[4] = uint8_t(0);
    frame100.data[5] = uint8_t(0);
    frame100.data[6] = uint8_t(0);
    frame100.data[7] = uint8_t(1);

    ret = write(sockfd, &frame100, sizeof(frame100));
    if (sizeof(frame100) != ret) {
        perror("write error");
    }
    ROS_INFO("Confirmation CAN message was sent to the mower.");
}

void canSend::startListenerThread() {
    listenThread = thread(&canSend::listenForCANMessages, this);
}

void canSend::listenForCANMessages() {
    struct pollfd fds[1];
    fds[0].fd = sockfd;
    fds[0].events = POLLIN;
    int nbytes;
    while (runListener) {
        int ret = poll(fds, 1, 1000);  // 等待1000毫秒
        if (ret > 0 && (fds[0].revents & POLLIN)) {
            nbytes = read(sockfd, &frame101, sizeof(frame101));
            if (nbytes > 0 && frame101.can_id == 0x200) {
                ROS_INFO("Received confirmation CAN message");
                flag = true;
            }
        }
        if (nbytes < 0) {
            perror("CAN read error");
        }
    }
}

void canSend::stopListener() {
    runListener = false;
}

void canSend::publishCmd(double linear_vel, double angular_vel) {
    auto speed_mps = max(min(linear_vel, MAX_LINEAR_SPEED_ON_CAN), -MAX_LINEAR_SPEED_ON_CAN);
    auto angular_rad_per_sec = max(min(angular_vel, MAX_ANGULAR_SPEED_ON_CAN), -MAX_ANGULAR_SPEED_ON_CAN);

    ROS_INFO("Linear Speed: %f", linear_vel);
    ROS_INFO("Angular Speed: %f", angular_vel);

    flag_L = linear_vel >= 0 ? 1 : 0;
    flag_A = angular_vel >= 0 ? 1 : 0;

    frame100.can_id = 0x100;
    frame100.can_dlc = 8;

    if (flag) {
        frame100.data[0] = uint8_t(16);
        frame100.data[1] = uint8_t(linear_vel * 100);
        frame100.data[2] = uint8_t(angular_vel * 100); 
        frame100.data[3] = uint8_t(flag_L);
        frame100.data[4] = uint8_t(flag_A);
        frame100.data[5] = uint8_t(0);
        frame100.data[6] = uint8_t(0);
        frame100.data[7] = uint8_t(2);
    } else {
        frame100.data[0] = uint8_t(16);
        frame100.data[1] = uint8_t(linear_vel * 100);
        frame100.data[2] = uint8_t(angular_vel * 100); 
        frame100.data[3] = uint8_t(flag_L);
        frame100.data[4] = uint8_t(flag_A);
        frame100.data[5] = uint8_t(0);
        frame100.data[6] = uint8_t(0);
        frame100.data[7] = uint8_t(1);   
    }
    
    ret = write(sockfd, &frame100, sizeof(frame100)); // send data
    if (sizeof(frame100) != ret) {
        perror("write error");
    }
    usleep(200);
    cout << "Size of Frame100: " << ret << endl;
}

// send control command to chassis
void canSend::publishCANCmd(double linear_vel, double angular_vel) {
    auto speed_mps = max(min(linear_vel, MAX_LINEAR_SPEED_ON_CAN), -MAX_LINEAR_SPEED_ON_CAN);
    auto angular_rad_per_sec = max(min(angular_vel, MAX_ANGULAR_SPEED_ON_CAN), -MAX_ANGULAR_SPEED_ON_CAN);
    auto left_mps = speed_mps - (wheel_base / 2 * angular_rad_per_sec);
    auto right_mps = speed_mps + (wheel_base / 2 * angular_rad_per_sec);

    ROS_INFO("Speed of left wheel: %f", left_mps);
    ROS_INFO("Speed in the middle: %f", speed_mps);
    ROS_INFO("Speed of right wheel: %f", right_mps);

    flagLeft = left_mps < 0 ? 0 : 1;
    flagRight = right_mps < 0 ? 0 : 1;

    int16_t left_rpm = int16_t(speedToRPM(left_mps)*RPM_TO_CAN_SCALE_FACTOR_10MPH_MAX);
    int16_t right_rpm = int16_t(speedToRPM(right_mps)*RPM_TO_CAN_SCALE_FACTOR_10MPH_MAX);

    // ROS_INFO("rpm of left wheel: %u", left_rpm);
    // ROS_INFO("rpm of right wheel: %u", right_rpm);

    cout << "left rpm: " << flagLeft << " " << left_rpm << endl;
    cout << "right rpm: " << flagRight << " " << right_rpm << endl;

    frame100.can_id = 0x100;
    frame100.can_dlc = 8;
    
    /**
    * @brief CAN message logic
    * frame100.data[0] : lower position of right motor rpm
    * frame100.data[1] : higher position of right motor rpm
    * frame100.data[2] : lower position of left motor rpm
    * frame100.data[3] : higher position of left mptpr rpm
    * frame100.data[4] : direction of right motor (0: backwards; 1: forwards)
    * frame100.data[5] : direction of left motor (0: backwards; 1: forwards)
    * frame100.data[6] : TODO
    * frame100.data[7] : control logic (1: Orin is connected; 2: Mower is ready to receive;)
    */
   
    if (flag) {
        frame100.data[0] = uint8_t(right_rpm & 0xFF);
        frame100.data[1] = uint8_t((right_rpm >> 8) & 0xFF); 
        frame100.data[2] = uint8_t(left_rpm & 0xFF); 
        frame100.data[3] = uint8_t((left_rpm >> 8) & 0xFF);
        frame100.data[4] = uint8_t(flagRight);
        frame100.data[5] = uint8_t(flagLeft);
        frame100.data[6] = uint8_t(0);
        frame100.data[7] = uint8_t(2);
    } else {
        frame100.data[0] = uint8_t(right_rpm & 0xFF);
        frame100.data[1] = uint8_t((right_rpm >> 8) & 0xFF); 
        frame100.data[2] = uint8_t(left_rpm & 0xFF); 
        frame100.data[3] = uint8_t((left_rpm >> 8) & 0xFF);
        frame100.data[4] = uint8_t(flagRight);
        frame100.data[5] = uint8_t(flagLeft);
        frame100.data[6] = uint8_t(0);
        frame100.data[7] = uint8_t(1);
    }
    
    ret = write(sockfd, &frame100, sizeof(frame100)); // send data
    if (sizeof(frame100) != ret) {
        perror("write error");
    }
    usleep(200);
    cout << "Size of Frame100: " << ret << endl;
}

canSend::~canSend() {
    for (int i = 0; i < 8; ++i)
        frame100.data[i] = uint8_t(0);
    ret = write(sockfd, &frame100, sizeof(frame100));
    stopListener();
    if (listenThread.joinable())
        listenThread.join();
    close(sockfd);
    exit(EXIT_SUCCESS);
}
