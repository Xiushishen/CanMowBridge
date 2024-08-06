#include "imu_convert.hpp"

using namespace std;

imuConvert::imuConvert() : nh_("~") {
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (0 > sockfd) {
        perror("socket error");
        exit(EXIT_FAILURE);
    }
    strcpy(ifr.ifr_name, "can0");
    ioctl(sockfd, SIOCGIFINDEX, &ifr);
    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = ifr.ifr_ifindex;
    if (0 > bind(sockfd, (struct sockaddr *)&can_addr, sizeof(can_addr))) {
        perror("bind error");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    pub_imu_ = nh_.advertise<sensor_msgs::Imu>("/mower_imu", 100);

    receive_imu_thread = std::thread(&imuConvert::receiveImuData, this);
}

imuConvert::~imuConvert() {
    cout << "imu converter node is shutting down." << endl;
    if (receive_imu_thread.joinable()) {
        receive_imu_thread.join();
    }
    close(sockfd);
    exit(EXIT_SUCCESS);
}

void imuConvert::receiveImuData() {
    while (ros::ok()) {
        if (0 > read(sockfd, &frame, sizeof(struct can_frame))) {
            perror("read error");
            break;
        }
        if (frame.can_id & CAN_ERR_FLAG) {
            cout << "Error frame" << endl;
            break;
        }
        if (frame.can_id & CAN_RTR_FLAG) {
            cout << "Remote request" << endl;
            continue;
        }
        received_new_can = true;
        if (received_new_can && frame.can_id == 0x101) {
            // TODO: write codes to process IMU information from CANbus
            cout << "Received CAN messages from AC F4-A 80V 300A motor controller" << endl;
            received_new_can = false;
        }
    }
}
