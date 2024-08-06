#include "can_receive.hpp"

using namespace std;

canReceive::canReceive(string &config_path)
        : config_file_path_(config_path), nh_("~") {
    auto yaml = YAML::LoadFile(config_file_path_);
    wheel_base = yaml["wheel_base"].as<float>();
    wheel_diameter = yaml["wheel_diameter"].as<float>();
    publish_tf = yaml["publish_tf"].as<bool>();
    rpm_motor_to_wheel = yaml["rpm_motor_to_wheel"].as<float>();
}

canReceive::canReceive() : nh_("~") {
    nh_.param<float>("wheel_base", wheel_base, 1.04);
    nh_.param<float>("wheel_diameter", wheel_diameter, 0.6096);
    nh_.param<bool>("publish_tf", publish_tf, false);

    /**
     * find the place where we run the node
     */
    // std::array<char, 1024> cwd{};
    // if (getcwd(cwd.data(), cwd.size()) != nullptr) {
    //     std::cout << "Current working directory: " << cwd.data() << std::endl;
    // } else {
    //     perror("getcwd() error");
    // }

    config_file_path_ = "../robotics/optimus_launch_ws/src/can_ros1/config/default.yaml";
    auto yaml = YAML::LoadFile(config_file_path_);
    
    rpm_motor_to_wheel = yaml["rpm_motor_to_wheel"].as<float>();

    position_x = 0.0;
    position_y = 0.0;
    yaw = 0.0;

    ifr = {0};
    can_addr = {0};
    frame = {0};

    initMarker();

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

    /* 设置过滤规则 */ 
    // setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0); 

    // time_last_integration = ros::Time::now();
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/mower_odom", 10);
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/wheel_encoder_marker", 10);

    // receiveCanData();
    receive_thread = thread(&canReceive::receiveCANData, this);
}

void canReceive::stop() {
    keep_running_ = false;
}

float canReceive::rpmToSpeed(const int16_t rpm) {
    float rpm_wheel = static_cast<float>(rpm) / rpm_motor_to_wheel;
    float vel_wheel = (rpm_wheel / 60.0) * (M_PI * wheel_diameter);
    vel_wheel = abs(vel_wheel) <= 0.01 ? 0 : vel_wheel; // to filter noise speed data
    return vel_wheel;
}

int16_t canReceive::decodeRPM(const uint8_t &lowbyte, const uint8_t &highbyte) {
    uint8_t lower = lowbyte;
    uint8_t higher = highbyte;
    int16_t rpm_motor = 0;
    rpm_motor = static_cast<int16_t>((highbyte << 8) | lowbyte);
    rpm_motor = abs(rpm_motor) <= 5 ? 0 : rpm_motor;
    return rpm_motor;
}

float canReceive::decodeRPMToSpeed(const uint8_t &lowbyte, const uint8_t &highbyte) {
    uint8_t lower = lowbyte;
    uint8_t higher = highbyte;
    int16_t rpm_motor = 0;
    rpm_motor = static_cast<int16_t>((highbyte << 8) | lowbyte);
    return rpmToSpeed(rpm_motor);
}

void canReceive::receiveCANData() {
    int count = 0;
    while (ros::ok()&& keep_running_) {
        if (0 > read(sockfd, &frame, sizeof(struct can_frame))) {
            perror("read error");
            break;
        }
        if (frame.can_id & CAN_ERR_FLAG) {
            printf("Error frame!\n");
            break;
        }
        /* 校验帧格式 */
        // if (frame.can_id & CAN_EFF_FLAG)
        //     ROS_INFO("extended frame <0x%08x> ", frame.can_id & CAN_EFF_MASK);
        // else
        //     ROS_INFO("standard frame <0x%03x> ", frame.can_id & CAN_SFF_MASK);

        if (frame.can_id & CAN_RTR_FLAG) {
            printf("remote request\n");
            continue;
        }
        // cout << "Length of data: " << frame.can_dlc << endl;
        received_new_can = true;
        // float dt = 0.032;
        if (received_new_can && (frame.can_id == 0x503 || frame.can_id == 0x502)) {
            received_new_can = false;
            float current_left_speed = prev_left_speed;
            float current_right_speed = prev_right_speed;
            auto current_left_rpm = prev_left_rpm;
            auto current_right_rpm = prev_right_rpm;
            if (frame.can_id == 0x503 && frame.can_dlc == 8) {
                if (confirmed_left) {
                    ros::Time time_now_left = ros::Time::now();
                    dt_left_ = time_now_left - time_last_integration_left;
                    time_last_integration_left = time_now_left;
                    current_left_speed = decodeRPMToSpeed(frame.data[0], frame.data[1]);
                    prev_left_speed = current_left_speed;
                    current_left_rpm = decodeRPM(frame.data[0], frame.data[1]);
                    ds_left = current_left_speed * dt_left_.toSec();
                    // ds_left = current_left_speed * dt;
                    distance_left_accumulate += ds_left;
                    cout << "Left Wheel Distance: " << distance_left_accumulate << endl;
                    cout << "Left-Wheel Timer: "   << dt_left_ << endl; 
                    cout << "Left-Wheel Speed: "    << current_left_speed << " /" << " Left-Wheel RPM: " << current_left_rpm << endl; 
                    prev_left_rpm = current_left_rpm;
                }
                confirmed_left = false;
            }
            if (frame.can_id == 0x502 && frame.can_dlc == 8) {
                if (confirmed_right) {
                    ros::Time time_now_right = ros::Time::now();
                    dt_right_ = time_now_right - time_last_integration_right;
                    time_last_integration_right = time_now_right;
                    current_right_speed = decodeRPMToSpeed(frame.data[0], frame.data[1]);
                    prev_right_speed = current_right_speed;
                    current_right_rpm = decodeRPM(frame.data[0], frame.data[1]);
                    ds_right = current_right_speed * dt_right_.toSec();
                    // ds_right = current_right_speed * dt;
                    distance_right_accumulate += ds_right;
                    cout << "Right-Wheel Distance: " << distance_right_accumulate << endl;
                    cout << "Right-Wheel Timer: "    << dt_right_ << endl; 
                    cout << "Right-Wheel Speed: "    << current_right_speed << " /" << " Right-Wheel RPM: " << current_right_rpm << endl; 
                    prev_right_rpm = current_right_rpm;
                }
                confirmed_right = false;
            }
            if (!confirmed_left && !confirmed_right) {
                confirmed_left = true;
                confirmed_right = true;
                nav_msgs::Odometry odom;
                float dt = (dt_left_.toSec() + dt_right_.toSec()) / 2.0f;
                float ds = (ds_left + ds_right) / 2.0f;
                float dtheta = (ds_right - ds_left) / (wheel_base);
                position_x = position_x + ds * cos(yaw + dtheta / 2.0f);
                position_y = position_y + ds * sin(yaw + dtheta / 2.0f);
                total_distance += ds;
                cout << "Distance: " << total_distance << endl;
                yaw = fmod(yaw + dtheta, 2 * M_PI);
                if (yaw > M_PI) {
                    yaw -= 2 * M_PI;
                } else if (yaw < -M_PI) {
                    yaw += 2 * M_PI;
                }
                float speed = ds / dt;
                odom.twist.twist.linear.x = speed; 
                odom.twist.twist.angular.z = dtheta / dt;
                odom.pose.pose.position.x = position_x;
                odom.pose.pose.position.y = position_y;
                odom.pose.covariance[0] = 1e-8; // x 位置
                odom.pose.covariance[7] = 1e-8; // y 位置
                odom.pose.covariance[14] = 0; // z 位置
                odom.pose.covariance[21] = 1e-7; // roll 方向
                odom.pose.covariance[28] = 1e-7; // pitch 方向
                odom.pose.covariance[35] = 1e-6; // yaw 方向

                odom.twist.covariance[0] = 1e-6; // dx 线速度
                odom.twist.covariance[7] = 1e-7; // dy 线速度
                odom.twist.covariance[14] = 1e-7; // dz 线速度
                odom.twist.covariance[21] = 1e-7; // droll 角速度
                odom.twist.covariance[28] = 1e-7; // dpitch 角速度
                odom.twist.covariance[35] = 1e-6; // dyaw 角速度

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                odom.pose.pose.orientation.x = q.x();
                odom.pose.pose.orientation.y = q.y();
                odom.pose.pose.orientation.z = q.z();
                odom.pose.pose.orientation.w = q.w();
            
                if (count % 10 == 0) {
                    geometry_msgs::Point p;
                    p.x = position_x;
                    p.y = position_y;
                    marker_.points.push_back(p);
                    marker_.header.stamp = ros::Time::now();
                    marker_.lifetime = ros::Duration(0);
                    pub_marker_.publish(marker_);
                }
                count++;

                odom.header.frame_id = "odom";
                odom.child_frame_id = "base_link";
                odom.header.stamp = ros::Time::now();
                pub_odom_.publish(odom);
                ROS_INFO("Odom from ZTR is being pulished!");

                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
                // cout << "odom_quat:" << odom_quat << endl << endl;
                cout << "Odometry Quaternion:" << endl;
                cout << odom_quat << endl;
            
                if (publish_tf) { 
                    geometry_msgs::TransformStamped tf_msg;
                    tf_msg.header.stamp = ros::Time::now();
                    tf_msg.header.frame_id = "odom";
                    tf_msg.child_frame_id = "base_link";
                    tf_msg.transform.translation.x = position_x;
                    tf_msg.transform.translation.y = position_y;
                    tf_msg.transform.translation.z = 0.0;
                    tf_msg.transform.rotation = odom_quat;
                    tf_broadcaster_.sendTransform(tf_msg);
                }
            }
        }
    }
}

canReceive::~canReceive() {
    stop();
    if (receive_thread.joinable())
        receive_thread.join();
    close(sockfd);
    exit(EXIT_SUCCESS);
}

