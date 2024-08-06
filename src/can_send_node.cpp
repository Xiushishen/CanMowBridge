#include "can_send.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_send_node");
    canSend can_send;
    ros::spin();
    return 0;
}
