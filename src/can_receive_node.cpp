#include "can_receive.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_receive_node");
    canReceive can_receive;
    ros::spin();
    return 0;
}
