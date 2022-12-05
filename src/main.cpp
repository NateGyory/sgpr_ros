#include <iostream>
#include "ros/ros.h"

int main(int argc, char **argv)
{
    std::cout << "hello world" << std::endl;
    ros::init(argc, argv, "sgpr_ros_node");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 1;
}
