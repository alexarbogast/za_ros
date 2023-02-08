/* This node used to provide an external hardware interface to za_control */
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "za_control_node");

    ros::NodeHandle public_node_handle;
    ros::NodeHandle node_handle("~");
}