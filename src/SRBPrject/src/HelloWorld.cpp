#include "ros/ros.h"
#include <string>

int main() {
    ros::init(argc, argv, "hello_world") ;
    ros::NodeHandle nh ;

    ros::Rate rate(5) ;
    while(ros::ok()) {
        ROS_INFO_STREAM("Hello World") ;
        rate.sleep ;
    }

    return(0) ;
}


