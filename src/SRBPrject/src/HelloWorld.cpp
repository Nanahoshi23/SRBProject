// 2021 3/17 Nanahoshi

#include <ros/ros.h>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "hello_world") ;
    ros::NodeHandle nh ;

    ros::Rate rate(5) ;
    while(ros::ok()) {
        ROS_INFO_STREAM("Hello World") ;
        rate.sleep() ;
    }

    return(0) ;
}
