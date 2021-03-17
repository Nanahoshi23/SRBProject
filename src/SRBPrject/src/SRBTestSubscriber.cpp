#include <ros/ros.h>
#include <std_msgs/String.h>

void msgCallBack(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str()) ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SRBTestSubscriber") ;
    ros::NodeHandle nh ;

    ros::Subscriber srb_test_sub = nh.subscribe("SRBTestSubscriber", 1000, msgCallBack) ;
    ros::spin() ;
}
