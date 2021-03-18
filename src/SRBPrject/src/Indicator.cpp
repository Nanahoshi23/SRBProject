#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "slab_srb_msgs/diff_driver_cmd.h"
#include "slab_srb_msgs/diff_driver_fb.h"
// #include "Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <jsk_rviz_plugins/OverlayText.h>
#include "sensor_msgs/NavSatFix.h"


class VisualIndicatorPublisher
{
    private :
        // インジケータ用変数

        std_msgs::Float32 m_cmd_velocity ;
        std_msgs::Float32 m_cmd_angler_velocity ;
        std_msgs::Float32 m_raw_velocity ;
        std_msgs::Float32 m_raw_angler_velocity ;

        std_msgs::Float32 m_r_motor_cmd_rps ;
        std_msgs::Float32 m_l_motor_cmd_rps ;
        std_msgs::Float32 m_r_motor_raw_rps ;
        std_msgs::Float32 m_l_motor_raw_rps ;

        std_msgs::Float32 m_wheel_odom_x ;
        std_msgs::Float32 m_wheel_odom_y ;
        std_msgs::Float32 m_wheel_odom_th ;

        std_msgs::Float32 m_gnss_odom_x ;
        std_msgs::Float32 m_gnss_odom_y ;
        std_msgs::Float32 m_gnss_odom_th ;
        jsk_rviz_plugins::OverlayText m_gnss_data ;

        std_msgs::Float32 m_imu_data_x ; 
        std_msgs::Float32 m_imu_data_y ;
        std_msgs::Float32 m_imu_data_th ;

        std_msgs::Float32 m_imu_data_x_vel ; 
        std_msgs::Float32 m_imu_data_y_vel ;
        std_msgs::Float32 m_imu_data_th_vel ;

        std_msgs::Float32 m_fusion_odom_x ;
        std_msgs::Float32 m_fusion_odom_y ;
        std_msgs::Float32 m_fusion_odom_th ;

        std_msgs::Float32 m_robot_pose_x ;
        std_msgs::Float32 m_robot_pose_y ;
        std_msgs::Float32 m_robot_pose_th ;

        std_msgs::Float32 m_gazebo_truth_odom_x ;
        std_msgs::Float32 m_gazebo_truth_odom_y ;
        std_msgs::Float32 m_gazebo_truth_odom_th ;


        // TF
        tf::TransformListener m_tf_listener;
        tf::StampedTransform  m_trans_slam ;

        // インジケータ用変数のパブリッシャ
        ros::Publisher m_cmd_velocity_pub ;
        ros::Publisher m_cmd_angler_velocity_pub ;
        ros::Publisher m_raw_velocity_pub ;
        ros::Publisher m_raw_angler_velocity_pub ;

        ros::Publisher m_r_motor_cmd_rps_pub ;
        ros::Publisher m_l_motor_cmd_rps_pub ;
        ros::Publisher m_r_motor_raw_rps_pub ;
        ros::Publisher m_l_motor_raw_rps_pub ;

        ros::Publisher m_wheel_odom_x_pub ;
        ros::Publisher m_wheel_odom_y_pub ;
        ros::Publisher m_wheel_odom_th_pub ;

        ros::Publisher m_gnss_odom_x_pub ;
        ros::Publisher m_gnss_odom_y_pub ;
        ros::Publisher m_gnss_odom_th_pub ;

        ros::Publisher m_gnss_data_pub ;

        ros::Publisher m_imu_data_x_pub ;
        ros::Publisher m_imu_data_y_pub ;
        ros::Publisher m_imu_data_th_pub ;

        ros::Publisher m_imu_data_x_vel_pub ; 
        ros::Publisher m_imu_data_y_vel_pub  ;
        ros::Publisher m_imu_data_th_vel_pub ;

        ros::Publisher m_fusion_odom_x_pub ;
        ros::Publisher m_fusion_odom_y_pub ;
        ros::Publisher m_fusion_odom_th_pub ;

        ros::Publisher m_robot_pose_x_pub ;
        ros::Publisher m_robot_pose_y_pub ;
        ros::Publisher m_robot_pose_th_pub ;

        ros::Publisher m_gazebo_truth_odom_x_pub ;
        ros::Publisher m_gazebo_truth_odom_y_pub ;
        ros::Publisher m_gazebo_truth_odom_th_pub ;

        geometry_msgs::Twist       m_cmd_twist ;
        slab_srb_msgs::diff_driver_cmd m_cmd_val ;
        slab_srb_msgs::diff_driver_fb  m_raw_val ;
        nav_msgs::Odometry         m_wheel_odom ;
        nav_msgs::Odometry         m_gnss_odom ;
        sensor_msgs::Imu           m_imu_data ;
        nav_msgs::Odometry         m_fusion_odom ;
        nav_msgs::Odometry         m_gazebo_truth_odom ;

        ros::Subscriber m_cmd_twist_sub ;
        ros::Subscriber m_cmd_val_sub ;
        ros::Subscriber m_raw_val_sub ;
        ros::Subscriber m_wheel_odom_sub ;
        ros::Subscriber m_gnss_odom_sub ;
        ros::Subscriber m_gnss_data_sub ;
        ros::Subscriber m_imu_data_sub ;
        ros::Subscriber m_fusion_odom_sub ;
        ros::Subscriber m_gazebo_truth_odom_sub ;
        // ros::Subscriber m_robot_pose_sub ;

        void m_cmd_twist_cb(const geometry_msgs::Twist::ConstPtr& cmd_twist) ;
        void m_cmd_val_cb(const slab_srb_msgs::diff_driver_cmd::ConstPtr& cmd_val ) ;
        void m_raw_val_cb(const slab_srb_msgs::diff_driver_fb::ConstPtr& raw_val ) ;
        void m_wheel_odom_cb(const nav_msgs::Odometry::ConstPtr& wheel_odom ) ;
        void m_gnss_odom_cb(const nav_msgs::Odometry::ConstPtr& gnss_odom) ;
        void m_gnss_data_cb(const sensor_msgs::NavSatFix::ConstPtr& gnss_data) ;
        void m_imu_data_cb(const sensor_msgs::Imu::ConstPtr& imu_data ) ;
        void m_fusion_odom_cb(const nav_msgs::Odometry::ConstPtr& fusion_odom) ;
        void m_gazebo_truth_odom_cb(const nav_msgs::Odometry::ConstPtr& gazebo_truth_odom) ;
        // void m_robot_pose_cb(const ::ConstPtr& pulse_count) ; 

    public:
        ros::NodeHandle m_nh ;        // ノードハンドラ　ここで定義シていいものなのか？
        ros::NodeHandle m_pnh ;       // パラメータ通信用のノードハンドラ

        VisualIndicatorPublisher() ;
        ~VisualIndicatorPublisher() ;

        void Init() ;
        void Publish() ;
} ;

VisualIndicatorPublisher::VisualIndicatorPublisher() :
    m_tf_listener(ros::Duration(10))
{
}

VisualIndicatorPublisher::~VisualIndicatorPublisher()
{
}
    

void VisualIndicatorPublisher::Init()
{
    // m_pnh.getParam("/electric_car/electril_car_params/giar_rate_param" , gaiar_rate ) ;                // ギア比

    m_cmd_twist_sub   = m_nh.subscribe<geometry_msgs::Twist>("/slab_srb_robot/diff_drive_controller/cmd_vel", 10, &VisualIndicatorPublisher::m_cmd_twist_cb, this) ;
    m_cmd_val_sub     = m_nh.subscribe<slab_srb_msgs::diff_driver_cmd>("/slab_srb_robot/diff_driver_cmd", 10, &VisualIndicatorPublisher::m_cmd_val_cb, this) ;
    m_raw_val_sub     = m_nh.subscribe<slab_srb_msgs::diff_driver_fb>("/slab_srb_robot/diff_driver_fb", 10, &VisualIndicatorPublisher::m_raw_val_cb, this) ;
    m_wheel_odom_sub  = m_nh.subscribe<nav_msgs::Odometry>("/slab_srb_robot/diff_drive_controller/odom", 10, &VisualIndicatorPublisher::m_wheel_odom_cb, this) ;
     m_gnss_odom_sub  = m_nh.subscribe<nav_msgs::Odometry>("/slab_srb_robot/gnss/odom", 10, &VisualIndicatorPublisher::m_gnss_odom_cb, this) ;
    m_imu_data_sub    = m_nh.subscribe<sensor_msgs::Imu>("/slab_srb_robot/imu/data", 10, &VisualIndicatorPublisher::m_imu_data_cb, this) ;
    m_fusion_odom_sub = m_nh.subscribe<nav_msgs::Odometry>("/slab_srb_robot/fusion/odom", 10, &VisualIndicatorPublisher::m_fusion_odom_cb, this ) ;
    m_gnss_data_sub  = m_nh.subscribe<sensor_msgs::NavSatFix>("/slab_srb_robot/gnss/data", 10, &VisualIndicatorPublisher::m_gnss_data_cb, this) ;

    m_gazebo_truth_odom_sub  = m_nh.subscribe<nav_msgs::Odometry>("/slab_srb_robot/gazebo/tracking/groundtruth", 10, &VisualIndicatorPublisher::m_gazebo_truth_odom_cb, this) ;
    //


    m_cmd_velocity_pub          = m_nh.advertise<std_msgs::Float32>("visual_indicator/cmd_velocity", 10 ) ;
    m_cmd_angler_velocity_pub   = m_nh.advertise<std_msgs::Float32>("visual_indicator/cmd_angler_velocity", 10 ) ;
    m_raw_velocity_pub          = m_nh.advertise<std_msgs::Float32>("visual_indicator/raw_velocity", 10 ) ;
    m_raw_angler_velocity_pub   = m_nh.advertise<std_msgs::Float32>("visual_indicator/raw_angler_velocity", 10 ) ;

    m_r_motor_cmd_rps_pub       = m_nh.advertise<std_msgs::Float32>("visual_indicator/r_motor_cmd_rps", 10 ) ;  
    m_l_motor_cmd_rps_pub       = m_nh.advertise<std_msgs::Float32>("visual_indicator/l_motor_cmd_rps", 10 ) ;
    m_r_motor_raw_rps_pub       = m_nh.advertise<std_msgs::Float32>("visual_indicator/r_motor_raw_rpm", 10 ) ;
    m_l_motor_raw_rps_pub       = m_nh.advertise<std_msgs::Float32>("visual_indicator/l_motor_raw_rps", 10 ) ;

    m_wheel_odom_x_pub          = m_nh.advertise<std_msgs::Float32>("visual_indicator/wheel_odom_x", 10 ) ;
    m_wheel_odom_y_pub          = m_nh.advertise<std_msgs::Float32>("visual_indicator/wheel_odom_y", 10 ) ;
    m_wheel_odom_th_pub         = m_nh.advertise<std_msgs::Float32>("visual_indicator/wheel_odom_th", 10 ) ;

    m_gnss_odom_x_pub           = m_nh.advertise<std_msgs::Float32>("visual_indicator/gnss_odom_x", 10 ) ;
    m_gnss_odom_y_pub           = m_nh.advertise<std_msgs::Float32>("visual_indicator/gnss_odom_y", 10 ) ;
    m_gnss_odom_th_pub          = m_nh.advertise<std_msgs::Float32>("visual_indicator/gnss_odom_th", 10 ) ;

    m_gnss_data_pub            = m_nh.advertise<jsk_rviz_plugins::OverlayText>("visual_indicator/gnss_state", 10);

    m_imu_data_x_pub            = m_nh.advertise<std_msgs::Float32>("visual_indicator/imu_data_x", 10 ) ;
    m_imu_data_y_pub            = m_nh.advertise<std_msgs::Float32>("visual_indicator/imu_data_y", 10 ) ;
    m_imu_data_th_pub            = m_nh.advertise<std_msgs::Float32>("visual_indicator/imu_data_th", 10 ) ;

    m_imu_data_x_vel_pub            = m_nh.advertise<std_msgs::Float32>("visual_indicator/imu_data_x_vel", 10 ) ;
    m_imu_data_y_vel_pub            = m_nh.advertise<std_msgs::Float32>("visual_indicator/imu_data_y_vel", 10 ) ;
    m_imu_data_th_vel_pub            = m_nh.advertise<std_msgs::Float32>("visual_indicator/imu_data_th_vel", 10 ) ;

    m_fusion_odom_x_pub         = m_nh.advertise<std_msgs::Float32>("visual_indicator/fusion_odom_x", 10 ) ;
    m_fusion_odom_y_pub         = m_nh.advertise<std_msgs::Float32>("visual_indicator/fusion_odom_y", 10 ) ;
    m_fusion_odom_th_pub        = m_nh.advertise<std_msgs::Float32>("visual_indicator/fusion_odom_th", 10 ) ;

    m_robot_pose_x_pub          = m_nh.advertise<std_msgs::Float32>("visual_indicator/robot_pose_x", 10 ) ;
    m_robot_pose_y_pub          = m_nh.advertise<std_msgs::Float32>("visual_indicator/robot_pose_y", 10 ) ;
    m_robot_pose_th_pub         = m_nh.advertise<std_msgs::Float32>("visual_indicator/robot_pose_th", 10 ) ;

    m_gazebo_truth_odom_x_pub   = m_nh.advertise<std_msgs::Float32>("visual_indicator/gazebo_truth_x", 10 ) ;
    m_gazebo_truth_odom_y_pub   = m_nh.advertise<std_msgs::Float32>("visual_indicator/gazebo_truth_y", 10 ) ;
    m_gazebo_truth_odom_th_pub   = m_nh.advertise<std_msgs::Float32>("visual_indicator/gazebo_truth_th", 10 ) ;
}


void VisualIndicatorPublisher::m_cmd_twist_cb(const geometry_msgs::Twist::ConstPtr& cmd_twist)
{
        m_cmd_velocity.data        = cmd_twist->linear.x;
        m_cmd_angler_velocity.data = cmd_twist->angular.z ;
}


void VisualIndicatorPublisher::m_cmd_val_cb(const slab_srb_msgs::diff_driver_cmd::ConstPtr& cmd_val )
{
        m_r_motor_cmd_rps.data = (  cmd_val->r_motor_angular_velocity / (2.0*M_PI) ) ;
        m_l_motor_cmd_rps.data = (  cmd_val->l_motor_angular_velocity / (2.0*M_PI) ) ;
}


void VisualIndicatorPublisher::m_raw_val_cb(const slab_srb_msgs::diff_driver_fb::ConstPtr& raw_val )
{
        m_r_motor_raw_rps.data = ( raw_val->r_angular_velocity / (2.0*M_PI) );
        m_l_motor_raw_rps.data = ( raw_val->l_angular_velocity / (2.0*M_PI) );
}


void VisualIndicatorPublisher::m_wheel_odom_cb(const nav_msgs::Odometry::ConstPtr& wheel_odom )
{
        tf::Quaternion quat(
                wheel_odom->pose.pose.orientation.x,
                wheel_odom->pose.pose.orientation.y,
                wheel_odom->pose.pose.orientation.z,
                wheel_odom->pose.pose.orientation.w
                ) ;

        tf::Matrix3x3 matrix(quat) ;
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);

        m_raw_velocity.data        = wheel_odom->twist.twist.linear.x ;
        m_raw_angler_velocity.data = wheel_odom->twist.twist.angular.z ;

        m_wheel_odom_x.data        = wheel_odom->pose.pose.position.x ;
        m_wheel_odom_y.data        = wheel_odom->pose.pose.position.y  ;
        m_wheel_odom_th.data       = yaw ;

}


void VisualIndicatorPublisher::m_gnss_odom_cb(const nav_msgs::Odometry::ConstPtr& gnss_odom )
{
        m_gnss_odom_x.data        = gnss_odom->pose.pose.position.x ;
        m_gnss_odom_y.data        = gnss_odom->pose.pose.position.y  ;

        tf::Quaternion quat(
                gnss_odom->pose.pose.orientation.x,
                gnss_odom->pose.pose.orientation.y,
                gnss_odom->pose.pose.orientation.z,
                gnss_odom->pose.pose.orientation.w
                ) ;

        tf::Matrix3x3 matrix(quat) ;
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);

        m_gnss_odom_th.data = yaw - M_PI ;

}

void VisualIndicatorPublisher::m_gnss_data_cb(const sensor_msgs::NavSatFix::ConstPtr& gnss_data )
{
    m_gnss_data.action=jsk_rviz_plugins::OverlayText::ADD;
    m_gnss_data.width=400;
    m_gnss_data.height=100;
    m_gnss_data.left=0;
    m_gnss_data.top=500;

    std_msgs::ColorRGBA color1, single_color, froat_color, fix_color ;
    color1.r = 0;
    color1.g = 0;
    color1.b = 0;
    color1.a = 0.4;
    m_gnss_data.bg_color=color1;

    single_color.r = 255.0/255;
    single_color.g = 25.0/255;
    single_color.b = 25.0/255;
    single_color.a = 0.8;

    froat_color.r = 50.0/255;
    froat_color.g = 50.0/255;
    froat_color.b = 255.0/255;
    froat_color.a = 0.8;

    fix_color.r = 25.0/255;
    fix_color.g = 255.0/255;
    fix_color.b = 100.0/255;
    fix_color.a = 0.8;


    m_gnss_data.line_width=1;
    m_gnss_data.text_size=14;

    m_gnss_data.font="Ubuntu";
    if( gnss_data->position_covariance_type == 2 )
    {
        m_gnss_data.text="float";
        m_gnss_data.fg_color=froat_color;
    } else if( gnss_data->position_covariance_type == 3 ) {
        m_gnss_data.text = "fix" ;
        m_gnss_data.fg_color=fix_color;
    } else {
        m_gnss_data.text = "---" ;
        m_gnss_data.fg_color=single_color;
    }

}

void VisualIndicatorPublisher::m_imu_data_cb(const sensor_msgs::Imu::ConstPtr& imu_data )
{
        tf::Quaternion quat(
                imu_data->orientation.x,
                imu_data->orientation.y,
                imu_data->orientation.z,
                imu_data->orientation.w
                ) ;

        tf::Matrix3x3 matrix(quat) ;
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);

        m_imu_data_x.data = roll ;
        m_imu_data_y.data = pitch ;
        m_imu_data_th.data = yaw ;

        m_imu_data_x_vel.data   = imu_data->angular_velocity.x ;
        m_imu_data_y_vel.data   = imu_data->angular_velocity.y ;
        m_imu_data_th_vel.data  = imu_data->angular_velocity.z ;
}


void VisualIndicatorPublisher::m_fusion_odom_cb(const nav_msgs::Odometry::ConstPtr& fusion_odom)
{
    tf::Quaternion quat(
            fusion_odom->pose.pose.orientation.x,
            fusion_odom->pose.pose.orientation.y,
            fusion_odom->pose.pose.orientation.z,
            fusion_odom->pose.pose.orientation.w
            ) ;

    tf::Matrix3x3 matrix(quat) ;
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    m_raw_velocity.data        = fusion_odom->twist.twist.linear.x ;
    m_raw_angler_velocity.data = fusion_odom->twist.twist.angular.z ;

    m_fusion_odom_x.data       = fusion_odom->pose.pose.position.x ;
    m_fusion_odom_y.data       = fusion_odom->pose.pose.position.y  ;
    m_fusion_odom_th.data      = yaw ;


    try {
        m_tf_listener.lookupTransform("/slab_srb_robot/map","/slab_srb_robot/base_link",ros::Time(0), m_trans_slam );
        m_robot_pose_x.data  = m_trans_slam.getOrigin().x() ;
        m_robot_pose_y.data  = m_trans_slam.getOrigin().y() ;
        tf::Quaternion quat = m_trans_slam.getRotation() ;
        tf::Matrix3x3 matrix(quat) ;
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);

        m_robot_pose_th.data = yaw ;
    }
    catch ( tf::TransformException &ex ) {
        ROS_ERROR("%s",ex.what()) ;
        ros::Duration(1.0).sleep();
    }
}


void VisualIndicatorPublisher::m_gazebo_truth_odom_cb(const nav_msgs::Odometry::ConstPtr& gazebo_truth_odom)
{
    tf::Quaternion quat(
            gazebo_truth_odom->pose.pose.orientation.x,
            gazebo_truth_odom->pose.pose.orientation.y,
            gazebo_truth_odom->pose.pose.orientation.z,
            gazebo_truth_odom->pose.pose.orientation.w
            ) ;

    tf::Matrix3x3 matrix(quat) ;
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    m_raw_velocity.data        = gazebo_truth_odom->twist.twist.linear.x ;
    m_raw_angler_velocity.data = gazebo_truth_odom->twist.twist.angular.z ;

    m_gazebo_truth_odom_x.data   = gazebo_truth_odom->pose.pose.position.x ;
    m_gazebo_truth_odom_y.data     = gazebo_truth_odom->pose.pose.position.y  ;
    m_gazebo_truth_odom_th.data     = yaw ;
}



void VisualIndicatorPublisher::Publish()
{
    m_cmd_velocity_pub.publish(m_cmd_velocity) ;
    m_cmd_angler_velocity_pub.publish(m_cmd_angler_velocity) ;
    m_raw_velocity_pub.publish(m_raw_velocity) ;
    m_raw_angler_velocity_pub.publish(m_raw_angler_velocity) ;

    m_r_motor_cmd_rps_pub.publish(m_r_motor_cmd_rps) ;
    m_l_motor_cmd_rps_pub.publish(m_l_motor_cmd_rps) ;

    m_r_motor_raw_rps_pub.publish(m_r_motor_raw_rps) ;
    m_l_motor_raw_rps_pub.publish(m_l_motor_raw_rps) ;

    m_wheel_odom_x_pub.publish(m_wheel_odom_x) ;
    m_wheel_odom_y_pub.publish(m_wheel_odom_y) ;
    m_wheel_odom_th_pub.publish(m_wheel_odom_th) ;

    m_gnss_odom_x_pub.publish(m_gnss_odom_x) ;
    m_gnss_odom_y_pub.publish(m_gnss_odom_y) ;
    m_gnss_odom_th_pub.publish(m_gnss_odom_th) ;

    m_imu_data_x_pub.publish(m_imu_data_x) ;
    m_imu_data_y_pub.publish(m_imu_data_y) ;
    m_imu_data_th_pub.publish(m_imu_data_th) ;

    m_imu_data_x_vel_pub.publish(m_imu_data_x_vel) ;
    m_imu_data_y_vel_pub.publish(m_imu_data_y_vel) ;
    m_imu_data_th_vel_pub.publish(m_imu_data_th_vel) ;

    m_fusion_odom_x_pub.publish(m_fusion_odom_x) ;
    m_fusion_odom_y_pub.publish(m_fusion_odom_y) ;
    m_fusion_odom_th_pub.publish(m_fusion_odom_th) ;

    m_gnss_data_pub.publish(m_gnss_data) ;

    m_robot_pose_x_pub.publish(m_robot_pose_x) ;
    m_robot_pose_y_pub.publish(m_robot_pose_y) ;
    m_robot_pose_th_pub.publish(m_robot_pose_th) ;

    m_gazebo_truth_odom_x_pub.publish(m_gazebo_truth_odom_x) ;
    m_gazebo_truth_odom_y_pub.publish(m_gazebo_truth_odom_y) ;
    m_gazebo_truth_odom_th_pub.publish(m_gazebo_truth_odom_th) ;
}


int main(int argc, char **argv)
{
    // ノードを初期化
    ros::init(argc, argv, "visual_indicator_pub");

    VisualIndicatorPublisher visual_indicator_publisher ;
    visual_indicator_publisher.Init() ;

    ros::Rate loop_rate(20) ;
    while(ros::ok()) {
        visual_indicator_publisher.Publish() ;
        ros::spinOnce() ;
        loop_rate.sleep() ;
    }
    return 0;
}
