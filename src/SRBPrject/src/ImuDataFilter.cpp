#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"

sensor_msgs::Imu imu_data ;
ros::Publisher imu_data_pub;
tf::Quaternion imu_quat_now ;  // 今回観測した姿勢 
tf::Quaternion imu_quat_last ; // 前回観測した値

ros::Time     cb_current_time ;
ros::Time     cb_last_time ;
ros::Duration cb_dt ;



void imu_data_raw_cb(const sensor_msgs::Imu& imu_data_raw)
{
    imu_data = imu_data_raw ;
    imu_data.header.stamp    = ros::Time::now() ;

    // imu_data.header.frame_id = "dtw_robot1/imu_link" ;
    // orientation_covariance = { 0.017453292519943295, 0.0, 0.0,0.0, 0.017453292519943295, 0.0,0.0, 0.0, 0.15707963267948966};

    // angular_velocity_covariance =  {
    //     0.0004363323129985824, 0.0, 0.0,
    //     0.0, 0.0004363323129985824, 0.0,
    //     0.0, 0.0, 0.0004363323129985824
    // } ;
    

    imu_quat_last = tf::Quaternion(
        imu_quat_now.getAxis(),
        (2 * M_PI) - (imu_quat_now.getAngle())
        ) ;

    imu_quat_now = tf::Quaternion(
            imu_data_raw.orientation.x,
            imu_data_raw.orientation.y,
            imu_data_raw.orientation.z,
            imu_data_raw.orientation.w
            ) ;
    tf::Quaternion d_quat = imu_quat_now * imu_quat_last ;
   //  imu_quat_now.dot(imu_quat_last) ; // 差分

    // 姿勢の差分をrpyに変換
    tf::Matrix3x3 matrix(d_quat) ;
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);


    // 時間の更新
    cb_last_time    = cb_current_time ;
    cb_current_time = ros::Time::now();
    cb_dt = cb_current_time - cb_last_time ;

    // 前回のループからの経過時間
    double dt = cb_dt.sec + cb_dt.nsec * (0.000000001) ;

    // rad/sに変換するために制御周期で割る
    double roll_vel  = roll / dt ;
    double pitch_vel = pitch / dt ;
    double yaw_vel   = yaw / dt ;

    imu_data.angular_velocity.x = roll_vel ;
    imu_data.angular_velocity.y = pitch_vel ;
    imu_data.angular_velocity.z = yaw_vel ;
}

//------------------main------------------

int main(int argc, char **argv)
{
    // ノードを初期化
    ros::init(argc, argv, "slab_srb_imu_data_filter");
    ros::NodeHandle nh ;

    //Publisher
    imu_data_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    //Subscriber
    ros::Subscriber serial_sub = nh.subscribe("imu/data_raw", 10, imu_data_raw_cb);

    cb_current_time = ros::Time::now();

    ros::Rate loop_rate(20) ;
    while(ros::ok()) {
        imu_data_pub.publish(imu_data) ;
        ros::spinOnce() ;
        loop_rate.sleep() ;
    }
    return 0 ;
}

