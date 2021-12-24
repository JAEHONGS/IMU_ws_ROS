#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

double Roll_rate, Acc_X;
double Pitch_rate,Acc_Y;
double Yaw_rate, Acc_Z;
double Roll = 0;
double Pitch = 0;
double Yaw = 0;

void imudataCallback(const sensor_msgs::Imu imu)
{
    //ROS_INFO("I heard: Roll:%3.5f, Pitch:%3.5f, Yaw:%3.5f,Roll_rate:%3.5f, Pitch_rate:%3.5f,Yaw_rate:%3.5f, Acc_X : [%3.5f], Acc_Y: [%3.5f], Acc_Z: [%3.5f], ", imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z,imu.angular_velocity.x, imu.angular_velocity.z);
    Roll = imu.orientation.x;
    Pitch = imu.orientation.y;
    Yaw = imu.orientation.z;
    printf(" Roll:%3.5f, Pitch:%3.5f, Yaw:%3.5f \n",  Roll, Pitch, Yaw);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Subscriber sub_imu = nh.subscribe("imu/data_raw", 1000, imudataCallback);
    ros::spin();
    return 0;
}
