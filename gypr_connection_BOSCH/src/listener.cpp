#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

double Yaw = 0;
double Roll_s = 0;
double Roll = 0;
double Pitch = 0;
double Acc_x =0;
double Acc_y = 0;
double Acc_z = 0;
double Yaw_rate = 0;
double Roll_rate = 0;
double Pitch_rate = 0;

double Q_angle = 0.001f; // Process noise variance for the accelerometer
double Q_bias = 0.003f; // Process noise variance for the gyro bias
double R_measure = 0.03f; // Measurement noise variance - this is actually the variance of the measurement noise

double angle = 0.0f; // The angle calculated by the Kalman filter - part of the 2x1 state vector
double bias = 0.0f; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
double rate = 0.0f; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate


//double P = 0;
double P [2][2];  // Error covariance matrix - This is a 2x2 matrix


//P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
//P[0][1] = 0.0f;
//P[1][0] = 0.0f;
//P[1][1] = 0.0f;


double dt = 0.05;

# define M_PIl 3.141592653589793238462643383279502884L/* pi */

//void chatterCallback(const std_msgs::String::ConstPtr& msg)
//void chatterCallback(const sensor_msgs::ConstPtr& msg)

void *getAngle(double newAngle, double newRate)
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
  //ROS_INFO("newAngle:%3.5f,newRate:%3.5f", newAngle,newRate);

  P [0][0] = 0.0f;
  P [0][1] = 0.0f;
  P [1][0] = 0.0f;
  P [1][1] = 0.0f;
  double dt = 0.05;

  bias = 0.036666;
   //newRate = (Yaw_rate*M_PI)*0.00555556;
   //newAngle = (Yaw*M_PI)*0.0055556;
   //Yaw = (Yaw_s*0.05*M_PI)*0.0055556;
   //ROS_INFO("bias:%3.5f", bias);
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    double S = P[0][0] + R_measure; // Estimate error
    //ROS_INFO("S:%3.5f", S);
    /* Step 5 */
    double K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;


    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    double y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    double P00_temp = P[0][0];
    double P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    //angle = angle*180/M_PI;
    printf("newAngle:%3.5f,angle:%3.5f,dt:%3.5f \n", newAngle,angle,dt);
    //ROS_INFO("angle:%3.5f,bias:%3.5f,y:%3.5f,S:%3.5f,K[0]:%3.5f,K[1]:%3.5f,P00_temp:%3.5f,P01_temp:%3.5f", angle,bias,y,S,K[0],K[1],P00_temp,P01_temp);

    return 0;
}

void chatterCallback(const sensor_msgs::Imu imu)
{
  //sensor_msgs::Imu msg;
  //ROS_INFO("I heard: %3.5f", msg->header.seq);
  //ROS_INFO("I heard: Acc_X : [%3.5f], Acc_Y: [%3.5f], Acc_Z: [%3.5f], Roll_rate:%3.5f, Yaw_rate:%3.5f, Yaw:%3.5f", imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z,imu.angular_velocity.x, imu.angular_velocity.z, imu.orientation.z);
  //ROS_INFO("X_Angle: %3.5f, AX: %3.5f, AY: %3.5f, AZ:%3.5f, Roll:%3.5f, Yaw:%3.5f ",X_angle,Acc_X,Acc_Y, Acc_Z,Roll_rate,Yaw_rate);
  //ROS_INFO("I heard:%3.5f", imu->magnetic_field.x);
  //ROS_INFO("%s", msg->c_str());
  //ROS_INFO("Yaw:%3.5f", imu.orientation.z);

  //ROS_INFO("Roll:%3.5f,Pith_1:%3.5f,Yaw:%3.5f,Yaw_2:%3.5f", imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.z);


  //ROS_INFO("I heard: Acc_X : [%3.5f], Acc_Y: [%3.5f], Acc_Z: [%3.5f], Roll_rate:%3.5f, Yaw_rate:%3.5f", imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z,imu.angular_velocity.x, imu.angular_velocity.z);
  //ROS_INFO("I heard: Acc_X : [%3.5f], Acc_Y: [%3.5f], Acc_Z: [%3.5f], Roll_rate:%3.5f, Yaw_rate:%3.5f, Yaw:%3.5f", imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z,imu.angular_velocity.x, imu.angular_velocity.z, imu.orientation.z);
  Roll =  imu.orientation.x;

  Pitch = imu.orientation.y;
  Yaw = imu.orientation.z;
  Acc_x =imu.linear_acceleration.x;
  Acc_y = imu.linear_acceleration.y;
  Acc_z = imu.linear_acceleration.z;
  Yaw_rate = imu.angular_velocity.z;
  Roll_rate = imu.angular_velocity.x;
  Pitch_rate = 0;
  getAngle(Yaw,Yaw_rate);


}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("imu/data_raw", 1000, chatterCallback);

  ros::spin();

  return 0;
}
