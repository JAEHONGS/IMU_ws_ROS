#include <ros/ros.h>
#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include <stdint.h>
#include <iostream>
#include <iomanip>
#include <fstream>

#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <signal.h>
#include <tf/tf.h>


using namespace std;

#define PI       3.14159265358979323846   // pi

//double = ;

double Roll_rate, Acc_X;
double Pitch_rate,Acc_Y;
double Yaw_rate, Acc_Z;
double Roll = 0;
double Pitch = 0;
double Yaw = 0;

static void millisleep(int msecs)
{
    struct timespec rqtp, rmtp;
    int err;

    /* sleep in ms */
    rqtp.tv_sec = msecs / 1000;
    rqtp.tv_nsec = msecs % 1000 * 1000000;

    do {
        err = clock_nanosleep(CLOCK_MONOTONIC, 0, &rqtp, &rmtp);
        if (err != 0 && err != EINTR) {
            printf("t\n");
            break;
        }
        rqtp = rmtp;
    } while (err != 0);
}


int uart_int(void)
{
    int fd;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    millisleep(20);
    if(fd == -1)
    {
        printf("\n  Error! in Opening ttyUSB0  ");
        return -1;
    }
    else
    {
        printf("\n  ttyUSB0 Opened Successfully \n");

    }
    /*---------- Setting the Attributes of the serial port using termios structure --------- */

    /*RX init*/
    struct termios SerialPortSettings;	/* Create the structure                          */

    tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

    /* Setting the Baud rate */
    cfsetispeed(&SerialPortSettings, B38400); /* Set Read  Speed as 9600                       */
    cfsetospeed(&SerialPortSettings, B38400); /* Set Write Speed as 9600                       */

    /* 8N1 Mode */
    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */


    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */


    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing   raw  format  output*/

    /* Setting Time outs */
    SerialPortSettings.c_cc[VMIN] = 100; /* Read at least 10 characters */
    SerialPortSettings.c_cc[VTIME] = 1; /* Wait indefinetly   */
    millisleep(20);

    if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
        printf("\n  ERROR ! in Setting attributes");
    else
        printf("\n  BaudRate = 38400 \n  StopBits = 1 \n  Parity   = none");

    /*------------------------------- Read data from serial port -----------------------------*/

    tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */
    millisleep(20);
    close(fd); /* Close the serial port */

    return 0;

}



void signalHandler(int signum)
{
  ROS_INFO("%s is received, Terminating the node...",strsignal(signum));
  ros::shutdown();
  exit(signum);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_raw2");
    ros::NodeHandle nh;

//    ros::NodeHandle node;

//    geometry_msgs::Quaternion q;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();

    ros::Publisher chatter_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

    int fd;
    uart_int();
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
    millisleep(20);
    if(fd == -1)
        printf("\n  Error! in Opening ttyUSB0  ");
    else
        printf("\n  ttyUSB0 Opened Successfully \n ");

    int  bytes_read = 0;    /* Number of bytes read by the read() system call */
    int i = 0;
    unsigned char read_buffer[26] = {0};

    //unsigned char msg[] = "$MIB,RESET*87\r\n";
    unsigned char msg[] = "$MIB,RESET*87";
    write(fd, msg, sizeof(msg));
    sleep(1);
    unsigned char read_buffer1,read_buffer2,read_buffer3;
    while(ros::ok())
    {
        uint8_t index;
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        int16_t roll_rate;
        int16_t pitch_rate;
        int16_t yaw_rate;
        int16_t Acc_x;
        int16_t Acc_y;
        int16_t Acc_z;
        int16_t reserved;
        uint8_t check_sum;
        //read(fd, &read_buffer1,1);
        while(read(fd, &read_buffer1,1))
        {
            if(read_buffer1 !=0xA6)
                continue;
            read(fd, &read_buffer2,1);
            if (read_buffer2 !=0xA6)
                continue;

            bytes_read = read(fd, read_buffer,26); /* Read the data  */

            tcflush(fd, TCIOFLUSH);

            if(bytes_read > 0  )
            {
                printf("\n read_byte:%d", bytes_read); /* Print the number of bytes read */
                printf(" data: ");
                for(i = 0; i < bytes_read; i++)	 /*printing only the received characters*/

                //Assemble data
                index = read_buffer[0];
                roll = (read_buffer[1] & 0xFF) | ((read_buffer[2] << 8) & 0XFF00);
                pitch = (read_buffer[3] & 0xFF) | ((read_buffer[4] << 8) & 0XFF00);
                yaw = (read_buffer[5] & 0xFF) | ((read_buffer[6] << 8) & 0XFF00);

                roll_rate = (read_buffer[7] & 0xFF) | ((read_buffer[8] << 8) & 0xFF00);
                pitch_rate = (read_buffer[9] & 0xFF) | ((read_buffer[10] << 8) & 0xFF00);
                yaw_rate = (read_buffer[11] & 0xFF) | ((read_buffer[12] << 8) & 0xFF00);

                Acc_x = (read_buffer[13] & 0xFF) | ((read_buffer[14] << 8) & 0xFF00);
                Acc_y = (read_buffer[15] & 0xFF) | ((read_buffer[16] << 8) & 0XFF00);
                Acc_z = (read_buffer[17] & 0xFF) | ((read_buffer[18] << 8) & 0xFF00);

                reserved = read_buffer[19] + read_buffer[20] + read_buffer[21] + read_buffer[22] ;
                //Verify checksum
                check_sum = read_buffer[0] + read_buffer[1] + read_buffer[2] + read_buffer[3]
                        + read_buffer[4] + read_buffer[5] + read_buffer[6] + read_buffer[7]
                        + read_buffer[8] + read_buffer[9] + read_buffer[10] + read_buffer[11] + read_buffer[12] + read_buffer[13]
                        + read_buffer[14] + read_buffer[15] + read_buffer[16] + read_buffer[17] + read_buffer[18] + read_buffer[19]
                        + read_buffer[20] + read_buffer[21] + read_buffer[22];
                                    if( check_sum != read_buffer[23])
                                          {
                                            printf("Checksum mismatch error");
                                            //return false;
                                          }

                Roll = roll/100.0;
                Pitch = pitch/100.0;
                Yaw = - yaw/100.0;

                Roll_rate = roll_rate/100.0;
                Pitch_rate = pitch_rate/100.0;
                Yaw_rate = yaw_rate/100.0;

                Acc_X = Acc_x/100.0;
                Acc_Y = Acc_y/100.0;
                Acc_z = Acc_z/100.0;
                //q=tf::createQuaternionMsgFromRollPitchYaw(Roll,Pitch,Yaw);
                //std::cout<<"输出的四元数为：w="<<q.w<<",x="<<q.x<<",y="<<q.y<<",z="<<q.z<<std::endl;
                printf(" Roll:%3.5f, Pitch:%3.5f, Yaw:%3.5f, Roll_rate:%3.5f, Pitch_rate:%3.5f,Yaw_rate:%3.5f, Acc_X: %3.5f, Acc_Y: %3.5f, Acc_Z:%3.5f \n",  Roll, Pitch, Yaw,Roll_rate, Pitch_rate,Yaw_rate, Acc_X, Acc_Y, Acc_Z);
                millisleep(20);

//                printf("%0X ", read_buffer[i]);
//                millisleep(10);


                sensor_msgs::Imu imu;
                imu.header.stamp = ros::Time::now();
                imu.header.frame_id = "/imu_link";
                //imu.child_frame_id = child_frame_id;

                imu.orientation.x = Roll*  PI / 180;
                imu.orientation.y = Pitch*  PI / 180;
                imu.orientation.z = Yaw*  PI / 180;
                imu.orientation.w =1;

                imu.angular_velocity.x = Roll_rate;
                imu.angular_velocity.y = Pitch_rate;
                imu.angular_velocity.z = Yaw_rate;

                imu.linear_acceleration.x = Acc_X;
                imu.linear_acceleration.y = Acc_Y;
                imu.linear_acceleration.z = Acc_Z;


                imu.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
                imu.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
                imu.linear_acceleration_covariance = {-1,0,0,0,0,0,0,0,0};

              chatter_pub.publish(imu);
              ros::spinOnce();

                signal(SIGINT,signalHandler);
                millisleep(20);

            }

        }


        return 0;
        ros::shutdown();
    }


}
