#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <fstream>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include<sys/time.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <tf/tf.h>
#include <math.h>
#include <std_msgs/Float64.h>


// https://en.wikipedia.org/wiki/SocketCAN
//https://www.raspberrypi.org/forums/viewtopic.php?t=141052
//https://github.com/linux-can/can-utils

//struct can_frame {
//            canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
//            __u8    can_dlc; /* frame payload length in byte (0 .. 8) */
//            __u8    __pad;   /* padding */
//            __u8    __res0;  /* reserved / padding */
//            __u8    __res1;  /* reserved / padding */
//            __u8    data[8] __attribute__((aligned(8)));
//    };


#include <pthread.h>
#include <math.h>

#include <netdb.h>
#include <arpa/inet.h>

using namespace std;
//using namespace boost::asio;

#define CAN_MSG_COUNT	10000

#define OFFSET 0x800000

# define M_PIl 3.141592653589793238462643383279502884L/* pi */

static int running = 1;
static int sockfd;

double X_angle;
double Y_angle;

int Frame_ID;
int Frame_Length;
double Yaw_rate, Acc_Y;
double Roll_rate, Acc_X;
double Acc_Z;

// Test Variables
long double X_tmp_angle;
long double Y_tmp_angle;
double Yaw_tmp_rate, Acc_tmp_Y;
double Roll_tmp_rate, Acc_tmp_X;
double Acc_tmp_Z;
double Yaw_s = 0;
double Yaw = 0;
double Roll_s = 0;
double Roll = 0;
double Pitch = 0;

unsigned int hex_Yaw, hex_accY;//int tmp_Yaw, tmp_accY;
unsigned int hex_Roll, hex_accX;//int tmp_Roll, tmp_accX;
unsigned int hex_accZ;//int tmp_accZ;
unsigned int hex_angleX;//int tmp_angleX;
unsigned int hex_angleY;//int tmp_angleY;

std::string frame_id, child_frame_id;



char tmpmsg[8];
FILE *fp;

double rxCount;

struct test_frame {
  unsigned int    can_id;
  unsigned char    can_dlc;
  unsigned char    data[8] ;
};

struct test_frame tframe;
//struct can_frame tx_frame;

void *hexTodecimal (double x, double y)//(int x, int y)
{
  double dec_number[2];
  //  int sum[];
  //  double sum_1;
#define NUM 10000

  if(Frame_ID == 0x180)
  {
    dec_number[0] = x*0.00549319;
    dec_number[1] = y*0.00549319;

    X_angle = dec_number[0] ;
    Y_angle = dec_number[1] ;
  }
  else
  {
    dec_number[0] = x*0.005;
    dec_number[1] = y*0.000127465;

    if( Frame_ID == 0x174 )
    {
      Yaw_rate = dec_number[0];
      /*
     for (int i = 0; i < NUM; ++i)
     {
      sum[] = dec_number[0];
      sum_1 += sum[i] ;
     }
*/
      Acc_Y = dec_number[1] ;
    }
    else if(Frame_ID == 0x178)
    {
      Roll_rate = dec_number[0] ;
      Acc_X = dec_number[1] ;
    }
    else if(Frame_ID == 0x17C)
    {
      Acc_Z = dec_number[1] ;
    }
    else
    {
      ;
    }
  }
}


/*
void IMUupdate()
{
double norm;
//float xdata vx, vy, vz;
double ex, ey, ez;


ax = Acc_X;
ay = Acc_Y;
az = Acc_Z;


gx = Roll_rate;
gy = 0;
gz = Yaw_rate;


gx = gx*100/65536;
gy = gy*100/65536;
gz = gz*100/65536;

// 测量正常化
norm = sqrt(ax*ax + ay*ay + az*az);

ax = ax / norm;//单位化
ay = ay / norm;
az = az / norm;


// 估计方向的重力
vx = 2*(q1*q3 - q0*q2);
vy = 2*(q0*q1 + q2*q3);
vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

// 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
ex = (ay*vz - az*vy);
ey = (az*vx - ax*vz);
ez = (ax*vy - ay*vx);


// 积分误差比例积分增益
exInt = exInt + ex*Ki;
eyInt = eyInt + ey*Ki;
ezInt = ezInt + ez*Ki;


// 调整后的陀螺仪测量
gx = gx + Kp*ex + exInt;
gy = gy + Kp*ey + eyInt;
gz = gz + Kp*ez + ezInt;


// 整合四元数率和正常化
q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;


// 正常化四元
norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
q0 = q0 / norm;
q1 = q1 / norm;
q2 = q2 / norm;
q3 = q3 / norm;



}


/*

/*
void *hexTodecimal (double x1, double y1)//(int x, int y)
{
    double dec_number[2];

    if(Frame_ID == 0x180)
    {
        dec_number[0] = x1*0.00549319;
        dec_number[1] = y1*0.00549319;

        X_angle = dec_number[0] ;
        Y_angle = dec_number[1] ;
    }
    else
    {
        dec_number[0] = x1*0.005;
        dec_number[1] = y1*0.000127465;

        if( Frame_ID == 0x174 )
        {
            w = dec_number[0] ;
            y = dec_number[1] ;
        }
        else if(Frame_ID == 0x178)
        {
            Roll_rate = dec_number[0] ;
            x = dec_number[1] ;
        }
        else if(Frame_ID == 0x17C)
        {
            z = dec_number[1] ;
        }
        else
        {
            ;
        }
    }
}
*/
static void print_frame(struct can_frame *frame)
{
  int i;


  double tmp_Yaw, tmp_accY;//int tmp_Yaw, tmp_accY;
  double tmp_Roll, tmp_accX;//int tmp_Roll, tmp_accX;
  double tmp_accZ;//int tmp_accZ;
  double tmp_angleX;//int tmp_angleX;
  double tmp_angleY;//int tmp_angleY;

  //printf("%04x ", frame->can_id);
  if (frame->can_id & CAN_RTR_FLAG)
  {
    printf("remote request");
  }
  else
  {
    //printf(" [%d]", frame->can_dlc);
    for (i = 0; i < frame->can_dlc; i++)
    {
      //printf(" %02x ", frame->data[i]);
      //frame->data[i][i] = frame->data[i];
      //printf(" %02x  ", frame->data[i][i]);

    }
#if 1

    if(frame->can_id == 0x174)
    {
      Frame_ID = frame->can_id;

      tmp_Yaw =  ( (frame->data[1])<< 8)+ ( (frame->data[0]) ) ;
      tmp_Yaw = tmp_Yaw - 0x8000;
      tmp_accY =   ( (frame->data[5])<< 8)+ ( (frame->data[4]) ) ;
      tmp_accY = tmp_accY - 0x8000;

      //test
      Yaw_tmp_rate =  ( (((frame->data[1])<< 8)+ ( (frame->data[0]) )) - 0x8000 ) * 0.005;

      Acc_tmp_Y =  ((( (frame->data[5])<< 8)+ ((frame->data[4])) ) - 0x8000 ) * 0.000127465;
      // test
      hex_Yaw = ( (((frame->data[1])<< 8)+ ( (frame->data[0]) )) - 0x8000 ) ;
      hex_accY = ((( (frame->data[5])<< 8)+ ((frame->data[4])) ) - 0x8000 );
      hexTodecimal(tmp_Yaw, tmp_accY);
    }
    else if(frame->can_id == 0x178)
    {
      Frame_ID = frame->can_id;

      tmp_Roll = ( (frame->data[1])<< 8)+ ( (frame->data[0]) ) ;
      tmp_Roll = tmp_Roll - 0x8000;
      tmp_accX =( (frame->data[5])<< 8)+ ( (frame->data[4]) ) ;
      tmp_accX = tmp_accX - 0x8000;

      //test
      Roll_tmp_rate =  ( (((frame->data[1])<< 8)+ ( (frame->data[0]) )) - 0x8000 ) * 0.005;
      Acc_tmp_X =  ((( (frame->data[5])<< 8)+ ((frame->data[4])) ) - 0x8000 ) * 0.000127465;
      //test
      hex_Roll = ( (((frame->data[1])<< 8)+ ( (frame->data[0]) )) - 0x8000 ) ;
      hex_accX = ((( (frame->data[5])<< 8)+ ((frame->data[4])) ) - 0x8000 ) ;

      hexTodecimal(tmp_Roll, tmp_accX);

    }
    else if(frame->can_id == 0x17C)
    {
      Frame_ID = frame->can_id;

      tmp_accZ = ( (frame->data[5])<< 8)+ ( (frame->data[4]) ) ;
      tmp_accZ = tmp_accZ - 0x8000;

      //test
      Acc_tmp_Z =  ((( (frame->data[5])<< 8)+ ((frame->data[4])) ) - 0x8000 ) * 0.000127465;
      //test
      hex_accZ = ((( (frame->data[5])<< 8)+ ((frame->data[4])) ) - 0x8000 ) ;

      hexTodecimal(0, tmp_accZ);
    }
    // else if(frame->can_id == 0x180)

    if(frame->can_id == 0x180)
    {
      Frame_ID = frame->can_id;

      tmp_angleX =  ( (frame->data[1])<< 8)+ ( (frame->data[0]) ) ;
      tmp_angleX = tmp_angleX - 0x8000;
      tmp_angleY =  ((frame->data[5])<< 8)+ ( (frame->data[4]) ) ;
      tmp_angleY = tmp_angleY - 0x8000;

      //test
      X_tmp_angle=  ( (((frame->data[1])<< 8)+ ( (frame->data[0]) )) - 0x8000 ) * 0.00549316;
      Y_tmp_angle =  ((( (frame->data[5])<< 8)+ ((frame->data[4])) ) - 0x8000 ) * 0.00549316;
      //test
      hex_angleX = ( (((frame->data[1])<< 8)+ ( (frame->data[0]) )) - 0x8000 );
      hex_angleY = ((( (frame->data[5])<< 8)+ ((frame->data[4])) ) - 0x8000 ) ;

      //printf("hex_angle_X: %04x, hex_angle_Y: %04x\n",( (((frame->data[1])<< 8)+ ( (frame->data[0]) )) - 0x8000 ),((( (frame->data[5])<< 8)+ ((frame->data[4])) ) - 0x8000 ) );
      //printf("X_angle: %3.5f, Y_angle: %3.5f\n",((( (((frame->data[1])<< 8)+ ( (frame->data[0]) )) - 0x8000) * 0.00549316) ),((((( (frame->data[5])<< 8)+ ((frame->data[4])) ) - 0x8000 ))* 0.00549316 ));

      //printf("X_tmp_angle: %3.5f \n ",X_tmp_angle);
      //printf("frame->can_id: %02x frame->data[0]: %02x frame->data[1]: %02x frame->data[i][2]: %02x frame->data[i][3]: %02x frame->data[4]: %02x frame->data[5]: %02x frame->data[i][6]: %02x frame->data[i][7]: %02x  \n ",frame->can_id,frame->data[0], frame->data[1], frame->data[i][2], frame->data[i][3], frame->data[4], frame->data[5], frame->data[i][6], frame->data[i][7]);
      //hexTodecimal(tmp_angleX, tmp_angleY);
      //printf("frame->can_id: %02x frame->data[i][i]: %02x \n ", frame->can_id,frame->data[i][i]);

    }
#endif
  }

}

static int recv_frame(struct can_frame *frame)
{
  int ret;

  ret = recv(sockfd, frame, sizeof(*frame), 0);

  if (ret != sizeof(*frame))
  {
    if (ret < 0)
      perror("recv failed");
    else
      fprintf(stderr, "recv returned %d", ret);
    return -1;
  }

  return 0;
}


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

void *recv_pthread(void *threadid)
{
  struct can_frame rx_frame;

  while(running)
  {
    if (recv_frame(&rx_frame))
    {
      printf("Read NG: \n");
    }

    print_frame(&rx_frame);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gypo_connection");
  ros::NodeHandle nh;
  ros::NodeHandle node;
  ros::Time current_time, last_time;


  ros::Publisher imu_pub = nh.advertise<std_msgs::Float64>("/yaw", 1000);


  //string filePath_theta = "/home/jumin/development/Test//orchardTest_ws/src/theta_rate.txt";

  //ofstream writeFile_theta(filePath_theta.data());

   std_msgs::Float64 yaw_param;

  ros::Rate loop_rate(10);

  //CAN Protocol
  struct sockaddr_can addr;
  const char *intf_name;
  int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;
  long t = 0;

  struct sockaddr_in server_UDP;

#if 1//#ifdef Debug_TXT
  fp = fopen("Test_hex.txt","w");

  if(fp == NULL)
  {
    perror("fopen");
    return 1;
  }

#endif

  intf_name = "can3";

  printf("CAN interface = %s, family = %d, type = %d, proto = %d\n", intf_name, family, type, proto);

  unsigned char counter = 0;
  int send_pos = 0, recv_pos = 0, unprocessed = 0, loops = 0;
  int i;

  // Can create socket
  if ((sockfd = socket(family, type, proto)) < 0)
  {
    perror("CAN socket error");
    return 1;
  }

  // CAN assign local value
  addr.can_family = family;
  addr.can_ifindex = if_nametoindex(intf_name);

  if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    perror("CAN bind error");
    close(sockfd);
    return 1;
  }

  millisleep(50);

  pthread_t pth;


#if 1
  int status  = pthread_create(&pth,NULL,recv_pthread,(void *)t);
#endif

  millisleep(50);




  while (ros::ok())
  {
    int loops = 0;

    //if(writeFile_theta.is_open())     writeFile_theta << Yaw_rate << "\n";

    std::cout << "Yaw rate: " << Yaw_rate << endl;

    //if(Yaw_rate < 0.15 && Yaw_rate > -0.28)
    if(fabs(Yaw_rate) < 0.20)
    {
        Yaw_s = Yaw_s;
    }
    else
    {
        Yaw_s = Yaw_s + Yaw_rate;
    }

    //std::cout << "Yaw_s: " << Yaw_s << endl;

    Yaw = Yaw_s*0.05;

    double Yaw_11 = Yaw;

    Roll_s = Roll_s + Roll_rate;
    Roll = Roll_s*0.05;

    double Roll_11 = Roll;

    Pitch = 0;

    double Pith_rate = 0;

    printf("Yaw: %3.5f\n",Yaw);
    millisleep(50);
    yaw_param.data = Yaw;
    imu_pub.publish(yaw_param);

    ros::spinOnce();

    //loop_rate.sleep();

    //++loops;
  }

   //writeFile_theta.close();

  return 0;

}

