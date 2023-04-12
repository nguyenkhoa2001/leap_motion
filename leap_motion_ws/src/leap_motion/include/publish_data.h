#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <serial/serial.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#define MAX_BUFFER_SIZE 200
#define MODE_IMU_ANGLE			7
#define MODE_IMU_VEL		7
#define MODE_IMU_ACCEL			7
#define MODE_IMU_QUAT   7
#define BEGIN_LINE_UNIT   '\n'
#define END_LINE_UNIT     '\r'

float convertAsciiToFloat(uint8_t* buffer);
float convertAsciiQuaternionToFloat(uint8_t* buffer);
void implementAsciiData(uint8_t* input_buffer);
int readPositionAndPublish(std::string hand_type, float hand_palm_position[3]);

struct data
{
    uint8_t* vel_ascii[3];
    uint8_t* ang_ascii[3];
    uint8_t* acc_ascii[3];
    uint8_t* quat_ascii[4];
    float quat[4];
    float vel[3];
    float pos[3];
    float ang[3];
    float acc[3];
};
