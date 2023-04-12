#include "publish_data.h"

struct data imu_data;

int readPositionAndPublish(std::string hand_type, float hand_palm_position[3])
{
	if(hand_type == "Right hand") return 0;
    ros::init(argc, argv, "imu_publish_data");
    ros::NodeHandle n;
    ros::Publisher publish_angular_linear = n.advertise<geometry_msgs::Twist>("angular and linear", 1);
    ros::Publisher publish_pose = n.advertise<geometry_msgs::PoseStamped>("header and pose", 1);

    geometry_msgs::Twist angular_linear_data;
    geometry_msgs::PoseStamped header_pose_data;

    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;

    std_msgs::Header header;
    geometry_msgs::Pose pose;

    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(500);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(460800);
    sp.setTimeout(to);
 
    try
    {
        sp.open();
    }

    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    
    int rate_hz = 30;
    ros::Rate loop_rate(rate_hz);
    ros::Time init_time = ros::Time::now();
    float last_time = 0;
    int found_position;
    int seq = 0;


    float dt = 0;
    while(ros::ok())
    {
        float now_time = (float)((ros::Time::now() - init_time).toSec());
        dt = now_time - last_time;
        last_time = now_time;
        uint8_t buffer[MAX_BUFFER_SIZE];
        size_t n = sp.available();

        if (n >= (MAX_BUFFER_SIZE * 2))
		{
			sp.read(buffer,MAX_BUFFER_SIZE * 2);
            found_position = std::find(std::begin(buffer), std::end(buffer), BEGIN_LINE_UNIT) - std::begin(buffer);
            if (buffer[found_position + MAX_BUFFER_SIZE - 1] == END_LINE_UNIT)
            {                
                implementAsciiData(buffer + found_position);
                //std::cout << "Goc deg: " << imu.roll << "\t" << imu.pitch << "\t" << imu.yaw << std::endl;
            }
            sp.flushInput();
		}

        /*calculate velocity
        imu_data.vel[0] += imu_data.acc[0] * dt;
        imu_data.vel[1] += imu_data.acc[1] * dt;
        imu_data.vel[2] += imu_data.acc[2] * dt;*/

        /*Compare with current orientation?*/

        /*calculate displacement
        imu_data.pos[0] += imu_data.vel[0] * dt + 0.5 * imu_data.acc[0] * dt * dt;
        imu_data.pos[1] += imu_data.vel[1] * dt + 0.5 * imu_data.acc[1] * dt * dt;
        imu_data.pos[2] += imu_data.vel[2] * dt + 0.5 * imu_data.acc[2] * dt * dt;*/

        /*put data into object Twist of geometry_msgs*/
        //linear acceleration vector3
        linear.x = imu_data.acc[0];
        linear.y = imu_data.acc[1];
        linear.z = imu_data.acc[2];
        
        imu_data.pos[0] = hand_palm_position[0];
        imu_data.pos[1] = hand_palm_position[1];
        imu_data.pos[2] = hand_palm_position[2];

        //angular vector3
        angular.x = imu_data.ang[0];
        angular.y = imu_data.ang[1];
        angular.z = imu_data.ang[2];
    
        //Twist
        angular_linear_data.angular = angular;
        angular_linear_data.linear = linear;

        /*put data into object PostStamped of geometry_msgs*/
        //Header
        header.stamp = ros::Time::now();
        header.seq = seq++;
        header.frame_id = "";

        //Pose - position
        pose.position.x = imu_data.pos[0];
        pose.position.y = imu_data.pos[1];
        pose.position.z = imu_data.pos[2];

        //Pose - orientation
        pose.orientation.w = imu_data.quat[0];
        pose.orientation.x = imu_data.quat[1];
        pose.orientation.y = imu_data.quat[2];
        pose.orientation.z = imu_data.quat[3];

        /*publish data*/
        publish_angular_linear.publish(angular_linear_data);
        publish_pose.publish(header_pose_data);


        /**/
        loop_rate.sleep();
        
    }

    sp.close();
    return 0;
}

float convertAsciiToFloat(uint8_t* buffer)
{
    // sign - 100 - 10 - 1 - 0.1 - -0.01
    float final = (buffer[1] - 0x30) * 100;
    final += (buffer[2] - 0x30) * 10;
    final += (buffer[3] - 0x30) * 1;
    final += (buffer[5] - 0x30) * 0.1;
    final += (buffer[6] - 0x30) * 0.01;
    if (buffer[0] == '-')
    {
        final = - final;
    }
    return final;
}

float convertAsciiQuaternionToFloat(uint8_t* buffer)
{
    // sign - 1 - 0.1 - 0.01 - 0.001 - 0.0001
    float final = (buffer[1] - 0x30);
    final += (buffer[2] - 0x30) * 0.1;
    final += (buffer[3] - 0x30) * 0.01;
    final += (buffer[5] - 0x30) * 0.001;
    final += (buffer[6] - 0x30) * 0.0001;
    if (buffer[0] == '-')
    {
        final = - final;
    }
    return final;
}

void implementAsciiData(uint8_t* input_buffer)
{
    ++input_buffer;

    //copy angle roll-pitch-yaw
    memcpy(imu_data.ang_ascii[0], input_buffer, MODE_IMU_ANGLE);
    input_buffer += (MODE_IMU_ANGLE + 1);
    memcpy(imu_data.ang_ascii[1], input_buffer, MODE_IMU_ANGLE);
    input_buffer += (MODE_IMU_ANGLE + 1);
    memcpy(imu_data.ang_ascii[2], input_buffer, MODE_IMU_ANGLE);
    input_buffer += (MODE_IMU_ANGLE + 1);

    //copy acceleration x-y-z
    memcpy(imu_data.acc_ascii[0], input_buffer, MODE_IMU_ACCEL);
    input_buffer += (MODE_IMU_ACCEL + 1);
    memcpy(imu_data.acc_ascii[1], input_buffer, MODE_IMU_ACCEL);
    input_buffer += (MODE_IMU_ACCEL + 1);
    memcpy(imu_data.acc_ascii[2], input_buffer, MODE_IMU_ACCEL);
    input_buffer += (MODE_IMU_ACCEL + 1);

    //copy quaternion  w-x-y-z
    memcpy(imu_data.quat_ascii[0], input_buffer, MODE_IMU_QUAT);
    input_buffer += (MODE_IMU_QUAT + 1);
    memcpy(imu_data.quat_ascii[1], input_buffer, MODE_IMU_QUAT);
    input_buffer += (MODE_IMU_QUAT + 1);
    memcpy(imu_data.quat_ascii[2], input_buffer, MODE_IMU_QUAT);
    input_buffer += (MODE_IMU_QUAT + 1);
    memcpy(imu_data.quat_ascii[3], input_buffer, MODE_IMU_QUAT);
    input_buffer += (MODE_IMU_QUAT + 1);


    //transform angle in ascii into float32
    imu_data.ang[0] = convertAsciiToFloat(imu_data.ang_ascii[0]);
    imu_data.ang[1] = convertAsciiToFloat(imu_data.ang_ascii[1]);
    imu_data.ang[2] = convertAsciiToFloat(imu_data.ang_ascii[2]);

    //transform acceleration in ascii into float32
    imu_data.ang[0] = convertAsciiToFloat(imu_data.acc_ascii[0]);
    imu_data.acc[1] = convertAsciiToFloat(imu_data.acc_ascii[1]);
    imu_data.acc[2] = convertAsciiToFloat(imu_data.acc_ascii[2]);

    //transform quaternion in ascii into float32
    imu_data.quat[0] = convertAsciiQuaternionToFloat(imu_data.quat_ascii[0]);
    imu_data.quat[1] = convertAsciiQuaternionToFloat(imu_data.quat_ascii[1]);
    imu_data.quat[2] = convertAsciiQuaternionToFloat(imu_data.quat_ascii[2]);
    imu_data.quat[3] = convertAsciiQuaternionToFloat(imu_data.quat_ascii[3]);
}

