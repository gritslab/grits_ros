/**
 * @file quadrotor.cpp
 *
 * @brief Quadrotor ROS node
 *
 * @author Rowland O'Flaherty
 *
 * @date 2015 APR 22
 *
 * @copyright Copyright (C) 2015, see LICENSE file
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <ros/ros.h>

#include <fcntl.h>
#include <termios.h>
#include <math.h>

#include <geometry_msgs/TransformStamped.h>

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define UPDATE_RATE 100

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
enum MsgType {
    POSITION = 1,
    POSITION_DESIRED = 2
};

// size = 4
struct MsgHeader {
    uint8_t marker;
    uint8_t parity;
    uint8_t target;
    uint8_t type;
};
static uint8_t PARITY_BYTE_IND = 1;

// size 4+4*4 = 20
struct MsgPosition {
    struct MsgHeader header;
    float x;
    float y;
    float z;
    float yaw;
} pos;

// size 4+4*4 = 20
struct MsgPositionDesired {
    struct MsgHeader header;
    float x;
    float y;
    float z;
    float yaw;
};

//------------------------------------------------------------------------------
// Function Declarations
//------------------------------------------------------------------------------
void handle_quad_pose(const geometry_msgs::TransformStamped& msg);
uint8_t parityCalc(uint8_t* msg, uint8_t parityByteInd);
int set_interface_attribs(int fd, int speed, int parity);
void set_blocking(int fd, int should_block);

//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    ROS_INFO("Running Quad Node");

    ros::init(argc, argv, "quadrotor");

    ros::NodeHandle node_handle;

    ros::Subscriber sub_quad_pose;
    sub_quad_pose = node_handle.subscribe("/quad1/pose",
                                          1,
                                          handle_quad_pose);

    // Setup serial comms
    string portname = "/dev/ttyUSB0";
    memset(&pos, 0, sizeof(pos));
    pos.header.marker = 0xFF;
    pos.header.type = POSITION;
    pos.header.parity = parityCalc((uint8_t*)&pos, sizeof(pos));

    // MsgPositionDesired posDesired;
    // memset(&posDesired, 0, sizeof(posDesired));
    // posDesired.header.marker = 0xFF;
    // posDesired.header.type = POSITION_DESIRED;
    // posDesired.x = 0;
    // posDesired.y = 0;
    // posDesired.z = 0;
    // posDesired.header.parity = parityCalc((uint8_t*)&posDesired, sizeof(posDesired));

    ROS_INFO("Opening port: %s", portname.c_str());
    int fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        ROS_INFO("Unable to open port");
        return 0;
    }

    set_interface_attribs(fd, B57600, 0); // set speed to 57,600 bps, 8n1 (no parity)
    set_blocking(fd, 0); // set no blocking

    ros::Time t;

    // Loop
    ros::Rate loop_rate(UPDATE_RATE);
    while (ros::ok()) {
        ROS_INFO("Sending: %2.3f, %2.3f, %2.3f, %2.3f", pos.x, pos.y, pos.z, pos.yaw);
        write(fd, &pos, sizeof(pos));
        // write(fd, &posDesired, sizeof(posDesired));
        loop_rate.sleep();
        ros::spinOnce();
    }
    close(fd);
    return 0;
}

//------------------------------------------------------------------------------
// Function Definitions
//------------------------------------------------------------------------------
void handle_quad_pose(const geometry_msgs::TransformStamped& msg)
{
    pos.x = msg.transform.translation.x;
    pos.y = msg.transform.translation.y;
    pos.z = msg.transform.translation.z;

    float q0, q1, q2, q3;
    q0 = msg.transform.rotation.x;
    q1 = msg.transform.rotation.y;
    q2 = msg.transform.rotation.z;
    q3 = msg.transform.rotation.w;

    pos.yaw = 180*atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3))/M_PI;
    pos.header.parity = parityCalc((uint8_t*)&pos, sizeof(pos));
}

uint8_t parityCalc(uint8_t* msg, uint8_t msgSize)
{
    uint8_t parityByte = 0;
    for (uint8_t i=PARITY_BYTE_IND+1; i<msgSize; ++i) {
        parityByte ^= msg[i];
    }
    return parityByte;
}

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                cout << "error from tcgetattr" << endl;
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                cout << "error from tcsetattr" << endl;
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                cout << "error from tggetattr" << endl;
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                cout << "error setting term attributes" << endl;
}
