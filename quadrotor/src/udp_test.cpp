/**
* @file udp_test.cpp
 *
 * @brief UDP test node
 *
 * @author Rowland O'Flaherty
 *
 * @date 2015 MAY 29
 *
 * @copyright Copyright (C) Rowland O'Flaherty, see LICENSE file
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <sys/socket.h>
#include <arpa/inet.h>

// Messages
#include <std_msgs/String.h>
#include <optitrack/PoseXYZRPY.h>


//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Global Constants Declarations
//------------------------------------------------------------------------------
const int UPDATE_RATE = 40;
const int SERVICE_PORT = 21234;
const int BUF_SIZE = 2048;

//------------------------------------------------------------------------------
// Function Declarations
//------------------------------------------------------------------------------
void parse_buffer(unsigned char* buf, optitrack::PoseXYZRPY& pos_desired);

//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    // Initialize ROS
    ROS_INFO("Running UDP Test Node");
    ros::init(argc, argv, "udp_test");
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher data_pub;
    data_pub = nh.advertise<optitrack::PoseXYZRPY>("udp_data",1);
    optitrack::PoseXYZRPY pos_desired;

    // Setup UDP socket
    unsigned char buf[BUF_SIZE];
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = (int)(1e6/(double)UPDATE_RATE);

    int fd; // socket file descriptor
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("cannot create socket\n");
        return 0;
    }
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));

    struct sockaddr_in ros_addr;    // Address of ROS computer
    struct sockaddr_in remote_addr; // Address of remote computer
    socklen_t addr_len = sizeof(remote_addr);
    int recv_len;

    memset((char *)&ros_addr, 0, sizeof(ros_addr));
    ros_addr.sin_family = AF_INET;
    ros_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    ros_addr.sin_port = htons(SERVICE_PORT);

    if (bind(fd, (struct sockaddr *)&ros_addr, sizeof(ros_addr)) < 0) {
        perror("bind failed");
        return 0;
    }

    // Loop
    ros::Rate loop_rate(UPDATE_RATE);
    while (ros::ok()) {
        // Update
        int recv_len = recvfrom(fd, buf, BUF_SIZE, 0,
                           (struct sockaddr *)&remote_addr, &addr_len);
        // Publish
        if (recv_len > 0) {
            // Parse
            parse_buffer(buf, pos_desired);
            data_pub.publish(pos_desired);

            // buf[recv_len] = '\0';
            // msg.data = std::string(reinterpret_cast<const char*>(buf));
            // data_pub.publish(msg);
        }


        // Sleep
        loop_rate.sleep();
        ros::spinOnce();

    }
}

void parse_buffer(unsigned char* buf, optitrack::PoseXYZRPY& pos_desired)
{
    std::string msg(reinterpret_cast<const char*>(buf));
    std::string::size_type sz = 0;

    if (msg[0] == '$') {
        // x
        msg = msg.substr(sz+1);
        pos_desired.x = stof(msg, &sz);

        // y
        msg = msg.substr(sz+1);
        pos_desired.y = stof(msg, &sz);

        // z
        msg = msg.substr(sz+1);
        pos_desired.z = stof(msg, &sz);

        // yaw
        msg = msg.substr(sz+1);
        pos_desired.yaw = stof(msg, &sz);
    }
}
