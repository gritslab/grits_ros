/**
 * @file planner.cpp
 *
 * @brief ROS quadrotor planner node
 *
 * @author Rowland O'Flaherty
 *
 * @date 2015 MAY 27
 *
 * @copyright Copyright (C) 2015, see LICENSE file
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <Eigen/Dense>
#include <sys/socket.h>
#include <arpa/inet.h>

// Messages
#include <geometry_msgs/Vector3.h>
#include <optitrack/PoseXYZRPY.h>


//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
typedef Matrix<double, 6, 1> Vector6d;

enum FiniteState {
    INITIALIZING = 0,
    HOLDING = 1,
    RUNNING = 2
};


//------------------------------------------------------------------------------
// Global Constants Declarations
//------------------------------------------------------------------------------
const int UPDATE_RATE = 50;
const int SERVICE_PORT = 21234;
const int BUF_SIZE = 2048;

//------------------------------------------------------------------------------
// Global Variable Declarations
//------------------------------------------------------------------------------
bool state_valid = false;
Vector6d x;
FiniteState s;

//------------------------------------------------------------------------------
// Function Declarations
//------------------------------------------------------------------------------
void set_pos_desired(const Vector3d& pos, optitrack::PoseXYZRPY& pos_desired);
int setup_udp_socket(int& fd);
void parse_buffer(unsigned char* buf, optitrack::PoseXYZRPY& pos_desired);


// Subscriber handles
void handle_poseXYZRPY(const optitrack::PoseXYZRPY& msg);
void handle_vel(const geometry_msgs::Vector3& msg);

//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    // Initialize constants
    Vector6d x0;
    // x0 << -1.25, -.4, 1.75, 0.0, 0.0, 0.0;
    // x0 << -1.15, .5, 1.3, 0.0, 0.0, 0.0;
    x0 << -1.15, .5, .5, 0.0, 0.0, 0.0;
    // x0 << -2.101, 0.0,  0.750, 0.0, 0.0, 0.0;

    double pos_ball = .1;
    double vel_ball = .05;

    // Initialize variables
    double t;
    double t0;

    s = INITIALIZING;

    optitrack::PoseXYZRPY pos_desired;
    pos_desired.yaw = 270.0 * M_PI/180.0;

    double pos_del;
    double vel_del;

    double pos_del_max = 0;
    double vel_max = 0;

    // Initialize ROS
    ROS_INFO("Running Quadrotor Planner Node");
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber pos_sub = nh.subscribe("/poseXYZRPY", 1, handle_poseXYZRPY);
    ros::Subscriber vel_sub = nh.subscribe("/vel", 1, handle_vel);

    // Publishers
    ros::Publisher pos_desired_pub;
    pos_desired_pub = nh.advertise<optitrack::PoseXYZRPY>("pos_desired",1);

    // Setup UDP socket
    unsigned char buf[BUF_SIZE];
    struct sockaddr_in remote_addr; // Address of remote computer
    socklen_t addr_len = sizeof(remote_addr);
    int fd; // socket file descriptor
    if (setup_udp_socket(fd)) {
        return 1;
    }

    // Loop
    ros::Rate loop_rate(UPDATE_RATE);
    while (ros::ok()) {
        if (state_valid) {

        // Update
            t = ros::Time::now().toSec();
            int recv_len = recvfrom(fd, buf, BUF_SIZE, 0,
                                    (struct sockaddr *)&remote_addr, &addr_len);

            pos_del = (x.head(2) - x0.head(2)).norm();
            vel_del = (x.tail(3) - x0.tail(3)).norm();

            switch (s) {

                case INITIALIZING: {
                    if (pos_del < pos_ball && vel_del < vel_ball) {
                        t0 = t;
                        s = HOLDING;
                    } else {
                        set_pos_desired(x0.head(3), pos_desired);
                    }

                    break;
                }

                case HOLDING: {
                    set_pos_desired(x0.head(3), pos_desired);

                    if (recv_len > 0) {
                        s = RUNNING;
                    }

                    break;
                }

                case RUNNING: {
                    if (recv_len > 0) {
                        parse_buffer(buf, pos_desired);
                    }

                    break;
                }
            }

        // Print
            printf("State: %d, ", s);
            printf("Pos: % 2.3f, % 2.3f, % 2.3f, ", x(0), x(1), x(2));
            printf("Pos Des: % 2.3f, % 2.3f, % 2.3f, ",
                   pos_desired.x, pos_desired.y, pos_desired.z);
            printf("Pos Del: % 2.3f, ", pos_del);
            printf("Vel: % 2.3f, ", x.tail(3).norm());
            printf("\n");

        // Publish
            pos_desired_pub.publish(pos_desired);
        }

        // Sleep
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

//------------------------------------------------------------------------------
// Function Definitions
//------------------------------------------------------------------------------
void set_pos_desired(const Vector3d& pos, optitrack::PoseXYZRPY& pos_desired)
{
    pos_desired.x = pos(0);
    pos_desired.y = pos(1);
    pos_desired.z = pos(2);
}

int setup_udp_socket(int& fd)
{
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = (int)(1e6/(double)UPDATE_RATE);

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("cannot create socket\n");
        return 1;
    }
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));

    struct sockaddr_in ros_addr;    // Address of ROS computer
    int recv_len;

    memset((char *)&ros_addr, 0, sizeof(ros_addr));
    ros_addr.sin_family = AF_INET;
    ros_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    ros_addr.sin_port = htons(SERVICE_PORT);

    if (bind(fd, (struct sockaddr *)&ros_addr, sizeof(ros_addr)) < 0) {
        perror("bind failed");
        return 1;
    }

    return 0;
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

// Subscriber handles
void handle_poseXYZRPY(const optitrack::PoseXYZRPY& msg)
{
    x(0) = msg.x;
    x(1) = msg.y;
    x(2) = msg.z;
}

void handle_vel(const geometry_msgs::Vector3& msg)
{
    x(3) = msg.x;
    x(4) = msg.y;
    x(5) = msg.z;

    state_valid = true;
}
