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
    RUNNING = 1
};


//------------------------------------------------------------------------------
// Global Constants Declarations
//------------------------------------------------------------------------------
const int UPDATE_RATE = 25;

//------------------------------------------------------------------------------
// Global Variable Declarations
//------------------------------------------------------------------------------
// States
Vector6d x;
FiniteState s;

//------------------------------------------------------------------------------
// Function Declarations
//------------------------------------------------------------------------------
void set_pos_desired(const Vector3d& pos, optitrack::PoseXYZRPY& pos_desired);

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
    x0 << 1.0, 0.0, 1.5, 0.0, 0.0, 0.0;

    double move_dist = 1.0;   // meters
    double move_period = 30; // seconds

    double pos_ball = .05;
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

    // Loop
    ros::Rate loop_rate(UPDATE_RATE);
    while (ros::ok()) {

        // Update
        t = ros::Time::now().toSec();

        pos_del = (x.head(3) - x0.head(3)).norm();
        vel_del = (x.tail(3) - x0.tail(3)).norm();

        switch (s) {

            case INITIALIZING: {
                if (pos_del < pos_ball && vel_del < vel_ball) {
                    t0 = t;
                    s = RUNNING;
                } else {
                    set_pos_desired(x0.head(3), pos_desired);
                }

                printf("State: %d, ", s);
                printf("Pos: % 2.3f, % 2.3f, % 2.3f, ", x(0), x(1), x(2));
                printf("Pos Del: % 2.3f, ", pos_del);
                printf("Vel: % 2.3f, % 2.3f, % 2.3f, ", x(3), x(4), x(5));
                printf("Vel Del: % 2.3f, ", vel_del);
                printf("\n");

                break;
            }

            case RUNNING: {
                Vector3d p_bar = x0.head(3);
                double t_del = t - t0;
                p_bar(0) = move_dist*cos(2*M_PI*t_del/move_period);
                set_pos_desired(p_bar, pos_desired);

                double vel = x.tail(3).norm();

                pos_del_max = pos_del > pos_del_max ? pos_del : pos_del_max;
                vel_max = vel > vel_max ? vel : vel_max;

                printf("State: %d, ", s);
                printf("Pos Bar X: % 2.3f, ", p_bar(0));
                printf("Pos Del: % 2.3f, ", pos_del);
                printf("Pos Max Del: % 2.3f, ", pos_del_max);
                printf("Vel: % 2.3f, ", vel);
                printf("Vel Del: % 2.3f, ", vel_del);
                printf("Vel Bar: % 2.3f, ", move_dist*sin(2*M_PI*t_del/move_period));
                printf("Vel Max: % 2.3f, ", vel_max);
                printf("\n");


                break;
            }
        }

        // Publish
        pos_desired_pub.publish(pos_desired);

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
}
