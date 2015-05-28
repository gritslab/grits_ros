/**
 * @file state_machine.cpp
 *
 * @brief ROS quadrotor state machine node
 *
 * @author Rowland O'Flaherty
 *
 * @date Rowland O'Flaherty MAY 27
 *
 * @copyright Copyright (C) 2015, see LICENSE file
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <ros/ros.h>

// Messages

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define UPDATE_RATE 25

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;

//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    ROS_INFO("Running Quadrotor State Machine Node");

    ros::init(argc, argv, "state_machine");

    ros::NodeHandle nh;


}
