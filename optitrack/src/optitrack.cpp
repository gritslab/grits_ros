/**
 * @file optitrack.cpp
 *
 * @brief This is a ROS node that publishes optitrack data.
 *
 * @author Rowland O'Flaherty
 *
 * @date Rowland O'Flaherty MAY 04
 *
 * @copyright Copyright (C) Rowland O'Flaherty, see LICENSE file
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <optitrack/PoseXYZRPY.h>

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define UPDATE_RATE 100

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;

//------------------------------------------------------------------------------
// Global
//------------------------------------------------------------------------------
Vector3d raw_pos;
Quaterniond raw_quat;

//------------------------------------------------------------------------------
// Declarations
//------------------------------------------------------------------------------
void VRPN_CALLBACK update_trackable_data (void *, const vrpn_TRACKERCB t);

void quat2euler(const Quaterniond& quat,
                double &phi, double &theta, double &psi);
double wrap_to_2pi(double theta);
double wrap_to_pi(double theta);

struct Trackable {
    Quaterniond aircraft_nominal;
    Quaterniond local;
    Quaterniond global;

    Vector3d pos;
    Quaterniond quat;
    double roll, pitch, yaw;

    vrpn_Connection* connection;
    vrpn_Tracker_Remote* tracker;

    geometry_msgs::Pose raw_msg;
    ros::Publisher raw_pub;

    geometry_msgs::Pose pose_msg;
    ros::Publisher pose_pub;

    geometry_msgs::Pose2D pose2D_msg;
    ros::Publisher pose2D_pub;

    optitrack::PoseXYZRPY poseXYZRPY_msg;
    ros::Publisher poseXYZRPY_pub;

    Trackable(ros::NodeHandle& nh, string server_ip)
    :
    aircraft_nominal(0, sqrt(.5), sqrt(.5), 0),
    local(sqrt(.5), sqrt(.5), 0, 0),
    global(-sqrt(.5), sqrt(.5), 0, 0)
    {
        string connection_str = server_ip + ":" + "3883";
        connection = vrpn_get_connection_by_name(connection_str.c_str());

        string trackable_name = "quad1";
        tracker = new vrpn_Tracker_Remote(trackable_name.c_str(), connection);
        tracker->register_change_handler(NULL, update_trackable_data);

        raw_pub = nh.advertise<geometry_msgs::Pose>("raw",1);
        pose_pub = nh.advertise<geometry_msgs::Pose>("pose",1);
        pose2D_pub = nh.advertise<geometry_msgs::Pose2D>("pose2D",1);
        poseXYZRPY_pub = nh.advertise<optitrack::PoseXYZRPY>("poseXYZRPY",1);
    }

    void update()
    {
        tracker->mainloop();
        connection->mainloop();

        pos = global.inverse()*raw_pos;
        quat = aircraft_nominal.inverse()*(global.inverse()*raw_quat)*local;

        quat2euler(quat, roll, pitch, yaw);
    }

    void publish()
    {
        // Raw
        raw_msg.position.x = raw_pos(0);
        raw_msg.position.y = raw_pos(1);
        raw_msg.position.z = raw_pos(2);

        raw_msg.orientation.w = raw_quat.w();
        raw_msg.orientation.x = raw_quat.x();
        raw_msg.orientation.y = raw_quat.y();
        raw_msg.orientation.z = raw_quat.z();

        raw_pub.publish(raw_msg);

        // Pose
        pose_msg.position.x = pos(0);
        pose_msg.position.y = pos(1);
        pose_msg.position.z = pos(2);

        pose_msg.orientation.w = quat.w();
        pose_msg.orientation.x = quat.x();
        pose_msg.orientation.y = quat.y();
        pose_msg.orientation.z = quat.z();

        pose_pub.publish(pose_msg);

        // Pose2D
        pose2D_msg.x = pos(0);
        pose2D_msg.y = pos(1);
        pose2D_msg.theta = wrap_to_pi(M_PI/2 - yaw);

        pose2D_pub.publish(pose2D_msg);

        // Pose XYZ RPY
        poseXYZRPY_msg.x = pos(0);
        poseXYZRPY_msg.y = pos(1);
        poseXYZRPY_msg.z = pos(2);

        poseXYZRPY_msg.roll = roll;
        poseXYZRPY_msg.pitch = pitch;
        poseXYZRPY_msg.yaw = wrap_to_2pi(yaw);

        poseXYZRPY_pub.publish(poseXYZRPY_msg);
    }
};

//------------------------------------------------------------------------------
// Function Definitions
//------------------------------------------------------------------------------
void VRPN_CALLBACK update_trackable_data(void *, const vrpn_TRACKERCB t)
{
    raw_pos << t.pos[0], t.pos[1], t.pos[2];
    raw_quat.w() = t.quat[3];
    raw_quat.x() = t.quat[0];
    raw_quat.y() = t.quat[1];
    raw_quat.z() = t.quat[2];
}

void quat2euler(const Quaterniond& quat,
                double &phi, double &theta, double &psi)
{
    double r = quat.w();
    double i = quat.x();
    double j = quat.y();
    double k = quat.z();

    phi = atan2(2*(r*i + j*k), 1 - 2*(pow(i, 2) + pow(j, 2)));
    theta = asin(2*(r*j - k*i));
    psi = atan2(2*(r*k + i*j), 1 - 2*(pow(j, 2) + pow(k, 2)));
}

double wrap_to_2pi(double theta)
{
    if (theta >= 0) {
        return fmod(theta, 2*M_PI);
    } else {
        return 2*M_PI + fmod(theta, 2*M_PI);
    }
}

double wrap_to_pi(double theta)
{
    return wrap_to_2pi(theta + M_PI) - M_PI;
}

//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    ROS_INFO("Running optitrack node");

    ros::init(argc, argv, "optitrack");

    ros::NodeHandle node_handle;

    string vrpn_server_ip = "192.168.2.145";

    Trackable trackable(node_handle, vrpn_server_ip);

    ros::Rate loop_rate(UPDATE_RATE);

    while(ros::ok())
    {
        trackable.update();
        trackable.publish();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
