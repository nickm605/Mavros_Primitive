#ifndef MAVROS_PRIMITIVE
#define MAVROS_PRIMITIVE

#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <mavros/CommandTOL.h>
#include <mavros/Waypoint.h>
#include <mavros/WaypointClear.h>
#include <mavros/WaypointPull.h>
#include <mavros/WaypointPush.h>
#include <mavros/WaypointList.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <string>

struct gps {
    double latitude;
    double longitude;
};

/**
 * Demonstration of onboard control with the Odroid U3 and Pixhawk
 * Utilizes ROS services to relay commands to autopilot
 * Loads initial mission and updates position based on AR tags
 *
 * Brian Wright 2-2-15
 * Nicholas Montgomery 4-13-15
 */

class MavrosPrimitive
{
public:
    MavrosPrimitive();
    ~MavrosPrimitive();

    static void waypointListCallback(const mavros::WaypointList::ConstPtr& msg);
    static void arTagCallback(const  geometry_msgs::PoseStamped::ConstPtr& msg);
    //void clear_waypoints();
    void get_waypoints();
    void load_initial_mission();
    void load_ar_tag_waypoint(float x, float y);
    gps offsetToGPSWaypoint(double x, double y, gps current_gps, double yaw);
    void load_end_of_mission();

    static void *runInSeparateThread(void*);

private:
    ros::NodeHandle nh_;
    ros::ServiceClient waypoint_clear_client_;
    ros::ServiceClient waypoint_pull_client_;
    ros::ServiceClient waypoint_push_client_;

    
};

#endif //MAVROS_PRIMITIVE
