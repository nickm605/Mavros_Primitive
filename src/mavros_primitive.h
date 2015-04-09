#ifndef MAVROS_PRIMITIVE
#define MAVROS_PRIMITIVE

#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <mavros/CommandTOL.h>
#include <mavros/Waypoint.h>
#include <mavros/WaypointPull.h>
#include <mavros/WaypointPush.h>
#include <mavros/WaypointList.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"

struct gps {
    double latitude;
    double longitude;
};

/**
 * Primitive demo of publishing to pixhawk autopilot using mavros package, not for actual flight function yet
 * W to send takeoff
 * S to send Land
 *
 * Utilizes Ros services to relay commands to autopilot
 *
 * Brian Wright 2-2-15
 */

class MavrosPrimitive
{
public:
    MavrosPrimitive();
    ~MavrosPrimitive();

    //void handleKeyboardInput(int character);
    //int getch();
    static void waypointListCallback(const mavros::WaypointList::ConstPtr& msg);
    static void arTagCallback(const  geometry_msgs::PoseStamped::ConstPtr& msg);
    //void land();
    //void takeOff();
    void get_waypoints();
    void load_waypoints();
    void load_ar_tag_waypoint(float x, float y);
    gps offsetToGPSWaypoint(double x, double y, gps current_gps, double yaw);

private:
    ros::NodeHandle nh_;
    //ros::ServiceClient takeoff_client_;
    //ros::ServiceClient land_client_;
    ros::ServiceClient waypoint_pull_client_;
    ros::ServiceClient waypoint_push_client_;
};

#endif //MAVROS_PRIMITIVE
