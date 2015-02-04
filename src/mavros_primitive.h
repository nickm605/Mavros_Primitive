#ifndef MAVROS_PRIMITIVE
#define MAVROS_PRIMITIVE

#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <mavros/CommandTOL.h>



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

    void handleKeyboardInput(int character);
    int getch();

private:

    void land();
    void takeOff();

    ros::NodeHandle nh_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient land_client_;
};

#endif //MAVROS_PRIMITIVE
