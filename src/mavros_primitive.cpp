#include "mavros_primitive.h"


MavrosPrimitive::MavrosPrimitive()
{    
    takeoff_client_ = nh_.serviceClient<mavros::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = nh_.serviceClient<mavros::CommandTOL>("/mavros/cmd/land");
}

MavrosPrimitive::~MavrosPrimitive()
{

}

void MavrosPrimitive::handleKeyboardInput(int character)
{
    switch(character)
    {
    case 'w':
        ROS_ERROR("send take off");
        takeOff();
    break;
    case 's':
        ROS_ERROR("send land");
        land();
    break;
    }
}

void MavrosPrimitive::takeOff()
{
    //build takeoff command ,,
    //(NOTE) just using empty to demonstrate functionality
    mavros::CommandTOL srv;    
    srv.request.min_pitch = 0.1f;
    srv.request.yaw = 0.1f;
    srv.request.latitude = 0.1f;
    srv.request.longitude= 0.1f;
    srv.request.altitude = 0.1f;

    //send
    if(takeoff_client_.call(srv))
    {
        ROS_ERROR("connected takeoff");
        //check response of the service
        //NOTE: Pixhawk may not be successful on start as you need to arm correctly before you takeoff?
        if(srv.response.success)
        {
            ROS_ERROR("Sent Takeoff %d", srv.response.result);
        }
    }
    else
    {
        ROS_ERROR("Failed to send takeoff");
    }

}

void MavrosPrimitive::land()
{
    //build land command
    //(NOTE) just using empty to demonstrate functionality
    mavros::CommandTOL srv;
    srv.request.min_pitch = 0.1f;
    srv.request.yaw = 0.1f;
    srv.request.latitude = 0.1f;
    srv.request.longitude= 0.1f;
    srv.request.altitude = 0.1f;


    //send
    if(land_client_.call(srv))
    {
        ROS_ERROR("connected land");
        if(srv.response.success)
        {
            ROS_ERROR("Sent Land %d", srv.response.result);
        }
    }
    else
    {
        ROS_ERROR("Failed to send Land");
    }

}

//read input with a non-blocking getchar from http://answers.ros.org/question/63491/keyboard-key-pressed/
//linux specific?
int MavrosPrimitive::getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavros_primitive_node");
    //run obstacle avoidance with ros
    MavrosPrimitive mp;

    ROS_ERROR("Mavros Primitive running...");

    while (ros::ok())
    {
        mp.handleKeyboardInput(mp.getch());
    }

    return 0;
}
