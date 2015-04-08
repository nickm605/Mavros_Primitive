#include "mavros_primitive.h"

mavros::WaypointListPtr wpl = boost::make_shared<mavros::WaypointList>();

MavrosPrimitive::MavrosPrimitive()
{    
    takeoff_client_ = nh_.serviceClient<mavros::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = nh_.serviceClient<mavros::CommandTOL>("/mavros/cmd/land");
    waypoint_pull_client_ = nh_.serviceClient<mavros::WaypointPull>("/mavros/mission/pull");
    waypoint_push_client_ = nh_.serviceClient<mavros::WaypointPush>("/mavros/mission/push");
}

MavrosPrimitive::~MavrosPrimitive()
{

}

void MavrosPrimitive::handleKeyboardInput(int character)
{
    switch(character)
    {
    case 'w':
        ROS_INFO("send take off");
        takeOff();
    break;
    case 's':
        ROS_INFO("send land");
        land();
    break;
    case 'g':
        ROS_INFO("Get waypoints");
        get_waypoints();
    break;
    case 'l':
        ROS_INFO("Load waypoints");
        load_waypoints();
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
            ROS_INFO("Sent Takeoff %d", srv.response.result);
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
        ROS_INFO("connected land");
        if(srv.response.success)
        {
            ROS_INFO("Sent Land %d", srv.response.result);
        }
    }
    else
    {
        ROS_ERROR("Failed to send Land");
    }

}

void MavrosPrimitive::get_waypoints()
{
    // read waypoints from Pixhawk
    mavros::WaypointPull srv;
    //send
    if(waypoint_pull_client_.call(srv))
    {
        ROS_INFO("Getting waypoints");
        if(srv.response.success)
        {
            ROS_INFO("Number of waypoints received: %d", srv.response.wp_received);
        }
    }
    else
    {
        ROS_ERROR("Failed to get waypoints");
    }

}

void MavrosPrimitive::load_waypoints()
{
    // read waypoints from Pixhawk
    mavros::WaypointPush srv;

    mavros::Waypoint wp;
    wp.x_lat = wpl->waypoints[0].x_lat + 0.0001;
    wp.y_long = wpl->waypoints[0].y_long;
    wp.z_alt = 15.0;
    wpl->waypoints.push_back(wp);
    srv.request.waypoints = wpl->waypoints;
    //send
    if(waypoint_push_client_.call(srv))
    {
        ROS_INFO("Pushing waypoints");
        if(srv.response.success)
        {
            ROS_INFO("Number of waypoints transferred: %d", srv.response.wp_transfered);
        }
    }
    else
    {
        ROS_ERROR("Failed to push waypoints");
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

void MavrosPrimitive::chatterCallback(const mavros::WaypointList::ConstPtr& msg) {

    ROS_INFO("HERE");
    
    ROS_INFO("Received %d waypoints", msg->waypoints.size());

    for(int i = 0; i < msg->waypoints.size(); i++) {

        ROS_INFO("Waypoint (%d) is current: %d x_lat: %f", i, msg->waypoints[i].is_current, msg->waypoints[i].x_lat);
    }

    wpl->waypoints = msg->waypoints;

    MavrosPrimitive mp;

    mp.load_waypoints();

    ROS_INFO("send land");
    mp.land();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavros_primitive_node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/mavros/mission/waypoints", 1000, MavrosPrimitive::chatterCallback);

    ros::spin();

   

    //run obstacle avoidance with ros
/*
    
    ROS_INFO("Mavros Primitive starting...");

    while (ros::ok())
    {
        mp.handleKeyboardInput(mp.getch());
    }
*/
    return 0;
}