#include "mavros_primitive.h"

mavros::WaypointListPtr wpl = boost::make_shared<mavros::WaypointList>();
gps current_gps;

MavrosPrimitive::MavrosPrimitive()
{    
    //takeoff_client_ = nh_.serviceClient<mavros::CommandTOL>("/mavros/cmd/takeoff");
    //land_client_ = nh_.serviceClient<mavros::CommandTOL>("/mavros/cmd/land");
    waypoint_pull_client_ = nh_.serviceClient<mavros::WaypointPull>("/mavros/mission/pull");
    waypoint_push_client_ = nh_.serviceClient<mavros::WaypointPush>("/mavros/mission/push");
    current_gps.latitude = -1;
}

MavrosPrimitive::~MavrosPrimitive()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavros_primitive_node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/mavros/mission/waypoints", 1000, MavrosPrimitive::waypointListCallback);

    ros::Subscriber sub2 = n.subscribe("/pose_stamped", 1000, MavrosPrimitive::arTagCallback);

    ros::spin();

    return 0;

    //run obstacle avoidance with ros
    /*
    
    ROS_INFO("Mavros Primitive starting...");

    while (ros::ok())
    {
        mp.handleKeyboardInput(mp.getch());
    }
    */
}

void MavrosPrimitive::waypointListCallback(const mavros::WaypointList::ConstPtr& msg) {

    ROS_INFO("Received %d waypoints", msg->waypoints.size());

    bool load_when_done = false;
    if(wpl->waypoints.size() == 0) {

        load_when_done = true;
    }

    for(int i = 0; i < msg->waypoints.size(); i++) {

        ROS_INFO("Waypoint (%d) frame: %d command: %d is current: %d x_lat: %f", i, msg->waypoints[i].frame, msg->waypoints[i].command, msg->waypoints[i].is_current, msg->waypoints[i].x_lat);

        if (msg->waypoints[i].is_current) {
            current_gps.latitude = msg->waypoints[i].x_lat;
            current_gps.longitude = msg->waypoints[i].y_long;
        }
    }

    wpl->waypoints = msg->waypoints;

    if(load_when_done) {

        MavrosPrimitive mp;
        mp.load_waypoints();
    }
}

void MavrosPrimitive::arTagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    
    // When do we stop? How do we wait?

    ROS_INFO("Received x: %f y: %f", msg->pose.position.x, msg->pose.position.y);

    if (wpl->waypoints.size() < 3) {

        ROS_INFO("Waypoint list doesnt match expectation. Size: %d", wpl->waypoints.size());
        return;
    }

    MavrosPrimitive mp;

    mp.load_ar_tag_waypoint(msg->pose.position.x, msg->pose.position.y);
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

void MavrosPrimitive::load_ar_tag_waypoint(float x, float y)
{
    mavros::WaypointPush srv;

    if (current_gps.latitude == -1) {

        ROS_INFO("Current GPS unknown");
        return;
    }

    gps new_gps = offsetToGPSWaypoint(x, y, current_gps, 0);
    ROS_INFO("New waypoint from ar tag calculation: Lat: %f Long: %f", new_gps.latitude, new_gps.longitude);

    mavros::Waypoint wp;
    wp.x_lat = new_gps.latitude;
    wp.y_long = new_gps.longitude;
    wp.z_alt = 15.0;
    wp.command = 16;
    wp.frame = 3;
    wpl->waypoints.push_back(wp);
    srv.request.waypoints = wpl->waypoints;
    //send
    if(waypoint_push_client_.call(srv))
    {
        ROS_INFO("Pushing with ar tag based waypoint");
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

gps MavrosPrimitive::offsetToGPSWaypoint(double x, double y, gps current_gps, double yaw) {

    double pi = 3.14159265359;

    double longitude_length = cos(2*pi / 360 * current_gps.latitude) * 111325;
    double latitude_length = 111325;

    double lon_adjustment_raw = cos(yaw)*x + sin(yaw)*y;
    double lat_adjustment_raw = -1*sin(yaw)*x + cos(yaw)*y;

    double lon_adjustment = lon_adjustment_raw / longitude_length;
    double lat_adjustment = lat_adjustment_raw / latitude_length;

    gps gps_returned;
    gps_returned.longitude = current_gps.longitude + lon_adjustment;
    gps_returned.latitude = current_gps.latitude + lat_adjustment;

    return gps_returned;
}


void MavrosPrimitive::load_waypoints()
{
    if (wpl->waypoints.size() < 3) {

        ROS_INFO("Waypoint list doesn't match expectation. Size: %d", wpl->waypoints.size());
        return;
    }

    mavros::WaypointPush srv;

    //move east
    mavros::Waypoint wp_east;
    wp_east.x_lat = wpl->waypoints[2].x_lat;
    wp_east.y_long = wpl->waypoints[2].y_long + 0.0001;
    wp_east.z_alt = 15.0;
    wp_east.command = 16;
    wp_east.frame = 3;
    wpl->waypoints.push_back(wp_east);

    //move south
    mavros::Waypoint wp_south;
    wp_south.x_lat = wpl->waypoints[2].x_lat - 0.0001;
    wp_south.y_long = wpl->waypoints[2].y_long + 0.0001;
    wp_south.z_alt = 15.0;
    wp_south.command = 16;
    wp_south.frame = 3;
    wpl->waypoints.push_back(wp_south);

    //move west
    mavros::Waypoint wp_west;
    wp_west.x_lat = wpl->waypoints[2].x_lat - 0.0001;
    wp_west.y_long = wpl->waypoints[2].y_long;
    wp_west.z_alt = 15.0;
    wp_west.command = 16;
    wp_west.frame = 3;
    wpl->waypoints.push_back(wp_west);

    //move north
    mavros::Waypoint wp_north;
    wp_north.x_lat = wpl->waypoints[2].x_lat;
    wp_north.y_long = wpl->waypoints[2].y_long;
    wp_north.z_alt = 15.0;
    wp_north.command = 16;
    wp_north.frame = 3;
    wpl->waypoints.push_back(wp_north);

    //land
    mavros::Waypoint wp_land;
    wp_land.x_lat = wpl->waypoints[6].x_lat;
    wp_land.y_long = wpl->waypoints[6].y_long;
    wp_land.z_alt = 0.0;
    wp_land.command = 21;
    wp_land.frame = 3;
    wpl->waypoints.push_back(wp_land);

    //takeoff
    mavros::Waypoint wp_takeoff;
    wp_takeoff.x_lat = wpl->waypoints[6].x_lat;
    wp_takeoff.y_long = wpl->waypoints[6].y_long;
    wp_takeoff.z_alt = 15.0;
    wp_takeoff.command = 22;
    wp_takeoff.frame = 3;
    wpl->waypoints.push_back(wp_takeoff);

    //rtl
    mavros::Waypoint wp_rtl;
    wp_rtl.command = 20;
    wp_rtl.frame = 3;
    wpl->waypoints.push_back(wp_rtl);

    srv.request.waypoints = wpl->waypoints;

    //send
    if(waypoint_push_client_.call(srv))
    {
        ROS_INFO("Pushing new waypoints");
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


/*
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
*/
/*
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
        ROS_INFO("connected takeoff");
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
*/
/*
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
*/

/*
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
*/



