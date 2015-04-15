#include "mavros_primitive.h"

mavros::WaypointListPtr wpl = boost::make_shared<mavros::WaypointList>();
gps current_gps;
bool reachedLoiter;
double lat_tolerance;
double long_tolerance;

static FILE* myfile;

MavrosPrimitive::MavrosPrimitive()
{    
    waypoint_clear_client_ = nh_.serviceClient<mavros::WaypointPull>("/mavros/mission/clear");
    waypoint_pull_client_ = nh_.serviceClient<mavros::WaypointPull>("/mavros/mission/pull");
    waypoint_push_client_ = nh_.serviceClient<mavros::WaypointPush>("/mavros/mission/push");
    
    current_gps.latitude = 37.212738;
    current_gps.longitude = -80.438042;

    reachedLoiter = false;

    lat_tolerance = 0.000001;
    long_tolerance = 0.000001;

    myfile = fopen("/home/odroid/catkin_ws/src/ar_track_alvar/ar_track_alvar/flight_log.txt", "a");
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

    //spin separate thread
    pthread_t thread;
    pthread_create(&thread, NULL, &MavrosPrimitive::runInSeparateThread, NULL);

    ros::spin();

    return 0;
}

void *MavrosPrimitive::runInSeparateThread(void*) {

    while(true) {

        sleep(1);
	MavrosPrimitive mp;
        mp.get_waypoints();
    }
}

void MavrosPrimitive::get_waypoints()
{
    // read waypoints from Pixhawk
    mavros::WaypointPull srv;
    //send
    waypoint_pull_client_.call(srv);
}

void MavrosPrimitive::waypointListCallback(const mavros::WaypointList::ConstPtr& msg) {

    fprintf(myfile, "\n\n%d waypoints\n", msg->waypoints.size());

    for(int i = 0; i < msg->waypoints.size(); i++) {

        fprintf(myfile, "Waypoint (%d) frame: %d command: %d is current: %d lat: %f long: %f\n", i, msg->waypoints[i].frame, msg->waypoints[i].command, msg->waypoints[i].is_current, msg->waypoints[i].x_lat,  msg->waypoints[i].y_long);
    }

    if(wpl->waypoints.size() == 0) {

        MavrosPrimitive mp;
        //mp.clear_waypoints();
        mp.load_initial_mission();
        return;
    }

    if(wpl->waypoints[wpl->waypoints.size() - 1].is_current) {

        reachedLoiter = true;
    }
}

/*
void MavrosPrimitive::clear_waypoints()
{
    // clear waypoints from Pixhawk
    mavros::WaypointClear srv;
    //send
    waypoint_clear_client_.call(srv)
}
*/

void MavrosPrimitive::load_initial_mission()
{
    mavros::WaypointPush srv;

    //home
    mavros::Waypoint wp_home;
    wpl->waypoints.push_back(wp_home);

    //takeoff
    mavros::Waypoint wp_takeoff;
    wp_takeoff.x_lat = current_gps.latitude;
    wp_takeoff.y_long = current_gps.longitude;
    wp_takeoff.z_alt = 10.0;
    wp_takeoff.command = 22;
    wp_takeoff.frame = 3;
    wpl->waypoints.push_back(wp_takeoff);

    //move to initial waypoint
    mavros::Waypoint wp_neighborhood;
    wp_neighborhood.x_lat = current_gps.latitude;
    wp_neighborhood.y_long = current_gps.longitude;
    wp_neighborhood.z_alt = 10.0;
    wp_neighborhood.command = 16;
    wp_neighborhood.frame = 3;
    wpl->waypoints.push_back(wp_neighborhood);

    //loiter unlimited
    mavros::Waypoint wp_loiter;
    wp_loiter.x_lat = current_gps.latitude;
    wp_loiter.y_long = current_gps.longitude;
    wp_loiter.z_alt = 10.0;
    wp_loiter.command = 17;
    wp_loiter.frame = 3;
    wpl->waypoints.push_back(wp_loiter);

    srv.request.waypoints = wpl->waypoints;

    //send
    waypoint_push_client_.call(srv);
}

void MavrosPrimitive::arTagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    ROS_INFO("Received x: %f y: %f", msg->pose.position.x, msg->pose.position.y);
    fprintf(myfile, "\n\nReceived x: %f y: %f", msg->pose.position.x, msg->pose.position.y);

    if(reachedLoiter) {

        reachedLoiter = false;
        MavrosPrimitive mp;
        mp.load_ar_tag_waypoint(msg->pose.position.x, msg->pose.position.y);
    }
}

void MavrosPrimitive::load_ar_tag_waypoint(float x, float y)
{
    wpl->waypoints.clear();
    mavros::WaypointPush srv;

    gps new_gps = offsetToGPSWaypoint(x, y, current_gps, 0);
    ROS_INFO("New waypoint from ar tag calculation: Lat: %0.8f Long: %0.8f", new_gps.latitude, new_gps.longitude);

    if(fabs(new_gps.latitude - current_gps.latitude) < lat_tolerance && fabs(new_gps.longitude - current_gps.longitude) < long_tolerance) {

        MavrosPrimitive mp;
        mp.load_end_of_mission();
        return;
    }

    current_gps = new_gps;

    //home
    mavros::Waypoint wp_home;
    wp_home.x_lat = current_gps.latitude;
    wp_home.y_long = current_gps.longitude;
    wp_home.z_alt = 10.0;
    wp_home.command = 16;
    wp_home.frame = 3;
    wpl->waypoints.push_back(wp_home);

    //move to new waypoint
    mavros::Waypoint wp_ar;
    wp_ar.x_lat = current_gps.latitude;
    wp_ar.y_long = current_gps.longitude;
    wp_ar.z_alt = 10.0;
    wp_ar.command = 16;
    wp_ar.frame = 3;
    wpl->waypoints.push_back(wp_ar);

    //loiter unlimited
    mavros::Waypoint wp_loiter;
    wp_loiter.x_lat = current_gps.latitude;
    wp_loiter.y_long = current_gps.longitude;
    wp_loiter.z_alt = 10.0;
    wp_loiter.command = 17;
    wp_loiter.frame = 3;
    wpl->waypoints.push_back(wp_loiter);

    srv.request.waypoints = wpl->waypoints;
    //send
    waypoint_push_client_.call(srv);
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

void MavrosPrimitive::load_end_of_mission()
{
    mavros::WaypointPush srv;

    //home
    mavros::Waypoint wp_home;
    wp_home.x_lat = current_gps.latitude;
    wp_home.y_long = current_gps.longitude;
    wp_home.z_alt = 10.0;
    wp_home.command = 16;
    wp_home.frame = 3;
    wpl->waypoints.push_back(wp_home);

    //land
    mavros::Waypoint wp_land;
    wp_land.x_lat = current_gps.latitude;
    wp_land.y_long = current_gps.longitude;
    wp_land.z_alt = 0.0;
    wp_land.command = 21;
    wp_land.frame = 3;
    wpl->waypoints.push_back(wp_land);

    //set servo

    //takeoff
    mavros::Waypoint wp_takeoff;
    wp_takeoff.x_lat = current_gps.latitude;
    wp_takeoff.y_long = current_gps.longitude;
    wp_takeoff.z_alt = 10.0;
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
    waypoint_push_client_.call(srv);
}

/*
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
    waypoint_push_client_.call(srv);
}
*/