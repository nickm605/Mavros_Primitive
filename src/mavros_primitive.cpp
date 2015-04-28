#include "mavros_primitive.h"

mavros::WaypointListPtr wpl = boost::make_shared<mavros::WaypointList>();
gps current_gps;
bool reachedLoiter;
double x_tolerance;
double y_tolerance;

double testAltitude;

static FILE* myfile;

bool first_call;

MavrosPrimitive::MavrosPrimitive()
{    
    //waypoint_clear_client_ = nh_.serviceClient<mavros::WaypointPull>("/mavros/mission/clear");
    waypoint_pull_client_ = nh_.serviceClient<mavros::WaypointPull>("/mavros/mission/pull");
    waypoint_push_client_ = nh_.serviceClient<mavros::WaypointPush>("/mavros/mission/push");
    waypoint_set_current_client_ = nh_.serviceClient<mavros::WaypointSetCurrent>("/mavros/mission/set_current");
}

MavrosPrimitive::~MavrosPrimitive()
{

}

int main(int argc, char** argv)
{
    std::string fileID;
    if(argc == 4) {
        std::string s(argv[1]);
        fileID = s;
    }
    else {
        fileID = "default";
    }
    std::string fileLocation = "/home/odroid/catkin_ws/src/ar_track_alvar/ar_track_alvar/flight_log-" + fileID + ".txt";

    ros::init(argc, argv, "mavros_primitive_node");
     
    current_gps.latitude = 37.212738;
    current_gps.longitude = -80.438042;

    reachedLoiter = false;

    first_call = true;

    x_tolerance = 0.1;
    y_tolerance = 0.1;

    testAltitude = 10.0;

    myfile = fopen(fileLocation.c_str(), "w");

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
    waypoint_pull_client_.call(srv);
}

void MavrosPrimitive::waypointListCallback(const mavros::WaypointList::ConstPtr& msg) {

    fprintf(myfile, "\n\n%d waypoints\n", msg->waypoints.size());

    for(int i = 0; i < msg->waypoints.size(); i++) {

        fprintf(myfile, "Waypoint (%d) frame: %d command: %d is current: %d lat: %f long: %f\n", i, msg->waypoints[i].frame, msg->waypoints[i].command, msg->waypoints[i].is_current, msg->waypoints[i].x_lat,  msg->waypoints[i].y_long);
    }
    
    if(first_call) {

        first_call = false;
        MavrosPrimitive mp;
        mp.load_initial_mission();
        return;
    }

    if(!reachedLoiter) {
        if(msg->waypoints[msg->waypoints.size() - 1].is_current &&
	   msg->waypoints[msg->waypoints.size() - 1].command == 17) {

	    fprintf(myfile, "\n--------------\nReached loiter\n--------------\n");
            reachedLoiter = true;
        }
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
    wpl->waypoints.clear();

    fprintf(myfile, "\n\n----------------------\nLoading initial mission\n----------------------\n\n");

    //home
    mavros::Waypoint wp_home;
    wp_home.x_lat = current_gps.latitude;
    wp_home.y_long = current_gps.longitude;
    wp_home.z_alt = testAltitude;
    wp_home.command = 16;
    wp_home.frame = 3;
    wpl->waypoints.push_back(wp_home);

    //takeoff
    mavros::Waypoint wp_takeoff;
    wp_takeoff.x_lat = current_gps.latitude;
    wp_takeoff.y_long = current_gps.longitude;
    wp_takeoff.z_alt = testAltitude;
    wp_takeoff.command = 22;
    wp_takeoff.frame = 3;
    wpl->waypoints.push_back(wp_takeoff);

    //move to initial waypoint
    mavros::Waypoint wp_neighborhood;
    wp_neighborhood.x_lat = current_gps.latitude;
    wp_neighborhood.y_long = current_gps.longitude;
    wp_neighborhood.z_alt = testAltitude;
    wp_neighborhood.command = 16;
    wp_neighborhood.frame = 3;
    wpl->waypoints.push_back(wp_neighborhood);

    //loiter unlimited
    mavros::Waypoint wp_loiter;
    wp_loiter.x_lat = current_gps.latitude;
    wp_loiter.y_long = current_gps.longitude;
    wp_loiter.z_alt = testAltitude;
    wp_loiter.command = 17;
    wp_loiter.frame = 3;
    wpl->waypoints.push_back(wp_loiter);

    mavros::WaypointPush push_srv;
    push_srv.request.waypoints = wpl->waypoints;
    waypoint_push_client_.call(push_srv);
}

/*
void MavrosPrimitive::set_waypoint()
{

    // Go to new mission item
    mavros::WaypointSetCurrent set_current_srv;
    set_current_srv.request.wp_seq = 2;
    if(waypoint_set_current_client_.call(set_current_srv)) {

        if(!set_current_srv.response.success) {

            ROS_ERROR("FAILED");
        }
        else {
            ROS_ERROR("Success");
        }
    }
}
*/

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
    if(fabs(x) < x_tolerance && fabs(y) < y_tolerance) {

        MavrosPrimitive mp;
        mp.load_end_of_mission();
        return;
    }

    float invert_y = -1 * y;
    gps new_gps = offsetToGPSWaypoint(x, invert_y, current_gps, 0);
    ROS_INFO("New waypoint from AR tag calculation: Lat: %0.8f Long: %0.8f", new_gps.latitude, new_gps.longitude);
    fprintf(myfile, "\nNew waypoint from AR tag calculation: Lat: %0.8f Long: %0.8f", new_gps.latitude, new_gps.longitude);

    uint16_t index = wpl->waypoints.size();

    current_gps = new_gps;

    /*
    //home
    mavros::Waypoint wp_home;
    wp_home.x_lat = current_gps.latitude;
    wp_home.y_long = current_gps.longitude;
    wp_home.z_alt = testAltitude;
    wp_home.command = 16;
    wp_home.frame = 3;
    wpl->waypoints.push_back(wp_home);
    */

    //move to new waypoint
    mavros::Waypoint wp_ar;
    wp_ar.x_lat = current_gps.latitude;
    wp_ar.y_long = current_gps.longitude;
    wp_ar.z_alt = testAltitude;
    wp_ar.command = 16;
    wp_ar.frame = 3;
    wpl->waypoints.push_back(wp_ar);

    //loiter unlimited
    mavros::Waypoint wp_loiter;
    wp_loiter.x_lat = current_gps.latitude;
    wp_loiter.y_long = current_gps.longitude;
    wp_loiter.z_alt = testAltitude;
    wp_loiter.command = 17;
    wp_loiter.frame = 3;
    wpl->waypoints.push_back(wp_loiter);

    // Push waypoint list
    mavros::WaypointPush push_srv;
    push_srv.request.waypoints = wpl->waypoints;
    waypoint_push_client_.call(push_srv);

    // Go to new mission item
    mavros::WaypointSetCurrent set_current_srv;
    set_current_srv.request.wp_seq = index;
    waypoint_set_current_client_.call(set_current_srv);
}

gps MavrosPrimitive::offsetToGPSWaypoint(double x, double y, gps current_gps, double yaw) {

    double pi = 3.14159265359;

    double longitude_length = cos(2*pi / 360 * current_gps.latitude) * 111325;
    double latitude_length = 111325;

    double lon_adjustment_raw = cos(yaw)*x + sin(yaw)*y;
    double lat_adjustment_raw = -1*sin(yaw)*x + cos(yaw)*y;

    double lon_adjustment = 0.5 * lon_adjustment_raw / longitude_length;
    double lat_adjustment = 0.5 * lat_adjustment_raw / latitude_length;

    gps gps_returned;
    gps_returned.longitude = current_gps.longitude + lon_adjustment;
    gps_returned.latitude = current_gps.latitude + lat_adjustment;

    return gps_returned;
}

void MavrosPrimitive::load_end_of_mission()
{
    fprintf(myfile, "\n\n----------------------\nLoading end of mission\n----------------------\n");

    uint16_t index = wpl->waypoints.size();

    /*
    //home
    mavros::Waypoint wp_home;
    wp_home.x_lat = current_gps.latitude;
    wp_home.y_long = current_gps.longitude;
    wp_home.z_alt = testAltitude;
    wp_home.command = 16;
    wp_home.frame = 3;
    wpl->waypoints.push_back(wp_home);
    */

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
    wp_takeoff.z_alt = testAltitude;
    wp_takeoff.command = 22;
    wp_takeoff.frame = 3;
    wpl->waypoints.push_back(wp_takeoff);

    //rtl
    mavros::Waypoint wp_rtl;
    wp_rtl.command = 20;
    wp_rtl.frame = 3;
    wpl->waypoints.push_back(wp_rtl);

    // Push new waypoint list
    mavros::WaypointPush push_srv;
    push_srv.request.waypoints = wpl->waypoints;
    waypoint_push_client_.call(push_srv);

    // Go to new mission item
    mavros::WaypointSetCurrent set_current_srv;
    set_current_srv.request.wp_seq = index;
    waypoint_set_current_client_.call(set_current_srv);
}

