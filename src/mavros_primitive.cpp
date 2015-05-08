#include "mavros_primitive.h"

// Initialize empty waypoint list (to fill on callback)
mavros::WaypointListPtr wpl = boost::make_shared<mavros::WaypointList>();

// Stoes the current GPS as it changes in flight
gps current_gps;

// Tracks if the vehicle has stabilized in a loiter position
bool reachedLoiter;
int loiterSeconds;

// Tolerance for AR tag offset to progress to next stage / lond
double ar_tolerance;

// Flight altitude
double altitude;

// Increment to adjust for each AR tag
double increment;

// Tracks whether the algorithm is in stage two (decreased altitude)
bool stage_two;

// For logging
static FILE* myfile;

// Triggers loading initial mission
bool first_call;

/*
 * MavrosPrimitive Constructor
 */
MavrosPrimitive::MavrosPrimitive()
{    
    // Initialize clients for services
    waypoint_pull_client_ = nh_.serviceClient<mavros::WaypointPull>("/mavros/mission/pull");
    waypoint_push_client_ = nh_.serviceClient<mavros::WaypointPush>("/mavros/mission/push");
    waypoint_set_current_client_ = nh_.serviceClient<mavros::WaypointSetCurrent>("/mavros/mission/set_current");
}

MavrosPrimitive::~MavrosPrimitive()
{

}

/*
 * Main method for initializing the operation
 */
int main(int argc, char** argv)
{
    // Read the unique filename from the launch parameters, if given
    std::string fileID;
    if(argc == 4) {
        std::string s(argv[1]);
        fileID = s;
    }
    else {
        fileID = "default";
    }
    // Open the log file
    std::string fileLocation = "/home/odroid/catkin_ws/src/ar_track_alvar/ar_track_alvar/flight_log-" + fileID + ".txt";

    // Initialize the ROS node
    ros::init(argc, argv, "mavros_primitive_node");
     
    // Set the initial GPS to start searching
    current_gps.latitude = 37.1021963;
    current_gps.longitude = -76.3873581;

    // Initialize the flight parameters
    reachedLoiter = false;
    loiterSeconds = 0;

    first_call = true;

    ar_tolerance = 0.5;

    altitude = 10.0;

    increment = 0.6;

    stage_two = false;

    // Open the log file
    myfile = fopen(fileLocation.c_str(), "w");

    // Begin callbacks for the waypoint list and AR tag output
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/mavros/mission/waypoints", 1000, MavrosPrimitive::waypointListCallback);

    ros::Subscriber sub2 = n.subscribe("/pose_stamped", 1000, MavrosPrimitive::arTagCallback);

    // Spin separate thread for one-second updates
    pthread_t thread;
    pthread_create(&thread, NULL, &MavrosPrimitive::runInSeparateThread, NULL);

    ros::spin();

    return 0;
}

/*
 * Runs waypoint list updates in a separate thread (to deconflict with ros::spin())
 */
void *MavrosPrimitive::runInSeparateThread(void*) {

    // Get the waypoint list every second
    while(true) {

        sleep(1);
        MavrosPrimitive mp;
        mp.get_waypoints();
    }
}

/*
 * Uses a ROS service to trigger a waypoint list update
 */
void MavrosPrimitive::get_waypoints()
{
    // Read waypoints from Pixhawk
    mavros::WaypointPull srv;
    waypoint_pull_client_.call(srv);
}

/*
 * Callback method whenever the waypoint list updates
 */
void MavrosPrimitive::waypointListCallback(const mavros::WaypointList::ConstPtr& msg) {

    // Print out the full waypoint list
    fprintf(myfile, "\n\n%d waypoints\n", msg->waypoints.size());

    for(int i = 0; i < msg->waypoints.size(); i++) {

        fprintf(myfile, "Waypoint (%d) frame: %d command: %d is current: %d lat: %f long: %f\n", i, msg->waypoints[i].frame, msg->waypoints[i].command, msg->waypoints[i].is_current, msg->waypoints[i].x_lat,  msg->waypoints[i].y_long);
    }
    
    // If this is the first time the waypoint list callback triggers, load the initial mission
    if(first_call) {

        first_call = false;
        MavrosPrimitive mp;
        mp.load_initial_mission();
        return;
    }

    // If the vehicle has not yet reached loiter
    if(!reachedLoiter) {

        // If the current command is to loiter unlimited, increase the count of seconds loitering
        if(msg->waypoints[msg->waypoints.size() - 1].is_current &&
           msg->waypoints[msg->waypoints.size() - 1].command == 17) {

            loiterSeconds++;
        }
        // Otherwise, reset the count of seconds loitering to zero
        else {

            loiterSeconds = 0;
        }

        // If the vehicle has been loitering at least three seconds, update the status to "reachedLoiter = true"
        if(loiterSeconds >= 3) {

            fprintf(myfile, "\n--------------\nReached loiter\n--------------\n");
            reachedLoiter = true;
            loiterSeconds = 0;
        }
    }
}

/*
 * Loads the initial mission to takeoff, go to initial waypoint, and loiter unlimited
 */
void MavrosPrimitive::load_initial_mission()
{
    wpl->waypoints.clear();

    fprintf(myfile, "\n\n----------------------\nLoading initial mission\n----------------------\n\n");

    // Home
    mavros::Waypoint wp_home;
    wp_home.x_lat = current_gps.latitude;
    wp_home.y_long = current_gps.longitude;
    wp_home.z_alt = altitude;
    wp_home.command = 16;
    wp_home.frame = 3;
    wpl->waypoints.push_back(wp_home);

    // Takeoff
    mavros::Waypoint wp_takeoff;
    wp_takeoff.x_lat = current_gps.latitude;
    wp_takeoff.y_long = current_gps.longitude;
    wp_takeoff.z_alt = altitude;
    wp_takeoff.command = 22;
    wp_takeoff.frame = 3;
    wpl->waypoints.push_back(wp_takeoff);

    // Move to initial waypoint
    mavros::Waypoint wp_neighborhood;
    wp_neighborhood.x_lat = current_gps.latitude;
    wp_neighborhood.y_long = current_gps.longitude;
    wp_neighborhood.z_alt = altitude;
    wp_neighborhood.command = 16;
    wp_neighborhood.frame = 3;
    wpl->waypoints.push_back(wp_neighborhood);

    // Loiter unlimited
    mavros::Waypoint wp_loiter;
    wp_loiter.x_lat = current_gps.latitude;
    wp_loiter.y_long = current_gps.longitude;
    wp_loiter.z_alt = altitude;
    wp_loiter.command = 17;
    wp_loiter.frame = 3;
    wpl->waypoints.push_back(wp_loiter);

    // Push to Pixhawk
    mavros::WaypointPush push_srv;
    push_srv.request.waypoints = wpl->waypoints;
    waypoint_push_client_.call(push_srv);
}

/*
 * Callback whenever an AR tag is recognized
 */
void MavrosPrimitive::arTagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    // Log the location of the tag
    ROS_INFO("Received x: %f y: %f", msg->pose.position.x, msg->pose.position.y);
    fprintf(myfile, "\n\nReceived x: %f y: %f", msg->pose.position.x, msg->pose.position.y);

    // If the vehicle has stabilized in loiter, use the tag to generate a new GPS waypoint
    if(reachedLoiter) {

        reachedLoiter = false;
        MavrosPrimitive mp;
        mp.load_ar_tag_waypoint(msg->pose.position.x, msg->pose.position.y);
    }
}

/*
 * Give the Pixhawk a new waypoint from the AR tag vector output
 */
void MavrosPrimitive::load_ar_tag_waypoint(float x, float y)
{
    // If the tag offset is less than the tolerance
    if((x * x + y * y) < (ar_tolerance * ar_tolerance)) {

        // If not yet in stage two (at higher altitude), decrease the tolerance,
        // lower the altitude, and enter stage two of the search
        if(!stage_two) {

            stage_two = true;
            ar_tolerance = 0.3;
            altitude = 6.0;
	    increment = 0.3;
        }
        // Otherwise, trigger end of mission
        else {

            MavrosPrimitive mp;
            mp.load_end_of_mission();
            return;
        }
    }

    // Invert the y input (because up is negative in this frame)
    float invert_y = -1 * y;

    // Calculate a new GPS waypoint based on the x, y
    gps new_gps = offsetToGPSWaypoint(x, invert_y, current_gps, 0);

    // Log the result
    ROS_INFO("New waypoint from AR tag calculation: Lat: %0.8f Long: %0.8f", new_gps.latitude, new_gps.longitude);
    fprintf(myfile, "\nNew waypoint from AR tag calculation: Lat: %0.8f Long: %0.8f", new_gps.latitude, new_gps.longitude);

    // Get the index that the mission should jump to next (previous last index + 1)
    uint16_t index = wpl->waypoints.size();

    // Update the current GPS
    current_gps = new_gps;

    // Move to new waypoint
    mavros::Waypoint wp_ar;
    wp_ar.x_lat = current_gps.latitude;
    wp_ar.y_long = current_gps.longitude;
    wp_ar.z_alt = altitude;
    wp_ar.command = 16;
    wp_ar.frame = 3;
    wpl->waypoints.push_back(wp_ar);

    // Loiter unlimited
    mavros::Waypoint wp_loiter;
    wp_loiter.x_lat = current_gps.latitude;
    wp_loiter.y_long = current_gps.longitude;
    wp_loiter.z_alt = altitude;
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
    if(waypoint_set_current_client_.call(set_current_srv)) {

        fprintf(myfile, "\n--------------\nSUCCESS UPDATING MISSION ITEM\n--------------\n");
    }
    else {

        fprintf(myfile, "\n--------------\nFAILED TO UPDATE MISSION ITEM\n--------------\n");
    }
}

/*
 * Calculate GPS offset based on x, y, and yaw
 */
gps MavrosPrimitive::offsetToGPSWaypoint(double x, double y, gps current_gps, double yaw) {

    double pi = 3.14159265359;

    // Scale longitude changes to current latitude
    double longitude_length = cos(2*pi / 360 * current_gps.latitude) * 111325;
    double latitude_length = 111325;

    // Use 2D transformation based on yaw
    double lon_adjustment_raw = cos(yaw)*x + sin(yaw)*y;
    double lat_adjustment_raw = -1*sin(yaw)*x + cos(yaw)*y;

    // Calculate adjustment based on given increment
    double lon_adjustment = increment * lon_adjustment_raw / longitude_length;
    double lat_adjustment = increment * lat_adjustment_raw / latitude_length;

    // Create a new GPS coordinate referenced to the current_gps
    gps gps_returned;
    gps_returned.longitude = current_gps.longitude + lon_adjustment;
    gps_returned.latitude = current_gps.latitude + lat_adjustment;

    return gps_returned;
}

/*
 * Load the end of the mission (land, release package, takeoff, RTL)
 */
void MavrosPrimitive::load_end_of_mission()
{
    fprintf(myfile, "\n\n----------------------\nLoading end of mission\n----------------------\n");

    uint16_t index = wpl->waypoints.size();

    // Land
    mavros::Waypoint wp_land;
    wp_land.x_lat = current_gps.latitude;
    wp_land.y_long = current_gps.longitude;
    wp_land.z_alt = 0.0;
    wp_land.command = 21;
    wp_land.frame = 3;
    wpl->waypoints.push_back(wp_land);

    // Set servo
    mavros::Waypoint wp_servo;
    wp_servo.param1 = 9;
    wp_servo.param2 = 650;
    wp_servo.param3 = 2;
    wp_servo.param4 = 0;
    wp_servo.command = 184;
    wp_servo.frame = 3;
    wpl->waypoints.push_back(wp_servo);

    // Takeoff
    mavros::Waypoint wp_takeoff;
    wp_takeoff.x_lat = current_gps.latitude;
    wp_takeoff.y_long = current_gps.longitude;
    wp_takeoff.z_alt = altitude;
    wp_takeoff.command = 22;
    wp_takeoff.frame = 3;
    wpl->waypoints.push_back(wp_takeoff);

    // RTL
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
