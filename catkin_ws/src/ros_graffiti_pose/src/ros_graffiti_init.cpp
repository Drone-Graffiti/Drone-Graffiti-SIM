#include "ros_graffiti_pose/ros_graffiti_init.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "graffiti_init");

    ros::NodeHandle n("~");

    ekf_origin_publisher = n.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin", 100);
    set_home_position_client = n.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
    set_stream_rate_client = n.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

    ros::Rate loop_rate(EKF_SET_RATE);

    // Parameters
    if (!n.getParam("extnav_latitude", extnav_latitude)) { extnav_latitude = 0.0; }
    if (!n.getParam("extnav_longitude", extnav_longitude)) { extnav_longitude = 0.0; }
    if (!n.getParam("extnav_altitude", extnav_altitude)) { extnav_altitude = 0.0; }
    
    while (ros::ok()) {

        set_ekf_origin();

        set_stream_rate();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


void set_ekf_origin() 
{
    geographic_msgs::GeoPointStamped gp;
    gp.position.latitude = extnav_latitude;
    gp.position.longitude = extnav_longitude;
    gp.position.altitude = extnav_altitude;
    ekf_origin_publisher.publish(gp);

    mavros_msgs::CommandHome set_home_srv;
    set_home_srv.request.current_gps = false;
    set_home_srv.request.latitude = extnav_latitude;
    set_home_srv.request.longitude = extnav_longitude;
    set_home_srv.request.altitude = extnav_altitude;

    set_home_position_client.call(set_home_srv);
}

void set_stream_rate() 
{
  mavros_msgs::StreamRate stream_rate_srv;
  stream_rate_srv.request.message_rate = TELEMETRY_RATE;
  stream_rate_srv.request.on_off = true;
  stream_rate_srv.request.stream_id = mavros_msgs::StreamRate::Request::STREAM_POSITION;
  set_stream_rate_client.call(stream_rate_srv);

  stream_rate_srv.request.stream_id = mavros_msgs::StreamRate::Request::STREAM_EXTRA1;
  set_stream_rate_client.call(stream_rate_srv);
}



/*

mike@mike-robot:~$ rostopic pub /mavros/global_position/set_gp_origin geographic_msgs/GeoPointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
position:
  latitude: 1.0
  longitude: 1.0
  altitude: 0.0" 

publishing and latching message. Press ctrl-C to terminate
^Cmike@mike-robot:~$ ^C
mike@mike-robot:~$ ^C
mike@mike-robot:~$ rosservice call /mavros/cmd/set_home "current_gps: false
latitude: 1
longitude: 1
altitude: 0.0" 
success: True
result: 0


*/
