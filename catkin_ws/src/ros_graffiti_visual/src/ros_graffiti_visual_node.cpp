#include "ros_graffiti_visual/ros_graffiti_visual.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graffiti_visual");

    ros::NodeHandle n("~");

    wall_visual_publisher = n.advertise<visualization_msgs::Marker>("/paint/visual/wall", 1);
    text_publisher = n.advertise<visualization_msgs::Marker>("/paint/visual/text", 1);
    
    laser_array_subscriber = n.subscribe("/ranges", 100, teraranger_callback);
    altitude_subscriber = n.subscribe("/paint/laser/alt", 100, alt_range_callback);
    pozyx_subscriber = n.subscribe("/paint/pozyx/laser_scan", 100, pozyx_callback);
    ardupilot_local_position_subscriber = n.subscribe("/mavros/local_position/pose", 1, ardu_local_pos_callback);



    ros::Rate loop_rate(10);
  
    while (ros::ok()) {

        publish_wall();

        publish_tera();

        publish_pozyx();

        publish_alt();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


void publish_wall() 
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "wall";
    marker.id = 0;
    marker.type = (uint32_t)visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::MODIFY;
    
    marker.pose.position.x = 0;
    marker.pose.position.y = 0.1;
    marker.pose.position.z = 20;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.scale.x = 40;
    marker.scale.y = 0.2;
    marker.scale.z = 40;

    marker.color.r = 0.3;
    marker.color.g = 0.3;
    marker.color.b = 0.3;
    marker.color.a = 1.0;

  

    wall_visual_publisher.publish(marker);
}

void publish_tera() {

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "text";
    marker.id = 0;
    marker.type = (uint32_t)visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose.position.x = ardu.pose.position.x;
    marker.pose.position.y = ardu.pose.position.y;
    marker.pose.position.z = ardu.pose.position.z + 1.2;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    std::string text = "tera: ";

    if (tera.ranges.size() > 0) {
        for (int i = 0; i < tera.ranges.size(); i++) {
            //float t = tera.ranges[i].range;
            text = text + float2str(tera.ranges[i].range) + " | ";
        }
    } else {
        text = text + "no info";
    }
    if ((ros::Time::now() - tera.header.stamp).toSec() > 1.0) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    marker.lifetime = ros::Duration();
    marker.text = text;

    text_publisher.publish(marker);
}


void publish_pozyx() {

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "text";
    marker.id = 1;
    marker.type = (uint32_t)visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose.position.x = ardu.pose.position.x;
    marker.pose.position.y = ardu.pose.position.y;
    marker.pose.position.z = ardu.pose.position.z + 1.3;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    std::string text = "pozyx: [Q=";

    text += std::to_string((int)pozyx.scan_time) + "%] ";

    if (pozyx.ranges.size() > 0) {
        for (int i = 0; i < pozyx.ranges.size(); i++) {
            text = text + float2str(pozyx.ranges[i]) + " | ";
        }
    } else {
        text = text + "no info";
    }
    if ((ros::Time::now() - pozyx.header.stamp).toSec() > 1.0) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    marker.lifetime = ros::Duration();
    marker.text = text;

    text_publisher.publish(marker);
}



void publish_alt() {

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "text";
    marker.id = 2;
    marker.type = (uint32_t)visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose.position.x = ardu.pose.position.x;
    marker.pose.position.y = ardu.pose.position.y;
    marker.pose.position.z = ardu.pose.position.z + 1.4;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    std::string text = "alt: ";

    if (altitude.ranges.size() > 0) {
        text = text + float2str(altitude.ranges[0]);
    } else {
        text = text + "no info";
    }
    if ((ros::Time::now() - altitude.header.stamp).toSec() > 1.0) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    marker.lifetime = ros::Duration();
    marker.text = text;

    text_publisher.publish(marker);
}




// -------------------------
// Callbacks
// -------------------------
void teraranger_callback(const teraranger_array::RangeArray::ConstPtr &scan) {
    tera = *scan;
}

void alt_range_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    altitude = *scan;
}

void pozyx_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    pozyx = *scan;
}

void ardu_local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &pos)
{
    ardu = *pos;
}

std::string float2str(float v)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << v;
    std::string ret(stream.str());
    return ret;
}
