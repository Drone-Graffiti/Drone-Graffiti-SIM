#include "ros_graffiti_pose/ros_graffiti_pose.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "graffiti_pose");

    ros::NodeHandle n("~");

    fake_gps_position_publisher = n.advertise<geometry_msgs::PoseStamped>("/mavros/fake_gps/vision", 1);
    extnav_position_publisher_ned = n.advertise<geometry_msgs::PoseStamped>("/mavros/extnav/pose_ned", 1);
    extnav_position_publisher_enu = n.advertise<geometry_msgs::PoseStamped>("/mavros/extnav/pose_enu", 1); // for monitoring only
    extnav_velocity_publisher_ned = n.advertise<geometry_msgs::Vector3>("/mavros/extnav/vel_ned", 1);
    pozyx_visual_publisher = n.advertise<visualization_msgs::MarkerArray>("/paint/visual/pozyx", 1);
    navi_cmd_publisher = n.advertise<std_msgs::String>("/paint/command", 1);
    log_publisher = n.advertise<std_msgs::Float32MultiArray>("/mavros/extnav/log", 1);

    laser_array_subscriber = n.subscribe("/ranges", 1, teraranger_callback);
    altitude_subscriber = n.subscribe("/paint/laser/alt", 1, alt_range_callback);
    pozyx_subscriber = n.subscribe("/paint/pozyx/laser_scan", 1, pozyx_callback);
    ardupilot_local_position_subscriber = n.subscribe("/mavros/local_position/pose", 1, ardu_local_pos_callback);
    gazebo_twist_subscriber = n.subscribe("/paint/gazebo/model_twist", 1, gazebo_twist_callback);
    gazebo_position_subscriber = n.subscribe("/paint/gazebo/model", 1, gazebo_position_callback);
    of_subscriber = n.subscribe("/paint/of_out", 1, of_callback);
    
    ros::Rate loop_rate(200);

    // Parameters
    if (!n.getParam("use_fakegps", use_fakegps)) { use_fakegps = false; }
    if (!n.getParam("use_extnav", use_extnav)) { use_extnav = false; }
    if (!n.getParam("left_laser_index", left_laser_index)) {ROS_ERROR("LEFT LASER INDEX MUST BE SET!"); return 0; }
    if (!n.getParam("right_laser_index", right_laser_index)) {ROS_ERROR("RIGHT LASER INDEX MUST BE SET!"); return 0; }
    if (!n.getParam("altimeter_pitch_angle", altimeter_pitch_angle)) { altimeter_pitch_angle = 0; }
    if (!n.getParam("use_velocity", use_velocity)) { use_velocity = false; }
    if (!n.getParam("send_gazebo_velocity", send_gazebo_velocity)) { send_gazebo_velocity = false; }
    if (!n.getParam("pozyx_distance_from_wall", pozyx_distance_from_wall)) { pozyx_distance_from_wall = 1; }
    if (!n.getParam("wall_left_border", wall_left_border)) { wall_left_border = 0.5; }
    if (!n.getParam("wall_right_border", wall_right_border)) { wall_right_border = 3.5; }
    if (!n.getParam("wall_upper_border", wall_upper_border)) { wall_upper_border = 4.0; }
    if (!n.getParam("save_position_data", save_position_data)) { save_position_data = false; }
    if (!n.getParam("turn_on_raw_logging", turn_on_raw_logging)) { turn_on_raw_logging = false; }
    
    // 0-3: pozyx
    // 4-5: tera
    // 6: alt
    // 7-8: optical
    // 9-11: reserved
    raw_log.data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // anchor positions list
    std::string apl;
    std::vector<std::string> apl_vec;
    anchor_positions.clear();
    if (!n.getParam("positions_list", apl)) { apl = ""; }
    ROS_INFO("Anchor positions, %s:", apl.c_str());
    boost::algorithm::split(apl_vec, apl, boost::algorithm::is_any_of(";"));

    for (auto it = apl_vec.begin(); it != apl_vec.end(); ++it){
        anchor_positions.push_back(std::stof(*it));
        ROS_INFO("  : %f", anchor_positions.back());
    }
    
    // switch altimeter_pitch_angle to rad
    altimeter_pitch_angle = altimeter_pitch_angle * M_PI / 180.0;

    ros::Time last_sent = ros::Time::now();
    ros::Time last_visualization_sent = ros::Time::now();
    
    while (ros::ok()) {

        // EKF accepts messages with > 0.07 sec between
        if ((ros::Time::now() - last_sent).toSec() > (0.075)) {

            handle_errors();

            last_sent = ros::Time::now();

            calculate_pose();
            
            if (turn_on_raw_logging) {
                send_raw_data_logs();
            }
            
            if (use_fakegps) {
                send_position_to_fake_gps();  
            }

            if (use_extnav) {
                send_position_to_extnav();
            }
            
            if (use_velocity) {
                if (send_gazebo_velocity) {
                    send_gazebo_velocity_to_extnav();
                } else {
                    send_velocity_to_extnav();
                }
            }

            if (save_position_data) {
                save_position_data_to_csv();
            }
        }

        // visualization send
        if ((ros::Time::now() - last_visualization_sent).toSec() > 1) {
            last_visualization_sent = ros::Time::now();

            send_pozyx_visualization();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// -------------------------
// Fake GPS & EXT NAV
// -------------------------

void send_position_to_fake_gps() {
    calculated_pose_enu.header.stamp = ros::Time::now();
    fake_gps_position_publisher.publish(calculated_pose_enu);
}

void send_position_to_extnav() {
    calculated_pose_ned.header.stamp = ros::Time::now();
    calculated_pose_enu.header.stamp = ros::Time::now();
    
    calculated_pose_enu.header.frame_id = "map";
   
    extnav_position_publisher_ned.publish(calculated_pose_ned);
    extnav_position_publisher_enu.publish(calculated_pose_enu);
}

void send_velocity_to_extnav() {
    if (vel_counter.x > 0) {
        velocity_ned.x = vel_sum.x/ vel_counter.x;
        //ROS_INFO(">> POSXY pos_sum=%f pos_counter=%f vel_sum=%f vel_counter=%f vel=%f", pos_sum.x, pos_counter.x, vel_sum.x, vel_counter.x, velocity_ned.x );
        
        vel_counter.x = 0;
        vel_sum.x = 0;
    }
    if (vel_counter.y > 0) {
        //velocity_ned.y = vel_sum.y/ vel_counter.y;
        //ROS_INFO(">> POSXY pos_sum=%f pos_counter=%f vel_sum=%f vel_counter=%f vel=%f", pos_sum.y, pos_counter.y, vel_sum.y, vel_counter.y, velocity_ned.y );
        vel_counter.y = 0;
        vel_sum.y = 0;
    }

    uint32_t list_size = median_y_velocity_list.size();
    if (list_size > 0) {
        median_y_velocity_list.sort();
        auto it = std::next(median_y_velocity_list.begin(), list_size / 2);
        velocity_ned.y = *it;
        median_y_velocity_list.clear();
    }
    
    extnav_velocity_publisher_ned.publish(velocity_ned);
}

void send_gazebo_velocity_to_extnav() {
    geometry_msgs::Vector3 v;
    v.x = gazebo_twist.twist.linear.x;
    v.y = -gazebo_twist.twist.linear.y;
    v.z = -gazebo_twist.twist.linear.z;
    extnav_velocity_publisher_ned.publish(v);
}

void send_pozyx_visualization() {
    visualization_msgs::MarkerArray ma;

    for (int i = 0; i < anchor_positions.size(); i++) {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "pozyx";
        marker.id = i;
        marker.type = (uint32_t)visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::MODIFY;

        marker.pose.position.x = anchor_positions[i];
        marker.pose.position.y = -pozyx_distance_from_wall;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 1.0;

        ma.markers.push_back(marker);
    }

    pozyx_visual_publisher.publish(ma);
}

void send_raw_data_logs() {
    // pozyx
    if (pozyx_current_scan.ranges.size() > 3) {
        raw_log.data[0] = pozyx_current_scan.ranges[0];
        raw_log.data[1] = pozyx_current_scan.ranges[1];
        raw_log.data[2] = pozyx_current_scan.ranges[2];
        raw_log.data[3] = pozyx_current_scan.ranges[3];
    }
    
    // tera
    if (current_tera_scan.ranges.size() > 0) {
        raw_log.data[4] = current_tera_scan.ranges[left_laser_index].range;
        raw_log.data[5] = current_tera_scan.ranges[right_laser_index].range;
    }
    
    // alt
    if (current_alt_scan.ranges.size() > 0) {
        raw_log.data[6] = current_alt_scan.ranges[0];  
    }

    // optical flow
    raw_log.data[7] = current_of.vector.x;
    raw_log.data[8] = current_of.vector.y;
    raw_log.data[9] = current_of.vector.z;

    log_publisher.publish(raw_log);
}

// -------------------------
// Callbacks
// -------------------------
void teraranger_callback(const teraranger_array::RangeArray::ConstPtr &scan) {

    current_tera_scan = *scan;

    if ( (scan->ranges[left_laser_index].range == std::numeric_limits<double>::infinity()) ||
          (scan->ranges[right_laser_index].range == std::numeric_limits<double>::infinity())) {
        tera_err_count++;
        return;
    }

    // another sanity check
    if ( (scan->ranges[left_laser_index].range > 1000) ||
          (scan->ranges[right_laser_index].range > 1000)) {
        tera_err_count++;
        return;
    }

    if (std::isnormal(scan->ranges[left_laser_index].range)) {
        horizontal_laser_ranges[LEFT_LASER_RANGEFINDER] =  scan->ranges[left_laser_index].range * cos(ardu_pitch);
    }

    if (std::isnormal(scan->ranges[right_laser_index].range)) {
        horizontal_laser_ranges[RIGHT_LASER_RANGEFINDER] = scan->ranges[right_laser_index].range * cos(ardu_pitch);
    }

    double distance;
    ros::Duration dt = scan->header.stamp - lastTerarangerUpdate;

    if (calculate_x(distance, horizontal_laser_ranges[LEFT_LASER_RANGEFINDER], horizontal_laser_ranges[RIGHT_LASER_RANGEFINDER] )) {
        tera_err_count = 0;
        //ROS_INFO(">>>> %f,    %f", dt.toSec(), (distance - prev_point_ned.x) );
        if (dt.toSec() > 0 && dt.toSec() < 1) {
            //velocity_ned.x = (distance - prev_point_ned.x) / dt.toSec();
            vel_sum.x += (distance - prev_point_ned.x) / dt.toSec();
            vel_counter.x++;
        } 
        prev_point_ned.x = distance;
    } else {
        tera_err_count++;
    }

    lastTerarangerUpdate = scan->header.stamp;
}

void alt_range_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    current_alt_scan = *scan;
    // take altimeter_pitch_angle into the account!        
    altitude_range = scan->ranges[0] * cos(ardu_pitch - altimeter_pitch_angle) * cos(ardu_roll);

    ros::Duration dt = scan->header.stamp - lastAltitudeUpdate;

    if (dt.toSec() > 0 &&  dt.toSec() < 1) {
        velocity_ned.z = -(prev_point_ned.z + altitude_range) / dt.toSec();  // Apply filter ??
    } else {
        velocity_ned.z = 0;
    }

    prev_point_ned.z = -altitude_range;

    lastAltitudeUpdate = scan->header.stamp;
 // ROS_INFO(">>> GET ALT %f, pitch = %f, altimiter_pitch = %f", altitude_range, ardu_pitch, altimeter_pitch_angle);
}

void pozyx_callback(const sensor_msgs::LaserScan::ConstPtr &scan) 
{   
    pozyx_current_scan = *scan;

    float y = 0;

    if (calculate_y(scan->ranges, anchor_positions, y) >= 0) {
        // add to median list
        if (!std::isnan(y)) {
            median_y_list.push_back(y);
        } else {
            pozyx_err_count++;
            return;
        }

        // clear error counter;
        pozyx_err_count = 0;

        pos_sum.y += y;
        pos_counter.y++;  
 
        ros::Duration dt = scan->header.stamp - lastPozyxUpdate;

        if (dt.toSec() > 0 &&  dt.toSec() < 1) {
            median_y_velocity_list.push_back((y - prev_point_ned.y) / dt.toSec());
            vel_sum.y += (y - prev_point_ned.y) / dt.toSec();
            vel_counter.y++;
        }

        prev_point_ned.y = y;
    
        lastPozyxUpdate = scan->header.stamp;
    } else {
        pozyx_err_count++;
    }
}

void ardu_local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &pos)
{
    ardupilot_pose = *pos;

    ardupilot_first_message_recieved = true;

    // fill RPY with TF2 lib
    tf2::Quaternion q;
    tf2::fromMsg(ardupilot_pose.pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(ardu_roll, ardu_pitch, ardu_yaw);    
}

void gazebo_twist_callback(const geometry_msgs::TwistStamped::ConstPtr &twist) {
    gazebo_twist = *twist;
}

void gazebo_position_callback(const geometry_msgs::PoseStamped::ConstPtr &pose) {
    gazebo_pose = *pose;
}

void of_callback(const geometry_msgs::Vector3Stamped::ConstPtr &vec) {
    current_of = *vec;
}

// -------------------------
// Math
// -------------------------

 void calculate_pose() {
    // Distance from Wall (Y coordinate)

    double r1_range = horizontal_laser_ranges[LEFT_LASER_RANGEFINDER];
    double r2_range = horizontal_laser_ranges[RIGHT_LASER_RANGEFINDER];
    //r3_range - size of third side of triangle
    double r3_range = sqrt(r1_range * r1_range + r2_range * r2_range - 2 * r1_range * r2_range * cos(2 * SENSORS_HALF_ANGLE));
    // sanity check
    if (r3_range == 0) { return; }
    // half-perimeter of triangle
    double hp = (r1_range + r2_range + r3_range) / 2;
    // height of triangle
    double h = 2 * sqrt(hp * (hp - r1_range) * (hp - r2_range) * (hp - r3_range) ) / r3_range;
    calculated_pose_ned.pose.position.x = -h;

    // Y from pozyx
    if (pos_counter.y > 0) {
       calculated_pose_ned.pose.position.y = pos_sum.y / pos_counter.y;
       pos_sum.y = 0;
       pos_counter.y = 0;
    } 
    uint32_t list_size = median_y_list.size();
    if (list_size > 0) {
        median_y_list.sort();
        auto it = std::next(median_y_list.begin(), list_size / 2);
      //  calculated_pose_ned.pose.position.y = *it;
        //ROS_INFO("ned.y=%f, size=%d", calculated_pose_ned.pose.position.y, list_size);
        median_y_list.clear();
    }
    
    
    // Z from altitude rangefinder
    calculated_pose_ned.pose.position.z = -altitude_range;
    
    // calculate orientation (
    // pitch & roll are taken from autopilot
    // and yaw is calculated from lasers
    if (r2_range == 0) { return; }

    // zero yaw means north, i.e. means looking 
    // directly to the wall
    double tau = acos(h / r2_range);
    double yaw = SENSORS_HALF_ANGLE - tau;
    
    tf2::Quaternion q_ned, q_enu;
    
    // DO NOT USE q.setEuler!
    // It changes yaw, pitch and roll order, i.e. yaw is 
    // rotation around Y axis. WHAT???

    // use setRPY instead
    
    q_ned.setRPY(ardu_roll, ardu_pitch, yaw);
    q_enu.setRPY(ardu_roll, ardu_pitch, -yaw + M_PI / 2);
    
    calculated_pose_ned.pose.orientation = tf2::toMsg(q_ned);
    calculated_pose_enu.pose.orientation = tf2::toMsg(q_enu);
    calculated_pose_enu.pose.position.x = calculated_pose_ned.pose.position.y;
    calculated_pose_enu.pose.position.y = calculated_pose_ned.pose.position.x;
    calculated_pose_enu.pose.position.z = -calculated_pose_ned.pose.position.z;

     // check we are inside the wall
    if (calculated_pose_ned.pose.position.y < wall_left_border ||
    calculated_pose_ned.pose.position.y > wall_right_border ||
    calculated_pose_ned.pose.position.z < -wall_upper_border ) {
        borders_err_count++;
    } else {
        borders_err_count = 0;
    }
}

bool calculate_x(double &x, double r1_range, double r2_range) {
    //r3_range - size of third side of triangle
    double r3_range = sqrt(fabs(r1_range * r1_range + r2_range * r2_range - 2 * r1_range * r2_range * cos(2 * SENSORS_HALF_ANGLE)));
    // sanity check
    if (r3_range == 0) { return false; }
    // half-perimeter of triangle
    double hp = (r1_range + r2_range + r3_range) / 2;
    // height of triangle
    x = -(2 * sqrt(fabs(hp * (hp - r1_range) * (hp - r2_range) * (hp - r3_range))) / r3_range);

    return true;
}

float calculate_y(std::vector<float> ranges, std::vector<float> places, float &result) {
    int n = ranges.size();
    result = 0;

    if (n != places.size()) {
        ROS_ERROR("ERR 1");
        return -1;
    }

    float d_to_wall = fabs(calculated_pose_ned.pose.position.x);
    float altimeter_alt = fabs(calculated_pose_ned.pose.position.z);

    std::vector<float> coords;
    std::vector<float> heights_squared;
    std::vector<float> divergences;
    for (int i = 0; i < n - 1; i++){
        for (int j = i + 1; j < n; j++) {

            // triangle sides
            float a = ranges[i]; // "left" side
            float b = ranges[j]; // "right" side
            if (a <= 0 || b <= 0) {
                //ROS_ERROR("Bad A or B");
                continue;
            }

            // floor side           
            // check if triangle basement is too small
            float c = abs(places[i] - places[j]);
            if (c < 1) {
                //ROS_ERROR("Bad C");
                continue;
            }

            //ensure we have a triangle
            if (((a + c) < b) || ((b + c) < a) || ((a + b) < c)) {
               // ROS_WARN("NOT A TRIANGLE (%f %f %f)", a ,b, c);
                continue;
            }

            // halfperimeter
            float p = (a + b + c) / 2.0;
            // square of height
            float h_squared = 4 * p * (p - a) * (p - b) * (p - c) / (c * c);
            // distance from A point (intersection of a and c)

            // check for possible float error
            // d can be small and less than zero
            float d = a * a - h_squared;
            if (d > 0) {
                d = sqrt(d);
            } else {
                d = 0;
            }

            // determine sign of d. if angle AC > 90 degree d = -d
            float ac_angle = (a * a + c * c - b * b) / (2.0 * a * c);
            //sanity check
            if (ac_angle > 1) {
                ac_angle = 1;
            } else if (ac_angle < -1) {
                ac_angle = -1;
            }
            ac_angle = acos(ac_angle);
            

            if (ac_angle > M_PI / 2.0) {
                d = -d;
            }
        
            coords.push_back(places[i] + d);
            heights_squared.push_back(h_squared);
            // div by each of triangles. 
            //float flr_s = d_to_wall - pozyx_distance_from_wall;
            //float pozyx_calc_height_s = sqrt(fabs(h_squared - flr_s * flr_s));
            //divergences.push_back(fabs(pozyx_calc_height_s - altimeter_alt));
        }
    }

    if (coords.size() == 0) {
        return -1;
    }

    float sum = 0;
    for (float v : coords) {
        sum += v;
    }

    result = sum / coords.size();

    // calculate vertical divergence
    sum = 0;
    for (float v : heights_squared) {
        sum += v;
    }

    float heights_squared_averege = sum / heights_squared.size();
    float flr = d_to_wall - pozyx_distance_from_wall;
    float pozyx_calc_height = sqrt(fabs(heights_squared_averege - flr * flr));
    float divergence = fabs(pozyx_calc_height - altimeter_alt);


    /*
    // div by each triangle
    sum = 0;
    for (float v : divergences) {
        sum += v;
    }
    float divergence = sum; */

    // last sanity check :(
    if (std::isnan(result))  {
        ROS_ERROR("ERR 3");
        return -1;
    }

    return divergence; 
}


void log_pozyx() {
    


}

void handle_errors() {

    bool do_land = false;

    if (pozyx_err_count > 10) {
        ROS_ERROR("POZYX ERROR COUNT!");
        do_land = true;
    }

    if (tera_err_count > 10) {
        ROS_ERROR("TERARANGER ERROR COUNT!");
        do_land = true;
    }

    if ((ros::Time::now() - pozyx_current_scan.header.stamp).toSec() > 0.25) {
        ROS_ERROR("POZYX TIMEOUT!");
        do_land = true;
    }

    if ((ros::Time::now() - lastTerarangerUpdate).toSec() > 0.25) {
        ROS_ERROR("TERARANGER TIMEOUT!");
        do_land = true;
    }

    if ((ros::Time::now() - lastAltitudeUpdate).toSec() > 0.25) {
        ROS_ERROR("Altimeter TIMEOUT!");
        do_land = true;
    }

    if (ardupilot_first_message_recieved && (ros::Time::now() - ardupilot_pose.header.stamp).toSec() > 0.25) {
        ROS_ERROR("Autopilot TIMEOUT!");
        do_land = true;
    }
    
    // check we are inside the wall
    if (borders_err_count > 3) {
        ROS_ERROR("OUT OF BORDERS!");
        do_land = true;
    }

    if (do_land) {
        std_msgs::String navi_cmd;
        navi_cmd.data = "LAND";
        navi_cmd_publisher.publish(navi_cmd);
    }
    
}

void save_position_data_to_csv() {
    if (!csv_file.is_open()) {
        std::string fname = "/home/mike/" + std::to_string(ros::Time::now().toSec()) + ".csv";
        csv_file.open(fname);
        csv_file << "ROS_TIME;GAZEBO_X;GAZEBO_Y;GAZEBO_Z;BOARD_X;BOARD_Y;BOARD_Z;POZYX_1;POZYX_2;POZYX_3;POZYX_4;\n";
    }

    if (csv_file.is_open()) {
        csv_file << std::to_string(ros::Time::now().toSec()) + ";";
        csv_file << std::to_string(gazebo_pose.pose.position.x) + ";";
        csv_file << std::to_string(gazebo_pose.pose.position.y) + ";";
        csv_file << std::to_string(gazebo_pose.pose.position.z) + ";";
        csv_file << std::to_string(calculated_pose_ned.pose.position.x) + ";";
        csv_file << std::to_string(calculated_pose_ned.pose.position.y) + ";";
        csv_file << std::to_string(calculated_pose_ned.pose.position.z) + ";";     
        if (pozyx_current_scan.ranges.size() > 3)  {
            csv_file << std::to_string(pozyx_current_scan.ranges[0]) + ";";      
            csv_file << std::to_string(pozyx_current_scan.ranges[1]) + ";";
            csv_file << std::to_string(pozyx_current_scan.ranges[2]) + ";";
            csv_file << std::to_string(pozyx_current_scan.ranges[3]) + ";";
        } else {
            csv_file << std::to_string(0) + ";";
            csv_file << std::to_string(0) + ";";
            csv_file << std::to_string(0) + ";";
            csv_file << std::to_string(0) + ";";
        }
        csv_file << "\n";
    }

}