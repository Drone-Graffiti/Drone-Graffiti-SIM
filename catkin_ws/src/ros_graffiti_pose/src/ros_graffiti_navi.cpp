 #include "ros_graffiti_pose/ros_graffiti_navi.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "graffiti_navi");

    ros::NodeHandle n("~");

    desired_position_publisher = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    sprayer_state_publisher = n.advertise<std_msgs::UInt8>("/paint/gazebo/sprayer", 1);
    waypoints_publisher = n.advertise<visualization_msgs::Marker>("/paint/visual/wps", 1);
    wpnum_publisher = n.advertise<std_msgs::Int32>("/paint/wpnum", 1);

    ardupilot_local_position_subscriber = n.subscribe("/mavros/local_position/pose", 1, ardu_local_pos_callback);
    command_subscriber = n.subscribe("/paint/command", 1, command_callback);
    rcout_subscriber = n.subscribe("/mavros/rc/out", 1, rcout_callback);

    cmd_long_client = n.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    land_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

    ros::Rate loop_rate(10);

    // Parameters
    if (!n.getParam("distance_to_wall", distance_to_wall)) { distance_to_wall = 1.0; }
    if (!n.getParam("svg_file_name", svg_file_name)) { svg_file_name = ""; }
    if (!n.getParam("max_painting_distance_from_wall", max_painting_distance_from_wall)) { max_painting_distance_from_wall = 0.5; }
    if (!n.getParam("max_painting_deviation", max_painting_deviation)) { max_painting_deviation = 0.2; }
    if (!n.getParam("sprayer_servo_id", sprayer_servo_id)) { sprayer_servo_id = 10; }
    if (!n.getParam("sprayer_servo_paint_pwm", sprayer_servo_paint_pwm)) { sprayer_servo_paint_pwm = 1150; }
    if (!n.getParam("sprayer_servo_stop_pwm", sprayer_servo_stop_pwm)) { sprayer_servo_stop_pwm = 2000; }
    if (!n.getParam("wall_left_border", wall_left_border)) { wall_left_border = 0.5; }
    if (!n.getParam("wall_right_border", wall_right_border)) { wall_right_border = 3.5; }
    if (!n.getParam("wall_upper_border", wall_upper_border)) { wall_upper_border = 4.0; }
    
    parse_svg(svg_file_name);

    while (ros::ok()) {

        set_desired_position_smooth();

        publish_sprayer_state();

        handle_sprayer_servo();

        //publish_wps_visual();
        publish_wps_visual_smooth();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void ardu_local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &pos)
{
    ardupilot_pose = *pos;
}

void command_callback(const std_msgs::String::ConstPtr &cmd) {
    std::vector<std::string> command;
    boost::split(command, cmd->data, [](char c) {return c == ';';});

    ROS_INFO("NAVI CMD Arrived. %s", cmd->data.c_str());

    if (command.size() < 1) { return; }

    if (command[0] == "STARTPAINT") {
        if (wps_long.size() > 0) {
            user_allows_to_paint = true;
            paint_state = 1;
            // first move to current_wp close enough and
            // then start moving along the trajectory
            moving_to_position = true;
        }
        return;    
    }

    if (command[0] == "STOPPAINT") {
        user_allows_to_paint = false;
        paint_state = 0;
        return;    
    }

    if (command[0] == "SETWP" && command.size() == 2) {
        uint16_t v = std::stoi(command[1]);
        if (v > wps_long.size() - 1) {v = wps_long.size() - 1;}
        current_wp = v;
        return;
    }

    if (command[0] == "SETSCALE" && command.size() == 7) {
        if (paint_state != 0) {
            ROS_WARN("Attention! Rescaling while painting is forbidden!");
            return;
        }
        
        svg_h_offset = std::stof(command[1]);
        svg_v_offset = std::stof(command[2]);
        svg_h_scale = std::stof(command[3]);
        svg_v_scale = std::stof(command[4]);
        fps = std::stof(command[5]);
        speed = std::stof(command[6]);

        ROS_INFO("Set offset to [%f, %f]", svg_h_offset, svg_v_offset);
        ROS_INFO("Rescaling to [%f, %f]", svg_h_scale, svg_v_scale);
        ROS_INFO("Set FPS to [%f]", fps);
        ROS_INFO("Set speed to [%f] m/sec", speed);

        parse_svg(svg_file_name);
        return;
    }

     if (command[0] == "LAND") {
        user_allows_to_paint = false;
        paint_state = 0;
        mavros_msgs::CommandTOL cmd_tol;
        land_client.call(cmd_tol);
        return;    
    }
}

void rcout_callback(const mavros_msgs::RCOutConstPtr &rcout) {
    current_rcout = *rcout;
}

// ------------------------
// NAVIGATION
// ------------------------

void set_desired_position_smooth() {
    if (paint_state == 0) return;

    if (moving_to_position) {
        set_waypoint_smooth(current_wp);

        double dist_to_current_wp = pow(ardupilot_pose.pose.position.z - wps_long[current_wp].z, 2) +
                                pow(ardupilot_pose.pose.position.y - wps_long[current_wp].y, 2) +
                                pow(ardupilot_pose.pose.position.x - wps_long[current_wp].x, 2);
        
        if (sqrt(dist_to_current_wp) < 0.2f && fps > 0) {
           painting_start_time = ros::Time(ros::Time::now().toSec() - ((double)current_wp / fps));
           moving_to_position = false;
        }
        return;
    }

    float dt = (ros::Time::now() - painting_start_time).toSec();

    set_waypoint_smooth(dt * fps);

}

void create_path_from_pathes_array() {
    uint32_t wp_idx = 0;

    float min_x, min_z, max_x, max_z;
    min_x = 100000;
    min_z = 100000;
    max_x = -100000;
    max_z = -100000;
    for (int i=0; i<pathes.size(); i++) {
        //1. remove M
        std::string new_s = pathes[i];
     
        std::string move_substring;
        // split by L. Should have a pairs of coord.
        std::vector<std::string> move_results;
        boost::split(move_results, new_s, [](char c) {return c == 'M';});
        for(std::vector<std::string>::iterator move_it = move_results.begin(); move_it != move_results.end(); ++move_it) {
            std::string move_pair_string = *move_it; // strings between M's. First point should have paint=0
            uint32_t move_first_idx = wp_idx;
            
            //ROS_INFO(">M> %s", move_pair_string.c_str());
            if (move_pair_string.length() == 0) {continue;}

            // inside M-subsytring let's parse L's
            std::vector<std::string> line_pairs;
            boost::split(line_pairs, move_pair_string, [](char c) {return c == 'L';});
            for(std::vector<std::string>::iterator it = line_pairs.begin(); it != line_pairs.end(); ++it) {
                std::string pair_string =  *it; // something like this: 3070.453125 5080.625
                //ROS_INFO(">LLLLLLL> %s", pair_string.c_str());
                if (pair_string.length() == 0) {continue;}

                std::vector<std::string> coordinates;
                boost::split(coordinates, pair_string, [](char c) {return c == ' ';});
                // create WP
                WP wp;
                wp.y = -distance_to_wall; // it's ok for graffiti positioning, but for 
                                                   // fake gps we need to correct this value
                wp.x = (std::stod(coordinates[0]) * svg_h_scale) + svg_h_offset;
                wp.z = svg_v_offset - (std::stod(coordinates[1]) * svg_v_scale);
                wp.speed = 100;
                wp.paint = 1;
                wps.push_back(wp);

                if (min_x > wp.x) {min_x = wp.x;}
                if (max_x < wp.x) {max_x = wp.x;}
                if (min_z > wp.z) {min_z = wp.z;}
                if (max_z < wp.z) {max_z = wp.z;}
                wp_idx++;
            }
            wps[move_first_idx].paint = 0;
        }
    }

    path_length = wp_idx;

    ROS_INFO("Pathes was transformed to %d waypoints", path_length);
    ROS_INFO("First Point = [%f, %f]", wps[0].x, wps[0].z);
    ROS_INFO("Perimeter = [%f, %f] - [%f, %f]", min_x, min_z, max_x, max_z);
}

void create_smooth_path() {
    float dl = speed / fps;

    float cur_t = 0;
    float cur_l = 0;
    wps_long.clear();

    for(int i = 0; i < wps.size()-1; i++) {
        WP a = wps[i];
        WP b = wps[i+1];
        
        float d_x = b.x - a.x;
        float d_z = b.z - a.z;
                
        float len = sqrt(d_x * d_x + d_z * d_z);
        if (len == 0) {
            continue;
        }
        d_x = d_x / len;
        d_z = d_z / len;

        while (cur_l < len) {
            WP c;
            c.x = a.x + d_x * cur_l;
            c.z = a.z + d_z * cur_l;
            c.y = a.y;
            c.paint = b.paint;
            wps_long.push_back(c);
            cur_l += dl;
        }

        cur_l = cur_l - len;
    }
}

void set_waypoint_smooth(uint32_t index) {
     /*
    "header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
    coordinate_frame: 1
    type_mask: 65528
    position: {x: -0.0, y: 0.0, z: 5.0}
    velocity: {x: 0.0, y: 0.0, z: 0.0}
    acceleration_or_force: {x: 0.0, y: 0.0, z: 0.0}
    yaw: 0.0
    yaw_rate: 0.0" 
    */

    //ROS_INFO("CURRENT_WP=%d [%f, %f]", index, wps[index].x, wps[index].z);

    mavros_msgs::PositionTarget desired_position;

    if (index < 0) {
        index = 0;
    }
    
    if (index > wps_long.size() - 1) {
        index = wps_long.size() - 1;
    }

    current_wp = index;

    desired_position.position.x = wps_long[index].x;
    desired_position.position.y = wps_long[index].y;
    desired_position.position.z = wps_long[index].z;

    desired_position.coordinate_frame = 1;
    //desired_position.type_mask = 65528; // no yaw control
    desired_position.type_mask = 64504; // yaw control
    desired_position.yaw = M_PI / 2; // PI/2 in ENU = 0 in NED. We need to set 0 angle.
  
    desired_position_publisher.publish(desired_position);


}

void publish_sprayer_state() {
    std_msgs::UInt8 state;
    state.data = allow_paint();
    sprayer_state_publisher.publish(state);
}

void parse_svg(std::string filename) 
{
    pathes.clear();
    path_length = 0;
    wps.clear();

    if (filename.length() == 0) {
        ROS_WARN("Empty SVG filename!");
        return;
    }

    pt::ptree tree;
    pt::read_xml(filename, tree);

    // get pathes
    int cnt = 0;
    BOOST_FOREACH(pt::ptree::value_type &v, tree.get_child("svg.g")) {
        if (v.first == "path") {
            std::string pth = v.second.get<std::string>("<xmlattr>.d", "");
            if (pth.length() > 0) {
                pathes.emplace_back(pth);
                cnt++;
            }
        }
    }
    
    ROS_INFO("%d pathes successfully loaded from %s", cnt, filename.c_str());

    create_path_from_pathes_array();

    create_smooth_path();
}

void handle_sprayer_servo() {
    mavros_msgs::CommandLong set_servo;
    
    set_servo.request.command = 183;
    set_servo.request.param1 = sprayer_servo_id;
    uint16_t pwm = allow_paint() ? sprayer_servo_paint_pwm : sprayer_servo_stop_pwm;

    // if already has same pwm onboard - skip
    if (current_rcout.channels.size() >= sprayer_servo_id && sprayer_servo_id > 0) {
        if (current_rcout.channels[sprayer_servo_id-1] == pwm ) {
            return;
        }
    }
    set_servo.request.param2 = pwm;
    cmd_long_client.call(set_servo);
}

bool allow_paint() {

    if (!user_allows_to_paint) {
        return false;
    }

    if (moving_to_position) {
        return false;
    }

    if (current_wp == wps_long.size() - 1) {
        return false;
    }

    //uint8_t state = wps[current_wp].paint;
    uint8_t state = wps_long[current_wp].paint;

    if (state > 0 && current_wp > 0) {
        // ENU!
        
        // check distance to wall
        if (fabs(ardupilot_pose.pose.position.y) > max_painting_distance_from_wall) {
            return false;
        }

        // check deviance
        // calculate distance from current uav point to line between 
        // current wp and prev wp

        double x0 = ardupilot_pose.pose.position.x;
        double z0 = ardupilot_pose.pose.position.z;
        double x1 = wps_long[current_wp].x;
        double z1 = wps_long[current_wp].z;
        double x2 = wps_long[current_wp - 1].x;
        double z2 = wps_long[current_wp - 1].z;

        double d = distance_to_segment(x1, z1, x2, z2, x0, z0);

        //ROS_INFO(">>> %f", d);
        if (d < max_painting_deviation) {
            return true;
        }
    }

    return false;
}



double distance_between_points(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
 
bool is_obtuse_angle(double opposite_line, double a, double b)
{
	double cos = (a*a + b*b - opposite_line*opposite_line) / (2 * a * b);
   	return cos < 0;
}
 
double distance_to_segment(double ax, double ay, double bx, double by, double x, double y)
{
	if ((ax == x && ay == y) || (bx == x && by == y)) {
        return 0;
    }
 
	double AB = distance_between_points(ax, ay, bx, by);
	double AC = distance_between_points(ax, ay, x, y);
 
	if (AB == 0) {
        return AC;
    }
	
	double BC = distance_between_points(bx, by, x, y);
 
	if (is_obtuse_angle(AC, BC, AB)) {
        return BC;
    }
	
    if (is_obtuse_angle(BC, AC, AB)) {
        return AC;
    }
	
	double p = (AC + BC + AB) / 2;
	return 2 * sqrt(fabs(p * (p - AB) * (p - BC) * (p - AC))) / AB;
}
 

void publish_wps_visual_smooth() {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "wps";
    marker.id = 0;
    marker.type = (uint32_t)visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::MODIFY;

    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    std_msgs::ColorRGBA c;
    c.a = 1;

    for (int i = 0; i < wps_long.size(); i++) {
         if (i==0) {
            p1.y = ardupilot_pose.pose.position.y;
            p1.x = ardupilot_pose.pose.position.x;
            p1.z = ardupilot_pose.pose.position.z;
            p2.y = wps_long[current_wp].y;
            p2.x = wps_long[current_wp].x;
            p2.z = wps_long[current_wp].z;
         } else {
            p1.y = wps_long[i-1].y;
            p1.x = wps_long[i-1].x;
            p1.z = wps_long[i-1].z;
            p2.y = wps_long[i].y;
            p2.x = wps_long[i].x;
            p2.z = wps_long[i].z;
         }
        
         marker.points.push_back(p1);
         marker.points.push_back(p2);
         if (i == 0) {
            c.r = 0;
            c.g = 0;
            c.b = 1;
         } else if (current_wp == i) {
            c.r = 1;
            c.g = 0;
            c.b = 0;
         } else if (i < current_wp) {
            c.r = 0;
            c.g = 1;
            c.b = 0;
         } else {
            c.r = 1;
            c.g = 1;
            c.b = 0.3;
         }

         if (wps_long[i].paint == 0) {
             c.r *= 0.5;
             c.g *= 0.5;
             c.b *= 0.5;
         }



         marker.colors.push_back(c);
         marker.colors.push_back(c);
    }   

    // add wall boorders
    p1.x = wall_left_border;
    p1.y = -distance_to_wall;
    p1.z = wall_upper_border;
    p2.x = wall_left_border;
    p2.y = -distance_to_wall;
    p2.z = 0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    p2 = p1;
    p1.x = wall_right_border;
    p1.y = -distance_to_wall;
    p1.z = wall_upper_border;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    p2 = p1;
    p1.x = wall_right_border;
    p1.y = -distance_to_wall;
    p1.z = 0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    c.r = 0;
    c.g = 1;
    c.b = 1;
    marker.colors.push_back(c);
    marker.colors.push_back(c);
    marker.colors.push_back(c);
    marker.colors.push_back(c);
    marker.colors.push_back(c);
    marker.colors.push_back(c);



    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;


    marker.scale.x = 0.05;

    
    waypoints_publisher.publish(marker);

    std_msgs::Int32 wpnum;
    wpnum.data = current_wp;
    wpnum_publisher.publish(wpnum);
}