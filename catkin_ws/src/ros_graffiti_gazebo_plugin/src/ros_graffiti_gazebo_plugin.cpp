#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/LaserScan.h"

#include <vector>


#include <ignition/math4/ignition/math.hh>

namespace gazebo
{
  class GraffitiPlugin : public ModelPlugin
  {
    public: GraffitiPlugin() 
    {
      std::string node_name = "gazebo_grffiti_node";
      int argc = 0;
      ros::init(argc, NULL, node_name);
      this->node = new ros::NodeHandle("~");

      this->virtual_pozyx_laserscan_publisher = this->node->advertise<sensor_msgs::LaserScan>("/paint/pozyx/laser_scan", 1);
      this->model_position_publisher = this->node->advertise<geometry_msgs::PoseStamped>("/paint/gazebo/model", 1);
      this->model_velocity_publisher = this->node->advertise<geometry_msgs::TwistStamped>("/paint/gazebo/model_twist", 1);

      current_anchor = 0;

      anchor_ranges.ranges = {0,0,0,0};

    }

    public: ~GraffitiPlugin()
    {
      delete this->node;
    }


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->world = this->model->GetWorld();
      anchors.push_back(this->world->ModelByName("anchor1"));
      anchors.push_back(this->world->ModelByName("anchor2"));
      anchors.push_back(this->world->ModelByName("anchor3"));
      anchors.push_back(this->world->ModelByName("anchor4"));

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&GraffitiPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

      if ((ros::Time::now() - pozyx_sent_time).toSec() > 0.006) {
        pozyx_sent_time = ros::Time::now();

      
        // send virtual pozyx value
        ignition::math::Pose3d pose = this->model->WorldPose();
        ignition::math::Pose3d anchor_pose = anchors[current_anchor]->WorldPose();
        ignition::math::Vector3d dist = pose.CoordPositionSub(anchor_pose);
        anchor_ranges.ranges[current_anchor] = dist.Length() + ignition::math::Rand::DblUniform(-0.15, 0.15);
        anchor_ranges.header.stamp = ros::Time::now();
        current_anchor++;
        current_anchor = current_anchor % 4;
        // quality here
        anchor_ranges.scan_time = 100;
        // --- old
        //geometry_msgs::PointStamped point_stamped;
        //point_stamped.point.x = -pose.Pos().Y() + ignition::math::Rand::DblUniform(-0.15, 0.15);
        //point_stamped.header.stamp = ros::Time::now();
        //virtual_pozyx_position_publisher.publish(point_stamped);
        // --- old
        virtual_pozyx_laserscan_publisher.publish(anchor_ranges);
                
        // send model position
        geometry_msgs::PoseStamped geom_pose;
        geom_pose.pose.orientation.w = pose.Rot().W();
        geom_pose.pose.orientation.x = pose.Rot().X();
        geom_pose.pose.orientation.y = pose.Rot().Y();
        geom_pose.pose.orientation.z = pose.Rot().Z();
        geom_pose.pose.position.x = pose.Pos().X();
        geom_pose.pose.position.y = pose.Pos().Y();
        geom_pose.pose.position.z = pose.Pos().Z();
        model_position_publisher.publish(geom_pose);

        // send model velocity
        ignition::math::Vector3d linear_vel = this->model->WorldLinearVel();
        geometry_msgs::TwistStamped twist;
        twist.twist.linear.x = linear_vel.X();
        twist.twist.linear.y = linear_vel.Y();
        twist.twist.linear.z = linear_vel.Z();
        model_velocity_publisher.publish(twist);

        ros::spinOnce();
      }
    }

    ros::Publisher virtual_pozyx_laserscan_publisher;

    // model pose publisher (for gazebo visual plugin)
    ros::Publisher model_position_publisher;
    // model velocity publisher for ground truth tests
    ros::Publisher model_velocity_publisher;


    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;
    private: std::vector<physics::ModelPtr> anchors;
    private: sensor_msgs::LaserScan anchor_ranges;
    private: uint8_t current_anchor;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: ros::NodeHandle* node;

    ros::Time pozyx_sent_time;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GraffitiPlugin)
}

