#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>


#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/DynamicLines.hh>


#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/UInt8.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Vector3.h"

#include <ignition/math4/ignition/math.hh>

namespace gazebo
{
  class GraffitiVisualPlugin : public VisualPlugin
  {
    public: GraffitiVisualPlugin() 
    {
      std::string node_name = "gazebo_grffiti_visual_node";
      int argc = 0;
      ros::init(argc, NULL, node_name);
      this->node = new ros::NodeHandle("~");

    
    }

    public: ~GraffitiVisualPlugin()
    {
      delete this->node;
    }


    public: void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
    {
      visual_ = _parent;

      model_pose_subscriber = this->node->subscribe("/paint/gazebo/model", 1, &GraffitiVisualPlugin::modelPositionCallback, this);
      sprayer_subscriber = this->node->subscribe("/paint/gazebo/sprayer", 1, &GraffitiVisualPlugin::sprayerCallback, this);

      scale = this->visual_->GetGeometrySize();
      model_position = this->visual_->GetParent()->GetParent()->Position();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
    //  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      //    std::bind(&GraffitiPlugin::OnUpdate, this));

      
      line_tmp = this->visual_->CreateDynamicLine(rendering::RENDERING_LINE_LIST);

      this->update_connection_ = event::Events::ConnectRender(boost::bind(&GraffitiVisualPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      if (sprayer_state > 0 ) { 
        // paint point coordinates relative to wall position and scale
        double x,y,z;

        // paint point size
        // TODO: s must depend on distance from wall
        double s = 0.03; 

        ignition::math::v4::Quaterniond q(model_pose.pose.orientation.w,
                                          model_pose.pose.orientation.x,
                                          model_pose.pose.orientation.y,
                                          model_pose.pose.orientation.z);
                                    
        ignition::math::v4::Vector3d v(1,0,0);
        ignition::math::v4::Vector3d rayVector = q.RotateVector(v);
        
        ignition::math::v4::Vector3d rayPoint(model_pose.pose.position.x,
                                              model_pose.pose.position.y,
                                              model_pose.pose.position.z);
        ignition::math::v4::Vector3d planeNormal(-1, 0, 0);
        ignition::math::v4::Vector3d planePoint(3, 0, 0);
        ignition::math::v4::Vector3d diff = rayPoint - planePoint;
        double prod1 = diff.Dot(planeNormal);  
        double prod2 = rayVector.Dot(planeNormal);
        if (prod2 != 0) {
          double prod3 = prod1 / prod2;
          ignition::math::v4::Vector3d inter = rayPoint - rayVector * prod3;
          x = -0.51; // wall surface. 
          y = inter.Y();
          z = inter.Z();
        }
                      
      //  rendering::DynamicLines *line_tmp = this->visual_->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  
        line_tmp->AddPoint(x, (y - model_position.Y()) / scale.Y(), (z + s) / scale.Z());
        line_tmp->AddPoint(x, (y - model_position.Y())/ scale.Y(), (z - s) / scale.Z());
        
        line_tmp->AddPoint(x, (y + s / 2 - model_position.Y()) / scale.Y(), (z + s / 2) / scale.Z());
        line_tmp->AddPoint(x, (y - s / 2 - model_position.Y()) / scale.Y(), (z - s / 2) / scale.Z());
        
        line_tmp->AddPoint(x, (y - s / 2 - model_position.Y()) / scale.Y(), (z + s / 2) / scale.Z());
        line_tmp->AddPoint(x, (y + s / 2 - model_position.Y()) / scale.Y(), (z - s / 2) / scale.Z());

        line_tmp->AddPoint(x, (y + s - model_position.Y()) / scale.Y(), z / scale.Z());
        line_tmp->AddPoint(x, (y - s - model_position.Y()) / scale.Y(), z / scale.Z());
                
        
      line_tmp->setMaterial("Gazebo/Purple");
      line_tmp->setVisibilityFlags(GZ_VISIBILITY_GUI);
       // line_tmp->setMaterial("Gazebo/Purple");
       // line_tmp->setVisibilityFlags(GZ_VISIBILITY_GUI);

        this->visual_->SetVisible(true); 
      }
      ros::spinOnce();
    }



    public: void modelPositionCallback (const geometry_msgs::PoseStamped::ConstPtr msg) 
    {
        model_pose = *msg;
    }


    public: void sprayerCallback (const std_msgs::UInt8::ConstPtr msg) 
    {
      sprayer_state = msg->data;
    }


    private:
      rendering::VisualPtr visual_;
      rendering::ScenePtr scene_;
      rendering::DynamicLines *line;

      uint8_t sprayer_state;

      geometry_msgs::PoseStamped model_pose;

    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;

    private: ros::NodeHandle* node;

    // UAV model pose subscriber
    ros::Subscriber model_pose_subscriber;
    // sprayer state subscriber
    ros::Subscriber sprayer_subscriber;

    ignition::math::v4::Vector3d scale;
    ignition::math::v4::Vector3d model_position;
     rendering::DynamicLines *line_tmp;

  };


  // Register this plugin with the simulator
  GZ_REGISTER_VISUAL_PLUGIN(GraffitiVisualPlugin)

}

