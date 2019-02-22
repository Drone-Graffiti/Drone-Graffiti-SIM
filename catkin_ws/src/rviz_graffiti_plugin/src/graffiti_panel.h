#ifndef GRAFFITI_PANEL_H
#define GRAFFITI_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QPainter>
#include <QLineEdit>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QScrollBar>

#include <geometry_msgs/Twist.h>

#include <rosgraph_msgs/Log.h>
#include <sensor_msgs/BatteryState.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <vector>


class QLineEdit;

namespace rviz_graffiti_plugin
{

class GraffitiPanel: public rviz::Panel
{

Q_OBJECT
public:
  GraffitiPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:

protected Q_SLOTS:

  void guided();
  void takeOff();
  void land();
  void arm();
  void disarm();
  void reboot();
  void start_paint();
  void wpnum_changed();
  void send_to_uav();

  void off_scale_changed();
  

protected:

  QPushButton* guided_button_;
  QPushButton* takeoff_button_;
  QPushButton* land_button_;
  QPushButton* arm_button_;
  QPushButton* disarm_button_;
  QPushButton* start_paint_button_;
  QPushButton* send_param_to_uav_button_;
  QPushButton* reboot_button_;
  

  QLabel* connected_label_;
  QLabel* armed_label_;
  QLabel* mode_label_;
  QLabel* battery_label_;
  QLabel* sysstatus_label_;
  std::vector<QLabel*> sensor_text_label_;
  
  QLineEdit* takeoff_alt_edit_;
  QLineEdit* wpnum_edit_;
  QTextEdit* log_edit_;

  QLineEdit* h_off_edit_;
  QLineEdit* v_off_edit_;
  QLineEdit* h_scale_edit_;
  QLineEdit* v_scale_edit_;
  QLineEdit* fps_edit_;
  QLineEdit* speed_edit_;
  
  ros::Subscriber state_subscriber;
  ros::Subscriber battery_subscriber;
  ros::Subscriber wpnum_subscriber;
  ros::Subscriber rosout_subscriber;
  ros::Subscriber sensor_text_subscriber;

  ros::Publisher navi_cmd_publisher;

  // The ROS node handle.
  ros::NodeHandle nh_;

  private:
    ros::ServiceClient takeoff_service;
    ros::ServiceClient land_service;
    ros::ServiceClient mode_service;
    ros::ServiceClient arming_service;
    ros::ServiceClient command_service;

    void stateCallback(const mavros_msgs::StateConstPtr &state);
    void batteryCallback(const sensor_msgs::BatteryStateConstPtr &battery);
    void wpnumCallback(const std_msgs::Int32ConstPtr &data);
    void rosoutCallback(const rosgraph_msgs::LogConstPtr &log);
    void sensorTextCallback(const visualization_msgs::MarkerConstPtr &txt);

    bool current_paint_button_state = false;
    int32_t current_wp = 0;
    int32_t last_recieved_wp = -1;

};

}

#endif // GRAFFITI_PANEL_H
