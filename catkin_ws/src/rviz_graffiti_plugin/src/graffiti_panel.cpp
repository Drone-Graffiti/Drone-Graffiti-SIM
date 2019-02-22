#include <stdio.h>

#include "graffiti_panel.h"

namespace rviz_graffiti_plugin
{

GraffitiPanel::GraffitiPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  
  QHBoxLayout* main_layout = new QHBoxLayout;
  QVBoxLayout* left_layout = new QVBoxLayout;
  QVBoxLayout* right_layout = new QVBoxLayout;
  QHBoxLayout* button_layout = new QHBoxLayout;

  button_layout = new QHBoxLayout;
  connected_label_ = new QLabel(" - STATUS UNKNOWN -", this);
  connected_label_->setStyleSheet("QLabel { color : grey; }");
  button_layout->addWidget(connected_label_);
  button_layout->setAlignment(Qt::Alignment::enum_type::AlignHCenter);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  armed_label_ = new QLabel("ARMED?", this);
  armed_label_->setStyleSheet("QLabel { color : grey; }");
  button_layout->addWidget(armed_label_);
  mode_label_ = new QLabel("MODE?", this);
  mode_label_->setStyleSheet("QLabel { color : grey; }");
  button_layout->addWidget(mode_label_);
  //button_layout->setAlignment(Qt::Alignment::enum_type::AlignHCenter);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  battery_label_ = new QLabel("BATTERY?", this);
  battery_label_->setStyleSheet("QLabel { color : grey; }");
  button_layout->addWidget(battery_label_);
  sysstatus_label_ = new QLabel("SYS?", this);
  sysstatus_label_->setStyleSheet("QLabel { color : grey; }");
  button_layout->addWidget(sysstatus_label_);
  //button_layout->setAlignment(Qt::Alignment::enum_type::AlignHCenter);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  guided_button_ = new QPushButton("GUIDED", this);
  button_layout->addWidget(guided_button_);
  arm_button_ = new QPushButton("ARM", this);
  button_layout->addWidget(arm_button_);
  disarm_button_ = new QPushButton("DISARM", this);
  button_layout->addWidget(disarm_button_);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  takeoff_button_ = new QPushButton("TAKEOFF", this);
  button_layout->addWidget(takeoff_button_);
  land_button_ = new QPushButton("LAND", this);
  button_layout->addWidget(land_button_);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  QLabel* takeoff_alt_label_ = new QLabel("Takeoff Alt:", this);
  button_layout->addWidget(takeoff_alt_label_);
  takeoff_alt_edit_ = new QLineEdit(this);
  button_layout->addWidget(takeoff_alt_edit_);
  button_layout->setAlignment(Qt::Alignment::enum_type::AlignLeft);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  button_layout->setMargin(10);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  start_paint_button_ = new QPushButton("START PAINTING FROM", this);
  button_layout->addWidget(start_paint_button_);
  wpnum_edit_= new QLineEdit(this);
  button_layout->addWidget(wpnum_edit_);
  left_layout->addLayout(button_layout);
  
  button_layout = new QHBoxLayout;
  button_layout->setMargin(10);
  left_layout->addLayout(button_layout);


  button_layout = new QHBoxLayout;
  QLabel* l1 = new QLabel("H_OFFSET", this);
  button_layout->addWidget(l1);
  h_off_edit_= new QLineEdit(this);
  button_layout->addWidget(h_off_edit_);
  QLabel* l2 = new QLabel("V_OFFSET", this);
  button_layout->addWidget(l2);
  v_off_edit_= new QLineEdit(this);
  button_layout->addWidget(v_off_edit_);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  QLabel* l3 = new QLabel("H_SCALE ", this);
  button_layout->addWidget(l3);
  h_scale_edit_= new QLineEdit(this);
  button_layout->addWidget(h_scale_edit_);
  QLabel* l4 = new QLabel("V_SCALE ", this);
  button_layout->addWidget(l4);
  v_scale_edit_= new QLineEdit(this);
  button_layout->addWidget(v_scale_edit_);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  QLabel* l5 = new QLabel("FPS ", this);
  button_layout->addWidget(l5);
  fps_edit_= new QLineEdit(this);
  button_layout->addWidget(fps_edit_);
  QLabel* l6 = new QLabel("SPEED (M/S) ", this);
  button_layout->addWidget(l6);
  speed_edit_= new QLineEdit(this);
  button_layout->addWidget(speed_edit_);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  send_param_to_uav_button_ = new QPushButton("SEND TO UAV", this);
  button_layout->addWidget(send_param_to_uav_button_);
  button_layout->setAlignment(Qt::Alignment::enum_type::AlignRight);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  button_layout->setMargin(20);
  left_layout->addLayout(button_layout);
  
  for (int i = 0; i < 3; i++) {
    button_layout = new QHBoxLayout;
    QLabel* l5 = new QLabel(QString::number(i), this);
    button_layout->addWidget(l5);
    button_layout->setAlignment(Qt::Alignment::enum_type::AlignHCenter);
    left_layout->addLayout(button_layout);
    sensor_text_label_.push_back(l5);
  }

  button_layout = new QHBoxLayout;
  button_layout->setMargin(10);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  reboot_button_ = new QPushButton("REBOOT", this);
  button_layout->addWidget(reboot_button_);
  button_layout->setAlignment(Qt::Alignment::enum_type::AlignRight);
  left_layout->addLayout(button_layout);

  button_layout = new QHBoxLayout;
  left_layout->addLayout(button_layout, 10);

  log_edit_ = new QTextEdit(this);
  right_layout->addWidget(log_edit_);

  main_layout->addLayout(left_layout);
  main_layout->addLayout(right_layout);
  setLayout( main_layout );

  connect(takeoff_button_, SIGNAL(clicked()), this, SLOT(takeOff()));
  connect(land_button_, SIGNAL(clicked()), this, SLOT(land()));
  connect(guided_button_, SIGNAL(clicked()), this, SLOT(guided()));
  connect(arm_button_, SIGNAL(clicked()), this, SLOT(arm()));
  connect(disarm_button_, SIGNAL(clicked()), this, SLOT(disarm()));
  connect(start_paint_button_, SIGNAL(clicked()), this, SLOT(start_paint()));
  connect(wpnum_edit_, SIGNAL(editingFinished()), this, SLOT(wpnum_changed()));
  connect(send_param_to_uav_button_, SIGNAL(clicked()), this, SLOT(send_to_uav()));
  connect(reboot_button_, SIGNAL(clicked()), this, SLOT(reboot()));

  connect(h_off_edit_, SIGNAL(editingFinished()), this, SLOT(off_scale_changed()));
  connect(v_off_edit_, SIGNAL(editingFinished()), this, SLOT(off_scale_changed()));
  connect(h_scale_edit_, SIGNAL(editingFinished()), this, SLOT(off_scale_changed()));
  connect(v_scale_edit_, SIGNAL(editingFinished()), this, SLOT(off_scale_changed()));
  connect(fps_edit_, SIGNAL(editingFinished()), this, SLOT(off_scale_changed()));
  connect(speed_edit_, SIGNAL(editingFinished()), this, SLOT(off_scale_changed()));

  takeoff_service = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  land_service = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mode_service = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  arming_service = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  command_service = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

  state_subscriber = nh_.subscribe("/mavros/state", 1, &GraffitiPanel::stateCallback, this);
  battery_subscriber = nh_.subscribe("/mavros/battery", 1, &GraffitiPanel::batteryCallback, this);
  wpnum_subscriber = nh_.subscribe("/paint/wpnum", 1, &GraffitiPanel::wpnumCallback, this);
  rosout_subscriber = nh_.subscribe("/rosout", 50, &GraffitiPanel::rosoutCallback, this);
  sensor_text_subscriber = nh_.subscribe("/paint/visual/text", 10, &GraffitiPanel::sensorTextCallback, this);

  navi_cmd_publisher = nh_.advertise<std_msgs::String>("/paint/command", 1);
}


void GraffitiPanel::guided() {
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = "GUIDED";
    mode_service.call(mode_cmd);
}

void GraffitiPanel::takeOff() {
    mavros_msgs::CommandTOL takeoff_cmd;

    try {
      float takeoff_alt = takeoff_alt_edit_->text().toFloat();
      takeoff_cmd.request.altitude = takeoff_alt;
      takeoff_service.call(takeoff_cmd);
    } catch (std::exception ex) {

    }
}

void GraffitiPanel::land() {
    mavros_msgs::CommandTOL land_cmd;
    land_service.call(land_cmd);
}

void GraffitiPanel::arm() {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    arming_service.call(arm_cmd);
}

void GraffitiPanel::disarm() {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    arming_service.call(arm_cmd);
}

void GraffitiPanel::reboot() {
  ROS_INFO("REBOOTING");
  mavros_msgs::CommandLong restart_cmd;
  restart_cmd.request.command = 246;
  restart_cmd.request.param1 = 1;
  command_service.call(restart_cmd);
}


void GraffitiPanel::start_paint() {
  std_msgs::String navi_cmd;
  current_paint_button_state = !current_paint_button_state;

  if (current_paint_button_state) {
    start_paint_button_->setText("STOP PAINTING, CURRENT WP IS");
    navi_cmd.data = "STARTPAINT";
    navi_cmd_publisher.publish(navi_cmd);
    wpnum_edit_->setEnabled(false);
  } else {
    start_paint_button_->setText("START PAINTING FROM");
    navi_cmd.data = "STOPPAINT";
    navi_cmd_publisher.publish(navi_cmd);
    wpnum_edit_->setEnabled(true);
  }
   
}

void GraffitiPanel::wpnum_changed() {
    bool ok = false;
    int v = wpnum_edit_->text().toInt(&ok);

    if (ok) {
      std_msgs::String navi_cmd;
      navi_cmd.data = "SETWP;" + wpnum_edit_->text().toStdString();
      navi_cmd_publisher.publish(navi_cmd);
    }
}

void GraffitiPanel::off_scale_changed() {
    bool ok = false;
    float h_off = h_off_edit_->text().toFloat(&ok);
    if (!ok) {return;}
    float v_off = v_off_edit_->text().toFloat(&ok);
    if (!ok) {return;}
    float h_sca = h_scale_edit_->text().toFloat(&ok);
    if (!ok) {return;}
    float v_sca = v_scale_edit_->text().toFloat(&ok);
    if (!ok) {return;}
    float fps = fps_edit_->text().toFloat(&ok);
    if (!ok) {return;}
    float spd = speed_edit_->text().toFloat(&ok);
    if (!ok) {return;}

    std_msgs::String navi_cmd;
    navi_cmd.data = "SETSCALE;" + h_off_edit_->text().toStdString();
    navi_cmd.data += ";" + v_off_edit_->text().toStdString();
    navi_cmd.data += ";" + h_scale_edit_->text().toStdString();
    navi_cmd.data += ";" + v_scale_edit_->text().toStdString();
    navi_cmd.data += ";" + fps_edit_->text().toStdString();
    navi_cmd.data += ";" + speed_edit_->text().toStdString();

    navi_cmd_publisher.publish(navi_cmd);
}

void GraffitiPanel::send_to_uav() {
  off_scale_changed();
}

void GraffitiPanel::stateCallback(const mavros_msgs::StateConstPtr &state) {

  if (state->mode == "GUIDED") {
    mode_label_->setStyleSheet("QLabel { color : green; }");
  } else {
    mode_label_->setStyleSheet("QLabel { color : red; }");
  }
  
  mode_label_->setText(QString(state->mode.c_str()));

  switch (state->system_status) 
  {
    case 0 :
      sysstatus_label_->setStyleSheet("QLabel { color : red; }");
      sysstatus_label_->setText("UNKNOWN");
      break;
    case 1 :
      sysstatus_label_->setStyleSheet("QLabel { color : red; }");
      sysstatus_label_->setText("BOOT");
      break;
    case 2 :
      sysstatus_label_->setStyleSheet("QLabel { color : red; }");
      sysstatus_label_->setText("CALIBRATING");
      break;
    case 3 :
      sysstatus_label_->setStyleSheet("QLabel { color : green; }");
      sysstatus_label_->setText("STANDBY");
      break;
    case 4 :
      sysstatus_label_->setStyleSheet("QLabel { color : green; }");
      sysstatus_label_->setText("AIRBORNE");
      break;
    case 5 :
      sysstatus_label_->setStyleSheet("QLabel { color : red; }");
      sysstatus_label_->setText("FAILSAFE");
      break;
    case 6 :
      sysstatus_label_->setStyleSheet("QLabel { color : red; }");
      sysstatus_label_->setText("EMERGENCY");
      break;
    case 7 :
      sysstatus_label_->setStyleSheet("QLabel { color : red; }");
      sysstatus_label_->setText("POWER OFF");
      break;
    case 8 :
      sysstatus_label_->setStyleSheet("QLabel { color : red; }");
      sysstatus_label_->setText("FLIGHT TERMINATE");
      break;
  }

  // enabe button only for airborne or disarmed
  start_paint_button_->setEnabled((state->system_status == 4) || !state->armed);

  // enabe scaling only for disarmed
  h_off_edit_->setEnabled(!state->armed);
  v_off_edit_->setEnabled(!state->armed);
  h_scale_edit_->setEnabled(!state->armed);
  v_scale_edit_->setEnabled(!state->armed);
  fps_edit_->setEnabled(!state->armed);
  speed_edit_->setEnabled(!state->armed);
  send_param_to_uav_button_->setEnabled(!state->armed);

  // enable reboot only for disarmmed
  reboot_button_->setEnabled(!state->armed);
  
  if (state->armed == true) {
    armed_label_->setStyleSheet("QLabel { color : green; }");
    armed_label_->setText("ARMED");
  } else {
    armed_label_->setStyleSheet("QLabel { color : red; }");
    armed_label_->setText("DISARMED");
  }

  if (state->connected == true) {
    connected_label_->setStyleSheet("QLabel { color : green; }");
    connected_label_->setText("CONNECTED");
  } else {
    connected_label_->setStyleSheet("QLabel { color : red; }");
    connected_label_->setText("DISCONNECTED");

    armed_label_->setStyleSheet("QLabel { color : grey; }");
    mode_label_->setStyleSheet("QLabel { color : grey; }");
    battery_label_->setStyleSheet("QLabel { color : grey; }");
    sysstatus_label_->setStyleSheet("QLabel { color : grey; }");
  }
}

void GraffitiPanel::batteryCallback(const sensor_msgs::BatteryStateConstPtr &battery) {

  float voltage = battery->voltage;

  if (voltage > 14) {
    battery_label_->setStyleSheet("QLabel { color : green; }");
  } else if (voltage > 13.5){
    battery_label_->setStyleSheet("QLabel { color : yellow; }");
  } else {
    battery_label_->setStyleSheet("QLabel { color : red; }");
  }
  
  battery_label_->setText("VOLT: " + QString::number(voltage));
}

void GraffitiPanel::wpnumCallback(const std_msgs::Int32ConstPtr &data) {
    current_wp = data->data;
    if (current_wp != last_recieved_wp) {
      wpnum_edit_->setText(QString::number(current_wp));
      last_recieved_wp = current_wp;
    }
}

void GraffitiPanel::rosoutCallback(const rosgraph_msgs::LogConstPtr &log) {
  std::string l = "\n" + log->msg + log_edit_->toPlainText().toStdString();
  log_edit_->setText(QString::fromStdString(l));
  log_edit_->setReadOnly(true);
}

void GraffitiPanel::sensorTextCallback(const visualization_msgs::MarkerConstPtr &txt) {
  int id = txt->id;
  
  if (id > sensor_text_label_.size()) {
    return;
  }
  sensor_text_label_[id]->setText(QString::fromStdString(txt->text));
  std::string style = "QLabel { color : rgb(" + 
                std::to_string((int)txt->color.r * 128) + ", " +
                std::to_string((int)txt->color.g * 128) +", " +
                std::to_string((int)txt->color.b * 128) + "); }";

  sensor_text_label_[id]->setStyleSheet(QString::fromStdString(style));
}

void GraffitiPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "TakeoffAlt",  takeoff_alt_edit_->text());
  config.mapSetValue( "HOFFSET",  h_off_edit_->text());
  config.mapSetValue( "VOFFSET",  v_off_edit_->text());
  config.mapSetValue( "HSCALE",  h_scale_edit_->text());
  config.mapSetValue( "VSCALE",  v_scale_edit_->text());
  config.mapSetValue( "FPS",  fps_edit_->text());
  config.mapSetValue( "SPEED",  speed_edit_->text());

}


// Load all configuration data for this panel from the given Config object.
void GraffitiPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString val;
  if( config.mapGetString( "TakeoffAlt", &val)) {
    takeoff_alt_edit_->setText(val);
  }
  if( config.mapGetString( "HOFFSET", &val)) {
    h_off_edit_->setText(val);
  }
  if( config.mapGetString( "VOFFSET", &val)) {
    v_off_edit_->setText(val);
  }
  if( config.mapGetString( "HSCALE", &val)) {
    h_scale_edit_->setText(val);
  }
  if( config.mapGetString( "VSCALE", &val)) {
    v_scale_edit_->setText(val);
  }
  if( config.mapGetString( "FPS", &val)) {
    fps_edit_->setText(val);
  }
  if( config.mapGetString( "SPEED", &val)) {
    speed_edit_->setText(val);
  }
}


} // end namespace rviz_graffiti_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_graffiti_plugin::GraffitiPanel,rviz::Panel )

