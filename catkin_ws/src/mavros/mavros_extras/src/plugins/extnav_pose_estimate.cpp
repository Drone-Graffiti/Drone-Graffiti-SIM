/**
 * @brief ExtNavPoseEstimate plugin
 * @file extnav_pose_estimate.cpp
 * @author Mike Charikov <mcharikov@ugcs.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018, Mike Charikov
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>


namespace mavros {
namespace extra_plugins{
/**
 * @brief ExtNavPoseEstimate plugin
 *
 * Sends external nav drone position data to FCU.
 */

class ExtNavPoseEstimatePlugin : public plugin::PluginBase
{
public:
	ExtNavPoseEstimatePlugin() : PluginBase(),
		mp_nh("~extnav")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		extnav_pose_sub = mp_nh.subscribe("pose_ned", 1, &ExtNavPoseEstimatePlugin::extnav_pose_ned_cb, this);
        extnav_vel_sub = mp_nh.subscribe("vel_ned", 1, &ExtNavPoseEstimatePlugin::extnav_vel_ned_cb, this);
		extnav_pozyx_logger = mp_nh.subscribe("log", 1, &ExtNavPoseEstimatePlugin::log_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { };
	}

private:
	ros::NodeHandle mp_nh;

	ros::Subscriber extnav_pose_sub;
    ros::Subscriber extnav_vel_sub;
	ros::Subscriber extnav_pozyx_logger;


	void extnav_pose_ned_send (uint64_t usec,
		 double roll, double pitch, double yaw,
		 double x, double y, double z)
	{
		mavlink::common::msg::VISION_POSITION_ESTIMATE vpe;

		vpe.usec = usec;
		vpe.x = x;
		vpe.y = y;
		vpe.z = z;
		vpe.roll = roll;
		vpe.pitch = pitch;
		vpe.yaw = yaw;

		UAS_FCU(m_uas)->send_message_ignore_drop(vpe);
	}

	void extnav_pose_ned_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
	{
    	tf2::Quaternion q_ned;
		tf2::fromMsg(pose->pose.orientation, q_ned);

		double yaw, pitch, roll;
		tf2::getEulerYPR(q_ned, yaw, pitch, roll);
				
		extnav_pose_ned_send(pose->header.stamp.toNSec() / 1000, roll, pitch, yaw,
				pose->pose.position.x, pose->pose.position.y, pose->pose.position.z );
	}

    void extnav_vel_ned_cb(const geometry_msgs::Vector3::ConstPtr &vel)
    {
        mavlink::common::msg::VISION_SPEED_ESTIMATE vve;

        vve.usec = ros::Time::now().toNSec() / 1000;
        vve.x = vel->x;
        vve.y = vel->y;
        vve.z = vel->z;

        UAS_FCU(m_uas)->send_message_ignore_drop(vve);
    }

	void log_cb(const std_msgs::Float32MultiArrayConstPtr &arr) {
		mavlink::ardupilotmega::msg::DATA64 data64;
		data64.type = 66;
		if (arr->data.size() != 12) {
			return;
		}
		for (int i = 0; i < 12; i++) {
			int32_t v = (int32_t)(arr->data[i] * 1000);
			data64.data[i * 4] = (v >> 24) & 0xFF;
			data64.data[i * 4 + 1] = (v >> 16) & 0xFF;
			data64.data[i * 4 + 2] = (v >> 8) & 0xFF;
			data64.data[i * 4 + 3] = v  & 0xFF;
		}

		UAS_FCU(m_uas)->send_message_ignore_drop(data64);
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ExtNavPoseEstimatePlugin, mavros::plugin::PluginBase)
