/**
 * @brief MocapPoseEstimate plugin
 * @file mocap_pose_estimate.cpp
 * @author Tony Baltovski <tony.baltovski@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016 Vladimir Ermakov, Tony Baltovski.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


namespace mavros {
namespace extra_plugins{
/**
 * @brief MocapPoseEstimate plugin
 *
 * Sends motion capture data to FCU.
 */
class MocapPoseEstimatePlugin : public plugin::PluginBase
{
public:
	MocapPoseEstimatePlugin() : PluginBase(),
		mp_nh("~mocap")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		bool use_tf;
		bool use_pose;

		/** @note For VICON ROS package, subscribe to TransformStamped topic */
		mp_nh.param("use_tf", use_tf, false);

		/** @note For Optitrack ROS package, subscribe to PoseStamped topic */
		mp_nh.param("use_pose", use_pose, true);

		if (use_tf && !use_pose) {
			mocap_tf_sub = mp_nh.subscribe("tf", 1, &MocapPoseEstimatePlugin::mocap_tf_cb, this);
		}
		else if (use_pose && !use_tf) {
			mocap_pose_sub = mp_nh.subscribe("pose", 1, &MocapPoseEstimatePlugin::mocap_pose_cb, this);
		}
		else {
			ROS_ERROR_NAMED("mocap", "Use one motion capture source.");
		}
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle mp_nh;

	ros::Subscriber mocap_pose_sub;
	ros::Subscriber mocap_tf_sub;

	/* -*- low-level send -*- */
	void mocap_pose_send
		(uint64_t usec,
			Eigen::Quaterniond &q,
			Eigen::Vector3d &v)
	{
		mavlink::common::msg::ATT_POS_MOCAP pos;

		pos.time_usec = usec;
		ftf::quaternion_to_mavlink(q, pos.q);
		
		ROS_INFO(">>>> %f %f %f %f", pos.q[0], pos.q[1], pos.q[2],pos.q[3]);
		

		pos.x = v.x();
		pos.y = v.y();
		pos.z = v.z();

		UAS_FCU(m_uas)->send_message_ignore_drop(pos);
	}

		/* -*- low-level send reworked -*- */
	void mocap_pose_send_reworked
		(uint64_t usec,
			geometry_msgs::Pose pose)
	{
		mavlink::common::msg::ATT_POS_MOCAP pos;

		pos.time_usec = usec;
		
		pos.x = pose.position.x;
		pos.y = pose.position.y;
		pos.z = pose.position.z;

		pos.q[0] = pose.orientation.w;
		pos.q[1] = pose.orientation.x;
		pos.q[2] = pose.orientation.y;
		pos.q[3] = pose.orientation.z;

		UAS_FCU(m_uas)->send_message_ignore_drop(pos);
	}

	/* -*- mid-level helpers -*- */
	void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
	{
//		Eigen::Quaterniond q_enu;

//		tf::quaternionMsgToEigen(pose->pose.orientation, q_enu);



		//double yyy = ftf::quaternion_get_yaw(q_enu);

		//ROS_INFO(">>>>> YAW %f", yyy);
				
//		auto q = ftf::transform_orientation_enu_ned(
//					ftf::transform_orientation_baselink_aircraft(q_enu));

//		auto q = q_enu;

		
		//Eigen::Quaterniond q = Eigen::Quaterniond(1, 0, 0, 0);
		

    //  auto rpy = ftf::quaternion_to_rpy(q_enu);
	//	ROS_INFO("RPY = %f, %f, %f", rpy.x(), rpy.y(), rpy.z());


	//	auto position = ftf::transform_frame_enu_ned(
	//			Eigen::Vector3d(
	//				pose->pose.position.x,
	//				pose->pose.position.y,
	//				pose->pose.position.z));

//		mocap_pose_send(pose->header.stamp.toNSec() / 1000,
	//			q,
	//			position);

		geometry_msgs::Pose p = pose->pose;
		
		p.position.z = -p.position.z;
		p.position.x = pose->pose.position.y;
		p.position.y = pose->pose.position.x;
		
		mocap_pose_send_reworked(pose->header.stamp.toNSec() / 1000, p);
	}

	/* -*- callbacks -*- */
	void mocap_tf_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
	{
		Eigen::Quaterniond q_enu;

		tf::quaternionMsgToEigen(trans->transform.rotation, q_enu);

		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(q_enu));

		auto position = ftf::transform_frame_enu_ned(
				Eigen::Vector3d(
					trans->transform.translation.x,
					trans->transform.translation.y,
					trans->transform.translation.z));

		mocap_pose_send(trans->header.stamp.toNSec() / 1000,
				q,
				position);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MocapPoseEstimatePlugin, mavros::plugin::PluginBase)
