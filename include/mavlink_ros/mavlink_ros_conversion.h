/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */

#ifndef MAVLINK_ROS_CONVERSION_H_
#define MAVLINK_ROS_CONVERSION_H_

#include <ros/ros.h>
#include "mavlink.h"

class MavlinkRosConversion {
public:
  MavlinkRosConversion( std::string frame_id, bool verbose );
  void mavlink( mavlink_message_t& message );
  void attitude( mavlink_message_t& message );
  void highresImu( mavlink_message_t& message );
  void opticalFlow( mavlink_message_t& message );
  void gps( mavlink_message_t& message );
private:
  ros::Publisher m_mavlink_pub;
  ros::Publisher m_imu_pub;
  ros::Publisher m_imu_raw_pub;
  ros::Publisher m_mag_pub;
  ros::Publisher m_temp_pub;
  ros::Publisher m_pressure_pub;
  ros::Publisher m_opt_flow_pub;

  mavlink_highres_imu_t m_imu_raw;
  std::string m_frame_id;
  bool m_verbose;
};




#endif /* MAVLINK_ROS_CONVERSION_H_ */
