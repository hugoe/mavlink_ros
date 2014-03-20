/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */

#include <mavlink_ros/mavlink_ros_conversion.h>

#include <mavlink_ros/Mavlink.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <px_comm/OpticalFlow.h>

#include <mavlink_ros/math.h>

MavlinkRosConversion::MavlinkRosConversion( std::string frame_id, bool verbose )
: m_frame_id( frame_id ), m_verbose( verbose ) {
  ros::NodeHandle mavlink_nh( "mavlink" );
  m_mavlink_pub = mavlink_nh.advertise<mavlink_ros::Mavlink>( "from", 1000 );

  ros::NodeHandle nh( "fcu" );
  m_imu_pub = nh.advertise<sensor_msgs::Imu>( "imu", 10 );
  m_mag_pub = nh.advertise<sensor_msgs::MagneticField>( "mag", 10 );
  m_temp_pub = nh.advertise<sensor_msgs::Temperature>( "temperature", 10 );
  m_pressure_pub = nh.advertise<sensor_msgs::FluidPressure>( "abs_pressure", 10 );

  ros::NodeHandle raw_nh( "fcu/raw" );
  m_imu_raw_pub = raw_nh.advertise<sensor_msgs::Imu>( "imu", 10 );

  ros::NodeHandle opt_flow_nh( "opt_flow" );
  m_opt_flow_pub = opt_flow_nh.advertise<px_comm::OpticalFlow>( "opt_flow", 10 );
}

void MavlinkRosConversion::mavlink( mavlink_message_t& message ) {
  mavlink_ros::Mavlink rosmavlink_msg;

  rosmavlink_msg.header.stamp = ros::Time::now();
  rosmavlink_msg.len = message.len;
  rosmavlink_msg.seq = message.seq;
  rosmavlink_msg.sysid = message.sysid;
  rosmavlink_msg.compid = message.compid;
  rosmavlink_msg.msgid = message.msgid;

  for( int i = 0; i < ( message.len + 7 ) / 8; i++ ) {
    ( rosmavlink_msg.payload64 ).push_back( message.payload64[ i ] );
  }

  m_mavlink_pub.publish( rosmavlink_msg );
}

void MavlinkRosConversion::attitude( mavlink_message_t& message ) {
  if( m_imu_pub.getNumSubscribers() > 0 ) {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode( &message, &att );

    sensor_msgs::ImuPtr imu_msg( new sensor_msgs::Imu );

    angle2quaternion( att.roll, -att.pitch, -att.yaw, &( imu_msg->orientation.w ), &( imu_msg->orientation.x ), &( imu_msg->orientation.y ),
    &( imu_msg->orientation.z ) );

    // TODO: check/verify that these are body-fixed
    imu_msg->angular_velocity.x = att.rollspeed;
    imu_msg->angular_velocity.y = -att.pitchspeed;
    imu_msg->angular_velocity.z = -att.yawspeed;

    // take this from imu high res message, this is supposed to arrive before this one and should pretty much be in sync then
    imu_msg->linear_acceleration.x = m_imu_raw.xacc;
    imu_msg->linear_acceleration.y = -m_imu_raw.yacc;
    imu_msg->linear_acceleration.z = -m_imu_raw.zacc;

    // TODO: can we fill in the covariance here from a parameter that we set from the specs/experience?
    for( sensor_msgs::Imu::_orientation_covariance_type::iterator it = imu_msg->orientation_covariance.begin();
    it != imu_msg->orientation_covariance.end(); ++it )
      *it = 0;

    for( sensor_msgs::Imu::_angular_velocity_covariance_type::iterator it = imu_msg->angular_velocity_covariance.begin();
    it != imu_msg->angular_velocity_covariance.end(); ++it )
      *it = 0;

    for( sensor_msgs::Imu::_linear_acceleration_covariance_type::iterator it = imu_msg->linear_acceleration_covariance.begin();
    it != imu_msg->linear_acceleration_covariance.end(); ++it )
      *it = 0;

    imu_msg->header.frame_id = m_frame_id;
    imu_msg->header.seq = m_imu_raw.time_usec / 1000;
    imu_msg->header.stamp = ros::Time::now();

    m_imu_pub.publish( imu_msg );
  }
}

void MavlinkRosConversion::highresImu( mavlink_message_t& message ) {
  /* decode message */
  mavlink_msg_highres_imu_decode( &message, &m_imu_raw );

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.seq = m_imu_raw.time_usec / 1000;
  header.frame_id = m_frame_id;

  if( m_imu_raw_pub.getNumSubscribers() > 0 ) {

    sensor_msgs::ImuPtr imu_msg( new sensor_msgs::Imu );

    imu_msg->angular_velocity.x = m_imu_raw.xgyro;
    imu_msg->angular_velocity.y = -m_imu_raw.ygyro;
    imu_msg->angular_velocity.z = -m_imu_raw.zgyro;

    imu_msg->linear_acceleration.x = m_imu_raw.xacc;
    imu_msg->linear_acceleration.y = -m_imu_raw.yacc;
    imu_msg->linear_acceleration.z = -m_imu_raw.zacc;

    // TODO: can we fill in the covariance here from a parameter that we set from the specs/experience?
    for( sensor_msgs::Imu::_angular_velocity_covariance_type::iterator it = imu_msg->angular_velocity_covariance.begin();
    it != imu_msg->angular_velocity_covariance.end(); ++it )
      *it = 0;

    for( sensor_msgs::Imu::_linear_acceleration_covariance_type::iterator it = imu_msg->linear_acceleration_covariance.begin();
    it != imu_msg->linear_acceleration_covariance.end(); ++it )
      *it = 0;

    imu_msg->orientation_covariance[ 0 ] = -1;

    imu_msg->header = header;

    m_imu_raw_pub.publish( imu_msg );

    if( m_verbose )
      ROS_INFO_THROTTLE( 1, "Published IMU message (sys:%d|comp:%d):\n", message.sysid, message.compid );
  }
  if( m_mag_pub.getNumSubscribers() > 0 ) {
    const double gauss_to_tesla = 1.0e-4;
    sensor_msgs::MagneticFieldPtr mag_msg( new sensor_msgs::MagneticField );

    mag_msg->magnetic_field.x = m_imu_raw.xmag * gauss_to_tesla;
    mag_msg->magnetic_field.y = m_imu_raw.ymag * gauss_to_tesla;
    mag_msg->magnetic_field.z = m_imu_raw.zmag * gauss_to_tesla;

    // TODO: again covariance
    for( sensor_msgs::MagneticField::_magnetic_field_covariance_type::iterator it = mag_msg->magnetic_field_covariance.begin();
    it != mag_msg->magnetic_field_covariance.end(); ++it )
      *it = 0;

    mag_msg->header = header;
    m_mag_pub.publish( mag_msg );
  }

  if( m_temp_pub.getNumSubscribers() > 0 ) {
    sensor_msgs::TemperaturePtr temp_msg( new sensor_msgs::Temperature );
    temp_msg->header = header;
    temp_msg->temperature = m_imu_raw.temperature;
    temp_msg->variance = 0.;
    m_temp_pub.publish( temp_msg );
  }

  if( m_pressure_pub.getNumSubscribers() > 0 ) {
    const double millibar_to_pascal = 100.;
    sensor_msgs::FluidPressurePtr pressure_msg( new sensor_msgs::FluidPressure );
    pressure_msg->header = header;
    pressure_msg->fluid_pressure = m_imu_raw.abs_pressure * millibar_to_pascal;
    pressure_msg->variance = 0.;
    m_pressure_pub.publish( pressure_msg );
  }
}

void MavlinkRosConversion::opticalFlow( mavlink_message_t& message ) {
  /* decode message */
  mavlink_optical_flow_t flow;
  mavlink_msg_optical_flow_decode( &message, &flow );

  ros::Time flow_stamp( flow.time_usec / 1000000, ( flow.time_usec % 1000000 ) * 1000 );
  ros::Time host_stamp = ros::Time::now();

  px_comm::OpticalFlow opt_flow_msg;
  opt_flow_msg.header.stamp = host_stamp;
  //flowStamp + m_initialTimestampOffset; //correctedStamp; //ros::Time(flow.time_usec / 1000000, (flow.time_usec % 1000000) * 1000 );
  opt_flow_msg.px4stamp = flow_stamp;
  opt_flow_msg.header.frame_id = m_frame_id;
  opt_flow_msg.ground_distance = flow.ground_distance;
  opt_flow_msg.flow_x = flow.flow_x;
  opt_flow_msg.flow_y = flow.flow_y;
  opt_flow_msg.velocity_x = flow.flow_comp_m_x;
  opt_flow_msg.velocity_y = flow.flow_comp_m_y;
  opt_flow_msg.quality = flow.quality;

  m_opt_flow_pub.publish( opt_flow_msg );
}

void MavlinkRosConversion::gps( mavlink_message_t& message ) {

}
