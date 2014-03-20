/*=====================================================================
 
 MAVCONN Micro Air Vehicle Flying Robotics Toolkit
 
 (c) 2009, 2010 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>
 
 This file is part of the MAVCONN project
 
 MAVCONN is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 MAVCONN is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.
 
 ======================================================================*/

/**
 * @file
 *   @brief The serial interface process
 *
 *   This process connects any external MAVLink UART device to ROS
 *
 *   @author Lorenz Meier, <mavteam@student.ethz.ch>
 *
 */

#include "ros/ros.h"

#include <mavlink_ros/Mavlink.h>
#include <mavlink_ros/mavlink_ros_conversion.h>
#include <mavlink_ros/ros_mavlink_conversion.h>
#include <mavlink_ros/serial_helpers.h>

#include "mavlink.h"
#include <glib.h>

// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

using std::string;
using namespace std;

struct timeval tv;		  ///< System time

int g_baud = 115200;                 ///< The serial baud rate

// Settings
int g_sysid = 42;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int g_compid = 110;
int g_serial_compid = 0;
std::string g_port = "/dev/ttyUSB0";              ///< The serial port name, e.g. /dev/ttyUSB0
bool g_verbose = false;             ///< Enable verbose output
bool g_debug = false;               ///< Enable debug functions and output
int g_fd;

/**
 * Grabs all mavlink-messages from the ROS-Topic "mavlink" and publishes them on ROS
 */
ros::Subscriber g_mavlink_sub;

MavlinkRosConversion* g_mavlink_ros;

/**
 * @brief Serial function
 *
 * This function blocks waiting for serial data in it's own thread
 * and forwards the data once received.
 */
void* serial_wait( void* serial_ptr ) {
  int fd = *( (int*)serial_ptr );

  mavlink_status_t lastStatus;
  lastStatus.packet_rx_drop_count = 0;

  // Blocking wait for new data
  while( 1 ) {
    //if (debug) printf("Checking for new data on serial port\n");
    // Block until data is available, read only one byte to be able to continue immediately
    //char buf[MAVLINK_MAX_PACKET_LEN];
    uint8_t cp;
    mavlink_message_t message;
    mavlink_status_t status;
    uint8_t msgReceived = false;
    //tcflush(fd, TCIFLUSH);
    if( read( fd, &cp, 1 ) > 0 ) {
      // Check if a message could be decoded, return the message in case yes
      msgReceived = mavlink_parse_char( MAVLINK_COMM_1, cp, &message, &status );
      if( lastStatus.packet_rx_drop_count != status.packet_rx_drop_count ) {
        if( g_verbose || g_debug )
          ROS_ERROR( "mavlink: DROPPED %d PACKETS", status.packet_rx_drop_count );
        if( g_debug ) {
          unsigned char v = cp;
          fprintf( stderr, "%02x ", v );
        }
      }
      lastStatus = status;
    }
    else {
      ROS_ERROR( "mavlink: Could not read from port %s", g_port.c_str() );
    }

    // If a message could be decoded, handle it
    if( msgReceived ) {
      // Do not send images over serial port

      // DEBUG output
      if( g_debug ) {
        ROS_DEBUG( "mavlink: Forwarding SERIAL -> ROS: " );
        unsigned int i;
        uint8_t buffer[ MAVLINK_MAX_PACKET_LEN ];
        unsigned int messageLength = mavlink_msg_to_send_buffer( buffer, &message );
        if( messageLength > MAVLINK_MAX_PACKET_LEN ) {
          ROS_ERROR( "mavlink: FATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n" );
        }
        else {
          for( i = 0; i < messageLength; i++ ) {
            unsigned char v = buffer[ i ];
            fprintf( stderr, "%02x ", v );
          }
          fprintf( stderr, "\n" );
        }
      }

      if( g_verbose || g_debug )
        ROS_INFO( "Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid );

      /**
       * Serialize the Mavlink-ROS-message
       */
      g_mavlink_ros->mavlink( message );

      switch( message.msgid ) {
      //https://pixhawk.ethz.ch/mavlink/#ATTITUDE
      case MAVLINK_MSG_ID_ATTITUDE:
        g_mavlink_ros->attitude( message );
        break;

        // https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU
      case MAVLINK_MSG_ID_HIGHRES_IMU:
        g_mavlink_ros->highresImu( message );
        break;

        // https://pixhawk.ethz.ch/mavlink/#OPTICAL_FLOW
      case MAVLINK_MSG_ID_OPTICAL_FLOW:
        g_mavlink_ros->opticalFlow( message );
        break;
      }
    }
  }
  return NULL;
}

void mavlinkCallback( const mavlink_ros::Mavlink &mavlink_ros_msg ) {

  /**
   * Convert mavlink_ros::Mavlink to mavlink_message_t
   */
  mavlink_message_t msg;
  msg.msgid = mavlink_ros_msg.msgid;

  static uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;

  //Copy payload from mavlink_msg (from ROS) to the new "real" mavlink message
  copy( mavlink_ros_msg.payload64.begin(), mavlink_ros_msg.payload64.end(), msg.payload64 );

  mavlink_finalize_message_chan( &msg, mavlink_ros_msg.sysid, mavlink_ros_msg.compid, MAVLINK_COMM_0, mavlink_ros_msg.len,
  mavlink_crcs[ msg.msgid ] );

  /**
   * Send mavlink_message to UART
   */
  if( g_verbose )
    ROS_INFO( "Sent Mavlink from ROS to UART, Message-ID: [%i]", mavlink_ros_msg.msgid );

  // Send message over serial port
  static uint8_t buffer[ MAVLINK_MAX_PACKET_LEN ];
  int messageLength = mavlink_msg_to_send_buffer( buffer, &msg );
  if( g_debug )
    printf( "Writing %d bytes\n", messageLength );
  int written = write( g_fd, (char*)buffer, messageLength );
  tcflush( g_fd, TCOFLUSH );
  if( messageLength != written )
    fprintf( stderr, "ERROR: Wrote %d bytes but should have written %d\n", written, messageLength );
}

int main( int argc, char **argv ) {
  ros::init( argc, argv, "mavlink_ros_serial" );

  // Handling Program options
  ros::NodeHandle ph( "~" );
  ph.param<std::string>( "portname", g_port, "/dev/ttyUSB0" );
  ph.param<int>( "baudrate", g_baud, 115200 );
  ph.param<bool>( "verbose", g_verbose, false );
  ph.param<int>( "sysid", g_sysid, 42 );
  ph.param<int>( "compid", g_compid, 110 );
  ph.param<bool>( "debug", g_debug, false );
  std::string frame_id;
  ph.param<std::string>( "frame_id", frame_id, "base_link" );

  // SETUP SERIAL PORT

  // Exit if opening port failed
  // Open the serial port.
  ROS_INFO( "mavlink: Trying to connect to %s.. ", g_port.c_str() );
  g_fd = open_port( g_port );
  if( g_fd == -1 ) {
    ROS_ERROR( "mavlink: failure, could not open port." );
    exit( EXIT_FAILURE );
  }
  else {
    ROS_INFO( "mavlink: success." );
  }

  ROS_INFO( "Trying to configure %s.. ", g_port.c_str() );
  bool setup = setup_port( g_fd, g_baud, 8, 1, false, false );
  if( !setup ) {
    ROS_ERROR( "mavlink: failure, could not configure port." );
    exit( EXIT_FAILURE );
  }
  else {
    ROS_INFO( "mavlink: success." );
  }
  int* fd_ptr = &g_fd;

  // SETUP ROS
  g_mavlink_ros = new MavlinkRosConversion( frame_id, g_verbose );

  ros::NodeHandle mavlink_nh( "mavlink" );
  g_mavlink_sub = mavlink_nh.subscribe( "to", 1000, mavlinkCallback );

  GThread* serial_thread;
  GError* err;
  if( !g_thread_supported() ) {
    g_thread_init( NULL );
    // Only initialize g thread if not already done
  }

  // Run indefinitely while the ROS and serial threads handle the data
  ROS_INFO( "mavlink: READY, waiting for serial/ROS data." );

  if( ( serial_thread = g_thread_create( (GThreadFunc)serial_wait, (void *)fd_ptr, TRUE, &err ) ) == NULL ) {
    ROS_ERROR( "mavlink: Failed to create serial handling thread: %s!!", err->message );
    g_error_free( err );
  }

  int noErrors = 0;
  if( g_fd == -1 || g_fd == 0 ) {
    ROS_ERROR( "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", g_port.c_str(), g_baud );
    exit( EXIT_FAILURE );
  }
  else {
    ROS_INFO( "Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", g_port.c_str(), g_baud );
  }

  // FIXME ADD MORE CONNECTION ATTEMPTS
  if( g_fd == -1 || g_fd == 0 ) {
    exit( noErrors );
  }

  // Ready to roll
  ROS_INFO( "MAVLINK SERIAL TO ROS BRIDGE STARTED ON MAV %d (COMPONENT ID:%d) - RUNNING..\n\n", g_sysid, g_compid );

  /**
   * Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
   */
  ros::spin();

  close_port( g_fd );

  delete g_mavlink_ros;

  return 0;
}
