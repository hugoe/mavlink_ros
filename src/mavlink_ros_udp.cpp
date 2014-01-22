#include "ros/ros.h"

#include <mavlink_ros/Mavlink.h>
#include "mavlink.h"

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>

ros::Subscriber mavlink_sub;
ros::Publisher mavlink_pub;

using boost::asio::ip::udp;

class udpNode {
public:
  udpNode( ros::NodeHandle& nh );
private:
  void readCallback( const boost::system::error_code& error, size_t bytes_transferred );
  void readStart( unsigned int timeout_ms );

  void mavlinkCallback( const mavlink_ros::Mavlink::ConstPtr& mavlink_ros_msg );
  boost::shared_ptr<boost::asio::ip::udp::socket> m_udp_socket;
  boost::asio::ip::udp::endpoint m_remote_endpoint;
  boost::asio::io_service m_io_service;
  boost::thread m_udp_thread;

  ros::Publisher m_mavlink_pub;
  ros::Subscriber m_mavlink_sub;

  uint8_t m_rx_buffer[ MAVLINK_MAX_PACKET_LEN ];
  uint8_t m_tx_buffer[ MAVLINK_MAX_PACKET_LEN ];

  bool m_verbose;
};

udpNode::udpNode( ros::NodeHandle& nh )
: m_remote_endpoint( boost::asio::ip::address::from_string( "127.0.0.1" ), 14550 ), m_verbose( false ) {
  try {
    m_udp_socket.reset( new udp::socket( m_io_service, udp::endpoint( udp::v4(), 14551 ) ) );
  }
  catch( std::exception& e ) {
    ROS_ERROR_STREAM( e.what() );
    exit( -1 );
  }
  m_mavlink_sub = nh.subscribe<mavlink_ros::Mavlink>( "from", 1000, &udpNode::mavlinkCallback, this );
  m_mavlink_pub = nh.advertise<mavlink_ros::Mavlink>( "to", 1000 );
  ROS_ERROR( "mavlink_ros_udp running1" );

  readStart( 1000 );
  m_udp_thread = boost::thread( boost::bind( &boost::asio::io_service::run, &m_io_service ) );
}

void udpNode::readCallback( const boost::system::error_code& error, size_t bytes_transferred ) {
  ROS_ERROR( "readCallback" );
  if( !error ) {
    mavlink_message_t message;
    mavlink_status_t status;
    for( size_t i = 0; i < bytes_transferred; i++ ) {
      bool msg_received = mavlink_parse_char( MAVLINK_COMM_0, m_rx_buffer[ i ], &message, &status );
      if( msg_received ) {
        mavlink_ros::Mavlink rosmavlink_msg;
        rosmavlink_msg.compid = message.compid;
        rosmavlink_msg.sysid = message.sysid;
        rosmavlink_msg.len = message.len;
        rosmavlink_msg.seq = message.seq;
        rosmavlink_msg.msgid = message.msgid;
        for( int i = 0; i < (message.len + 7) / 8; i++ ) {
          ( rosmavlink_msg.payload64 ).push_back( message.payload64[ i ] );
        }

        m_mavlink_pub.publish( rosmavlink_msg );
      }
    }
  }

  readStart( 1000 );
}

void udpNode::readStart( unsigned int timeout_ms ) {
  m_udp_socket->async_receive( boost::asio::buffer( m_rx_buffer, sizeof( m_rx_buffer ) ),
  boost::bind( &udpNode::readCallback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}

void udpNode::mavlinkCallback( const mavlink_ros::Mavlink::ConstPtr& mavlink_ros_msg ) {
  /**
   * Convert mavlink_ros::Mavlink to mavlink_message_t
   */
  mavlink_message_t msg;
  msg.msgid = mavlink_ros_msg->msgid;
  msg.len = mavlink_ros_msg->len;
  msg.sysid = mavlink_ros_msg->sysid;
  msg.compid = mavlink_ros_msg->compid;
  msg.seq = mavlink_ros_msg->seq;

  static uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;

  //Copy payload from mavlink_msg (from ROS) to the new "real" mavlink message
  std::copy( mavlink_ros_msg->payload64.begin(), mavlink_ros_msg->payload64.end(), msg.payload64 );

  mavlink_finalize_message_chan( &msg, mavlink_ros_msg->sysid, mavlink_ros_msg->compid, MAVLINK_COMM_0, mavlink_ros_msg->len,
  mavlink_crcs[ msg.msgid ] );

  /**
   * Send mavlink_message to UART
   */
  if( m_verbose )
    ROS_INFO( "Sent Mavlink from ROS to UDP, Message-ID: [%i]", mavlink_ros_msg->msgid );

  if( msg.msgid == MAVLINK_MSG_ID_HEARTBEAT ) {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode( &msg, &heartbeat );
    ROS_ERROR_STREAM( "RX: " << (int)heartbeat.mavlink_version << " " << (int)heartbeat.type << " " << (int)heartbeat.system_status );
  }

  size_t message_length = mavlink_msg_to_send_buffer( m_tx_buffer, &msg );
  size_t sent_bytes = m_udp_socket->send_to( boost::asio::buffer( m_tx_buffer, message_length ), m_remote_endpoint );
  //if( message_length != boost::asio::send_to( m_udp_socket, boost::asio::buffer( m_tx_buffer, sizeof( m_tx_buffer ) ) ) {
  if( message_length != sent_bytes ) {
    ROS_ERROR( "Unable to send message over udp." );
    ROS_ERROR_STREAM( message_length << " " << sent_bytes );
  }
}

int main( int argc, char** argv ) {
  ros::init( argc, argv, "mavlink_ros_udp" );

  try {
    ROS_ERROR( "mavlink_ros_udp not yet running" );
    ros::NodeHandle nh( "/mavlink" );
    udpNode node( nh );
    ROS_ERROR( "mavlink_ros_udp running2" );

    ros::spin();

  }
  catch( std::exception& e ) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
