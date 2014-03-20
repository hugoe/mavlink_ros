/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */

#include <ros/ros.h>
#include <mavlink_ros/serial_helpers.h>

/**
 *
 *
 * Returns the file descriptor on success or -1 on error.
 */

int open_port( std::string& port ) {
  int fd; /* File descriptor for the port */

  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C
  fd = open( port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
  if( fd == -1 ) {
    /* Could not open the port. */
    return ( -1 );
  }
  else {
    fcntl( fd, F_SETFL, 0 );
  }

  return ( fd );
}

bool setup_port( int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control ) {
  //struct termios options;

  struct termios config;
  if( !isatty( fd ) ) {
    ROS_ERROR( "mavlink: ERROR: file descriptor %d is NOT a serial port", fd );
    return false;
  }
  if( tcgetattr( fd, &config ) < 0 ) {
    ROS_ERROR( "mavlink: ERROR: could not read configuration of fd %d", fd );
    return false;
  }
  //
  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //
  config.c_iflag &= ~( IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON );
  //
  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  //
  config.c_oflag &= ~( OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST );

#ifdef OLCUC
  config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
  config.c_oflag &= ~ONOEOT;
#endif

  //
  // No line processing:
  // echo off, echo newline off, canonical mode off,
  // extended input processing off, signal chars off
  //
  config.c_lflag &= ~( ECHO | ECHONL | ICANON | IEXTEN | ISIG );
  //
  // Turn off character processing
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  //
  config.c_cflag &= ~( CSIZE | PARENB );
  config.c_cflag |= CS8;
  //
  // One input byte is enough to return from read()
  // Inter-character timer off
  //
  config.c_cc[ VMIN ] = 1;
  config.c_cc[ VTIME ] = 10; // was 0

  // Get the current options for the port
  //tcgetattr(fd, &options);

  switch( baud ) {
  case 1200:
    if( cfsetispeed( &config, B1200 ) < 0 || cfsetospeed( &config, B1200 ) < 0 ) {
      ROS_ERROR( "mavlink: Could not set desired baud rate of %d Baud", baud );
      return false;
    }
    break;
  case 1800:
    if( cfsetispeed( &config, B1800 ) < 0 || cfsetospeed( &config, B1800 ) < 0 ) {
      ROS_ERROR( "mavlink: Could not set desired baud rate of %d Baud", baud );
      return false;
    }
    break;
  case 9600:
    if( cfsetispeed( &config, B9600 ) < 0 || cfsetospeed( &config, B9600 ) < 0 ) {
      ROS_ERROR( "mavlink: Could not set desired baud rate of %d Baud", baud );
      return false;
    }
    break;
  case 19200:
    if( cfsetispeed( &config, B19200 ) < 0 || cfsetospeed( &config, B19200 ) < 0 ) {
      ROS_ERROR( "mavlink: Could not set desired baud rate of %d Baud", baud );
      return false;
    }
    break;
  case 38400:
    if( cfsetispeed( &config, B38400 ) < 0 || cfsetospeed( &config, B38400 ) < 0 ) {
      ROS_ERROR( "mavlink: Could not set desired baud rate of %d Baud", baud );
      return false;
    }
    break;
  case 57600:
    if( cfsetispeed( &config, B57600 ) < 0 || cfsetospeed( &config, B57600 ) < 0 ) {
      ROS_ERROR( "mavlink: Could not set desired baud rate of %d Baud", baud );
      return false;
    }
    break;
  case 115200:
    if( cfsetispeed( &config, B115200 ) < 0 || cfsetospeed( &config, B115200 ) < 0 ) {
      ROS_ERROR( "mavlink: Could not set desired baud rate of %d Baud", baud );
      return false;
    }
    break;

    // These two non-standard (by the 70'ties ) rates are fully supported on
    // current Debian and Mac OS versions (tested since 2010).
  case 460800:
    if( cfsetispeed( &config, B460800 ) < 0 || cfsetospeed( &config, B460800 ) < 0 ) {
      ROS_ERROR( "mavlink: Could not set desired baud rate of %d Baud", baud );
      return false;
    }
    break;
  case 921600:
    if( cfsetispeed( &config, B921600 ) < 0 || cfsetospeed( &config, B921600 ) < 0 ) {
      ROS_ERROR( "mavlink: Could not set desired baud rate of %d Baud", baud );
      return false;
    }
    break;
  default:
    ROS_ERROR( "mavlink: Desired baud rate %d could not be set, aborting.", baud );
    return false;

    break;
  }

  //
  // Finally, apply the configuration
  //
  if( tcsetattr( fd, TCSAFLUSH, &config ) < 0 ) {
    ROS_ERROR( "mavlink: could not set configuration of fd %d\n", fd );
    return false;
  }
  return true;
}

void close_port( int fd ) {
  close( fd );
}

