/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */

#ifndef SERIAL_HELPERS_H_
#define SERIAL_HELPERS_H_

// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

int open_port( std::string& port );
bool setup_port( int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control );
void close_port( int fd );

#endif /* SERIAL_HELPERS_H_ */
