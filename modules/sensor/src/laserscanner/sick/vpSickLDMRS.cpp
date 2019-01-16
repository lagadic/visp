/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Sick LD-MRS laser driver on UNIX platform.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpTime.h>
#include <visp3/sensor/vpSickLDMRS.h>
//#include <strings.h>
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>

/*!

  \file vpSickLDMRS.cpp

  \brief Driver for the Sick LD-MRS laser scanner.
*/

/*!

  Default constructor that initialize the Ethernet address to
  "131.254.12.119", set the port to 12002 and allocates memory for the
  body messages.
*/
vpSickLDMRS::vpSickLDMRS()
  : socket_fd(-1), body(NULL), vAngle(), time_offset(0), isFirstMeasure(true), maxlen_body(104000)
{
  ip = "131.254.12.119";
  port = 12002;
  body = new unsigned char[maxlen_body];

  vAngle.resize(4); // Vertical angle of the 4 layers
  vAngle[0] = vpMath::rad(-1.2);
  vAngle[1] = vpMath::rad(-0.4);
  vAngle[2] = vpMath::rad(0.4);
  vAngle[3] = vpMath::rad(1.2);
}

/*!
  Destructor that deallocate the memory for the body messages.
*/
vpSickLDMRS::~vpSickLDMRS()
{
  if (body)
    delete[] body;
}

/*!
  Initialize the connection with the Sick LD-MRS laser scanner.

  \param ip_address : Ethernet address of the laser.
  \param com_port : Ethernet port of the laser.

  \return true if the device was initialized, false otherwise.

*/
bool vpSickLDMRS::setup(const std::string &ip_address, int com_port)
{
  setIpAddress(ip_address);
  setPort(com_port);
  return (this->setup());
}

/*!
  Initialize the connection with the Sick LD-MRS laser scanner.

  \return true if the device was initialized, false otherwise.
*/
bool vpSickLDMRS::setup()
{
  struct sockaddr_in serv_addr;
  int res;
  struct timeval tv;
  fd_set myset;

  // Create the TCP socket
  socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (socket_fd < 0) {
    fprintf(stderr, "Failed to create socket\n");
    return false;
  }
  // bzero(&serv_addr, sizeof(serv_addr));
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;                    // Internet/IP
  serv_addr.sin_addr.s_addr = inet_addr(ip.c_str()); // IP address
  serv_addr.sin_port = htons(port);                  // server port

  // Establish connection
  res = connect(socket_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
  if (errno == EINPROGRESS) {
    tv.tv_sec = 3;
    tv.tv_usec = 0;
    FD_ZERO(&myset);
    FD_SET(static_cast<unsigned int>(socket_fd), &myset);
    res = select(socket_fd + 1, NULL, &myset, NULL, &tv);
    if (res < 0 && errno != EINTR) {
      fprintf(stderr, "Error connecting to server %d - %s\n", errno, strerror(errno));
      return false;
    } else if (res > 0) {
      fprintf(stderr, "ok");
    } else {
      fprintf(stderr, "Timeout in select() - Cancelling!\n");
      return false;
    }
  }

  return true;
}

/*!
  Get the measures of the four scan layers.

  \return true if the measures are retrieven, false otherwise.

*/
bool vpSickLDMRS::measure(vpLaserScan laserscan[4])
{
  unsigned int *uintptr;
  unsigned short *ushortptr;
  static unsigned char header[24];
  ushortptr = (unsigned short *)header;
  uintptr = (unsigned int *)header;

  assert(sizeof(header) == 24);
  // std::cout << "size " << sizeof(header) << std::endl;

  double time_second = 0;

  if (isFirstMeasure) {
    time_second = vpTime::measureTimeSecond();
  }

  // read the 24 bytes header
  if (recv(socket_fd, header, sizeof(header), MSG_WAITALL) == -1) {
    printf("recv\n");
    perror("recv");
    return false;
  }

  if (ntohl(uintptr[0]) != vpSickLDMRS::MagicWordC2) {
    printf("Error, wrong magic number !!!\n");
    return false;
  }

  // get the message body
  uint16_t msgtype = ntohs(ushortptr[7]);
  uint32_t msgLength = ntohl(uintptr[2]);

  ssize_t len = recv(socket_fd, body, msgLength, MSG_WAITALL);
  if (len != (ssize_t)msgLength) {
    printf("Error, wrong msg length: %d of %d bytes.\n", (int)len, msgLength);
    return false;
  }

  if (msgtype != vpSickLDMRS::MeasuredData) {
    // printf("The message in not relative to measured data !!!\n");
    return true;
  }

  // decode measured data

  // get the measurement number
  unsigned short measurementId;
  ushortptr = (unsigned short *)body;
  measurementId = ushortptr[0];

  // get the start timestamp
  uintptr = (unsigned int *)(body + 6);
  unsigned int seconds = uintptr[1];
  unsigned int fractional = uintptr[0];
  double startTimestamp = seconds + fractional / 4294967296.; // 4294967296. = 2^32

  // get the end timestamp
  uintptr = (unsigned int *)(body + 14);
  seconds = uintptr[1];
  fractional = uintptr[0];
  double endTimestamp = seconds + fractional / 4294967296.; // 4294967296. = 2^32

  // compute the time offset to bring the measures in the Unix time reference
  if (isFirstMeasure) {
    time_offset = time_second - startTimestamp;
    isFirstMeasure = false;
  }

  startTimestamp += time_offset;
  endTimestamp += time_offset;

  // get the number of steps per scanner rotation
  unsigned short numSteps = ushortptr[11];

  // get the start/stop angle
  short startAngle = (short)ushortptr[12];
  short stopAngle = (short)ushortptr[13];
  //   std::cout << "angle in [" << startAngle << "; " << stopAngle
  // 	    << "]" << std::endl;

  // get the number of points of this measurement
  unsigned short numPoints = ushortptr[14];

  int nlayers = 4;
  for (int i = 0; i < nlayers; i++) {
    laserscan[i].clear();
    laserscan[i].setMeasurementId(measurementId);
    laserscan[i].setStartTimestamp(startTimestamp);
    laserscan[i].setEndTimestamp(endTimestamp);
    laserscan[i].setNumSteps(numSteps);
    laserscan[i].setStartAngle(startAngle);
    laserscan[i].setStopAngle(stopAngle);
    laserscan[i].setNumPoints(numPoints);
  }

  // decode the measured points
  double hAngle; // horizontal angle in rad
  double rDist;  // radial distance in meters
  vpScanPoint scanPoint;

  if (numPoints > USHRT_MAX - 2)
    throw(vpException(vpException::ioError, "Out of range number of point"));

  for (int i = 0; i < numPoints; i++) {
    ushortptr = (unsigned short *)(body + 44 + i * 10);
    unsigned char layer = ((unsigned char)body[44 + i * 10]) & 0x0F;
    unsigned char echo = ((unsigned char)body[44 + i * 10]) >> 4;
    // unsigned char flags = (unsigned char)  body[44+i*10+1];

    if (echo == 0) {
      hAngle = (2.f * M_PI / numSteps) * (short)ushortptr[1];
      rDist = 0.01 * ushortptr[2]; // cm to meters conversion

      // vpTRACE("layer: %d d: %f hangle: %f", layer, rDist, hAngle);
      scanPoint.setPolar(rDist, hAngle, vAngle[layer]);
      laserscan[layer].addPoint(scanPoint);
    }
  }
  return true;
}

#endif
