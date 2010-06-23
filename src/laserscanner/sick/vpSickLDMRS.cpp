/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Sick LD-MRS laser driver on UNIX platform.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifdef UNIX

#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <netdb.h>
#include <string.h>
#include <strings.h>
#include <math.h>

#include "visp/vpSickLDMRS.h"
#include "visp/vpMath.h"
#include "visp/vpDebug.h"

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
{
  ip = "131.254.12.119";
  port = 12002;
  body = new unsigned char [104000];
}

/*!
  Destructor that deallocate the memory for the body messages.
*/
vpSickLDMRS::~vpSickLDMRS()
{
  if (body) 
    delete [] body;
}

/*! 
  Initialize the connexion with the Sick LD-MRS laser scanner.

  \param ip : Ethernet address of the laser.
  \param port : Ethernet port of the laser.

  \return true if the device was initialized, false otherwise.
  
*/
bool vpSickLDMRS::setup(std::string ip, int port)
{
  setIpAddress( ip );
  setPort( port );
  return ( this->setup() );
}

/*! 
  Initialize the connexion with the Sick LD-MRS laser scanner.

  \return true if the device was initialized, false otherwise.
*/
bool vpSickLDMRS::setup()
{
  struct sockaddr_in serv_addr;
  struct hostent *server;
  long arg,oldarg;
  int res;
  struct timeval tv;
  fd_set myset;

  server = gethostbyname( ip.c_str() );

  if (!server) {
    fprintf(stderr, "Impossible to resolve \"%s\"", ip.c_str());
    return false;
  }

  socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  bzero(&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy(server->h_addr, &serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(port);
  if( (arg = fcntl(socket_fd, F_GETFL, NULL)) < 0) { 
     fprintf(stderr, "Error fcntl(..., F_GETFL) (%s)\n", strerror(errno)); 
     return 1; 
  } 
  oldarg=arg;
  arg |= O_NONBLOCK; 
  if( fcntl(socket_fd, F_SETFL, arg) < 0) { 
     fprintf(stderr, "Error fcntl(..., F_SETFL) (%s)\n", strerror(errno)); 
     return 1; 
  } 
  res = connect(socket_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) ;
  if (errno == EINPROGRESS) { 
    tv.tv_sec = 3; 
    tv.tv_usec = 0; 
    FD_ZERO(&myset); 
    FD_SET(socket_fd, &myset); 
    res = select(socket_fd+1, NULL, &myset, NULL, &tv); 
    if (res < 0 && errno != EINTR) { 
      fprintf(stderr, "Error connecting %d - %s\n", errno, strerror(errno)); 
      return 1; 
    } 
    else if (res > 0) { 
      fprintf(stderr,"ok");
    }
    else {
      fprintf(stderr, "Timeout in select() - Cancelling!\n"); 
      return 1;
    }
  }
  if( fcntl(socket_fd, F_SETFL, oldarg) < 0) { 
     fprintf(stderr, "Error fcntl(..., F_SETFL) (%s)\n", strerror(errno)); 
     return 1; 
  } 
  return true;
}

/*!
  Get the measures of the four scan layers.

  \return true if the measures are retrieven, false otherwise.

*/
bool vpSickLDMRS::measure(vpLaserScan laserscan[4])
{
  vpSickLDMRSHeader header;

  // read the 24 bytes header
  if (recv(socket_fd, (void *)&header, sizeof(header), MSG_WAITALL) == -1) {
    perror("recv");
    return false;
  }

  if (header.getMagicWord() != vpSickLDMRSHeader::MagicWordC2) {
    printf("Error, wrong magic number 0x%x!!!\n", header.getMagicWord());
    return false;
  }

  // get the message body
  unsigned int msgLenght = header.getMsgLenght();
  unsigned int len = recv(socket_fd, body, msgLenght, MSG_WAITALL);
  if (len != msgLenght){
    printf("Error, wrong msg lenght.\n");
    return false;
  }

  // check if the message contains measured data
  if (header.getDataType() != vpSickLDMRSHeader::MeasuredData) {
    //printf("The message in not relative to measured data !!!\n");
    return false;
  }

  // decode measured data

  // get the measurement number
  unsigned short measurementId;
  unsigned short *shortptr = (unsigned short *) body;
  measurementId = shortptr[0];

  // get the start timestamp
  unsigned int *intptr=(unsigned int *) (body+6);
  unsigned int seconds = intptr[1];
  unsigned int fractional=intptr[0];
  double startTimestamp = seconds + fractional / 4294967296.; // 4294967296. = 2^32
  // get the end timestamp
  intptr=(unsigned int *) (body+14);
  seconds = intptr[1];
  fractional=intptr[0];
  double endTimestamp = seconds + fractional / 4294967296.; // 4294967296. = 2^32
  // get the number of steps per scanner rotation
  unsigned short numSteps = shortptr[11];

  // get the start/stop angle
  short startAngle = (short)shortptr[12];
  short stopAngle = (short)shortptr[13];
//   std::cout << "angle in [" << startAngle << "; " << stopAngle 
// 	    << "]" << std::endl;
  
  // get the number of points of this measurement
  unsigned short numPoints = shortptr[14];

  int nlayers = 4;
  for (int i=0; i < nlayers; i++) {
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
  double vAngle; // vertical angle in rad
  double rDist; // radial distance in meters
  vpScanPoint scanPoint;
  for (int i=0; i < numPoints; i++) {
    shortptr = (unsigned short *) (body+44+i*10);
    unsigned char layer = ((unsigned char)  body[44+i*10])&0x0F;
    unsigned char echo  = ((unsigned char)  body[44+i*10])>>4;
    //unsigned char flags = (unsigned char)  body[44+i*10+1];
    
    if (echo==0) {
      hAngle = (2.f * M_PI / numSteps)*(short) shortptr[1];
      rDist = 0.01 * shortptr[2]; // cm to meters conversion
      switch(layer) {
      case 0: vAngle = vpMath::rad(-1.2); break;
      case 1: vAngle = vpMath::rad(-0.4); break;
      case 2: vAngle = vpMath::rad( 0.4); break;
      case 3: vAngle = vpMath::rad( 1.2); break;
      }
      //vpTRACE("layer: %d d: %f hangle: %f", layer, rDist, hAngle);
      scanPoint.setPolar(rDist, hAngle, vAngle);
      laserscan[layer].addPoint(scanPoint);
    }
  }
  return true;
}

#endif
