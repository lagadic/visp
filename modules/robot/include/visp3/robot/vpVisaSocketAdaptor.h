/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Visa simulator adpator.
 *
 * Authors:
 * Andrey Kudryavtsev (Femto-ST)
 *
 *****************************************************************************/

#ifndef __vpVisaApator_h_
#define __vpVisaApator_h_

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cstdlib>
#include <iomanip>
#include <math.h>
#include <vector>
#include <string>
#include <string.h>

#ifdef HAVE_VISP
#  include <visp3/core/vpConfig.h>
#  include <visp3/io/vpImageIo.h>
#  include <visp3/core/vpCameraParameters.h>
#endif

#ifdef VISP_HAVE_OPENCV
#  include <opencv2/core/mat.hpp>
#endif

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#  include <arpa/inet.h>
#  include <netdb.h>
#  include <netinet/in.h>
#  include <sys/socket.h>
#  include <unistd.h>
#else
#  include <io.h>
#  include <winsock2.h>
#endif

/*!
  \class vpVisaSocketAdapter

  \ingroup group_robot_simu_arm

  Allow to communicate by TCP/IP socket with Visa simulator.
*/
class
    #ifdef HAVE_VISP
    VISP_EXPORT
    #endif
    vpVisaSocketAdapter
{
public:
  vpVisaSocketAdapter();
  ~vpVisaSocketAdapter();
  bool connect(const std::string &host = "127.0.0.1", const unsigned int port = 1234);
  void disconnect();

  void getCalibMatrix(std::vector<double> &matrix) const;
#ifdef HAVE_VISP
  vpCameraParameters getCameraParameters() const;
#endif

#ifdef VISP_HAVE_CPPBASE64
  std::string getImage();

#if defined(VISP_HAVE_OPENCV) || defined(HAVE_OPENCV)
  cv::Mat getImageOpenCV();
#endif

#if defined(HAVE_VISP) && (defined(VISP_HAVE_OPENCV) || defined(HAVE_OPENCV))
  vpImage<unsigned char> getImageViSP();
#endif
#endif // VISP_HAVE_CPPBASE64

  void getJointPos(std::vector<double> &values) const;
#ifdef HAVE_VISP
  void getJointPos(vpColVector &values) const;
#endif
  void getToolTransform(std::vector<double> &matrix) const;

  bool homing() const;
  bool isConnected() const { return m_connected; }

  bool setJointPosAbs(const std::vector<double> &joints) const;
#ifdef HAVE_VISP
  bool setJointPosAbs(const vpColVector &joints) const;
#endif
  bool setJointPosRel(const std::vector<double> &joints) const;
#ifdef HAVE_VISP
  bool setJointPosRel(const vpColVector &joints) const;
#endif
  bool setJointVel(const std::vector<double> &velocities) const;
#ifdef HAVE_VISP
  bool setJointVel(const vpColVector &velocities) const;
#endif

private:
  bool sendCmd(const std::string &cmd, const std::vector<double> &args) const;

#ifdef _WIN32
  WSADATA m_WSAData; // configuration socket
  SOCKET m_sock;
  SOCKADDR_IN m_sin;
#elif defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))
  int m_sock;
  struct sockaddr_in m_server_socket;
#endif

  bool m_connected;
  char *m_bufferImage;
};

#endif // __vpVisaApator_h_
