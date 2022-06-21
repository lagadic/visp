/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * Motion capture using Qualisys device.
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_QUALISYS

#include <cmath>
#include <iostream>

#include <visp3/core/vpTime.h>
#include <visp3/sensor/vpMocapQualisys.h>

/*!
 * Default constructor.
 */
vpMocapQualisys::vpMocapQualisys()
  : m_rtProtocol(), m_basePort(22222), m_udpPort(6734), m_majorVersion(1), m_minorVersion(19), m_bigEndian(false),
    m_dataAvailable(false), m_streamFrames(false)
{
}

/*!
 * Destructor.
 */
vpMocapQualisys::~vpMocapQualisys() { close(); }

/*!
 * Close connexion with Qualisys by stopping the capture.
 */
void vpMocapQualisys::close()
{
  m_rtProtocol.StopCapture();
  m_rtProtocol.Disconnect();
}

/*!
 * Connect to Qualisys mocap server with IP address set using setServerAddress().
 *
 * \return true when connection succeed, false otherwise.
 */
bool vpMocapQualisys::connect()
{
  int n_attempt = 2;
  for (auto i = 0; i < n_attempt; i++) {
    if (!m_rtProtocol.Connected()) {
      if (!m_rtProtocol.Connect(m_serverAddr.c_str(), m_basePort, &m_udpPort, m_majorVersion, m_minorVersion,
                                m_bigEndian)) {
        std::cout << "Qualisys connection error: " << m_rtProtocol.GetErrorString() << std::endl;

        vpTime::sleepMs(1000);
      }
    } else {
      if (m_verbose) {
        std::cout << "Qualisys connected" << std::endl;
      }
      return verifyDataStreamed();
    }
  }

  std::cout << "Qualisys connection timeout" << std::endl;

  return false;
}

/*!
 * Reads and then streams the 6DOF data from Qualisys Mocap
 *
 * \return true when data is complete and streaming succeeds within 6 seconds each, false otherwise.
 */
bool vpMocapQualisys::verifyDataStreamed()
{
  bool readSettingsOK = false;

  for (auto i = 0; i < 6; i++) {
    if (!m_dataAvailable) {
      if (!m_rtProtocol.Read6DOFSettings(m_dataAvailable)) {
        if (m_verbose) {
          std::cout << "Reading 6DOF settings error: " << m_rtProtocol.GetErrorString() << std::endl;
        }

        vpTime::sleepMs(1000);
      }
    } else {
      if (m_verbose && !readSettingsOK) {
        std::cout << "Reading 6DOF settings succeded." << std::endl;
      }
      readSettingsOK = true;
    }
  }

  if (!readSettingsOK) {
    if (m_verbose) {
      std::cout << "Reading 6DOF settings timeout: " << std::endl;
    }
    return false;
  } else {
    for (auto i = 0; i < 6; i++) {
      if (!m_streamFrames) {
        if (!m_rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, m_udpPort, NULL, CRTProtocol::cComponent6d)) {
          if (m_verbose) {
            std::cout << "Streaming frames error: " << m_rtProtocol.GetErrorString() << std::endl;
          }

          vpTime::sleepMs(1000);
        }
        m_streamFrames = true;
      } else {
        if (m_verbose) {
          std::cout << "Starting to stream 6DOF data" << std::endl;
        }
        return true;
      }
    }

    std::cout << "Streaming frames timeout: " << std::endl;

    return false;
  }
}

/*!
 * Get the name and the pose of the body with number `iBody`.
 * \param[in] iBody : Indicates that we want to get the data for the body with number `iBody` in the packet.
 * \param[out] name : Corresponding name of the body.
 * \param[out] M : Corresponding homogeneous transformation matrix between the body's frame and the
 * reference frame.
 * \param[in] rtPacket : The CRTPacket containing the data.

 * \return true if the operation was successful.
 */
bool vpMocapQualisys::getBodyPose(int iBody, std::string &name, vpHomogeneousMatrix &M, CRTPacket *rtPacket)
{
  float fX, fY, fZ;
  float rotationMatrix[9];

  if (rtPacket->Get6DOFBody(iBody, fX, fY, fZ, rotationMatrix)) {
    const char *pTmpStr = m_rtProtocol.Get6DOFBodyName(iBody);
    if (pTmpStr) {
      name = std::string(pTmpStr);
    } else {
      if (m_verbose) {
        std::cout << "Unknown body" << std::endl;
      }
      return false;
    }

    M[0][3] = fX / 1000.;
    M[1][3] = fY / 1000.;
    M[2][3] = fZ / 1000.;
    M[3][3] = 1.;
    unsigned int k = 0;
    for (unsigned int j = 0; j < 3; j++) {
      for (unsigned int i = 0; i < 3; i++) {
        M[i][j] = rotationMatrix[k++];
      }
    }

    return true;
  } else {
    return false;
  }
}

/*!
 * Gets the pose of the different bodies.
 * \param[out] bodies_pose : Map of body names and corresponding poses.
 * \param[in] all_bodies : When true, consider all bodies, even those that are not visible and that contain nan in
 * their pose.
 * \return true if the operation was successful.
 */
bool vpMocapQualisys::getBodiesPose(std::map<std::string, vpHomogeneousMatrix> &bodies_pose, bool all_bodies)
{
  CRTPacket::EPacketType packetType;

  if (m_rtProtocol.Receive(packetType, true) == CNetwork::ResponseType::success) {
    if (packetType == CRTPacket::PacketData) {
      CRTPacket *rtPacket = m_rtProtocol.GetRTPacket();
      for (unsigned int iBody = 0; iBody < rtPacket->Get6DOFBodyCount(); iBody++) {
        std::string bodyName;

        vpHomogeneousMatrix bodyPose;
        if (!getBodyPose(iBody, bodyName, bodyPose, rtPacket)) {
          std::cout << "Error : Could not get pose from body n°" << iBody << std::endl;

          return false;
        }
        if (all_bodies) {
          bodies_pose[bodyName] = bodyPose;
        } else if (bodyPose.isValid()) {
          bodies_pose[bodyName] = bodyPose;
        }
      }
      return true;
    }
  }
  return false;
}

/*!
 * Gets the pose of a specific body.
 * \param[out] body_pose : The homogeneous transformation matrix for the specific body.
 * \param[in] body_name : The name of the body.
 * \return true if the operation was successful, false otherwise.
 */
bool vpMocapQualisys::getSpecificBodyPose(const std::string &body_name, vpHomogeneousMatrix &body_pose)
{
  std::map<std::string, vpHomogeneousMatrix> bodies_pose;
  if (getBodiesPose(bodies_pose, true)) {
    if (bodies_pose.find(body_name) != bodies_pose.end()) {
      body_pose = bodies_pose[body_name];
      if (m_verbose) {
        std::cout << "I found bodyName" << body_name << std::endl;
      }
      return true;
    } else {
      std::cout << "The body " << body_name << " was not found in Qualisys. Please check the name you typed."
                << std::endl;

      return false;
    }
  } else {
    std::cout << "Error : could not process data from Qualisys" << std::endl;

    return false;
  }
}
#else
// Work arround to avoid warning:
// libvisp_sensor.a(vpMocapQualisys.cpp.o) has no symbols
void dummy_vpMocapQualisys(){};
#endif
