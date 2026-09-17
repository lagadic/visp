/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_QUALISYS

#include <cmath>
#include <iostream>

#include <visp3/core/vpTime.h>
#include <visp3/sensor/vpMocapQualisys.h>

#include <qualisys_cpp_sdk/RTPacket.h>
#include <qualisys_cpp_sdk/RTProtocol.h>

BEGIN_VISP_NAMESPACE
#ifndef DOXYGEN_SHOULD_SKIP_THIS
class vpMocapQualisys::vpMocapQualisysImpl
{
public:
  vpMocapQualisysImpl()
    : m_rtProtocol(), m_base_port(22222), m_udp_port(6734), m_big_endian(false),
    m_dataAvailable(false), m_streamFrames(false), m_verbose(false), m_serverAddr()
  {}

  virtual ~vpMocapQualisysImpl() { close(); }

  void close()
  {
    m_rtProtocol.StopCapture();
    m_rtProtocol.Disconnect();
  }

  bool connect()
  {
    unsigned int n_attempt = 2;
    unsigned int attempt = 0;
    while (attempt < n_attempt && !m_rtProtocol.Connected()) {
      if (!m_rtProtocol.Connect(m_serverAddr.c_str(), m_base_port, &m_udp_port, MAJOR_VERSION, MINOR_VERSION, m_big_endian)) {
        std::cout << "Qualisys connection error: " << m_rtProtocol.GetErrorString() << std::endl;
        vpTime::sleepMs(1000);
      }
      else {
        if (m_verbose) {
          unsigned int major, minor;
          if (m_rtProtocol.GetVersion(major, minor)) {
            std::cout << "Qualisys RT Protocol Version " << major << "." << minor << std::endl;
          }

          std::string qtmVersion;
          if (m_rtProtocol.GetQTMVersion(qtmVersion)) {
            std::cout << "Qualisys connected to " << qtmVersion.data() << std::endl;
          }
        }
      }
      ++attempt;
    }

    return verifyDataStreamed(n_attempt);
  }

  bool verifyDataStreamed(unsigned int n_attempt)
  {
    bool readSettingsOK = false;

    unsigned int attempt = 0;
    while (attempt < n_attempt) {
      if (m_verbose) {
        std::cout << "Waiting for data available" << std::endl;
      }
      if (!m_dataAvailable && !m_rtProtocol.Read6DOFSettings(m_dataAvailable)) {
        vpTime::sleepMs(1000);
      }
      else {
        if (m_verbose && !readSettingsOK) {
          std::cout << "Reading 6DOF settings succeeded." << std::endl;
        }
        readSettingsOK = true;
      }
      ++attempt;
    }

    if (!readSettingsOK) {
      if (m_verbose) {
        std::cout << "Reading 6DOF settings timeout: " << std::endl;
      }
      return false;
    }

    attempt = 0;
    while (attempt < n_attempt) {
      if (!m_streamFrames) {
#if (VP_VERSION_INT(MAJOR_VERSION, MINOR_VERSION, 0) >= VP_VERSION_INT(1, 27, 0))
        if (!m_rtProtocol.StreamFrames(CRTProtocol::EStreamRate::RateAllFrames, 0, m_udp_port, nullptr, CRTProtocol::cComponent6d))
#else
        if (!m_rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, m_udp_port, nullptr, CRTProtocol::cComponent6d))
#endif
        {
          if (m_verbose) {
            std::cout << "Streaming frames error: " << m_rtProtocol.GetErrorString() << std::endl;
          }

          vpTime::sleepMs(1000);
        }
        m_streamFrames = true;
      }
      else {
        if (m_verbose) {
          std::cout << "Starting to stream 6DOF data" << std::endl;
        }
        return true;
      }
      ++attempt;
    }

    std::cout << "Streaming frames timeout: " << std::endl;
    return false;
  }

  bool getBodyPose(unsigned int iBody, std::string &name, vpHomogeneousMatrix &M, CRTPacket *rtPacket)
  {
    float fX, fY, fZ;
    float rotationMatrix[9];

    if (rtPacket->Get6DOFBody(iBody, fX, fY, fZ, rotationMatrix)) {
      const char *pTmpStr = m_rtProtocol.Get6DOFBodyName(iBody);
      if (pTmpStr) {
        name = std::string(pTmpStr);
      }
      else {
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
    }
    else {
      return false;
    }
  }

  bool getBodiesPose(std::map<std::string, vpHomogeneousMatrix> &bodies_pose, bool all_bodies)
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
          }
          else if (bodyPose.isValid()) {
            bodies_pose[bodyName] = bodyPose;
          }
        }
        return true;
      }
    }
    return false;
  }

  bool getSpecificBodyPose(const std::string &body_name, vpHomogeneousMatrix &body_pose)
  {
    std::map<std::string, vpHomogeneousMatrix> bodies_pose;
    if (getBodiesPose(bodies_pose, true)) {
      if (bodies_pose.find(body_name) != bodies_pose.end()) {
        body_pose = bodies_pose[body_name];
        if (m_verbose) {
          std::cout << "I found bodyName: " << body_name << std::endl;
        }
        return true;
      }
      else {
        std::cout << "The body " << body_name << " was not found in Qualisys. Please check the name you typed."
          << std::endl;

        return false;
      }
    }
    else {
      std::cout << "Error : could not process data from Qualisys" << std::endl;

      return false;
    }
  }

  void setServerAddress(const std::string &serverAddr) { m_serverAddr = serverAddr; }

  void setBigEndian(bool big_endian) { m_big_endian = big_endian; }

  void setBasePort(unsigned short port) { m_base_port = port; }

  void setUDPPort(unsigned short port) { m_udp_port = port; }

  void setVerbose(bool verbose) { m_verbose = verbose; }

private:
  CRTProtocol m_rtProtocol;
  unsigned short m_base_port;
  unsigned short m_udp_port;
  bool m_big_endian;
  bool m_dataAvailable;
  bool m_streamFrames;
  bool m_verbose;
  std::string m_serverAddr;
};
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*
 **********************************************************************************************
 */

/*!
 * Default constructor with:
 * - base port set by default to 22222
 * - UDP port set by default to 6734
 * - data received as little endian (big endian set to false).
 *
 * \sa setServerAddress(), setBasePort(), setBigEndian(), setUDPPort()
 */
vpMocapQualisys::vpMocapQualisys() : m_impl(new vpMocapQualisysImpl()) {}

/*!
 * Destructor.
 */
vpMocapQualisys::~vpMocapQualisys() { delete m_impl; }

/*!
 * Close connexion with Qualisys by stopping the capture.
 */
void vpMocapQualisys::close() { m_impl->close(); }

/*!
 * Connect to Qualisys mocap server with:
 * - IP address set using setServerAddress()
 * - Base port set by default to 22222 or modified using setBasePort()
 * - UDP port set by default to 6734 or modified using setUDPPort()
 * - Endianness set by default to little endian or modified using setBigENdian()
 *
 * \return true when connection succeed, false otherwise.
 */
bool vpMocapQualisys::connect() { return m_impl->connect(); }

/*!
 * Gets the pose of the different bodies.
 * \param[out] bodies_pose : Map of body names and corresponding poses.
 * \param[in] all_bodies : When true, consider all bodies, even those that are not visible and that contain nan in
 * their pose.
 * \return true if the operation was successful.
 */
bool vpMocapQualisys::getBodiesPose(std::map<std::string, vpHomogeneousMatrix> &bodies_pose, bool all_bodies)
{
  return m_impl->getBodiesPose(bodies_pose, all_bodies);
}

/*!
 * Gets the pose of a specific body.
 * \param[out] body_pose : The homogeneous transformation matrix for the specific body.
 * \param[in] body_name : The name of the body.
 * \return true if the operation was successful, false otherwise.
 */
bool vpMocapQualisys::getSpecificBodyPose(const std::string &body_name, vpHomogeneousMatrix &body_pose)
{
  return m_impl->getSpecificBodyPose(body_name, body_pose);
}
/*!
 * Set mocap server address.
 * \param[in] serverAddr : Server address.
 *
 * \sa connect()
 */
void vpMocapQualisys::setServerAddress(const std::string &serverAddr) { m_impl->setServerAddress(serverAddr); }

/*!
 * Set the base port used to connect to the Qualisys server. By default, the base port is set to 22222.
 * \param[in] port : Base port number.
 *
 * \sa connect()
 */
void vpMocapQualisys::setBasePort(unsigned short port) { m_impl->setBasePort(port); }

/*!
 * Enable or disable big endian mode for the data received from the Qualisys server.
 *
 * \param[in] big_endian : When true, the data received from the server is considered as big endian, otherwise
 * it is considered as little endian.
 *
 * \sa connect()
 */
void vpMocapQualisys::setBigEndian(bool big_endian) { m_impl->setBigEndian(big_endian); }

/*!
 * Set the UDP port used to stream data from the Qualisys server. By default, the UDP port is set to 6734.
 * \param[in] port : UDP port number.
 *
 * \sa connect()
 */
void vpMocapQualisys::setUDPPort(unsigned short port) { m_impl->setUDPPort(port); }

/*!
 * Enable or disable verbose mode.
 * \param[in] verbose : When true enable verbose mode, otherwise disable verbose mode.
 */
void vpMocapQualisys::setVerbose(bool verbose) { m_impl->setVerbose(verbose); }
END_VISP_NAMESPACE
#else
// Work around to avoid warning:
// libvisp_sensor.a(vpMocapQualisys.cpp.o) has no symbols
void dummy_vpMocapQualisys() {}
#endif
