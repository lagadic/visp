/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Motion capture using Vicon device.
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_VICON

#include <cmath>
#include <iostream>

#include <visp3/core/vpTime.h>
#include <visp3/sensor/vpMocapVicon.h>

#include <DataStreamClient.h>
#include <IDataStreamClientBase.h>

using namespace ViconDataStreamSDK::CPP;

BEGIN_VISP_NAMESPACE
#ifndef DOXYGEN_SHOULD_SKIP_THIS
class vpMocapVicon::vpMocapViconImpl
{
public:
  vpMocapViconImpl() : m_DirectClient(), m_verbose(false), m_serverAddr() { }
  virtual ~vpMocapViconImpl() { close(); }

  void close()
  {
    m_DirectClient.DisableSegmentData();
    m_DirectClient.DisableMarkerData();
    m_DirectClient.DisableUnlabeledMarkerData();
    m_DirectClient.DisableDeviceData();
    if (m_verbose) {
      std::cout << "Disconnecting..." << std::endl;
    }
    m_DirectClient.Disconnect();
  }

  bool connect()
  {
    int n_attempt = 2;
    for (auto i = 0; i < n_attempt; i++) {
      if (!m_DirectClient.IsConnected().Connected) {
        // Direct connection

        const Output_Connect ConnectResult = m_DirectClient.Connect(m_serverAddr);
        const bool ok = (ConnectResult.Result == Result::Success);

        if (!ok) {
          if (m_verbose) {
            std::cout << "Warning - connection failed... ";
            switch (ConnectResult.Result) {
            case Result::ClientAlreadyConnected:
              std::cout << "Client Already Connected" << std::endl;
              break;
            case Result::InvalidHostName:
              std::cout << "Invalid Host Name" << std::endl;
              break;
            case Result::ClientConnectionFailed:
              std::cout << "Client Connection Failed" << std::endl;
              break;
            default:
              std::cout << "Unrecognized Error: " << ConnectResult.Result << std::endl;
              break;
            }
          }
          vpTime::sleepMs(1000);
        }
        if (ok) {
          if (m_verbose) {
            std::cout << "Successful connection to : " << m_serverAddr << std::endl;
          }
          return setupDataStreamed();
        }
      }
    }

    if (m_verbose) {
      std::cout << "Vicon connection timeout" << std::endl;
    }
    return false;
  }

  bool getBodiesPose(std::map<std::string, vpHomogeneousMatrix> &bodies_pose, bool all_bodies = false)
  {
    if (m_DirectClient.GetFrame().Result == Result::Success) {
      for (unsigned int iBody = 0; iBody < m_DirectClient.GetSubjectCount().SubjectCount; iBody++) {
        std::string bodyName = m_DirectClient.GetSubjectName(iBody).SubjectName;
        std::string rootSegment = m_DirectClient.GetSubjectRootSegmentName(bodyName).SegmentName;
        bool data_extraction_success = (m_DirectClient.GetSegmentGlobalRotationMatrix(bodyName, rootSegment).Result &&
                                        m_DirectClient.GetSegmentGlobalTranslation(bodyName, rootSegment).Result);
        vpHomogeneousMatrix bodyPose;

        if (!data_extraction_success) {
          std::cout << "Error : Could not get pose from body n°" << iBody << std::endl;

          return false;
        }
        else {
          bodyPose[0][3] = m_DirectClient.GetSegmentGlobalTranslation(bodyName, rootSegment).Translation[0] / 1000.0;
          bodyPose[1][3] = m_DirectClient.GetSegmentGlobalTranslation(bodyName, rootSegment).Translation[1] / 1000.0;
          bodyPose[2][3] = m_DirectClient.GetSegmentGlobalTranslation(bodyName, rootSegment).Translation[2] / 1000.0;
          bodyPose[3][3] = 1.0;

          // Vicon is row major
          unsigned int k = 0;
          for (unsigned int i = 0; i < 3; i++) {
            for (unsigned int j = 0; j < 3; j++) {
              bodyPose[i][j] = m_DirectClient.GetSegmentGlobalRotationMatrix(bodyName, rootSegment).Rotation[k++];
            }
          }
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
    return false;
  }

  bool getSpecificBodyPose(const std::string &body_name, vpHomogeneousMatrix &body_pose)
  {
    std::map<std::string, vpHomogeneousMatrix> bodies_pose;
    if (getBodiesPose(bodies_pose, true)) {
      if (bodies_pose.find(body_name) != bodies_pose.end()) {
        body_pose = bodies_pose[body_name];
        return true;
      }
      else {
        std::cout << "The body " << body_name << " was not found in Vicon. Please check the name you typed."
          << std::endl;

        return false;
      }
    }
    else {
      std::cout << "Error : could not process data from Vicon" << std::endl;

      return false;
    }
  }

  void setServerAddress(const std::string &serverAddr) { m_serverAddr = serverAddr; }

  void setVerbose(bool verbose) { m_verbose = verbose; }

  bool setupDataStreamed()
  {
    // We set up the kind of data we get.
    m_DirectClient.EnableSegmentData();
    m_DirectClient.EnableMarkerData();
    m_DirectClient.EnableUnlabeledMarkerData();
    m_DirectClient.EnableMarkerRayData();
    m_DirectClient.EnableDeviceData();
    m_DirectClient.EnableDebugData();
    // We set up which kind of connectiion we want.
    m_DirectClient.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

    // Set the global up axis
    m_DirectClient.SetAxisMapping(Direction::Forward, Direction::Left,
                                  Direction::Up); // Z-up

    return true;
  }

private:
  ViconDataStreamSDK::CPP::Client m_DirectClient;
  bool m_verbose;
  std::string m_serverAddr;
};
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*
 **********************************************************************************************
 */

/*!
 * Default constructor that creates a direct client (not a multi-cast client).
 */
vpMocapVicon::vpMocapVicon() : m_impl(new vpMocapViconImpl()) { }

/*!
 * Destructor.
 */
vpMocapVicon::~vpMocapVicon() { delete m_impl; }

/*!
 * Close connexion with Vicon by stopping the capture.
 */
void vpMocapVicon::close() { m_impl->close(); }

/*!
 * Connect to Vicon mocap server with IP address set using setServerAddress().
 *
 * \return true when connection succeed, false otherwise.
 */
bool vpMocapVicon::connect() { return m_impl->connect(); }

/*!
 * Gets the pose of the different bodies.
 * \param[out] bodies_pose : Map of body names and corresponding poses.
 * \param[in] all_bodies : When true, consider all bodies, even those that are not visible and that contain nan in
 * their pose.
 * \return true if the operation was successful.
 */
bool vpMocapVicon::getBodiesPose(std::map<std::string, vpHomogeneousMatrix> &bodies_pose, bool all_bodies)
{
  return m_impl->getBodiesPose(bodies_pose, all_bodies);
}

/*!
 * Gets the pose of a specific body.
 * \param[out] body_pose : The homogeneous transformation matrix for the specific body.
 * \param[in] body_name : The name of the body.
 * \return true if the operation was successful.
 */
bool vpMocapVicon::getSpecificBodyPose(const std::string &body_name, vpHomogeneousMatrix &body_pose)
{
  return m_impl->getSpecificBodyPose(body_name, body_pose);
}

/*!
 * Set mocap server address.
 * \param[in] serverAddr : Server address.
 *
 * \sa connect()
 */
void vpMocapVicon::setServerAddress(const std::string &serverAddr) { m_impl->setServerAddress(serverAddr); }

/*!
 * Enable or disable verbose mode.
 * \param[in] verbose : When true enable verbose mode, otherwise disable verbose mode.
 */
void vpMocapVicon::setVerbose(bool verbose) { m_impl->setVerbose(verbose); }
END_VISP_NAMESPACE
#else
// Work around to avoid warning:
// libvisp_sensor.a(vpMocapVicon.cpp.o) has no symbols
void dummy_vpMocapVicon() { };
#endif
