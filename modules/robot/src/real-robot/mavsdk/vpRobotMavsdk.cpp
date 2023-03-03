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
 * Interface to mavlink compatible controller using mavsdk 3rd party
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MAVSDK) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)

#include <iostream>
#include <math.h>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/calibration/calibration.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <visp3/core/vpExponentialMap.h> // For velocity computation
#include <visp3/robot/vpRobotMavsdk.h>

using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
class vpRobotMavsdk::vpRobotMavsdkImpl
{
public:
  vpRobotMavsdkImpl() : m_takeoffAlt(1.0) {}
  vpRobotMavsdkImpl(const std::string &connection_info) : m_takeoffAlt(1.0) { connect(connection_info); }

  virtual ~vpRobotMavsdkImpl()
  {
    if (m_has_flying_capability) {
      land();
    }
  }

private:
  /*!
   * Connects a first time to the robot in order to get the system running on it.
   * \param[in] mavsdk : the Mavsdk object we will use to subscribe to the system.
   * \returns Returns a shared pointer to the system.
   */
  std::shared_ptr<mavsdk::System> getSystem(mavsdk::Mavsdk &mavsdk)
  {
    std::cout << "Waiting to discover system..." << std::endl;
    auto prom = std::promise<std::shared_ptr<mavsdk::System> >{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
#if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
    mavsdk::Mavsdk::NewSystemHandle handle = mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]() {
#else
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
#endif
      auto system = mavsdk.systems().back();

      if (system->has_autopilot()) {
        std::cout << "Discovered autopilot" << std::endl;

        // Unsubscribe again as we only want to find one system.
#if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
        mavsdk.unsubscribe_on_new_system(handle);
#else
        mavsdk.subscribe_on_new_system(nullptr);
#endif
        prom.set_value(system);
      }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
      std::cerr << "No autopilot found." << std::endl;
      return {};
    }

    // Get discovered system now.
    return fut.get();
  }

  MAV_TYPE getVehicleType()
  {
    auto passthrough = mavsdk::MavlinkPassthrough{m_system};

    auto prom = std::promise<MAV_TYPE>{};
    auto fut = prom.get_future();
#if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
    mavsdk::MavlinkPassthrough::MessageHandle handle = passthrough.subscribe_message(
        MAVLINK_MSG_ID_HEARTBEAT, [&passthrough, &prom, &handle](const mavlink_message_t &message) {
#else
    passthrough.subscribe_message_async(MAVLINK_MSG_ID_HEARTBEAT,
                                        [&passthrough, &prom](const mavlink_message_t &message) {
#endif
          // Process only Heartbeat coming from the autopilot
          if (message.compid != MAV_COMP_ID_AUTOPILOT1) {
            return;
          }

          mavlink_heartbeat_t heartbeat;
          mavlink_msg_heartbeat_decode(&message, &heartbeat);

      // Unsubscribe again as we only want to find one system.
#if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
          passthrough.unsubscribe_message(handle);
#else
                                          passthrough.subscribe_message_async(MAVLINK_MSG_ID_HEARTBEAT, nullptr);
#endif

          prom.set_value(static_cast<MAV_TYPE>(heartbeat.type));
        });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
      std::cerr << "No heartbeat received to get vehicle type." << std::endl;
      return {};
    }

    // Get discovered system now.
    return fut.get();
  }

  void calibrate_accelerometer(mavsdk::Calibration &calibration)
  {
    std::cout << "Calibrating accelerometer..." << std::endl;

    std::promise<void> calibration_promise;
    auto calibration_future = calibration_promise.get_future();

    calibration.calibrate_accelerometer_async(create_calibration_callback(calibration_promise));

    calibration_future.wait();
  }

  std::function<void(mavsdk::Calibration::Result, mavsdk::Calibration::ProgressData)>
  create_calibration_callback(std::promise<void> &calibration_promise)
  {
    return [&calibration_promise](const mavsdk::Calibration::Result result,
                                  const mavsdk::Calibration::ProgressData progress_data) {
      switch (result) {
      case mavsdk::Calibration::Result::Success:
        std::cout << "--- Calibration succeeded!" << std::endl;
        calibration_promise.set_value();
        break;
      case mavsdk::Calibration::Result::Next:
        if (progress_data.has_progress) {
          std::cout << "    Progress: " << progress_data.progress << std::endl;
        }
        if (progress_data.has_status_text) {
          std::cout << "    Instruction: " << progress_data.status_text << std::endl;
        }
        break;
      default:
        std::cout << "--- Calibration failed with message: " << result << std::endl;
        calibration_promise.set_value();
        break;
      }
    };
  }

  void calibrate_gyro(mavsdk::Calibration &calibration)
  {
    std::cout << "Calibrating gyro..." << std::endl;

    std::promise<void> calibration_promise;
    auto calibration_future = calibration_promise.get_future();

    calibration.calibrate_gyro_async(create_calibration_callback(calibration_promise));

    calibration_future.wait();
  }

public:
  void connect(const std::string &connectionInfo)
  {
    m_address = connectionInfo;
    mavsdk::ConnectionResult connection_result = m_mavsdk.add_any_connection(connectionInfo);

    if (connection_result != mavsdk::ConnectionResult::Success) {
      std::cerr << "Connection failed: " << connection_result << std::endl;
      return;
    }

    m_system = getSystem(m_mavsdk);

    if (!m_system) {
      throw vpException(vpException::fatalError, "Unable to connect to: %s", connectionInfo.c_str());
    }

    m_mav_type = getVehicleType();

    m_has_flying_capability = hasFlyingCapability(m_mav_type);

    std::cout << (m_has_flying_capability ? "Connected to a flying vehicle" : "Connected to a non flying vehicle")
              << std::endl;

    m_telemetry = std::make_shared<mavsdk::Telemetry>(m_system);
    m_offboard = std::make_shared<mavsdk::Offboard>(m_system);
  }

  bool hasFlyingCapability(MAV_TYPE mav_type)
  {
    switch (mav_type) {
    case MAV_TYPE::MAV_TYPE_GROUND_ROVER:
    case MAV_TYPE::MAV_TYPE_SURFACE_BOAT:
    case MAV_TYPE::MAV_TYPE_SUBMARINE:
      return false;
    default:
      return true;
    }
  }

  bool isRunning() const
  {
    if (m_system == NULL) {
      return false;
    } else {
      return true;
    }
  }

  std::string getAddress() const
  {
    std::string sequence;
    std::stringstream ss(m_address);
    std::string actual_address;
    std::getline(ss, sequence, ':');
    if (sequence == "serial" || sequence == "udp" || sequence == "tcp") {
      getline(ss, sequence, ':');
      for (const char &c : sequence) {
        if (c != '/') {
          actual_address.append(1, c);
        }
      }
      return actual_address;
    } else {
      std::cout << "ERROR : The address parameter must start with \"serial:\" or \"udp:\" or \"tcp:\"." << std::endl;
      return std::string();
    }
  }

  float getBatteryLevel() const
  {
    mavsdk::Telemetry::Battery battery = m_telemetry.get()->battery();
    return battery.voltage_v;
  }

  void getPose(vpHomogeneousMatrix &ned_M_frd) const
  {
    auto quat = m_telemetry.get()->attitude_quaternion();
    auto posvel = m_telemetry.get()->position_velocity_ned();
    vpQuaternionVector q{quat.x, quat.y, quat.z, quat.w};
    vpTranslationVector t{posvel.position.north_m, posvel.position.east_m, posvel.position.down_m};
    ned_M_frd.buildFrom(t, q);
  }

  std::tuple<float, float> getHome() const
  {
    auto position = m_telemetry.get()->home();
    return {float(position.latitude_deg), float(position.longitude_deg)};
  }

  bool sendMocapData(const vpHomogeneousMatrix &M)
  {
    auto mocap = mavsdk::Mocap{m_system};
    mavsdk::Mocap::VisionPositionEstimate pose_estimate;

    vpColVector rot_ned = vpMath::enu2ned(vpRxyzVector(M.getRotationMatrix()));
    pose_estimate.angle_body.roll_rad = rot_ned[0];
    pose_estimate.angle_body.pitch_rad = rot_ned[1];
    pose_estimate.angle_body.yaw_rad = rot_ned[2];

    vpColVector pos_ned = vpMath::enu2ned(M.getTranslationVector());
    pose_estimate.position_body.x_m = pos_ned[0];
    pose_estimate.position_body.y_m = pos_ned[1];
    pose_estimate.position_body.z_m = pos_ned[2];

    pose_estimate.pose_covariance.covariance_matrix.push_back(NAN);
    pose_estimate.time_usec = 0; // We are using the back end timestamp

    const mavsdk::Mocap::Result set_position_result = mocap.set_vision_position_estimate(pose_estimate);
    if (set_position_result != mavsdk::Mocap::Result::Success) {
      std::cerr << "Set position failed: " << set_position_result << '\n';
      return false;
    } else {
      std::cout << "I managed to send the data" << std::endl;
      std::cout << "Translation : " << pose_estimate.position_body.x_m << " , " << pose_estimate.position_body.y_m
                << " , " << pose_estimate.position_body.z_m << std::endl;
      std::cout << "Roll : " << pose_estimate.angle_body.roll_rad << " , Pitch : " << pose_estimate.angle_body.pitch_rad
                << " , Yaw : " << pose_estimate.angle_body.yaw_rad << " ." << std::endl;
      return true;
    }
  }

  void setTakeOffAlt(double altitude)
  {
    if (altitude > 0) {
      m_takeoffAlt = altitude;
    } else {
      std::cerr << "ERROR : The take off altitude must be positive." << std::endl;
    }
  }

  void doFlatTrim()
  {
    // Instantiate plugin.
    auto calibration = mavsdk::Calibration(m_system);

    // Run calibrations
    calibrate_accelerometer(calibration);
    calibrate_gyro(calibration);
  }

  bool arm()
  {
    auto action = mavsdk::Action{m_system};
    // Arm vehicle
    std::cout << "Arming...\n";
    const mavsdk::Action::Result arm_result = action.arm();

    if (arm_result != mavsdk::Action::Result::Success) {
      std::cerr << "Arming failed: " << arm_result << std::endl;
      return false;
    }
    return true;
  }
  bool disarm()
  {
    auto action = mavsdk::Action{m_system};
    // Arm vehicle
    std::cout << "Disarming...\n";
    const mavsdk::Action::Result arm_result = action.disarm();

    if (arm_result != mavsdk::Action::Result::Success) {
      std::cerr << "Disarming failed: " << arm_result << std::endl;
      return false;
    }
    return true;
  }

  bool setGPSGlobalOrigin(double latitude, double longitude, double altitude)
  {
    auto passthrough = mavsdk::MavlinkPassthrough{m_system};
    mavlink_set_gps_global_origin_t gps_global_origin;
    gps_global_origin.latitude = latitude * 10000000;
    gps_global_origin.longitude = longitude * 10000000;
    gps_global_origin.altitude = altitude * 1000; // in mm
    gps_global_origin.target_system = m_system->get_system_id();
    mavlink_message_t msg;
    mavlink_msg_set_gps_global_origin_encode(passthrough.get_our_sysid(), passthrough.get_our_compid(), &msg,
                                             &gps_global_origin);
    auto resp = passthrough.send_message(msg);
    if (resp != mavsdk::MavlinkPassthrough::Result::Success) {
      std::cerr << "Set GPS global position failed: " << resp << std::endl;
      return false;
    }
    return true;
  }

  bool takeOff(bool interactive)
  {
    if (!m_has_flying_capability) {
      std::cerr << "Warning: Cannot takeoff this non flying vehicle" << std::endl;
      return true;
    }
    auto action = mavsdk::Action{m_system};
    std::cout << "Telemetry in_air() : " << m_telemetry.get()->in_air() << "\n";
    bool authorize_takeoff = false;

    if (!interactive) {
      authorize_takeoff = true;
    } else {
      if (m_telemetry.get()->flight_mode() == mavsdk::Telemetry::FlightMode::Offboard) {
        authorize_takeoff = true;
      } else {
        std::string answer;
        while (answer != "Y" && answer != "y" && answer != "N" && answer != "n") {
          std::cout << "Current flight mode is not the offboard mode. Do you want to force offboard mode ? (y/n)"
                    << std::endl;
          std::cin >> answer;
          if (answer == "Y" || answer == "y") {
            authorize_takeoff = true;
          }
        }
      }
    }

    if (m_telemetry.get()->in_air()) {
      std::cerr << "Cannot take off as the robot is already flying." << std::endl;
    } else if (authorize_takeoff) {
      // Arm vehicle
      std::cout << "Arming...\n";
      const mavsdk::Action::Result arm_result = action.arm();

      if (arm_result != mavsdk::Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << std::endl;
        return false;
      }

      /*const auto takeoff_result = action.takeoff();
      if (takeoff_result != mavsdk::Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return true;
      }*/

      /*auto telemetry = m_telemetry.get();
      auto in_air_promise = std::promise<void>{};
      auto in_air_future = in_air_promise.get_future();
      mavsdk::Telemetry::LandedStateHandle handle =
          telemetry->subscribe_landed_state(
              [telemetry, &in_air_promise,
               &handle](mavsdk::Telemetry::LandedState state) {
                if (state == mavsdk::Telemetry::LandedState::InAir) {
                  std::cout << "Taking off has finished\n.";
                  telemetry->unsubscribe_landed_state(handle);
                  in_air_promise.set_value();
                }
              });
      in_air_future.wait_for(seconds(10));
      if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "Takeoff timed out.\n";
        return 1;
      }*/

      const mavsdk::Offboard::VelocityBodyYawspeed stay{};
      m_offboard.get()->set_velocity_body(stay);

      if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
        mavsdk::Offboard::Result offboard_result = m_offboard.get()->start();
        if (offboard_result != mavsdk::Offboard::Result::Success) {
          std::cerr << "Offboard start failed: " << offboard_result << std::endl;
          return false;
        }
      }

      mavsdk::Telemetry::Odometry odom;
      odom = m_telemetry.get()->odometry();
      auto quat = m_telemetry.get()->attitude_quaternion();
      auto posvel = m_telemetry.get()->position_velocity_ned();
      vpQuaternionVector q{quat.x, quat.y, quat.z, quat.w};
      vpRotationMatrix R(q);
      vpRxyzVector rxyz(R);

      std::cout << "X_init, Y_init , Z_init = " << posvel.position.north_m << " , " << posvel.position.east_m << " , "
                << posvel.position.down_m << std::endl;
      double X_init = posvel.position.north_m;
      double Y_init = posvel.position.east_m;
      double Z_init = posvel.position.down_m;
      double yaw_init = vpMath::deg(rxyz[2]);

      std::cout << "Taking off using position NED." << std::endl;

      mavsdk::Offboard::PositionNedYaw takeoff{};
      takeoff.north_m = X_init;
      takeoff.east_m = Y_init;
      takeoff.down_m = Z_init - m_takeoffAlt;
      takeoff.yaw_deg = yaw_init;
      m_offboard.get()->set_position_ned(takeoff);
      sleep_for(seconds(5));

      /*auto offboard_result = m_offboard.get()->stop();
      if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return false;
      }
      std::cout << "Offboard stopped\n";*/
    }
    return true;
  }

  bool land()
  {
    if (!m_has_flying_capability) {
      std::cerr << "Warning: Cannot land this non flying vehicle" << std::endl;
      return true;
    }
    auto action = mavsdk::Action{m_system};

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Land) {
      std::cout << "Landing...\n";
      const mavsdk::Action::Result land_result = action.land();
      if (land_result != mavsdk::Action::Result::Success) {
        std::cerr << "Land failed: " << land_result << std::endl;
        return false;
      }

      // Check if vehicle is still in air
      while (m_telemetry.get()->in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
      }
    }

    std::cout << "Landed!" << std::endl;
    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(10));
    std::cout << "Finished..." << std::endl;
    return true;
  }

  void setPosition(float ned_delta_north, float ned_delta_east, float ned_delta_down, float ned_delta_yaw)
  {
    auto action = mavsdk::Action{m_system};

    const mavsdk::Offboard::VelocityBodyYawspeed stay{};
    m_offboard.get()->set_velocity_body(stay);

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      mavsdk::Offboard::Result offboard_result = m_offboard.get()->start();
      if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << std::endl;
        return;
      }
    }

    mavsdk::Telemetry::Odometry odom;
    mavsdk::Telemetry::EulerAngle angles;
    odom = m_telemetry.get()->odometry();
    angles = m_telemetry.get()->attitude_euler();

    double ned_X = odom.position_body.x_m;
    double ned_Y = odom.position_body.y_m;
    double ned_Z = odom.position_body.z_m;
    double ned_yaw = angles.yaw_deg;

    mavsdk::Offboard::PositionNedYaw position_target{};
    position_target.north_m = ned_X + ned_delta_north;
    position_target.east_m = ned_Y + ned_delta_east;
    position_target.down_m = ned_Z + ned_delta_down;
    position_target.yaw_deg = ned_yaw + vpMath::deg(ned_delta_yaw);
    m_offboard.get()->set_position_ned(position_target);
  }

  void setPosition(const vpHomogeneousMatrix &M)
  {
    auto XYZvec = vpRxyzVector(M.getRotationMatrix());
    if (XYZvec[0] != 0.0) {
      std::cerr << "ERROR : Can't move, rotation around X axis should be 0." << std::endl;
      return;
    }
    if (XYZvec[1] != 0.0) {
      std::cerr << "ERROR : Can't move, rotation around Y axis should be 0." << std::endl;
      return;
    }
    setPosition(M.getTranslationVector()[0], M.getTranslationVector()[1], M.getTranslationVector()[2], XYZvec[2]);
  }

  void setVelocity(const vpColVector &frd_vel_cmd, double delta_t)
  {
    setVelocity(frd_vel_cmd);

    sleep_for(milliseconds((int)(delta_t * 1000.0)));

    const mavsdk::Offboard::VelocityBodyYawspeed stay{};
    m_offboard.get()->set_velocity_body(stay);
  }

  void setVelocity(const vpColVector &frd_vel_cmd)
  {
    if (frd_vel_cmd.size() != 4) {
      throw(vpException(vpException::dimensionError,
                        "ERROR : Can't set velocity, dimension of the velocity vector %d should be equal to 4.",
                        frd_vel_cmd.size()));
    }

    auto action = mavsdk::Action{m_system};

    const mavsdk::Offboard::VelocityBodyYawspeed stay{};
    m_offboard.get()->set_velocity_body(stay);

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      mavsdk::Offboard::Result offboard_result = m_offboard.get()->start();
      if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << std::endl;
        return;
      }
    }
    mavsdk::Offboard::VelocityBodyYawspeed velocity_comm{};
    velocity_comm.forward_m_s = frd_vel_cmd[0];
    velocity_comm.right_m_s = frd_vel_cmd[1];
    velocity_comm.down_m_s = frd_vel_cmd[2];
    velocity_comm.yawspeed_deg_s = vpMath::deg(frd_vel_cmd[3]);
    m_offboard.get()->set_velocity_body(velocity_comm);
  }

  bool kill()
  {
    auto action = mavsdk::Action{m_system};
    const mavsdk::Action::Result kill_result = action.kill();
    if (kill_result != mavsdk::Action::Result::Success) {
      std::cerr << "Kill failed: " << kill_result << std::endl;
      return false;
    }
    return true;
  }

  void holdPosition()
  {
    if (m_telemetry.get()->in_air()) {
      if (m_telemetry.get()->gps_info().fix_type != mavsdk::Telemetry::FixType::NoGps) {
        auto action = mavsdk::Action{m_system};
        const mavsdk::Action::Result hold_result = action.hold();
        if (hold_result != mavsdk::Action::Result::Success) {
          std::cerr << "Hold failed: " << hold_result << std::endl;
          return;
        }
      } else {
        const mavsdk::Offboard::VelocityBodyYawspeed stay{};
        m_offboard.get()->set_velocity_body(stay);

        if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
          mavsdk::Offboard::Result offboard_result = m_offboard.get()->start();
          if (offboard_result != mavsdk::Offboard::Result::Success) {
            std::cerr << "Offboard start failed: " << offboard_result << std::endl;
            return;
          }
        }
        m_offboard.get()->set_velocity_body(stay);

        mavsdk::Telemetry::Odometry odom;
        mavsdk::Telemetry::EulerAngle angles;
        odom = m_telemetry.get()->odometry();
        angles = m_telemetry.get()->attitude_euler();

        double X_current = odom.position_body.x_m;
        double Y_current = odom.position_body.y_m;
        double Z_current = odom.position_body.z_m;
        double yaw_current = angles.yaw_deg;

        std::cout << "Holding using position NED." << std::endl;

        mavsdk::Offboard::PositionNedYaw hold_position{};
        hold_position.north_m = X_current;
        hold_position.east_m = Y_current;
        hold_position.down_m = Z_current;
        hold_position.yaw_deg = yaw_current;
        m_offboard.get()->set_position_ned(hold_position);

        auto offboard_result = m_offboard.get()->stop();
        if (offboard_result != mavsdk::Offboard::Result::Success) {
          std::cerr << "Offboard stop failed: " << offboard_result << '\n';
          return;
        }
        std::cout << "Offboard stopped\n";
      }
    }
  }

  void stopMoving()
  {
    const mavsdk::Offboard::VelocityBodyYawspeed stay{};
    m_offboard.get()->set_velocity_body(stay);

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      mavsdk::Offboard::Result offboard_result = m_offboard.get()->start();
      if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << std::endl;
        return;
      }
    }
    m_offboard.get()->set_velocity_body(stay);
    auto offboard_result = m_offboard.get()->stop();
    if (offboard_result != mavsdk::Offboard::Result::Success) {
      std::cerr << "Offboard stop failed: " << offboard_result << '\n';
      return;
    }
    std::cout << "Offboard stopped\n";
  }

  void setYawSpeed(double body_frd_wz)
  {
    auto action = mavsdk::Action{m_system};

    const mavsdk::Offboard::VelocityBodyYawspeed stay{};
    m_offboard.get()->set_velocity_body(stay);

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      mavsdk::Offboard::Result offboard_result = m_offboard.get()->start();
      if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << std::endl;
        return;
      }
    }
    mavsdk::Offboard::VelocityBodyYawspeed velocity_comm{};
    velocity_comm.forward_m_s = 0.0;
    velocity_comm.right_m_s = 0.0;
    velocity_comm.down_m_s = 0.0;
    velocity_comm.yawspeed_deg_s = vpMath::deg(body_frd_wz);
    m_offboard.get()->set_velocity_body(velocity_comm);
  }

  void setForwardSpeed(double body_frd_vx)
  {
    auto action = mavsdk::Action{m_system};

    const mavsdk::Offboard::VelocityBodyYawspeed stay{};
    m_offboard.get()->set_velocity_body(stay);

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      mavsdk::Offboard::Result offboard_result = m_offboard.get()->start();
      if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << std::endl;
        return;
      }
    }

    mavsdk::Offboard::VelocityBodyYawspeed velocity_comm{};
    velocity_comm.forward_m_s = body_frd_vx;
    velocity_comm.right_m_s = 0.0;
    velocity_comm.down_m_s = 0.0;
    velocity_comm.yawspeed_deg_s = 0.0;
    m_offboard.get()->set_velocity_body(velocity_comm);
  }

  void setLateralSpeed(double body_frd_vy)
  {
    auto action = mavsdk::Action{m_system};

    const mavsdk::Offboard::VelocityBodyYawspeed stay{};
    m_offboard.get()->set_velocity_body(stay);

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      mavsdk::Offboard::Result offboard_result = m_offboard.get()->start();
      if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << std::endl;
        return;
      }
    }
    mavsdk::Offboard::VelocityBodyYawspeed velocity_comm{};
    velocity_comm.forward_m_s = 0.0;
    velocity_comm.right_m_s = body_frd_vy;
    velocity_comm.down_m_s = 0.0;
    velocity_comm.yawspeed_deg_s = 0.0;
    m_offboard.get()->set_velocity_body(velocity_comm);
  }

  void setVerticalSpeed(double body_frd_vz)
  {
    auto action = mavsdk::Action{m_system};

    const mavsdk::Offboard::VelocityBodyYawspeed stay{};
    m_offboard.get()->set_velocity_body(stay);

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      mavsdk::Offboard::Result offboard_result = m_offboard.get()->start();
      if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << std::endl;
        return;
      }
    }
    mavsdk::Offboard::VelocityBodyYawspeed velocity_comm{};
    velocity_comm.forward_m_s = 0.0;
    velocity_comm.right_m_s = 0.0;
    velocity_comm.down_m_s = body_frd_vz;
    velocity_comm.yawspeed_deg_s = 0.0;
    m_offboard.get()->set_velocity_body(velocity_comm);
  }

  bool getFlyingCapability() { return m_has_flying_capability; }

private:
  //*** Attributes ***//
  std::string m_address{}; ///< Ip address of the robot to discover on the network
  mavsdk::Mavsdk m_mavsdk{};
  std::shared_ptr<mavsdk::System> m_system;
  std::shared_ptr<mavsdk::Telemetry> m_telemetry;
  std::shared_ptr<mavsdk::Offboard> m_offboard;

  double m_takeoffAlt{1.0}; ///< The altitude to aim for when calling the function takeoff

  MAV_TYPE m_mav_type{}; // Vehicle type
  bool m_has_flying_capability{false};

  // bool m_flatTrimFinished;  ///< Used to know when the robot has finished a flat trim
  // bool m_relativeMoveEnded; ///< Used to know when the robot has ended a relative move
  // bool m_settingsReset;     ///< Used to know when the robot a finished the settings reset

  // unsigned int m_batteryLevel; ///< Percentage of battery remaining
};
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 * Constructor.
 *
 * Initializes vehicle controller, by discovering vehicles connected either with an Ethernet TCP or UDP link, or with a
 * serial link the computer is currently connected to.
 *
 * \warning This constructor should be called after the vehicle is turned on, and after the computer is connected to the
 * vehicle Ethernet network or with a serial link.
 *
 * \warning If the connection to the vehicle failed, the program will throw an exception.
 *
 * After having called this constructor, it is recommended to check if the vehicle is running with isRunning() before
 * sending commands to the vehicle.
 *
 * \param[in] connection_info : Specify connection information. This parameter must be written following these
 * conventions:
 * - for TCP link: tcp://[server_host][:server_port]
 * - for UDP link: udp://[bind_host][:bind_port]
 * - for Serial link: serial:///path/to/serial/dev[:baudrate]
 *
 * Examples: udp://192.168.30.111:14550 or serial:///dev/ttyACMO
 *
 * For more information see [here](https://mavsdk.mavlink.io/main/en/cpp/guide/connections.html).
 *
 * \exception vpException::fatalError : If the program failed to connect to the vehicle.
 */
vpRobotMavsdk::vpRobotMavsdk(const std::string &connection_info) : m_impl(new vpRobotMavsdkImpl(connection_info)) {}

/*!
 * Default constructor without parameters. You need to use the connect() function afterwards.
 *
 * \sa connect()
 */
vpRobotMavsdk::vpRobotMavsdk() : m_impl(new vpRobotMavsdkImpl()) {}

/*!
 * Destructor.
 * When the vehicle has flying capabilities, lands the vehicle if not landed and safely disconnects everything.
 */
vpRobotMavsdk::~vpRobotMavsdk() { delete m_impl; }

/*!
 * Connects to the vehicle and setups the different controllers.
 * \param[in] connection_info : The connection information given to connect to the vehicle. You may use:
 * - for TCP link: tcp://[server_host][:server_port]
 * - for UDP link: udp://[bind_host][:bind_port]
 * - for Serial link: serial:///path/to/serial/dev[:baudrate]
 *
 * Examples: udp://192.168.30.111:14550 or serial:///dev/ttyACMO
 *
 * For more information see [here](https://mavsdk.mavlink.io/main/en/cpp/guide/connections.html).
 *
 * \sa getAddress()
 */
void vpRobotMavsdk::connect(const std::string &connection_info) { m_impl->connect(connection_info); }

/*!
 * Checks if the vehicle is running, ie if the vehicle is connected and ready to receive commands.
 */
bool vpRobotMavsdk::isRunning() const { return m_impl->isRunning(); }

/*!
 * Sends MoCap position data to the vehicle.
 *
 * We consider here that the MoCap global reference frame is ENU. The vehicle body frame has X axes aligned
 * with the vehicle front axes and Z axis going upward.
 *
 * \return true if the MoCap data was successfully sent to the vehicle, false otherwise.
 * \param[in] M : Homogeneous matrix containing the pose of the vehicle given by the MoCap system.
 * To be more precise, this matrix gives the pose of the vehicle body frame returned the MoCap (where
 * X axes is aligned with the front vehicle axis and Z axis is going upward) with respect to the
 * MoCap global reference frame defined as ENU.
 *
 * Internally we transform this pose in a NED global reference frame.
 */
bool vpRobotMavsdk::sendMocapData(const vpHomogeneousMatrix &M) { return m_impl->sendMocapData(M); }

/*!
 * Gives the address given to connect to the vehicle.
 * \return : A string corresponding to the Ethernet or serial address used for the connection to the vehicle.
 *
 * \sa connect()
 */
std::string vpRobotMavsdk::getAddress() const { return m_impl->getAddress(); }

/*!
 * Gets current battery level in volts.
 * \warning When the vehicle battery gets below a certain threshold (around 14.8 for a 4S battery), you should recharge
 * it.
 */
float vpRobotMavsdk::getBatteryLevel() const { return m_impl->getBatteryLevel(); }

/*!
 * Gets the current vehicle pose FRD in its local NED frame.
 * \param[in] ned_M_frd : Homogeneous matrix describing the position and attitude of the vehicle returned by telemetry.
 */
void vpRobotMavsdk::getPose(vpHomogeneousMatrix &ned_M_frd) const { m_impl->getPose(ned_M_frd); }

/*!
 * Gets the robot home position in GPS coord.
 *
 * \warning Only available if the GPS is initialized, for example
 * in simulation.
 */
std::tuple<float, float> vpRobotMavsdk::getHome() const { return m_impl->getHome(); }

/*!
 * Sends a flat trim command to the vehicle, to calibrate accelerometer and gyro.
 *
 * \warning Should be executed only when the vehicle is on a flat surface.
 */
void vpRobotMavsdk::doFlatTrim() {}

/*!
 * Sets the take off altitude.
 * \param[in] altitude : Desired altitude for take off in meters, equal to 1.0 m by default.
 * \warning The altitude must be positive.
 *
 * \sa takeOff()
 */
void vpRobotMavsdk::setTakeOffAlt(double altitude) { m_impl->setTakeOffAlt(altitude); }

/*!
 * Arms the vehicle.
 * \return true if arming is successful, false otherwise.
 */
bool vpRobotMavsdk::arm() { return m_impl->arm(); }

/*!
 * Disarms the vehicle.
 * \return true if disarming is successful, false otherwise.
 */
bool vpRobotMavsdk::disarm() { return m_impl->disarm(); }

/*!
 * Sends take off command when the vehicle has flying capabilities.
 * \param[in] interactive : If true asks the user if the offboard mode is to be forced through the terminal. If false
 * offboard mode is automatically set.
 * \return
 * - If the vehicle has flying capabilities, returns true if the take off is successful, false otherwise.
 * - If the vehicle doesn't have flying capabilities, returns true.
 * \warning This function is blocking.
 * \sa setTakeOffAlt(), land(), hasFlyingCapability()
 */
bool vpRobotMavsdk::takeOff(bool interactive) { return m_impl->takeOff(interactive); }

/*!
 * Makes the vehicle hold its position.
 * \warning The function simply switches to hold mode when the vehicle is equipped with a GPS. It can function without a
 * GPS, but it is still preferable to use it outdoors.
 */
void vpRobotMavsdk::holdPosition() { m_impl->holdPosition(); };

/*!
 * Stops any vehicle movement.
 * \warning Depending on the speed of the vehicle when the function is called, it may still move a bit until it
 * stabilizes.
 */
void vpRobotMavsdk::stopMoving() { m_impl->stopMoving(); }; // Resolve hold problem

/*!
 * Sends landing command if the vehicle has flying capabilities.
 * \return
 * - If the vehicle has flying capabilities, returns true if the landing is successful, false otherwise.
 * - If the vehicle doesn't have flying capabilities, returns true.
 * \sa takeOff(), hasFlyingCapability()
 */
bool vpRobotMavsdk::land() { return m_impl->land(); }

/*!
 * Moves the vehicle Front-Right-Down (FRD) body frame with respect to the global reference NED frame.
 *
 * \warning This function is blocking.
 *
 * \param[in] ned_delta_north : Relative displacement along X-front axis (meters).
 * \param[in] ned_delta_east : Relative displacement along Y-right axis (meters).
 * \param[in] ned_delta_down : Relative displacement along Z-down axis (meters).
 * \param[in] ned_delta_yaw : Relative rotation of the heading (radians).
 *
 * \sa setPosition(const vpHomogeneousMatrix &M)
 */
void vpRobotMavsdk::setPosition(float ned_delta_north, float ned_delta_east, float ned_delta_down, float ned_delta_yaw)
{
  m_impl->setPosition(ned_delta_north, ned_delta_east, ned_delta_down, ned_delta_yaw);
}

/*!
 * Changes the relative position of the vehicle based on a homogeneous transformation matrix expressed in the NED
 * global reference frame.
 *
 * \param[in] ned_M_delta : Homogeneous matrix that expressed the relative displacement of the vehicle expressed
 * in the NED global reference frame.
 *
 * \warning The rotation around the X and Y axes should be equal to 0, as the vehicle (drone or rover)
 * cannot rotate around these axes.
 * \warning This function is blocking.
 *
 * \sa setPosition(float ned_delta_north, float ned_delta_east, float ned_delta_down, float ned_delta_yaw)
 */
void vpRobotMavsdk::setPosition(const vpHomogeneousMatrix &ned_M_delta) { m_impl->setPosition(ned_M_delta); }

/*!
 * Sets the vehicle velocity in its own Front-Right-Down (FRD) body frame for a duration delta_t [s]. The vehicle stops
 * afterwards.
 *
 * \param[in] frd_vel_cmd : 4-dim vehicle FRD velocity commands, vx, vy, vz, wz. Translation velocities (vx, vy, vz)
 * should be expressed in m/s and rotation velocity (wz) in rad/s. \param[in] delta_t : Sampling time (in seconds), time
 * during which the velocity `vel_cmd` is applied. When this time is elapsed, the vehicle stays where it is.
 *
 * \warning The dimension of the velocity vector should be equal to 4, as the vehicle cannot rotate around X and Y axes.
 */
void vpRobotMavsdk::setVelocity(const vpColVector &frd_vel_cmd, double delta_t)
{
  m_impl->setVelocity(frd_vel_cmd, delta_t);
}

/*!
 * Sets the vehicle velocity in its own Front-Right-Down (FRD) body frame.
 *
 * \param[in] frd_vel_cmd : 4-dim vehicle velocity commands, vx, vy, vz, wz. Translation velocities (vx, vy, vz) should
 * be expressed in m/s and rotation velocity (wz) in rad/s.
 *
 * \warning The dimension of the velocity vector should be equal to 4, as the vehicle cannot rotate around X and Y axes.
 * \warning The vehicle applies this command until given another one.
 */
void vpRobotMavsdk::setVelocity(const vpColVector &frd_vel_cmd) { m_impl->setVelocity(frd_vel_cmd); }

/*!
 * Cuts the motors. Should only be used in emergency cases.
 * \return true if the cut motors command was successful, false otherwise.
 * \warning If your vehicle has flying capabilities, it will fall.
 */
bool vpRobotMavsdk::kill() { return m_impl->kill(); }

/*!
 * Sets the yaw speed, expressed in rad/s, in the Front-Right-Down body frame.
 *
 * \warning The vehicle will not stop moving in that direction until you send another motion command.
 *
 * \param[in] body_frd_wz : Desired FRD body frame yaw speed in rad/s.
 * - Positive values will make the vehicle turn to its right (clockwise)
 * - Negative values will make the vehicle turn to its left (counterclockwise)
 */
void vpRobotMavsdk::setYawSpeed(double body_frd_wz) { m_impl->setYawSpeed(body_frd_wz); }

/*!
 * Sets the forward speed, expressed in m/s, in the Front-Right-Down body frame.
 *
 * \warning The vehicle will not stop moving in that direction until you send another motion command.
 *
 * \param[in] body_frd_vx : Desired FRD body frame forward speed in m/s.
 * - Positive values will make the vehicle go forward
 * - Negative values will make the vehicle go backwards
 */
void vpRobotMavsdk::setForwardSpeed(double body_frd_vx) { m_impl->setForwardSpeed(body_frd_vx); }

/*!
 * Sets the lateral speed, expressed in m/s, in the Front-Right-Down body frame.
 *
 * \warning The vehicle will not stop moving in that direction until you send another motion command.
 *
 * \param[in] body_frd_vy : Desired FRD body frame lateral speed in m/s.
 * - Positive values will make the vehicle go right
 * - Negative values will make the vehicle go left
 */
void vpRobotMavsdk::setLateralSpeed(double body_frd_vy) { m_impl->setLateralSpeed(body_frd_vy); }

/*!
 * Allows to set GPS global origin to initialize the Kalman filter when the vehicle is not
 * equipped with a GPS.
 *
 * \param latitude : Latitude in deg (WGS84).
 * \param longitude : Longitude in deg (WGS84).
 * \param altitude : Altitude in meter. Positive for up.
 */
bool vpRobotMavsdk::setGPSGlobalOrigin(double latitude, double longitude, double altitude)
{
  return m_impl->setGPSGlobalOrigin(latitude, longitude, altitude);
}

/*!
 * Sets the vertical speed, expressed in m/s, in the Front-Right-Down body frame.
 *
 * \warning The vehicle will not stop moving in that direction until you send another motion command.
 *
 * \param[in] body_frd_vz : Desired FRD body frame vertical speed in m/s.
 * - Positive values will make the vehicle go down.
 * - Negative values will make the vehicle go up.
 */
void vpRobotMavsdk::setVerticalSpeed(double body_frd_vz) { m_impl->setVerticalSpeed(body_frd_vz); }

/*!
 * Return true if the vehicle has flying capabilities.
 * Ground rover, surface boat and submarine vehicles are considered with non flying capabilities, while
 * all the other vehicles are considered with flying capabilities.
 */
bool vpRobotMavsdk::hasFlyingCapability() { return m_impl->getFlyingCapability(); }

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpRobotMavsdk.cpp.o) has no
// symbols
void dummy_vpRobotMavsdk(){};
#endif
