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
    if (m_has_flying_capability && m_auto_land) {
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

    m_action = std::make_shared<mavsdk::Action>(m_system);
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

  void getPosition(vpHomogeneousMatrix &ned_M_frd) const
  {
    auto quat = m_telemetry.get()->attitude_quaternion();
    auto posvel = m_telemetry.get()->position_velocity_ned();
    vpQuaternionVector q{quat.x, quat.y, quat.z, quat.w};
    vpTranslationVector t{posvel.position.north_m, posvel.position.east_m, posvel.position.down_m};
    ned_M_frd.buildFrom(t, q);
  }

  void getPosition(float &ned_north, float &ned_east, float &ned_down, float &ned_yaw) const
  {
    auto odom = m_telemetry.get()->odometry();
    auto angles = m_telemetry.get()->attitude_euler();
    ned_north = odom.position_body.x_m;
    ned_east = odom.position_body.y_m;
    ned_down = odom.position_body.z_m;
    ned_yaw = vpMath::rad(angles.yaw_deg);
  }

  std::tuple<float, float> getHome() const
  {
    auto position = m_telemetry.get()->home();
    return {float(position.latitude_deg), float(position.longitude_deg)};
  }

  bool sendMocapData(const vpHomogeneousMatrix &enu_M_flu, int display_fps)
  {
    static double time_prev = vpTime::measureTimeMs();

    // We suppose here that the body frame which pose is given by the MoCap is FLU (Front Left Up).
    // Thus we need to transform this frame to FRD (Front Right Down).
    vpHomogeneousMatrix flu_M_frd;
    flu_M_frd.eye();
    flu_M_frd[1][1] = -1;
    flu_M_frd[2][2] = -1;
  
    vpHomogeneousMatrix enu_M_frd = enu_M_flu * flu_M_frd;
    auto mocap = mavsdk::Mocap{m_system};
    mavsdk::Mocap::VisionPositionEstimate pose_estimate;

    vpHomogeneousMatrix ned_M_frd = vpMath::enu2ned(enu_M_frd);
    vpRxyzVector ned_rxyz_frd = vpRxyzVector(ned_M_frd.getRotationMatrix());
    pose_estimate.angle_body.roll_rad = ned_rxyz_frd[0];
    pose_estimate.angle_body.pitch_rad = ned_rxyz_frd[1];
    pose_estimate.angle_body.yaw_rad = ned_rxyz_frd[2];

    vpTranslationVector ned_t_frd = ned_M_frd.getTranslationVector();
    pose_estimate.position_body.x_m = ned_t_frd[0];
    pose_estimate.position_body.y_m = ned_t_frd[1];
    pose_estimate.position_body.z_m = ned_t_frd[2];

    pose_estimate.pose_covariance.covariance_matrix.push_back(NAN);
    pose_estimate.time_usec = 0; // We are using the back end timestamp

    const mavsdk::Mocap::Result set_position_result = mocap.set_vision_position_estimate(pose_estimate);
    if (set_position_result != mavsdk::Mocap::Result::Success) {
      std::cerr << "Set position failed: " << set_position_result << '\n';
      return false;
    } else {
      if (display_fps > 0) {
        double display_time_ms = 1000./display_fps;
        if (vpTime::measureTimeMs() - time_prev > display_time_ms) {
          time_prev = vpTime::measureTimeMs();
          std::cout << "Send ned_M_frd MoCap data: " << std::endl;
          std::cout << "Translation [m]: " << pose_estimate.position_body.x_m << " , " << pose_estimate.position_body.y_m
                    << " , " << pose_estimate.position_body.z_m << std::endl;
          std::cout << "Roll [rad]: " << pose_estimate.angle_body.roll_rad << " , Pitch [rad]: " << pose_estimate.angle_body.pitch_rad
                    << " , Yaw [rad]: " << pose_estimate.angle_body.yaw_rad << " ." << std::endl;
        }
      }
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
    // Arm vehicle
    std::cout << "Arming...\n";
    const mavsdk::Action::Result arm_result = m_action.get()->arm();

    if (arm_result != mavsdk::Action::Result::Success) {
      std::cerr << "Arming failed: " << arm_result << std::endl;
      return false;
    }
    return true;
  }

  bool disarm()
  {
    // Arm vehicle
    std::cout << "Disarming...\n";
    const mavsdk::Action::Result arm_result = m_action.get()->disarm();

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

  bool takeControl()
  {
    if (m_verbose) {
      std::cout << "Starting offboard mode..." << std::endl;
    }

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      const mavsdk::Offboard::VelocityBodyYawspeed stay{};
      m_offboard.get()->set_velocity_body(stay);

      mavsdk::Offboard::Result offboard_result = m_offboard.get()->start();
      if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard mode failed: " << offboard_result << std::endl;
        return false;
      }
    }
    else if (m_verbose) {
      std::cout << "Already in offboard mode" << std::endl;
    }

    // Wait to ensure offboard mode active in telemetry
    double t = vpTime::measureTimeMs();
    while (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      if (vpTime::measureTimeMs() - t > 3. * 1000.) {
        std::cout << "Time out received in takeControl()" << std::endl;
        break;
      }
    };

    if (m_verbose) {
      std::cout << "Offboard mode started" << std::endl;
    }
    return true;
  }

  void setPositioningIncertitude(float position_incertitude, float yaw_incertitude)
  {
    m_position_incertitude = position_incertitude;
    m_yaw_incertitude = yaw_incertitude;
  }

  bool takeOff(bool interactive, int timeout_sec)
  {
    if (!m_has_flying_capability) {
      std::cerr << "Warning: Cannot takeoff this non flying vehicle" << std::endl;
      return true;
    }

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
      return true;
    } else if (authorize_takeoff) {
      // Arm vehicle
      if (! arm()) {
        return false;
      }

      vpTime::wait(2000);

      if (interactive) {
        std::string answer;
        while (answer != "Y" && answer != "y" && answer != "N" && answer != "n") {
          std::cout << "If vehicle armed ? (y/n)" << std::endl;
          std::cin >> answer;
          if (answer == "N" || answer == "n") {
            disarm();
            kill();
            return false;
          }
        }
      }

      // Takeoff
      if (m_telemetry.get()->gps_info().fix_type == mavsdk::Telemetry::FixType::NoGps) {
        // No GPS connected. 
        // When using odometry from MoCap, Action::takeoff() behavior is to takeoff at 0,0,0,alt
        // that is weird when the drone is not placed at 0,0,0.
        // That's why here use set_position_ned() to takeoff

        // Start off-board or guided mode
        takeControl();

        auto in_air_promise = std::promise<void>{};
        auto in_air_future = in_air_promise.get_future();

        mavsdk::Telemetry::Odometry odom = m_telemetry.get()->odometry();
        vpQuaternionVector q{odom.q.x, odom.q.y, odom.q.z, odom.q.w};
        vpRotationMatrix R(q);
        vpRxyzVector rxyz(R);

        double X_init = odom.position_body.x_m;
        double Y_init = odom.position_body.y_m;
        double Z_init = odom.position_body.z_m;
        double yaw_init = vpMath::deg(rxyz[2]);

        std::cout << "Takeoff using position NED." << std::endl;

        mavsdk::Offboard::PositionNedYaw takeoff{};
        takeoff.north_m = X_init;
        takeoff.east_m = Y_init;
        takeoff.down_m = Z_init - m_takeoffAlt;
        takeoff.yaw_deg = yaw_init;
        m_offboard.get()->set_position_ned(takeoff);
        // Possibility is to use set_position_velocity_ned(); to speed up takeoff

  #if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
        Telemetry::LandedStateHandle handle = m_telemetry.get()->subscribe_landed_state(
        [this, &in_air_promise, &handle](mavsdk::Telemetry::LandedState state) {
          if (state == mavsdk::Telemetry::LandedState::InAir) {
            std::cout << "Drone is taking off\n.";
            m_telemetry.get()->unsubscribe_landed_state(handle);
            in_air_promise.set_value();
          }
        });
  #else
        m_telemetry.get()->subscribe_landed_state([this, &in_air_promise](mavsdk::Telemetry::LandedState state) {
          if (state == mavsdk::Telemetry::LandedState::InAir) {
            std::cout << "Drone is taking off\n.";
            m_telemetry.get()->subscribe_landed_state(nullptr);
            in_air_promise.set_value();
          }
          std::cout << "state: " << state << std::endl;
        });
  #endif
        if (in_air_future.wait_for(seconds(timeout_sec)) == std::future_status::timeout) {
          std::cerr << "Takeoff failed: drone not in air.\n";
  #if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
          m_telemetry.get()->unsubscribe_landed_state(handle);
  #else
          m_telemetry.get()->subscribe_landed_state(nullptr);
  #endif
          return false;
        }
        // Add check with Altitude
        auto takeoff_finished_promise = std::promise<void>{};
        auto takeoff_finished_future = takeoff_finished_promise.get_future();

  #if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
        auto handle_odom = m_telemetry.get()->subscribe_odometry(
        [this, &takeoff_finished_promise, &handle, &Z_init](mavsdk::Telemetry::Odometry odom) {
          if (odom.position_body.z_m < 0.90 * (Z_init - m_takeoffAlt) + m_position_incertitude) {
            std::cout << "Takeoff altitude reached\n.";
            m_telemetry.get()->unsubscribe_odometry(handle_odom);
            takeoff_finished_promise.set_value();
          }
        });
  #else
        m_telemetry.get()->subscribe_odometry([this, &takeoff_finished_promise, &Z_init](mavsdk::Telemetry::Odometry odom) {
          if (odom.position_body.z_m < 0.90 * (Z_init - m_takeoffAlt) + m_position_incertitude) {
            std::cout << "Takeoff altitude reached\n.";
            m_telemetry.get()->subscribe_odometry(nullptr);
            takeoff_finished_promise.set_value();
          }
        });
  #endif
        if (takeoff_finished_future.wait_for(seconds(timeout_sec)) == std::future_status::timeout) {
          std::cerr << "Takeoff failed:  altitude not reached.\n";
  #if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
          m_telemetry.get()->unsubscribe_odometry(handle);
  #else
          m_telemetry.get()->subscribe_odometry(nullptr);
  #endif
          return false;
        }
      }
      else {
        // GPS connected, we use Action::takeoff()
        std::cout << "---- DEBUG: GPS detected: use action::takeoff()" << std::endl;

        mavsdk::Telemetry::Odometry odom = m_telemetry.get()->odometry();
        double Z_init = odom.position_body.z_m;

        m_action.get()->set_takeoff_altitude(m_takeoffAlt);
        const auto takeoff_result = m_action.get()->takeoff();
        if (takeoff_result != mavsdk::Action::Result::Success) {
          std::cerr << "Takeoff failed: " << takeoff_result << '\n';
          return false;
        }

        auto in_air_promise = std::promise<void>{};
        auto in_air_future = in_air_promise.get_future();
  #if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
        Telemetry::LandedStateHandle handle = m_telemetry.get()->subscribe_landed_state(
        [this, &in_air_promise, &handle](mavsdk::Telemetry::LandedState state) {
          if (state == mavsdk::Telemetry::LandedState::InAir) {
              std::cout << "Taking off has finished\n.";
              m_telemetry.get()->unsubscribe_landed_state(handle);
              in_air_promise.set_value();
          }
        });
  #else
        m_telemetry.get()->subscribe_landed_state([this, &in_air_promise](mavsdk::Telemetry::LandedState state) {
          if (state == mavsdk::Telemetry::LandedState::InAir) {
            std::cout << "Taking off has finished\n.";
            m_telemetry.get()->subscribe_landed_state(nullptr);
            in_air_promise.set_value();
          }
          std::cout << "state: " << state << std::endl;
        });
  #endif
        if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
          // Add check with Altitude
          std::cerr << "Takeoff timed out.\n";
  #if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
          m_telemetry.get()->unsubscribe_landed_state(handle);
  #else
          m_telemetry.get()->subscribe_landed_state(nullptr);
  #endif
          //return false;
        }
        // Add check with Altitude
        auto takeoff_finished_promise = std::promise<void>{};
        auto takeoff_finished_future = takeoff_finished_promise.get_future();

  #if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
        auto handle_odom = m_telemetry.get()->subscribe_odometry(
        [this, &takeoff_finished_promise, &handle, &Z_init](mavsdk::Telemetry::Odometry odom) {
          if (odom.position_body.z_m < 0.90 * (Z_init - m_takeoffAlt) + m_position_incertitude) {
            std::cout << "Takeoff altitude reached\n.";
            m_telemetry.get()->unsubscribe_odometry(handle_odom);
            takeoff_finished_promise.set_value();
          }
        });
  #else
        m_telemetry.get()->subscribe_odometry([this, &takeoff_finished_promise, &Z_init](mavsdk::Telemetry::Odometry odom) {
          if (odom.position_body.z_m < 0.90 * (Z_init - m_takeoffAlt) + m_position_incertitude) {
            std::cout << "Takeoff altitude reached\n.";
            m_telemetry.get()->subscribe_odometry(nullptr);
            takeoff_finished_promise.set_value();
          }
        });
  #endif
        if (takeoff_finished_future.wait_for(seconds(timeout_sec)) == std::future_status::timeout) {
          std::cerr << "Takeoff failed:  altitude not reached.\n";
#if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
          m_telemetry.get()->unsubscribe_odometry(handle);
#else
          m_telemetry.get()->subscribe_odometry(nullptr);
#endif
          return false;
        }
      } 
    }
    return true;
  }

  bool land()
  {
    if (!m_has_flying_capability) {
      std::cerr << "Warning: Cannot land this non flying vehicle" << std::endl;
      return true;
    }

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Land) {
      std::cout << "Landing...\n";
      const mavsdk::Action::Result land_result = m_action.get()->land();
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
    sleep_for(seconds(5));
    std::cout << "Finished..." << std::endl;
    return true;
  }

  bool setPosition(float ned_north, float ned_east, float ned_down, float ned_yaw, bool blocking, int timeout_sec)
  {
    mavsdk::Offboard::PositionNedYaw position_target{};

    position_target.north_m = ned_north;
    position_target.east_m = ned_east;
    position_target.down_m = ned_down;
    position_target.yaw_deg = vpMath::deg(ned_yaw);

    std::cout << "NED Pos to reach: " << position_target.north_m << " " << position_target.east_m << " " << position_target.down_m << " " << position_target.yaw_deg << std::endl;
    m_offboard.get()->set_position_ned(position_target);

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      if (m_verbose) {
        std::cout << "Cannot set vehicle position: offboard mode not started" << std::endl;
      }
      return false;
    }

    if (blocking) {
      // Add check with Altitude
      auto position_reached_promise = std::promise<void>{};
      auto position_reached_future = position_reached_promise.get_future();

  #if (VISP_HAVE_MAVSDK_VERSION > 0x010412)
      auto handle_odom = m_telemetry.get()->subscribe_odometry(
      [this, &position_reached_promise, &handle, &position_target](mavsdk::Telemetry::Odometry odom) {
        vpQuaternionVector q{odom.q.x, odom.q.y, odom.q.z, odom.q.w};
        vpRotationMatrix R(q);
        vpRxyzVector rxyz(R);
        double odom_yaw = vpMath::deg(rxyz[2]);
        double distance_to_target = std::sqrt(vpMath::sqr(odom.position_body.x_m - position_target.north_m)
             + vpMath::sqr(odom.position_body.y_m - position_target.east_m) 
             + vpMath::sqr(odom.position_body.z_m - position_target.down_m));
        if (distance_to_target < m_position_incertitude && std::fabs(odom_yaw - position_target.yaw_deg) < m_yaw_incertitude)
        {
          std::cout << "Position reached\n.";
          m_telemetry.get()->unsubscribe_odometry(handle_odom);
          position_reached_promise.set_value();
        }
      });
  #else
      m_telemetry.get()->subscribe_odometry([this, &position_reached_promise, &position_target](mavsdk::Telemetry::Odometry odom) {
        vpQuaternionVector q{odom.q.x, odom.q.y, odom.q.z, odom.q.w};
        vpRotationMatrix R(q);
        vpRxyzVector rxyz(R);
        double odom_yaw = vpMath::deg(rxyz[2]);
        double distance_to_target = std::sqrt(vpMath::sqr(odom.position_body.x_m - position_target.north_m)
             + vpMath::sqr(odom.position_body.y_m - position_target.east_m) 
             + vpMath::sqr(odom.position_body.z_m - position_target.down_m));
        if (distance_to_target < m_position_incertitude && std::fabs(odom_yaw - position_target.yaw_deg) < m_yaw_incertitude)
        {
          std::cout << "Position reached\n.";
          m_telemetry.get()->subscribe_odometry(nullptr);
          position_reached_promise.set_value();
        }
      });
  #endif
      if (position_reached_future.wait_for(seconds(timeout_sec)) == std::future_status::timeout) {
        std::cerr << "Positioning failed: position not reached.\n";
        return false;
      }
    }

std::cout << "---- DEBUG timeout: " << timeout_sec << std::endl;
    return true;
  }

  bool setPositionRelative(float ned_delta_north, float ned_delta_east, float ned_delta_down, float ned_delta_yaw, bool blocking, int timeout_sec)
  {
    mavsdk::Telemetry::Odometry odom;
    mavsdk::Telemetry::EulerAngle angles;
    mavsdk::Offboard::PositionNedYaw position_target{};

    position_target.north_m = ned_delta_north;
    position_target.east_m = ned_delta_east;
    position_target.down_m = ned_delta_down;
    position_target.yaw_deg = vpMath::deg(ned_delta_yaw);

    // Set a relative position
    odom = m_telemetry.get()->odometry();
    angles = m_telemetry.get()->attitude_euler();

    position_target.north_m += odom.position_body.x_m;
    position_target.east_m += odom.position_body.y_m;
    position_target.down_m += odom.position_body.z_m;
    position_target.yaw_deg += angles.yaw_deg;

    return setPosition(position_target.north_m, position_target.east_m, position_target.down_m, vpMath::rad(position_target.yaw_deg), blocking, timeout_sec);
  }

  bool setPosition(const vpHomogeneousMatrix &M, bool absolute, int timeout_sec)
  {
    auto XYZvec = vpRxyzVector(M.getRotationMatrix());
    if (XYZvec[0] != 0.0) {
      std::cerr << "ERROR : Can't move, rotation around X axis should be 0." << std::endl;
      return false;
    }
    if (XYZvec[1] != 0.0) {
      std::cerr << "ERROR : Can't move, rotation around Y axis should be 0." << std::endl;
      return false;
    }
    return setPosition(M.getTranslationVector()[0], M.getTranslationVector()[1], M.getTranslationVector()[2], XYZvec[2], absolute, timeout_sec);
  }

  bool setPositionRelative(const vpHomogeneousMatrix &M, bool blocking, int timeout_sec)
  {
    auto XYZvec = vpRxyzVector(M.getRotationMatrix());
    if (XYZvec[0] != 0.0) {
      std::cerr << "ERROR : Can't move, rotation around X axis should be 0." << std::endl;
      return false;
    }
    if (XYZvec[1] != 0.0) {
      std::cerr << "ERROR : Can't move, rotation around Y axis should be 0." << std::endl;
      return false;
    }
    return setPositionRelative(M.getTranslationVector()[0], M.getTranslationVector()[1], M.getTranslationVector()[2], XYZvec[2], blocking, timeout_sec);
  }

  bool setVelocity(const vpColVector &frd_vel_cmd)
  {
    if (frd_vel_cmd.size() != 4) {
      throw(vpException(vpException::dimensionError,
                        "ERROR : Can't set velocity, dimension of the velocity vector %d should be equal to 4.",
                        frd_vel_cmd.size()));
    }

    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      if (m_verbose) {
        std::cout << "Cannot set vehicle velocity: offboard mode not started" << std::endl;
      }
      return false;
    }
    mavsdk::Offboard::VelocityBodyYawspeed velocity_comm{};
    velocity_comm.forward_m_s = frd_vel_cmd[0];
    velocity_comm.right_m_s = frd_vel_cmd[1];
    velocity_comm.down_m_s = frd_vel_cmd[2];
    velocity_comm.yawspeed_deg_s = vpMath::deg(frd_vel_cmd[3]);
    m_offboard.get()->set_velocity_body(velocity_comm);

    return true;
  }

  bool kill()
  {
    const mavsdk::Action::Result kill_result = m_action.get()->kill();
    if (kill_result != mavsdk::Action::Result::Success) {
      std::cerr << "Kill failed: " << kill_result << std::endl;
      return false;
    }
    return true;
  }

  bool holdPosition()
  {
    if (m_telemetry.get()->in_air()) {
      if (m_telemetry.get()->gps_info().fix_type != mavsdk::Telemetry::FixType::NoGps) {
        // Action::hold() doesn't work with PX4 when in offboard mode
        const mavsdk::Action::Result hold_result = m_action.get()->hold();
        if (hold_result != mavsdk::Action::Result::Success) {
          std::cerr << "Hold failed: " << hold_result << std::endl;
          return false;
        }
      } else {
        if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
          if (m_verbose) {
            std::cout << "Cannot set vehicle velocity: offboard mode not started" << std::endl;
          }
          return false;
        }

        setPositionRelative(0., 0., 0., 0., false, 10.); // timeout not used
      }
    }
    return true;
  }

  bool stopMoving()
  {
    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      if (m_verbose) {
        std::cout << "Cannot stop moving: offboard mode not started" << std::endl;
      }
      return false;
    }

    const mavsdk::Offboard::VelocityBodyYawspeed stay{};
    m_offboard.get()->set_velocity_body(stay);

    return true;
  }

  bool releaseControl()
  {
    auto offboard_result = m_offboard.get()->stop();
    if (offboard_result != mavsdk::Offboard::Result::Success) {
      std::cerr << "Offboard stop failed: " << offboard_result << '\n';
      return false;
    }
    std::cout << "Offboard stopped\n";
    return true;
  }

  void setAutoLand(bool auto_land)
  {
    m_auto_land = auto_land;
  }

  bool setYawSpeed(double body_frd_wz)
  {
    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      if (m_verbose) {
        std::cout << "Cannot set vehicle velocity: offboard mode not started" << std::endl;
      }
      return false;
    }
    mavsdk::Offboard::VelocityBodyYawspeed velocity_comm{};
    velocity_comm.forward_m_s = 0.0;
    velocity_comm.right_m_s = 0.0;
    velocity_comm.down_m_s = 0.0;
    velocity_comm.yawspeed_deg_s = vpMath::deg(body_frd_wz);
    m_offboard.get()->set_velocity_body(velocity_comm);

    return true;
  }

  bool setForwardSpeed(double body_frd_vx)
  {
    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      if (m_verbose) {
        std::cout << "Cannot set vehicle velocity: offboard mode not started" << std::endl;
      }
      return false;
    }

    mavsdk::Offboard::VelocityBodyYawspeed velocity_comm{};
    velocity_comm.forward_m_s = body_frd_vx;
    velocity_comm.right_m_s = 0.0;
    velocity_comm.down_m_s = 0.0;
    velocity_comm.yawspeed_deg_s = 0.0;
    m_offboard.get()->set_velocity_body(velocity_comm);

    return true;
  }

  bool setLateralSpeed(double body_frd_vy)
  {
    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      if (m_verbose) {
        std::cout << "Cannot set vehicle velocity: offboard mode not started" << std::endl;
      }
      return false;
    }
    mavsdk::Offboard::VelocityBodyYawspeed velocity_comm{};
    velocity_comm.forward_m_s = 0.0;
    velocity_comm.right_m_s = body_frd_vy;
    velocity_comm.down_m_s = 0.0;
    velocity_comm.yawspeed_deg_s = 0.0;
    m_offboard.get()->set_velocity_body(velocity_comm);

    return true;
  }

  bool setVerticalSpeed(double body_frd_vz)
  {
    if (m_telemetry.get()->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard) {
      if (m_verbose) {
        std::cout << "Cannot set vehicle velocity: offboard mode not started" << std::endl;
      }
      return false;
    }
    mavsdk::Offboard::VelocityBodyYawspeed velocity_comm{};
    velocity_comm.forward_m_s = 0.0;
    velocity_comm.right_m_s = 0.0;
    velocity_comm.down_m_s = body_frd_vz;
    velocity_comm.yawspeed_deg_s = 0.0;
    m_offboard.get()->set_velocity_body(velocity_comm);

    return true;
  }

  bool getFlyingCapability() { return m_has_flying_capability; }

  void setVerbose(bool verbose)
  {
    m_verbose = verbose;
  }

  // void waitSystemReady()
  // {
  //   if (! m_system_ready) 
  //   {
  //     while (!m_telemetry.get()->health_all_ok()) {
  //       std::cout << "Waiting for system to be ready\n";
  //       sleep_for(seconds(1));
  //     }
  //     std::cout << "System is ready\n";
  //   }
  // }

private:
  //*** Attributes ***//
  std::string m_address{}; ///< Ip address of the robot to discover on the network
  mavsdk::Mavsdk m_mavsdk{};
  std::shared_ptr<mavsdk::System> m_system;
  std::shared_ptr<mavsdk::Action> m_action;
  std::shared_ptr<mavsdk::Telemetry> m_telemetry;
  std::shared_ptr<mavsdk::Offboard> m_offboard;

  double m_takeoffAlt{1.0}; ///< The altitude to aim for when calling the function takeoff

  MAV_TYPE m_mav_type{}; // Vehicle type
  bool m_has_flying_capability{false};

  bool m_system_ready{false};
  float m_position_incertitude{0.05};
  float m_yaw_incertitude{0.09}; // 5 deg
  bool m_verbose{false};
  bool m_auto_land{true};
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
 * Set default positioning incertitude to 0.05 meter in translation, and 5 degrees along yaw orientation.
 * These default values are used to determine when a position is reached and could be changed using setPositioningIncertitude().
 * When the vehicle has flying capabilities, call by default land() in the destructor. This behavior could be changed
 * using setAutoLand().
 * 
 * To control the vehicle using this class, you need to call takeControl() to start the off-board mode with PX4 or the 
 * guided mode with Ardupilot. After this call you can call setPosition() to move the vehicle to a desired position
 * and yaw orientation or call setVelocity() to move the vehicle in velocity.
 *
 * \param[in] connection_info : Specify connection information. This parameter must be written following these
 * conventions:
 * - for TCP link: tcp://[server_host][:server_port]
 * - for UDP link: udp://[bind_host][:bind_port]
 * - for Serial link: serial:///path/to/serial/dev[:baudrate]<br>
 * Examples: udp://192.168.30.111:14550 or serial:///dev/ttyACMO
 *
 * For more information see [here](https://mavsdk.mavlink.io/main/en/cpp/guide/connections.html).
 *
 * \exception vpException::fatalError : If the program failed to connect to the vehicle.
 * 
 * \sa setPositioningIncertitude(), setAutoLand(), takeControl(), releaseControl()
 */
vpRobotMavsdk::vpRobotMavsdk(const std::string &connection_info) : m_impl(new vpRobotMavsdkImpl(connection_info))
{
  m_impl->setPositioningIncertitude(0.05, vpMath::rad(5));
}

/*!
 * Default constructor without parameters. You need to use the connect() function afterwards.
 *
 * Set default positioning incertitude to 0.05 meter in translation, and 5 degrees along yaw orientation.
 * These default values are used to determine when a position is reached and could be changed using setPositioningIncertitude().
 * When the vehicle has flying capabilities, call by default land() in the destructor. This behavior could be changed
 * using setAutoLand().
 * 
 * To control the vehicle using this class, you need to call takeControl() to start the off-board mode with PX4 or the 
 * guided mode with Ardupilot. After this call you can call setPosition() to move the vehicle to a desired position
 * and yaw orientation or call setVelocity() to move the vehicle in velocity.
 *
 * \sa connect(), setPositioningIncertitude()
 */
vpRobotMavsdk::vpRobotMavsdk() : m_impl(new vpRobotMavsdkImpl()) 
{
  m_impl->setPositioningIncertitude(0.05, vpMath::rad(5));
}

/*!
 * Destructor.
 * When the vehicle has flying capabilities and when auto land mode is enabled, lands the vehicle if not landed
 * and safely disconnects everything.
 * 
 * \sa setAutoLand()
 */
vpRobotMavsdk::~vpRobotMavsdk() { delete m_impl; }

/*!
 * Connects to the vehicle and setups the different controllers.
 * \param[in] connection_info : The connection information given to connect to the vehicle. You may use:
 * - for TCP link: tcp://[server_host][:server_port]
 * - for UDP link: udp://[bind_host][:bind_port]
 * - for Serial link: serial:///path/to/serial/dev[:baudrate]<br>
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
 * We consider here that the MoCap global reference frame is ENU (East-North-Up).
 * The vehicle body frame if FLU (Front-Left-Up) where X axis is aligned
 * with the vehicle front axis and Z axis going upward.
 * 
 * Internally, this pose called `enu_M_flu` is transformed to match the requirements of the Pixhawk
 * into `ned_M_frd` corresponding to the FRD (Front-Right-Down) body frame position in the NED (North-East-Down) 
 * local reference frame.
 *
 * \return true if the MoCap data was successfully sent to the vehicle, false otherwise.
 * \param[in] enu_M_flu : Homogeneous matrix containing the pose of the vehicle given by the MoCap system.
 * To be more precise, this matrix gives the pose of the vehicle FLU body frame returned by the MoCap where
 * MoCap global reference frame is defined as ENU.
 * \param[in] display_fps : Display `ned_M_frd` pose internally sent through mavlink at the given framerate. A value of 0 can 
 * be used to disable this display.
 *
 * Internally we transform this FRD pose in a NED global reference frame as expected by Pixhawk convention.
 */
bool vpRobotMavsdk::sendMocapData(const vpHomogeneousMatrix &enu_M_flu, int display_fps)
{
  return m_impl->sendMocapData(enu_M_flu, display_fps);
}

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
 * Gets the current vehicle FRD position in its local NED frame.
 * \param[in] ned_M_frd : Homogeneous matrix describing the position and attitude of the vehicle returned by telemetry.
 */
void vpRobotMavsdk::getPosition(vpHomogeneousMatrix &ned_M_frd) const { m_impl->getPosition(ned_M_frd); }

/*!
 * Gets the current vehicle FRD position in its local NED frame.
 * \param[in] ned_north : Position of the vehicle along NED north axis in [m].
 * \param[in] ned_east : Position of the vehicle along NED east axis in [m].
 * \param[in] ned_down : Position of the vehicle along NED down axis in [m].
 * \param[in] ned_yaw : Yaw angle in [rad] of the vehicle along NED down axis.
 */
void vpRobotMavsdk::getPosition(float &ned_north, float &ned_east, float &ned_down, float &ned_yaw) const
{
  m_impl->getPosition(ned_north, ned_east, ned_down, ned_yaw); 
}

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
 * \param[in] timeout_sec : Time out in seconds to acchieve takeoff.
 * \return
 * - If the vehicle has flying capabilities, returns true if the take off is successful, false otherwise,
 *   typically when a timeout occurs. If the vehicle has flying capabilities and is already flying, return true.
 * - If the vehicle doesn't have flying capabilities, returns true.
 * \warning This function is blocking.
 * \sa setTakeOffAlt(), land(), hasFlyingCapability()
 */
bool vpRobotMavsdk::takeOff(bool interactive, int timeout_sec) { return m_impl->takeOff(interactive, timeout_sec); }

/*!
 * Sends take off command when the vehicle has flying capabilities.
 * \param[in] interactive : If true asks the user if the offboard mode is to be forced through the terminal. If false
 * offboard mode is automatically set.
 * \param[in] takeoff_altitude : Take off altitude in [m]. Should be a positive value.
 * \param[in] timeout_sec : Time out in seconds to acchieve takeoff.
 * \return
 * - If the vehicle has flying capabilities, returns true if the take off is successful, false otherwise,
 *   typically when a timeout occurs.
 * - If the vehicle doesn't have flying capabilities, returns true.
 * \warning This function is blocking.
 * \sa setTakeOffAlt(), land(), hasFlyingCapability()
 */
bool vpRobotMavsdk::takeOff(bool interactive, double takeoff_altitude, int timeout_sec) 
{
  m_impl->setTakeOffAlt(takeoff_altitude);
  return m_impl->takeOff(interactive, timeout_sec); 
}

/*!
 * Makes the vehicle hold its position.
 * \warning When the vehicle is equipped with a GPS, switches to hold mode. It means that takeControl()
 * needs to be called after. 
 *
 * \return true when success, false otherwise.
 */
bool vpRobotMavsdk::holdPosition() { return m_impl->holdPosition(); };

/*!
 * Stops any vehicle movement.
 * \warning Depending on the speed of the vehicle when the function is called, it may still move a bit until it
 * stabilizes.
 */
bool vpRobotMavsdk::stopMoving() { return m_impl->stopMoving(); }; 

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
 * \param[in] ned_north : Absolute position to reach along north axis (meters).
 * \param[in] ned_east : Absolute position to reach along east axis (meters).
 * \param[in] ned_down : Absolute position to reach along down axis (meters).
 * \param[in] ned_yaw : Absolute position to reach of the heading (radians).
 * \param[in] blocking : When true this function is blocking until the position is reached.
 * \param[in] timeout_sec : Timeout value in seconds applied when `blocking` is set to true.
 * \return true when positioning succeed, false otherwise, typically when timeout occurs before reaching the position. 
 *
 * \sa setPosition(const vpHomogeneousMatrix &, bool, float)
 * \sa setPositionRelative(float, float, float, float, bool, float)
 */
bool vpRobotMavsdk::setPosition(float ned_north, float ned_east, float ned_down, float ned_yaw, bool blocking, int timeout_sec)
{
  return m_impl->setPosition(ned_north, ned_east, ned_down, ned_yaw, blocking, timeout_sec);
}

/*!
 * Moves the vehicle Front-Right-Down (FRD) body frame with respect to the global reference NED frame.
 *
 * \param[in] ned_M_frd : Homogeneous matrix that express the FRD absolute position to reach by the vehicle expressed
 * in the NED global reference frame.
 * \param[in] blocking : When true this function is blocking until the position is reached.
 * \param[in] timeout_sec : Timeout value in seconds applied when `blocking` is set to true.
 * \return true when positioning succeed, false otherwise, typically when timeout occurs before reaching the position. 
 *
 * \warning The rotation around the FRD X and Y axes should be equal to 0, as the vehicle (drone or rover)
 * cannot rotate around these axes.
 * \warning This function is blocking.
 *
 * \sa setPosition(float, float, float, float, bool, float)
 * \sa setPositionRelative(const vpHomogeneousMatrix &, bool, float)
 */
bool vpRobotMavsdk::setPosition(const vpHomogeneousMatrix &ned_M_frd, bool blocking, int timeout_sec) 
{
  return m_impl->setPosition(ned_M_frd, blocking, timeout_sec); 
}

/*!
 * Moves the vehicle Front-Right-Down (FRD) body frame with respect to the global reference NED frame.
 *
 * \param[in] ned_delta_north : Relative displacement along north (meters).
 * \param[in] ned_delta_east : Relative displacement along east (meters).
 * \param[in] ned_delta_down : Relative displacement along down axis (meters).
 * \param[in] ned_delta_yaw : Relative rotation of the heading (radians).
 * \param[in] blocking : When true this function is blocking until the position is reached.
 * \param[in] timeout_sec : Timeout value in seconds applied when `blocking` is set to true.
 * \return true when positioning succeed, false otherwise, typically when timeout occurs before reaching the position. 
 *
 * \sa setPositionRelative(const vpHomogeneousMatrix &, bool, float)
 * \sa setPosition(float, float, float, float, bool, float)
 */
bool vpRobotMavsdk::setPositionRelative(float ned_delta_north, float ned_delta_east, float ned_delta_down, float ned_delta_yaw, bool blocking, int timeout_sec)
{
  return m_impl->setPositionRelative(ned_delta_north, ned_delta_east, ned_delta_down, ned_delta_yaw, blocking, timeout_sec);
}

/*!
 * Moves the vehicle Front-Right-Down (FRD) body frame with respect to the global reference NED frame.
 *
 * \param[in] delta_frd_M_frd : Homogeneous matrix that express the FRD absolute position to reach by the vehicle expressed
 * in the NED global reference frame.
 * \param[in] blocking : When true this function is blocking until the position is reached.
 * \param[in] timeout_sec : Timeout value in seconds applied when `blocking` is set to true.
 * \return true when positioning succeed, false otherwise, typically when timeout occurs before reaching the position. 
 *
 * \warning The rotation around the FRD X and Y axes should be equal to 0, as the vehicle (drone or rover)
 * cannot rotate around these axes.
 * \warning This function is blocking.
 *
 * \sa setPositionRelative(float, float, float, float, bool, float)
 * \sa setPosition(const vpHomogeneousMatrix &, bool, float)
 */
bool vpRobotMavsdk::setPositionRelative(const vpHomogeneousMatrix &delta_frd_M_frd, bool blocking, int timeout_sec)
{
  return m_impl->setPositionRelative(delta_frd_M_frd, blocking, timeout_sec); 
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
bool vpRobotMavsdk::setVelocity(const vpColVector &frd_vel_cmd) { return m_impl->setVelocity(frd_vel_cmd); }

/*!
 * Cuts the motors like a kill switch. Should only be used in emergency cases.
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
 *
 * \return true when success, false otherwise.
 */
bool vpRobotMavsdk::setYawSpeed(double body_frd_wz) { return m_impl->setYawSpeed(body_frd_wz); }

/*!
 * Sets the forward speed, expressed in m/s, in the Front-Right-Down body frame.
 *
 * \warning The vehicle will not stop moving in that direction until you send another motion command.
 *
 * \param[in] body_frd_vx : Desired FRD body frame forward speed in m/s.
 * - Positive values will make the vehicle go forward
 * - Negative values will make the vehicle go backwards
 *
 * \return true when success, false otherwise.
 */
bool vpRobotMavsdk::setForwardSpeed(double body_frd_vx) { return m_impl->setForwardSpeed(body_frd_vx); }

/*!
 * Sets the lateral speed, expressed in m/s, in the Front-Right-Down body frame.
 *
 * \warning The vehicle will not stop moving in that direction until you send another motion command.
 *
 * \param[in] body_frd_vy : Desired FRD body frame lateral speed in m/s.
 * - Positive values will make the vehicle go right
 * - Negative values will make the vehicle go left
 *
 * \return true when success, false otherwise.
 */
bool vpRobotMavsdk::setLateralSpeed(double body_frd_vy) { return m_impl->setLateralSpeed(body_frd_vy); }

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
 * Take control using software running outside of the autopilot:
 * - When using the PX4 flight stack start the off-board mode,
 * - When using Ardupilot stack start the guided mode.
 *
 * \return true when off-board or guided mode are successfully started, false otherwise.
 *
 * This method should be called before using setPosition(), setVelocity()
 * 
 * \sa releaseControl()
 */
bool vpRobotMavsdk::takeControl()
{
  return m_impl->takeControl();
}

/*!
 * Release control allowing running software outside of the autopilot:
 * - When using the PX4 flight stack stop the off-board mode,
 * - When using Ardupilot stack stop the guided mode.
 *
 * \return true when off-board or guided mode are successfully stopped, false otherwise.
 *
 * \sa takeControl()
 */
bool vpRobotMavsdk::releaseControl()
{
  return m_impl->releaseControl();
}

/*!
 * Enable/disable auto land mode in the destructor.
 * \param[in] auto_land : When true auto land mode is enabled and the destructor calls land() when
 * the vehicle has flying capabilities. When false the destructor doesn't call land().
 * 
 * \sa land()
 */
void vpRobotMavsdk::setAutoLand(bool auto_land)
{
  m_impl->setAutoLand(auto_land);
}

/*!
 * Incertitude used to decided if a position is reached when using setPosition() and setPositionRelative().
 * \param[in] position_incertitude : Position incertitude in [m].
 * \param[in] yaw_incertitude : Yaw angle incertitude in [rad].
 *
 * \sa setPosition(), setPositionRelative()
 */
void vpRobotMavsdk::setPositioningIncertitude(float position_incertitude, float yaw_incertitude)
{
  m_impl->setPositioningIncertitude(position_incertitude, yaw_incertitude);
}

/*!
 * Sets the vertical speed, expressed in m/s, in the Front-Right-Down body frame.
 *
 * \warning The vehicle will not stop moving in that direction until you send another motion command.
 *
 * \param[in] body_frd_vz : Desired FRD body frame vertical speed in m/s.
 * - Positive values will make the vehicle go down.
 * - Negative values will make the vehicle go up.
 *
 * \return true when success, false otherwise.
 */
bool vpRobotMavsdk::setVerticalSpeed(double body_frd_vz) { return m_impl->setVerticalSpeed(body_frd_vz); }

/*!
 * Enable/disable verbose mode.
 *
 * \param[in] verbose : When true enable verbose mode.
 */
void vpRobotMavsdk::setVerbose(bool verbose)
{
  m_impl->setVerbose(verbose);
}

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
