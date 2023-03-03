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

  virtual ~vpRobotMavsdkImpl() { land(); }

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

    m_telemetry = std::make_shared<mavsdk::Telemetry>(m_system);
    m_offboard = std::make_shared<mavsdk::Offboard>(m_system);
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

  void getPose(vpHomogeneousMatrix &pose) const
  {
    auto quat = m_telemetry.get()->attitude_quaternion();
    auto posvel = m_telemetry.get()->position_velocity_ned();
    vpQuaternionVector q{quat.x, quat.y, quat.z, quat.w};
    vpTranslationVector t{posvel.position.north_m, posvel.position.east_m, posvel.position.down_m};
    pose.buildFrom(t, q);
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

    vpRxyzVector XYZvec = vpRxyzVector(M.getRotationMatrix());
    pose_estimate.angle_body.roll_rad = XYZvec[0];
    pose_estimate.angle_body.pitch_rad = XYZvec[1];
    pose_estimate.angle_body.yaw_rad = XYZvec[2];

    pose_estimate.position_body.x_m = M.getTranslationVector()[0];
    pose_estimate.position_body.y_m = M.getTranslationVector()[1];
    pose_estimate.position_body.z_m = M.getTranslationVector()[2];

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

  bool takeOff(bool interactive)
  {
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
      takeoff.yaw_deg = 0.0;
      m_offboard.get()->set_position_ned(takeoff);
      sleep_for(seconds(10));

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

  void setPosition(float dX, float dY, float dZ, float dPsi)
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

    double X_current = odom.position_body.x_m;
    double Y_current = odom.position_body.y_m;
    double Z_current = odom.position_body.z_m;
    double yaw_current = angles.yaw_deg;

    mavsdk::Offboard::PositionNedYaw position_target{};
    position_target.north_m = X_current + dX;
    position_target.east_m = Y_current + dY;
    position_target.down_m = Z_current + dZ;
    position_target.yaw_deg = yaw_current + vpMath::deg(dPsi);
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

  void setVelocity(const vpColVector &vel_cmd, double delta_t)
  {
    if (vel_cmd.size() != 4) {
      std::cerr << "ERROR : Can't set velocity, dimension of the velocity vector should be equal to 4." << std::endl;
      return;
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
    velocity_comm.forward_m_s = vel_cmd[0];
    velocity_comm.right_m_s = vel_cmd[1];
    velocity_comm.down_m_s = vel_cmd[2];
    velocity_comm.yawspeed_deg_s = vpMath::deg(vel_cmd[3]);
    m_offboard.get()->set_velocity_body(velocity_comm);
    sleep_for(milliseconds((int)(delta_t * 1000.0)));
    m_offboard.get()->set_velocity_body(stay);
  }

  void setVelocity(const vpColVector &vel_cmd)
  {
    if (vel_cmd.size() != 4) {
      std::cerr << "ERROR : Can't set velocity, dimension of the velocity vector should be equal to 4." << std::endl;
      return;
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
    velocity_comm.forward_m_s = vel_cmd[0];
    velocity_comm.right_m_s = vel_cmd[1];
    velocity_comm.down_m_s = vel_cmd[2];
    velocity_comm.yawspeed_deg_s = vpMath::deg(vel_cmd[3]);
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

  void setYawSpeed(double wz)
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
    velocity_comm.yawspeed_deg_s = vpMath::deg(wz);
    m_offboard.get()->set_velocity_body(velocity_comm);
  }

  void setForwardSpeed(double vx)
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
    velocity_comm.forward_m_s = vx;
    velocity_comm.right_m_s = 0.0;
    velocity_comm.down_m_s = 0.0;
    velocity_comm.yawspeed_deg_s = 0.0;
    m_offboard.get()->set_velocity_body(velocity_comm);
  }

  void setLateralSpeed(double vy)
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
    velocity_comm.right_m_s = vy;
    velocity_comm.down_m_s = 0.0;
    velocity_comm.yawspeed_deg_s = 0.0;
    m_offboard.get()->set_velocity_body(velocity_comm);
  }

  void setVerticalSpeed(double vz)
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
    velocity_comm.down_m_s = vz;
    velocity_comm.yawspeed_deg_s = 0.0;
    m_offboard.get()->set_velocity_body(velocity_comm);
  }

private:
  //*** Attributes ***//
  std::string m_address; ///< Ip address of the robot to discover on the network
  mavsdk::Mavsdk m_mavsdk;
  std::shared_ptr<mavsdk::System> m_system;
  std::shared_ptr<mavsdk::Telemetry> m_telemetry;
  std::shared_ptr<mavsdk::Offboard> m_offboard;

  double m_takeoffAlt = 1.0; ///< The altitude to aim for when calling the function takeoff

  static bool m_running; ///< Used for checking if the robot is running ie if successfully connected and ready to
                         ///< receive commands

  // bool m_flatTrimFinished;  ///< Used to know when the robot has finished a flat trim
  // bool m_relativeMoveEnded; ///< Used to know when the robot has ended a relative move
  // bool m_settingsReset;     ///< Used to know when the robot a finished the settings reset

  // unsigned int m_batteryLevel; ///< Percentage of battery remaining
};
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 * Constructor.
 *
 * Initializes robot controller, by discovering robots connected either with an Ethernet TCP or UDP link, or with a
 * serial link the computer is currently connected to. Sets up signal handling to safely land/stop the robot when
 * something wrong happens or when the user stops the binary using CRTL-C.
 *
 * \warning This constructor should be called after the robot is turned on, and after the computer is connected to the
 * robot Ethernet network or with a serial link.
 *
 * \warning If the connection to the robot failed, the program will throw an exception.
 *
 * After having called this constructor, it is recommended to check if the robot is running with isRunning() before
 * sending commands to the robot.
 *
 * \param[in] connection_info : Specify connection information. This parameter must be written following these
 * conventions:
 * - for TCP link: tcp://[server_host][:server_port]
 * - for UDP link: udp://[bind_host][:bind_port]
 * - for Serial link: serial:///path/to/serial/dev[:baudrate]
 * For more information see [here](https://mavsdk.mavlink.io/main/en/cpp/guide/connections.html).
 *
 * \exception vpException::fatalError : If the program failed to connect to the robot.
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
 * Lands the drone if not landed or stops the robot if it is a rover and safely disconnects everything.
 */
vpRobotMavsdk::~vpRobotMavsdk() { delete m_impl; }

/*!
 * Connects to the robot and setups the different controllers.
 * \param[in] connection_info : The connection information given to connect to the robot. You may use:
 * - for TCP link: tcp://[server_host][:server_port]
 * - for UDP link: udp://[bind_host][:bind_port]
 * - for Serial link: serial:///path/to/serial/dev[:baudrate]
 *
 * Example: udp://192.168.30.111:14550.
 *
 * For more information see [here](https://mavsdk.mavlink.io/main/en/cpp/guide/connections.html).
 *
 * \sa getAddress()
 */
void vpRobotMavsdk::connect(const std::string &connection_info) { m_impl->connect(connection_info); }

/*!
 * Checks if the robot is running, ie if the robot is connected and ready to receive commands.
 */
bool vpRobotMavsdk::isRunning() const { return m_impl->isRunning(); }

/*!
 * Sends MoCap position data to the robot.
 * \return True if the MoCap data was successfully sent to the robot, false otherwise.
 * \param[in] M : Homogeneous matrix containing the pose of the robot for the MoCap system.
 */
bool vpRobotMavsdk::sendMocapData(const vpHomogeneousMatrix &M) { return m_impl->sendMocapData(M); }

/*!
 * Gives the address given to connect to the robot.
 * \return : A string corresponding to the Ethernet or serial address used for the connection to the robot.
 *
 * \sa connect()
 */
std::string vpRobotMavsdk::getAddress() const { return m_impl->getAddress(); }

/*!
 * Gets current battery level in Volts.
 * \warning When the robot battery gets below a certain threshold (around 14.8), you should recharge it.
 */
float vpRobotMavsdk::getBatteryLevel() const { return m_impl->getBatteryLevel(); }

/*!
 * Gets the current robot pose in its local NED frame.
 * \param[in] Pose : Homogeneous matrix describing the position and attitude of the robot.
 */
void vpRobotMavsdk::getPose(vpHomogeneousMatrix &Pose) const { m_impl->getPose(Pose); }

/*!
 * Gets the robot home position in GPS coord.
 *
 * \warning Only available if the GPS is initialized, for example
 * in simulation
 */
std::tuple<float, float> vpRobotMavsdk::getHome() const { return m_impl->getHome(); }

/*!
 * Sends a flat trim command to the robot, to calibrate accelerometer and gyro.
 *
 * \warning Should be executed only if the drone is landed and on a flat surface.
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
 * Arms the robot.
 * \return True if arming is successful, False otherwise.
 */
bool vpRobotMavsdk::arm() { return m_impl->arm(); }

/*!
 * Sends take off command. To use if your robot is a drone.
 * \param[in] interactive : If True asks the user if the offboard mode is to be forced through the terminal. If false
 * Offboard mode is automatically set.
 * \return True if the takeoff is successful, False otherwise.
 * \warning This function is blocking.
 * \sa setTakeOffAlt(), land()
 */
bool vpRobotMavsdk::takeOff(bool interactive) { return m_impl->takeOff(interactive); }

/*!
 * Makes the robot hold its position.
 * \warning The function simply switches to Hold mode when the robot is equipped with a GPS. it can function without a
 * GPS, but it still preferable to use it outdoors.
 */
void vpRobotMavsdk::holdPosition() { m_impl->holdPosition(); };

/*!
 * Stops any robot movement.
 * \warning Depending on the speed of the robot when the function is called, the robot may still move a bit until it
 * stabilizes.
 */
void vpRobotMavsdk::stopMoving() { m_impl->stopMoving(); }; // Resolve hold problem

/*!
 * Sends landing command. To use if your robot is a drone.
 * \return True if the landing is successful, false otherwise.
 * \sa takeOff()
 */
bool vpRobotMavsdk::land() { return m_impl->land(); }

/*!
 * Moves the robot by the given amounts \e dX, \e dY, \e dZ (meters) and rotate the heading by \e dPsi (radian) in the
 * Front-Right-Down (FRD) body frame. Doesn't do anything if the drone isn't flying or hovering. This function is
 * blocking.
 *
 * \param[in] dX : Relative displacement along X axis (meters).
 * \param[in] dY : Relative displacement along Y axis (meters).
 * \param[in] dZ : Relative displacement along Z axis (meters).
 * \param[in] dPsi : Relative rotation of the heading (radians).
 *
 * \sa setPosition(const vpHomogeneousMatrix &M, bool blocking)
 */
void vpRobotMavsdk::setPosition(float dX, float dY, float dZ, float dPsi) { m_impl->setPosition(dX, dY, dZ, dPsi); }

/*!
 * Changes the relative position of the robot based on a homogeneous transformation matrix.
 *
 * \param[in] M : Homogeneous matrix with translation be expressed in meters and rotation in radians.
 *
 * \warning The rotation around the X and Y axes should be equal to 0, as the robot (drone or rover)
 * cannot rotate around these axes.
 * \warning This function is blocking.
 *
 * \sa setPosition(float dX, float dY, float dZ, float dPsi, bool blocking)
 */
void vpRobotMavsdk::setPosition(const vpHomogeneousMatrix &M) { m_impl->setPosition(M); }

/*!
 * Sets the robot velocity in its own Front-Right-Down (FRD) frame for a duration delta_t. The robot stops afterwards.
 *
 * \param[in] vel_cmd : 4-dim robot velocity commands, vx, vy, vz, wz. Translation velocities (vx, vy, vz) should be
 * expressed in meters and rotation velocity (wz) in radians.
 * \param[in] delta_t : Sampling time (in seconds), time during which the velocity `vel_cmd` is applied.
 *
 * \warning The dimension of the velocity vector should be equal to 4, as the robot cannot rotate around X and Y axes.
 */
void vpRobotMavsdk::setVelocity(const vpColVector &vel_cmd, double delta_t) { m_impl->setVelocity(vel_cmd, delta_t); }

/*!
 * Sets the robot velocity in its own Front-Right-Down (FRD) frame.
 *
 * \param[in] vel_cmd : 4-dim robot velocity commands, vx, vy, vz, wz. Translation velocities (vx, vy, vz) should be
 * expressed in meters and rotation velocity (wz) in radians.
 *
 * \warning The dimension of the velocity vector should be equal to 4, as the robot cannot rotate around X and Y axes.
 * \warning The robot applies this command until given another one.
 */
void vpRobotMavsdk::setVelocity(const vpColVector &vel_cmd) { m_impl->setVelocity(vel_cmd); }

/*!
 * Cuts the motors. Should only be used in emergency cases.
 * \return True if the cut motors command was successful, false otherwise.
 * \warning If your robot is a drone, it will fall.
 */
bool vpRobotMavsdk::kill() { return m_impl->kill(); }

/*!
 * Sets the yaw speed, expressed as signed rotation speed.
 *
 * \warning The robot will not stop moving in that direction until you send another motion command.
 *
 * \param[in] wz : Desired yaw speed in rad/s.
 * Positive values will make the robot turn to its right / clockwise
 * Negative values will make the robot turn to its left / counterclockwise
 */
void vpRobotMavsdk::setYawSpeed(double wz) { m_impl->setYawSpeed(wz); }

/*!
 * Sets the forward speed, expressed in m/s , in the Front-Right-Down body frame.
 *
 * \warning The robot will not stop moving in that direction until you send another motion command.
 *
 * \param[in] vx : Desired forward speed in m/s.
 * - Positive values will make the robot go forward
 * - Negative values will make the robot go backwards
 */
void vpRobotMavsdk::setForwardSpeed(double vx) { m_impl->setForwardSpeed(vx); }

/*!
 * Sets the lateral speed, expressed in m/s, in the Front-Right-Down body frame.
 *
 * \warning The robot will not stop moving in that direction until you send another motion command.
 *
 * \param[in] vy : desired lateral speed in m/s.
 * - Positive values will make the robot go right
 * - Negative values will make the robot go left
 */
void vpRobotMavsdk::setLateralSpeed(double vy) { m_impl->setLateralSpeed(vy); }

/*!
 * Sets the vertical speed, expressed in m/s, in the Front-Right-Down body frame.
 *
 * \warning The robot will not stop moving in that direction until you send another motion command.
 *
 * \param[in] vz : desired vertical speed in m/s.
 * - Positive values will make the robot go down.
 * - Negative values will make the robot go up.
 */
void vpRobotMavsdk::setVerticalSpeed(double vz) { m_impl->setVerticalSpeed(vz); }

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpRobotMavsdk.cpp.o) has no
// symbols
void dummy_vpRobotMavsdk(){};
#endif
