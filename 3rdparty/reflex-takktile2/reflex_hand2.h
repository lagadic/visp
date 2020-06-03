/////////////////////////////////////////////////////////////////////////////
//
//   Copyright 2017 Open Source Robotics Foundation, Inc.
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef REFLEX_HAND2_H
#define REFLEX_HAND2_H

#include <netinet/in.h>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <stdio.h>
#include <iostream>
#include <errno.h>
#include <functional>

#define NUM_FINGERS                   3
#define NUM_IMUS                      4
#define NUM_SENSORS_PER_FINGER        14
#define NUM_SENSORS_PALM              11
#define NUM_TAKKTILE                  (NUM_FINGERS * NUM_SENSORS_PER_FINGER + NUM_SENSORS_PALM)
#define NUM_ENCODERS                  NUM_FINGERS
#define NUM_SERVOS                    4
#define NUM_DYNAMIXELS                NUM_SERVOS

namespace reflex_hand2 {

    typedef struct {
        uint8_t  header[6];
        uint32_t systime;
        uint16_t tactile_pressures[NUM_TAKKTILE];
        uint16_t tactile_temperatures[NUM_TAKKTILE];
        uint16_t encoders[NUM_FINGERS];
        uint8_t  dynamixel_error_states[NUM_DYNAMIXELS];
        uint16_t dynamixel_angles[NUM_DYNAMIXELS];
        uint16_t dynamixel_speeds[NUM_DYNAMIXELS];
        uint16_t dynamixel_loads[NUM_DYNAMIXELS];
        uint8_t  dynamixel_voltages[NUM_DYNAMIXELS];
        uint8_t  dynamixel_temperatures[NUM_DYNAMIXELS];
        uint16_t imus[NUM_IMUS * NUM_DYNAMIXELS];
        int8_t  imu_calibration_status[NUM_IMUS];
        uint16_t imu_calibration_data[NUM_IMUS * NUM_SENSORS_PALM];
    } __attribute__((packed)) mcu_state_format_1_t;

    class HandInfo {
    public:
        float    proximal[NUM_FINGERS];
        float    distal_approx[NUM_FINGERS];
        int      pressure[NUM_FINGERS][NUM_SENSORS_PER_FINGER];
        bool     contact[NUM_FINGERS][NUM_SENSORS_PER_FINGER];

        float    joint_angle[NUM_DYNAMIXELS];
        float    raw_angle[NUM_DYNAMIXELS];
        float    velocity[NUM_DYNAMIXELS];
        float    load[NUM_DYNAMIXELS];
        float    voltage[NUM_DYNAMIXELS];
        uint32_t temperature[NUM_DYNAMIXELS];
        std::string  error_state[NUM_DYNAMIXELS];
        HandInfo();
        ~HandInfo();
    };

  class ReflexHandState {
  public:
    uint32_t systime_us_;
    uint16_t tactile_pressures_[NUM_TAKKTILE];
    uint16_t tactile_temperatures_[NUM_TAKKTILE];
    uint16_t encoders_[NUM_FINGERS];
    uint8_t  dynamixel_error_states_[NUM_DYNAMIXELS];
    uint16_t dynamixel_angles_[NUM_DYNAMIXELS];
    uint16_t dynamixel_speeds_[NUM_DYNAMIXELS];
    uint16_t dynamixel_loads_[NUM_DYNAMIXELS];
    uint8_t  dynamixel_voltages_[NUM_DYNAMIXELS];
    uint8_t  dynamixel_temperatures_[NUM_DYNAMIXELS];
    ReflexHandState();
    ~ReflexHandState();
  };

  class ReflexHand {
  public:
    static const int PORT_BASE = 11333;
    static constexpr uint16_t DYN_MIN_RAW = 0;
    static constexpr uint16_t DYN_MAX_RAW = 4095;
    static constexpr uint16_t DYN_MIN_RAW_WRAPPED = 16383;  // For checking negative wraps
    static constexpr float DYN_POS_SCALE = (4 * 2 * 3.1415926f) / 4095;  // Assuming resolution divider of 4
    static constexpr float DYN_VEL_SCALE = 0.01194f;  // rad/s for every velocity command -- http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-28.htm#Actuator_Address_20
    static constexpr float ENC_SCALE = (2 * 3.1415926f) / 16383;

    enum ControlMode{CM_IDLE = 0, CM_VELOCITY = 1, CM_POSITION = 2};

    typedef std::function<void(const ReflexHandState * const)> StateCallback;
    void setStateCallback(StateCallback callback);

    int setupNetwork(const std::string &network_interface);
    ReflexHand();
    ReflexHand(const std::string &network_interface);
    ~ReflexHand();
    bool listen(const double max_seconds);
    void setServoTargets(const uint16_t *targets);
    void setServoControlModes(const ControlMode *modes);
    void setServoControlModes(const ControlMode mode);
    bool happy() { return happy_; }
    ReflexHandState rx_state_;
  private:
    enum CommandPacket { CP_SET_SERVO_MODE = 1, CP_SET_SERVO_TARGET = 2 };
    int tx_sock_, rx_sock_;
    sockaddr_in mcast_addr_;
    StateCallback state_cb_;

    bool happy_;
    void tx(const uint8_t *msg, const uint16_t msg_len, const uint16_t port);
    void rx(const uint8_t *msg, const uint16_t msg_len);
  };

}

std::ostream& operator<<(std::ostream &os, const reflex_hand2::HandInfo &hand);

#endif
