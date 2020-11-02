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

#ifndef REFLEX_DRIVER2_H
#define REFLEX_DRIVER2_H

#include <string>
#include <signal.h>
#include <vector>
#include <fstream>

#include <reflex_hand2.h>

namespace reflex_driver2 {

  class ReflexDriver {
  public:
    ReflexDriver();
    void open(const std::string &network_interface,
              const std::string &finger_file_name,
              const std::string &tactile_file_name,
              const std::string &motor_file_name);
    ~ReflexDriver()
    {
      disable_torque();
    }

    bool calibrated;
    reflex_hand2::ReflexHand reflex_hand;
    reflex_hand2::ReflexHand *rh;
    reflex_hand2::HandInfo hand_info;

    void listen(int max_seconds);
    void wait(int milliseconds);

    void set_angle_position(const float *targets);
    void set_motor_speed(const float *targets);
    void move_until_any_contact(const float *velocities);
    void move_until_each_contact(const float *velocities);
    void disable_torque();

    void calibrate_tactile();
    void calibrate_fingers();
    void calibrate();

    void populate_tactile_thresholds(int threshold);
    void set_tactile_thresholds(const int *thresholds);

    void reflex_hand_state_cb(const reflex_hand2::ReflexHandState * const state);

  private:
    const int TACTILE_BASE_IDX[NUM_FINGERS] = {0, 28, 14};                    // Constant that differ from driver1
    const uint16_t CALIBRATION_DYN_OFFSET[NUM_DYNAMIXELS] = {50, 50, 50, 0};  // Constant

    // Configurable default parameters
    const int DEFAULT_CONTACT_THRESHOLD = 20; // Minimum sensor reading that registers a sensor is making contact
    const float MOTOR_INCREMENT = 0.05;       // Amount the fingers move when moving until contact
    const float MAX_LOAD = 250;               // Maximum load a motor can take before stopping
    const float MIN_ANGLE = -0.001;           // Smallest angle the motors can move to before stopping

    std::fstream finger_file;
    std::fstream tactile_file;
    std::fstream motor_file;

    std::string finger_file_address;
    std::string tactile_file_address;
    std::string motor_file_address;

    // tactile file
    std::vector<std::vector<int>> tactile_offset_values; // Loaded from yaml and reset during calibration

    // finger file
    std::vector<float> dynamixel_zero_point;  // Loaded from yaml and reset during calibration
    std::vector<float> encoder_zero_point;    // Loaded from yaml and reset during calibration

    // motor constants
    std::vector<int> motor_to_joint_inverted;      // Loaded from yaml
    std::vector<float> motor_to_joint_gear_ratio; // Loaded from yaml

    bool acquire_tactile;         // Updated by calibrate methods and in reflex_hand_state_cb()
    bool acquire_fingers;         // Updated by calibrate methods and in reflex_hand_state_cb()
    bool first_capture;           // Updated by calibrate methods and in reflex_hand_state_cb()
    bool all_fingers_moved;       // Updated in reflex_hand_state_cb()

    int encoder_last_value[NUM_ENCODERS] {0, 0, 0};        // Updated constantly in reflex_hand_state_cb()
    int encoder_offset[NUM_ENCODERS] {-1, -1, -1};         // Updated constantly in reflex_hand_state_cb()
    float load_last_value[NUM_DYNAMIXELS] {0, 0, 0, 0};    // Updated constantly in reflex_hand_state_cb()
    int raw_cmd_last_value[NUM_DYNAMIXELS] {0, 0, 0, 0};   // Updated in set_raw_position and set_angle_position

    uint16_t calibration_dyn_increase[NUM_DYNAMIXELS] {6, 6, 6, 0};   // Updated in reflex_hand_state_cb() during calibration
    time_t latest_calibration_time;                                   // Updated in reflex_hand_state_cb() during calibration

    int contact_thresholds[NUM_FINGERS][NUM_SENSORS_PER_FINGER]; // Set by the user

    static void signal_handler(int signum) {
      if (signum == SIGINT || signum == SIGTERM) {
        printf("Exiting. Have a nice day.\n");
        exit(1);
      }
    }

    void split(std::string &input, std::string delim, std::vector<int> &split_vector);
    void split(std::string &input, std::string delim, std::vector<float> &split_vector);
    std::vector<std::string> extract_file_arrays(std::fstream &file, std::string &file_name);
    void load_params();

    bool approx_equal(float a, float b, float error);

    uint16_t pos_rad_to_raw(float rad_command, int motor_idx);
    uint16_t speed_rad_to_raw(float rad_per_s_command, int motor_idx);
    float speed_raw_to_rad(uint16_t raw_per_s_command, int motor_idx);
    float load_raw_to_signed(int load, int motor_idx);
    void check_for_potential_motor_wraps_and_rezero();

    void set_raw_position(const uint16_t *targets);
    void move_fingers_in(const reflex_hand2::ReflexHandState * const state);

    int pressure_offset(int finger, int sensor);
    int update_encoder_offset(int raw_value, int last_value, int current_offset);
    bool any_finger_touching();
    bool* each_finger_touching(bool *touching);

    float calc_proximal_angle(int raw_enc_value, int offset, float zero);
    float calc_motor_angle(int inversion, int raw_dyn_value, float ratio, float zero);
    float calc_distal_angle(float spool, float proximal);
    int calc_pressure(const reflex_hand2::ReflexHandState * const state, int finger, int sensor);
    int calc_contact(const reflex_hand2::ReflexHandState * const state, int finger, int sensor);

    void calibrate_tactile_sensors(const reflex_hand2::ReflexHandState* const state);
    void calibrate_encoders_locally(const reflex_hand2::ReflexHandState* const state);
    void calibrate_motors_locally(const reflex_hand2::ReflexHandState* const state);

    void log_current_tactile_locally(const reflex_hand2::ReflexHandState* const state);
    void log_current_tactile_to_file(const reflex_hand2::ReflexHandState* const state);
    void log_encoder_zero_to_file();
    void log_motor_zero_to_file_and_close();

    bool check_for_finger_movement(const reflex_hand2::ReflexHandState* const state);
    void populate_motor_state(const reflex_hand2::ReflexHandState* const state);
  };
}

#endif
