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

#include <chrono>
#include <thread>
#include <cmath>
#include <sstream>

#include <reflex_hand2.h>
#include <reflex_driver2.h>

using namespace reflex_driver2;

#define CALIBRATION_ERROR 0.05f  // Encoder delta signifying movement in calibration

ReflexDriver::ReflexDriver()
  : reflex_hand(), tactile_offset_values(NUM_FINGERS, std::vector<int>(0)) {
  acquire_tactile = false;
  acquire_fingers = false;
  first_capture = false;
  all_fingers_moved = false;
  calibrated = false;
}

void ReflexDriver::open(const std::string &network_interface_,
                        const std::string &finger_file_name_,
                        const std::string &tactile_file_name_,
                        const std::string &motor_file_name_)
{
  acquire_tactile = false;
  acquire_fingers = false;
  first_capture = false;
  all_fingers_moved = false;
  calibrated = false;

  reflex_hand.setupNetwork(network_interface_);
  finger_file_address =  finger_file_name_;
  tactile_file_address =  tactile_file_name_;
  motor_file_address =  motor_file_name_;

  load_params();
  populate_tactile_thresholds(DEFAULT_CONTACT_THRESHOLD);

  // Intialize the reflex_hand object
  printf("Starting reflex_hand_driver on network interface %s.\n",
           network_interface_.c_str());
  rh = &reflex_hand;
  if (!rh->happy()) {
    printf("Error during initialization. Bailing now. Have a nice day.\n");
    exit(1);
  }

  rh->setServoControlModes(reflex_hand2::ReflexHand::CM_POSITION);

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  latest_calibration_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

  printf("Entering main reflex_driver program...\n");

  if (!rh->listen(1)) {
    printf("Error in listen.\n");
  }
}


// Splits a string of integers into a vector
void ReflexDriver::split(std::string &input, std::string delim, std::vector<int> &split_vector) {
  std::stringstream stream;
  stream.str(input);
  std::string entry;
  std::string::size_type size;
  std::string::size_type position;
  int num;

  while ((position = input.find(delim)) != std::string::npos) {
    entry = input.substr(0, position);
    num = std::stoi(entry, &size);
    split_vector.push_back(num);
    input.erase(0, position + delim.length() - 1);
  }

  num = std::stoi(input, &size);
  split_vector.push_back(num);
}


// Splits a string of floats into a vector
void ReflexDriver::split(std::string &input, std::string delim, std::vector<float> &split_vector) {
  std::stringstream stream;
  stream.str(input);
  std::string entry;
  std::string::size_type size;
  std::string::size_type position;
  float num;

  while ((position = input.find(delim)) != std::string::npos) {
    entry = input.substr(0, position);
    num = std::stof(entry, &size);
    split_vector.push_back(num);
    input.erase(0, position + delim.length());
  }

  num = std::stof(input, &size);
  split_vector.push_back(num);
}


// Gets a string representation of an array out of a file
std::vector<std::string> ReflexDriver::extract_file_arrays(std::fstream &file, std::string &file_name) {
  std::string line;
  std::string values_substring;
  std::string::size_type comment_index;
  std::string::size_type begin_array_index;
  std::string::size_type end_array_index;

  std::vector<std::string> values;

  file.open(file_name.c_str());
  while(std::getline(file, line)) {
    comment_index = line.find('#');
    if (comment_index != std::string::npos) {
      continue;
    }
    begin_array_index = line.find('[');
    end_array_index = line.find(']');
    if (begin_array_index == std::string::npos || end_array_index == std::string::npos) {
      continue;
    }
    values.push_back(line.substr(begin_array_index + 1, end_array_index - begin_array_index - 1));
  }
  file.close();

  return values;
}


// Loads the values from the yaml files into local variables
void ReflexDriver::load_params() {
  // motor constants
  std::vector<std::string> motor_values = extract_file_arrays(motor_file, motor_file_address);

  split(motor_values[0], ", ", motor_to_joint_inverted);
  split(motor_values[1], ", ", motor_to_joint_gear_ratio);

  // finger params
  std::vector<std::string> finger_values = extract_file_arrays(finger_file, finger_file_address);
  split(finger_values[0], ", ", encoder_zero_point);
  split(finger_values[1], ", ", dynamixel_zero_point);

  // tactile params
  std::vector<std::string> tactile_values = extract_file_arrays(tactile_file, tactile_file_address);
  for (size_t i = 0; i < NUM_FINGERS; i++) {
    split(tactile_values[i], ", ", tactile_offset_values[i]);
  }
}


// Returns if two floats are close
bool ReflexDriver::approx_equal(float a, float b, float error) {
  return std::abs(a - b) <= std::abs(error);
}


// Tries to communicate with the ReflexHand object
void ReflexDriver::listen(int max_seconds) {
  rh->listen(max_seconds);
}


// Tries to communicate with the ReflexHand object
void ReflexDriver::wait(int milliseconds) {
  std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
  while (std::chrono::system_clock::now() - start_time < std::chrono::milliseconds(milliseconds)) {
    rh->listen(1);
  }
}


// Takes a rad command and returns Dynamixel command
uint16_t ReflexDriver::pos_rad_to_raw(float rad_command, int motor_idx) {
  size_t idx = static_cast<size_t>(motor_idx);
  float zeroed_command = motor_to_joint_inverted[idx] * rad_command + dynamixel_zero_point[idx];
  float motor_ratio = (motor_to_joint_gear_ratio[idx] / reflex_hand2::ReflexHand::DYN_POS_SCALE);
  uint16_t command = static_cast<uint16_t> (zeroed_command * motor_ratio);
  if (command > reflex_hand2::ReflexHand::DYN_MIN_RAW_WRAPPED) {
    printf("Finger %d set out of range (%d), reset to %d", motor_idx + 1,
             static_cast<uint16_t>(command), reflex_hand2::ReflexHand::DYN_MIN_RAW);
    command = reflex_hand2::ReflexHand::DYN_MIN_RAW;
  }
  return command;
}


// Takes a rad/s command and returns Dynamixel command
uint16_t ReflexDriver::speed_rad_to_raw(float rad_per_s_command, int motor_idx) {
  size_t idx = static_cast<size_t>(motor_idx);
  uint16_t command = static_cast<uint16_t>(std::abs(rad_per_s_command) *
                     (motor_to_joint_gear_ratio[idx] / reflex_hand2::ReflexHand::DYN_VEL_SCALE));
  if (command > 1023) {
    command = 1023;
  }
  if (motor_to_joint_inverted[idx] * rad_per_s_command < 0) {
    command += 1024;
  }
  if (command == 0) {
    command = 1;  // 0 doesn't actually stop the motor
  }
  return command;
}

// Takes a Dynamixel speed command and return rad/s
float ReflexDriver::speed_raw_to_rad(uint16_t raw_per_s_command, int motor_idx) {
  size_t idx = static_cast<size_t>(motor_idx);
  float rad_per_s_command = raw_per_s_command * reflex_hand2::ReflexHand::DYN_VEL_SCALE /
                     motor_to_joint_gear_ratio[idx] ;
  return rad_per_s_command * motor_to_joint_inverted[idx];
}



// Takes the load, converts it to a float, then does a rolling filter
float ReflexDriver::load_raw_to_signed(int load, int motor_idx) {
  size_t idx = static_cast<size_t>(motor_idx);
  if (load > 1023) {
    load = (load - 1023);
  } else {
    load = -1 * load;
  }
  float signed_load = static_cast<float> (motor_to_joint_inverted[idx] * load);
  float load_filter = 0.25;  // Rolling filter of noisy data
  float filter_load = load_filter * signed_load + (1 - load_filter) * load_last_value[motor_idx];
  load_last_value[motor_idx] = filter_load;
  return filter_load;
}


// Checks whether the motor positions will reset after mode switch, and corrects zero point
// When switching modes (VELOCITY and POSITION) the motor will wrap values if above 14024 or below 13000
void ReflexDriver::check_for_potential_motor_wraps_and_rezero() {
  // Since motor_wrap is not used, it produces a warning during build.
  // That's why the code below is commented. An issue was posted here
  // https://github.com/RightHandRobotics/reflex-api/issues/1
  // and here
  // https://github.com/RightHandRobotics/reflex-ros-pkg/issues/44
  //float motor_wrap;
  //for (size_t i = 0; i < NUM_SERVOS; i++) {
  //  motor_wrap = 1025.f * (reflex_hand2::ReflexHand::DYN_POS_SCALE / motor_to_joint_gear_ratio[i]);
  //}
}


// Takes raw Dynamixel values (0-4095) and writes them directly to the motors
// NOTE: The Dynamixels have a resolution divider of 4.
void ReflexDriver::set_raw_position(const uint16_t *targets) {
  rh->setServoControlModes(reflex_hand2::ReflexHand::CM_POSITION);
  for (int i = 0; i < NUM_SERVOS; i++) {
    raw_cmd_last_value[i] = targets[i];
  }
  rh->setServoTargets(targets);
}


// Steps the fingers in by a calibration amount
void ReflexDriver::move_fingers_in(const reflex_hand2::ReflexHandState * const state) {
  rh->setServoControlModes(reflex_hand2::ReflexHand::CM_POSITION);
  uint16_t targets[NUM_SERVOS];
  int motor_step;
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    motor_step = motor_to_joint_inverted[i] * calibration_dyn_increase[i];
    targets[i] = static_cast<uint16_t>(state->dynamixel_angles_[i] + motor_step);
  }
  set_raw_position(targets);
}


// Commands the motors from radians, using the zero references from
// yaml/finger_calibrate.yaml to translate into the raw Dynamixel values
void ReflexDriver::set_angle_position(const float *targets) {
  rh->setServoControlModes(reflex_hand2::ReflexHand::CM_POSITION);
  float adjusted_targets[NUM_SERVOS];
  uint16_t raw_targets[NUM_SERVOS];
  bool finger1_arrived = false, finger2_arrived = false, finger3_arrived = false, preshape_arrived = false;

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (targets[i] < 0) {
      adjusted_targets[i] = 0.0;
      printf("Position for finger %d was set to negative, changing to 0\n", i+1);
    } else {
      adjusted_targets[i] = targets[i];
    }
    raw_targets[i] = pos_rad_to_raw(adjusted_targets[i], i);
    raw_cmd_last_value[i] = raw_targets[i];
  }

  while (!(finger1_arrived && finger2_arrived && finger3_arrived && preshape_arrived)) {
    listen(2);

    rh->setServoTargets(raw_targets);

    for (size_t i = 0; i < NUM_SERVOS; i++) {
      if (hand_info.load[i] > MAX_LOAD) {
        printf("Finger %ld overloaded at %f, loosening\n", i+1, hand_info.load[i]);
        adjusted_targets[i] = hand_info.joint_angle[i] - 0.1f;
        raw_targets[i] = pos_rad_to_raw(adjusted_targets[i], static_cast<int>(i));
        raw_cmd_last_value[i] = raw_targets[i];
      }
    }

    finger1_arrived = approx_equal(hand_info.joint_angle[0], adjusted_targets[0], CALIBRATION_ERROR);
    finger2_arrived = approx_equal(hand_info.joint_angle[1], adjusted_targets[1], CALIBRATION_ERROR);
    finger3_arrived = approx_equal(hand_info.joint_angle[2], adjusted_targets[2], CALIBRATION_ERROR);
    preshape_arrived = approx_equal(hand_info.joint_angle[3], adjusted_targets[3], CALIBRATION_ERROR);
  }
}


// Moves the fingers in until any of them makes contact with an object
// Velocities are in rad/s
void ReflexDriver::move_until_any_contact(const float *velocities) {
  rh->setServoControlModes(reflex_hand2::ReflexHand::CM_POSITION);
  uint16_t raw_targets[NUM_SERVOS];
  float adjusted_velocities[NUM_FINGERS];
  for (int i = 0; i < NUM_FINGERS; i++) {
    raw_targets[i] = pos_rad_to_raw(hand_info.joint_angle[i], i);
    adjusted_velocities[i] = velocities[i];
  }
  raw_targets[NUM_SERVOS - 1] = pos_rad_to_raw(hand_info.joint_angle[NUM_SERVOS - 1], NUM_SERVOS - 1);
  float next_target;
  bool all_zero;
  bool any_touching = any_finger_touching();

  while(!any_touching) {
    listen(2);

    rh->setServoTargets(raw_targets);

    for (int i = 0; i < NUM_FINGERS; i++) {
      all_zero = true;
      for (int j = 0; j < NUM_FINGERS; j++) {
        all_zero = (std::fabs(adjusted_velocities[j]) < std::numeric_limits<float>::epsilon()) && all_zero;
      }
      if (all_zero) {
        return;
      }
      if (std::fabs(adjusted_velocities[i]) < std::numeric_limits<float>::epsilon()) {
        continue;
      }
      next_target = hand_info.joint_angle[i] + adjusted_velocities[i] * MOTOR_INCREMENT;
      raw_targets[i] = pos_rad_to_raw(next_target, i);
      if (next_target < MIN_ANGLE) {
        adjusted_velocities[i] = 0.0;
      }
    }

    any_touching = any_finger_touching();
  }
}


// Moves the fingers in until each finger makes contact with an object
// Velocities are in rad/s
void ReflexDriver::move_until_each_contact(const float *velocities) {
  rh->setServoControlModes(reflex_hand2::ReflexHand::CM_POSITION);
  uint16_t raw_targets[NUM_SERVOS];
  float next_targets[NUM_FINGERS];
  float modified_velocities[NUM_FINGERS];
  for (int i = 0; i < NUM_FINGERS; i++) {
    raw_targets[i] = pos_rad_to_raw(hand_info.joint_angle[i], i);
    modified_velocities[i] = velocities[i];
  }
  raw_targets[NUM_SERVOS - 1] = pos_rad_to_raw(hand_info.joint_angle[NUM_SERVOS - 1], NUM_SERVOS - 1);

  bool *touching_array = new bool[NUM_FINGERS];
  bool all_touching = false;
  bool all_zero;

  while(!all_touching) {
    listen(2);

    rh->setServoTargets(raw_targets);
    touching_array = each_finger_touching(touching_array);
    for (int i = 0; i < NUM_FINGERS; i++) {
      all_zero = true;
      for (int j = 0; j < NUM_FINGERS; j++) {
        all_zero = (std::fabs(modified_velocities[j]) < std::numeric_limits<float>::epsilon()) && all_zero;
      }
      if (all_zero) {
        break;
      }
      if (std::fabs(modified_velocities[i]) < std::numeric_limits<float>::epsilon() || touching_array[i] == true) {
        continue;
      }
      next_targets[i] = hand_info.joint_angle[i] + modified_velocities[i] * MOTOR_INCREMENT;
      raw_targets[i] = pos_rad_to_raw(next_targets[i], i);
    }

    all_touching = true;
    for (int i = 0; i < NUM_FINGERS; i++) {
//      all_touching = all_touching && (!modified_velocities[i] || touching_array[i]);
      all_touching = all_touching && ((std::fabs(modified_velocities[i]) < std::numeric_limits<float>::epsilon()) || touching_array[i]);
      if (touching_array[i] || next_targets[i] < MIN_ANGLE) {
        modified_velocities[i] = 0;
      }
    }
  }

  delete [] touching_array;
}


// Changes the travel speed of the motor
void ReflexDriver::set_motor_speed(const float *targets) {
  rh->setServoControlModes(reflex_hand2::ReflexHand::CM_VELOCITY);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  uint16_t raw_targets[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    raw_targets[i] = speed_rad_to_raw(targets[i], i);
  }
  rh->setServoTargets(raw_targets);
  std::this_thread::sleep_for(std::chrono::milliseconds(35));
  rh->setServoControlModes(reflex_hand2::ReflexHand::CM_POSITION);
  check_for_potential_motor_wraps_and_rezero();
  std::this_thread::sleep_for(std::chrono::milliseconds(35));
}


// Puts the dynamixels into idle mode
void ReflexDriver::disable_torque() {
  rh->setServoControlModes(reflex_hand2::ReflexHand::CM_IDLE);
}


// Sets the procedure to calibrate the tactile values in motion
void ReflexDriver::calibrate_tactile() {
  acquire_tactile = true;
}


// Sets the procedure to calibrate the fingers in motion
void ReflexDriver::calibrate_fingers() {
  acquire_fingers = true;
  first_capture = true;
  all_fingers_moved = false;
}


// Calibrates both the tactile sensors and fingers
void ReflexDriver::calibrate() {
  calibrate_tactile();
  calibrate_fingers();

  while(!calibrated) {
    listen(2);
  }
}


// Sets the tactile threshold levels to be all one value
void ReflexDriver::populate_tactile_thresholds(int threshold) {
for (int i = 0; i < NUM_FINGERS; i++) {
    for (int j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
      contact_thresholds[i][j] = threshold;
    }
  }
}


// Sets the threshold levels on tactile sensors for reporting contact
void ReflexDriver::set_tactile_thresholds(const int *thresholds) {
//  for (int i = 0; i < NUM_SENSORS_PER_FINGER; i++) {
//    for (int j= 0; j < NUM_FINGERS; j++) {
//      contact_thresholds[j][i] = thresholds[NUM_FINGERS*i + j];
//    }
//  }
  for (int i = 0; i < NUM_FINGERS; i++) {
    for (int j= 0; j < NUM_SENSORS_PER_FINGER; j++) {
      contact_thresholds[i][j] = thresholds[NUM_SENSORS_PER_FINGER*i + j];
    }
  }
}


// Returns the correct pressure calibration offset for finger[sensor]
int ReflexDriver::pressure_offset(int finger, int sensor) {
  return tactile_offset_values[static_cast<size_t>(finger)][static_cast<size_t>(sensor)];
}


// Given raw value and a past value, tracks encoder wraps: enc_offset variable
int ReflexDriver::update_encoder_offset(int raw_value, int last_value, int current_offset) {
  int enc_offset = current_offset;
  if (enc_offset == -1) {
    // This case happens upon startup
    enc_offset = 0;
  } else {
    // If the encoder value jumps, that means it has wrapped a revolution
    if (last_value - raw_value > 5000)
      enc_offset = enc_offset + 16383;
    else if (last_value - raw_value < -5000)
      enc_offset = enc_offset - 16383;
  }
  return enc_offset;
}


// Returns whether any finger is making contact
bool ReflexDriver::any_finger_touching() {
  for (int i = 0; i < NUM_FINGERS; i++) {
    for (int j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
      if (hand_info.contact[i][j]) {
        return true;
      }
    }
  }
  return false;
}


// Returns an array containing whether each finger is making contact
bool* ReflexDriver::each_finger_touching(bool *touching) {
  for (int i = 0; i < NUM_FINGERS; i++) {
    for (int j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
      touching[i] = hand_info.contact[i][j];
      if (touching[i]) {
        break;
      }
    }
  }
  return touching;
}


// Calculates actual proximal angle using raw sensor value, wrap offset
// for that finger, and calibrated "zero" point for that encoder
float ReflexDriver::calc_proximal_angle(int raw_enc_value, int offset, float zero) {
  int wrapped_enc_value = raw_enc_value + offset;
  float rad_value = wrapped_enc_value * reflex_hand2::ReflexHand::ENC_SCALE;
  return (zero - rad_value);
}


// Calculates joint angle from raw sensor value, motor gear ratio,
// and calibrated "zero" point for the joint
float ReflexDriver::calc_motor_angle(int inversion, int raw_dyn_value, float ratio, float zero) {
  float rad_value = raw_dyn_value * reflex_hand2::ReflexHand::DYN_POS_SCALE / ratio;
  float zeroed_value = rad_value - zero;
  return inversion * zeroed_value;
}


// Calculates distal angle, "tendon spooled out" - "proximal encoder" angles
// Could be improved
float ReflexDriver::calc_distal_angle(float joint_angle, float proximal) {
  float diff = joint_angle - proximal;
  return (diff < 0) ? 0 : diff;
}


// Takes hand state and returns calibrated tactile data for given finger and sensor
int ReflexDriver::calc_pressure(const reflex_hand2::ReflexHandState * const state, int finger, int sensor) {
  int raw_value = state->tactile_pressures_[TACTILE_BASE_IDX[finger] + sensor];
  return raw_value - pressure_offset(finger, sensor);
}


// Checks given finger/sensor for contact threshold
int ReflexDriver::calc_contact(const reflex_hand2::ReflexHandState * const state, int finger, int sensor) {
  int pressure_value = calc_pressure(state, finger, sensor);
  return pressure_value > contact_thresholds[finger][sensor];
}


// Opens tactile calibration data file, changes local tactile_offset, and saves
// current tactile values to file as the new calibrated "zero"
void ReflexDriver::calibrate_tactile_sensors(const reflex_hand2::ReflexHandState * const state) {
  tactile_file.open(tactile_file_address.c_str(), std::ios::out|std::ios::trunc);
  tactile_file << "# Captured sensor values from unloaded state\n";
  log_current_tactile_locally(state);
  log_current_tactile_to_file(state);

  tactile_file.close();

  acquire_tactile = false;
}


// Captures current encoder position locally as "zero" and save to file
void ReflexDriver::calibrate_encoders_locally(const reflex_hand2::ReflexHandState * const state) {
  for (size_t i = 0; i < NUM_FINGERS; i++) {
    encoder_zero_point[i] = state->encoders_[i] * reflex_hand2::ReflexHand::ENC_SCALE;
    encoder_offset[i] = 0;
  }
}


// Saves current dynamixel location (plus an offset) as "zero" and then
// writes the dynamixels to the spot
void ReflexDriver::calibrate_motors_locally(const reflex_hand2::ReflexHandState * const state) {
  uint16_t targets[NUM_SERVOS];
  int motor_offset;
  float motor_scalar;

  for (size_t i = 0; i < NUM_SERVOS; i++) {
    motor_offset = motor_to_joint_inverted[i] * CALIBRATION_DYN_OFFSET[i];
    motor_scalar = reflex_hand2::ReflexHand::DYN_POS_SCALE / motor_to_joint_gear_ratio[i];
    dynamixel_zero_point[i] = (state->dynamixel_angles_[i] - motor_offset) * motor_scalar;
    targets[i] = static_cast<uint16_t>(state->dynamixel_angles_[i] - motor_offset);
  }
  set_raw_position(targets);
}


// Saves local variables tactile_offset_values with current tactile position
void ReflexDriver::log_current_tactile_locally(const reflex_hand2::ReflexHandState * const state) {
  for (size_t i = 0; i < NUM_FINGERS; i++) {
    for (size_t j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
      tactile_offset_values[i][j] = state->tactile_pressures_[static_cast<size_t>(TACTILE_BASE_IDX[i]) + j];
    }
  }
}


// Writes the current tactile data to the data file
void ReflexDriver::log_current_tactile_to_file(const reflex_hand2::ReflexHandState * const state) {
  for (int i = 0; i < NUM_FINGERS; i++) {
    tactile_file << "tactile_offset_f" << i + 1 << ": [";

    for (int j = 0; j < NUM_SENSORS_PER_FINGER - 1; j++) {
      tactile_file << state->tactile_pressures_[TACTILE_BASE_IDX[i] + j] << ", ";
    }
    tactile_file << state->tactile_pressures_[TACTILE_BASE_IDX[i] + NUM_SENSORS_PER_FINGER - 1] << "]\n";
  }
}


// Opens data file and writes calibrated encoder zeros to it
void ReflexDriver::log_encoder_zero_to_file() {
  finger_file.open(finger_file_address.c_str(), std::ios::out|std::ios::trunc);
  finger_file << "# Calibration constants for [f1, f2, f3, preshape]\n";
  finger_file << "encoder_zero_reference: ["
              << encoder_zero_point[0] << ", "
              << encoder_zero_point[1] << ", "
              << encoder_zero_point[2] << "]\n";
}


// Writes calibrated dynamixel zeros to the data file and closes it
void ReflexDriver::log_motor_zero_to_file_and_close() {
  finger_file << "motor_zero_reference: ["
              << dynamixel_zero_point[0] << ", "
              << dynamixel_zero_point[1] << ", "
              << dynamixel_zero_point[2] << ", "
              << dynamixel_zero_point[3] << "]\n";
  finger_file.close();
}


// Checks whether all fingers have moved more than CALIBRATION_ERROR, halts
// the motion of fingers past that point
bool ReflexDriver::check_for_finger_movement(const reflex_hand2::ReflexHandState * const state) {
  bool all_fingers_moved = true;
  for (size_t i = 0; i < NUM_FINGERS; i++) {
    float enc_pos = encoder_zero_point[i] -
                    state->encoders_[i] * reflex_hand2::ReflexHand::ENC_SCALE;
    if (std::abs(enc_pos) > CALIBRATION_ERROR) {
      calibration_dyn_increase[i] = 0;
    } else {
      calibration_dyn_increase[i] = 6;
      all_fingers_moved = false;
    }
  }
  return all_fingers_moved;
}


// Captures the current hand state
void ReflexDriver::populate_motor_state(const reflex_hand2::ReflexHandState * const state) {
  char buffer[10];
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    hand_info.raw_angle[i] = state->dynamixel_angles_[i];
    //hand_info.velocity[i] = (float) state->dynamixel_speeds_[i];
    hand_info.velocity[i] = speed_raw_to_rad(state->dynamixel_speeds_[i], static_cast<int>(i));
    hand_info.load[i] = load_raw_to_signed(state->dynamixel_loads_[i], static_cast<int>(i));
    hand_info.voltage[i] = static_cast<float>(state->dynamixel_voltages_[i]);
    hand_info.temperature[i] = state->dynamixel_temperatures_[i];
    sprintf(buffer, "0x%02x", state->dynamixel_error_states_[i]);
    hand_info.error_state[i] = buffer;
  }
}


// Takes in hand state data and writes to hand_info
// Also does calibration when certain booleans are enabled
void ReflexDriver::reflex_hand_state_cb(const reflex_hand2::ReflexHandState * const state) {
  for (size_t i = 0; i < NUM_FINGERS; i++) {
    int i_ = static_cast<int>(i);
    encoder_offset[i] = update_encoder_offset(state->encoders_[i],
                                              encoder_last_value[i],
                                              encoder_offset[i]);
    encoder_last_value[i] = state->encoders_[i];
    hand_info.joint_angle[i] = calc_motor_angle(motor_to_joint_inverted[i],
                                                     state->dynamixel_angles_[i],
                                                     motor_to_joint_gear_ratio[i],
                                                     dynamixel_zero_point[i]);
    hand_info.proximal[i] = calc_proximal_angle(state->encoders_[i],
                                                      encoder_offset[i],
                                                      encoder_zero_point[i]);
    hand_info.distal_approx[i] = calc_distal_angle(hand_info.joint_angle[i],
                                                   hand_info.proximal[i]);
    for (size_t j = 0; j < 5; j++) {
      int j_ = static_cast<int>(j);
      hand_info.pressure[i][j] = calc_pressure(state, i_, j_);
      hand_info.contact[i][j] = calc_contact(state, i_, j_);
    }
    hand_info.pressure[i][5] = calc_pressure(state, i_, 10);
    hand_info.contact[i][5] = calc_contact(state, i_, 5);
    hand_info.pressure[i][6] = calc_pressure(state, i_, 11);
    hand_info.contact[i][6] = calc_contact(state, i_, 6);
    hand_info.pressure[i][7] = calc_pressure(state, i_, 7);
    hand_info.contact[i][7] = calc_contact(state, i_, 7);
    hand_info.pressure[i][8] = calc_pressure(state, i_, 8);
    hand_info.contact[i][8] = calc_contact(state, i_, 8);
    hand_info.pressure[i][9] = calc_pressure(state, i_, 9);
    hand_info.contact[i][9] = calc_contact(state, i_, 9);
    hand_info.pressure[i][10] = calc_pressure(state, i_, 13);
    hand_info.contact[i][10] = calc_contact(state, i_, 10);
    hand_info.pressure[i][11] = calc_pressure(state, i_, 12);
    hand_info.contact[i][11] = calc_contact(state, i_, 11);
    hand_info.pressure[i][12] = calc_pressure(state, i_, 5);
    hand_info.contact[i][12] = calc_contact(state, i_, 12);
    hand_info.pressure[i][13] = calc_pressure(state, i_, 6);
    hand_info.contact[i][13] = calc_contact(state, i_, 13);
  }
  hand_info.joint_angle[NUM_SERVOS - 1] = calc_motor_angle(motor_to_joint_inverted[NUM_SERVOS - 1],
                                                          state->dynamixel_angles_[NUM_SERVOS - 1],
                                                          motor_to_joint_gear_ratio[NUM_SERVOS - 1],
                                                          dynamixel_zero_point[NUM_SERVOS - 1]);
  populate_motor_state(state);

  // Captures the current tactile data and save it as a zero reference
  if (acquire_tactile) {
    calibrate_tactile_sensors(state);
  }

  if (acquire_fingers &&
      (std::chrono::system_clock::now() > std::chrono::system_clock::from_time_t(latest_calibration_time) + std::chrono::milliseconds(50))) {
    if (first_capture) {
      calibrate_encoders_locally(state);
      first_capture = false;
    }
    all_fingers_moved = check_for_finger_movement(state);
    if (all_fingers_moved) {
      acquire_fingers = false;
      printf("FINISHED FINGER CALIBRATION: Encoder movement detected\n");
      calibrate_motors_locally(state);
      log_encoder_zero_to_file();
      log_motor_zero_to_file_and_close();
    } else {
      move_fingers_in(state);
    }
    latest_calibration_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  }
  calibrated = !acquire_tactile && !acquire_fingers;
  return;
}
