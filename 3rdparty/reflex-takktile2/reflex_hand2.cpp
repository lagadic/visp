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

#include <string.h>

#include <reflex_hand2.h>

using namespace reflex_hand2;

std::ostream& operator<<(std::ostream &os, const HandInfo &hand) {
  for (int i = 0; i < NUM_FINGERS; i++) {
    os << "Finger " << i + 1 << ": " << std::endl;

    os << "\tProximal: " << hand.proximal[i] << std::endl;
    os << "\tDistal Approx: " << hand.distal_approx[i] << std::endl;

    os << "\tPressures: ";
    for (int j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
      os << hand.pressure[i][j] << ", ";
    }
    os << std::endl;

    os << "\tContact: ";
    for (int j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
      os << hand.contact[i][j] << ", ";
    }
    os << std::endl;

    os << "\tJoint Angle: " << hand.joint_angle[i] << " rad" << std::endl;
    os << "\tJoint Angle: " << hand.joint_angle[i]*(180.f/3.1415926f) << " deg" << std::endl;
    os << "\tVelocity: " << hand.velocity[i] << " rad/s" << std::endl;
    os << "\tVelocity: " << hand.velocity[i]*(180.f/3.1415926f) << " deg/s" << std::endl;
    os << "\tError State: " << hand.error_state[i] << std::endl;
  }

  os << "Preshape: " << std::endl;
  os << "\tJoint Angle: " << hand.joint_angle[3] << std::endl;
  os << "\tVelocity: " << hand.velocity[3] << std::endl;
  os << "\tError State: " << hand.error_state[3] << std::endl;
  return os;
}

HandInfo::HandInfo() {
  for (int i = 0; i < NUM_FINGERS; i++) {
    proximal[i] = 0;
    distal_approx[i] = 0;
    for (int j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
      pressure[i][j] = 0;
      contact[i][j] = false;
    }
  }
  for (int i = 0; i < NUM_DYNAMIXELS; i++) {
    joint_angle[i] = 0;
    raw_angle[i] = 0;
    velocity[i] = 0;
    load[i] = 0;
    voltage[i] = 0;
    temperature[i] = 0;
    error_state[i] = "";
  }
}

HandInfo::~HandInfo() {}


ReflexHandState::ReflexHandState() {
  systime_us_ = 0;
  for (int i = 0; i < NUM_TAKKTILE; i++) {
    tactile_pressures_[i] = tactile_temperatures_[i] = 0;
  }
  for (int i = 0; i < NUM_FINGERS; i++) {
    encoders_[i] = 0;
  }
  for (int i = 0; i < NUM_DYNAMIXELS; i++) {
    dynamixel_error_states_[i] = 0;
    dynamixel_angles_[i] = 0;
    dynamixel_speeds_[i] = 0;
    dynamixel_loads_[i] = 0;
    dynamixel_voltages_[i] = 0;
    dynamixel_temperatures_[i] = 0;
  }
}


ReflexHandState::~ReflexHandState() {}


int tx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
int rx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
sockaddr_in mcast_addr_;
ReflexHandState rx_state_;

ReflexHand::ReflexHand(): happy_(false) {
}

ReflexHand::ReflexHand(const std::string &network_interface): happy_(true) {
  setupNetwork(network_interface);
}


ReflexHand::~ReflexHand() {}


bool ReflexHand::listen(const double max_seconds) {
  static uint8_t rxbuf[2000] = {0};
  fd_set rdset;
  FD_ZERO(&rdset);
  FD_SET(rx_sock_, &rdset);
  timeval timeout;
  timeout.tv_sec = static_cast<time_t>(max_seconds);
  timeout.tv_usec = static_cast<suseconds_t>((max_seconds - timeout.tv_sec) * 1e6);
  int rv = select(rx_sock_ + 1, &rdset, NULL, NULL, &timeout);
  // printf("File: %d\n", rv); // check or uncheck for debugging
  if (rv > 0 && FD_ISSET(rx_sock_, &rdset)) {
    int nbytes = recvfrom(rx_sock_, rxbuf, sizeof(rxbuf), 0, NULL, NULL);
    // printf("Received: %d bytes\n", nbytes); // check or uncheck for debugging
    rx(rxbuf, nbytes);
    usleep(100);
  } else {
    printf("Can't communicate with the hand.\n");
  }

  return true;
}


void ReflexHand::tx(const uint8_t *msg, const uint16_t msg_len, const uint16_t port) {
  mcast_addr_.sin_port = htons(port);
  int nsent = sendto(tx_sock_, msg, msg_len, 0, (sockaddr *)&mcast_addr_, sizeof(mcast_addr_));
  if (nsent < 0) {
    printf("woah. sendto() returned %d\n", nsent);
    exit(1);
  }
}


void ReflexHand::setServoTargets(const uint16_t *targets) {
  uint8_t msg[1 + 2*NUM_SERVOS];
  msg[0] = CP_SET_SERVO_TARGET;
  for (int i = 0; i < NUM_SERVOS; i++) { // rearrange from uint16_t to uint8_t
    msg[1 + 2*i] = (targets[i] >> 8) & 0xff;  // take second byte
    msg[2 + 2*i] = targets[i] & 0xff;         // take first byte
  }
  tx(msg, sizeof(msg), PORT_BASE);
}


void ReflexHand::setServoControlModes(const ControlMode *modes) {
  uint8_t msg[NUM_SERVOS+1];
  msg[0] = CP_SET_SERVO_MODE;
  for (int i = 0; i < NUM_SERVOS; i++)
    msg[i+1] = static_cast<uint8_t>(modes[i]);
  tx(msg, sizeof(msg), PORT_BASE);
}


void ReflexHand::setServoControlModes(const ControlMode mode) {
  const ControlMode modes[4] = { mode, mode, mode, mode };
  setServoControlModes(modes);
}


void ReflexHand::rx(const uint8_t *msg, const uint16_t msg_len) {
  // first, check the packet format "magic byte" and the length
  if (msg[0] != 1) {
    printf("unexpected hand shake byte received on UDP multicast port: 0x%02x.\n", msg[0]);
    return;
  }
  if (msg_len != sizeof(mcu_state_format_1_t)) { // The leftover palm data adds 44 bytes
    printf("expected packet length %d, but saw %d instead.\n", (int)sizeof(mcu_state_format_1_t), msg_len - 44);
    return;
  }

  mcu_state_format_1_t *rx_state_msg = (mcu_state_format_1_t *)msg;
  rx_state_.systime_us_ = rx_state_msg->systime;

  for (int i = 0; i < NUM_FINGERS; i++)
    rx_state_.encoders_[i] = rx_state_msg->encoders[i];
  for (int i = 0; i < NUM_TAKKTILE; i++) {
    rx_state_.tactile_pressures_[i]    = rx_state_msg->tactile_pressures[i];
    rx_state_.tactile_temperatures_[i] = rx_state_msg->tactile_temperatures[i];
  }
  for (int i = 0; i < NUM_DYNAMIXELS; i++) {
    rx_state_.dynamixel_error_states_[i] = rx_state_msg->dynamixel_error_states[i];
    rx_state_.dynamixel_angles_[i]       = rx_state_msg->dynamixel_angles[i];
    rx_state_.dynamixel_speeds_[i]       = rx_state_msg->dynamixel_speeds[i];
    rx_state_.dynamixel_loads_[i]        = rx_state_msg->dynamixel_loads[i];
    rx_state_.dynamixel_voltages_[i]     = rx_state_msg->dynamixel_voltages[i];
    rx_state_.dynamixel_temperatures_[i] = rx_state_msg->dynamixel_temperatures[i];
  }

  // now that we have stuff the rx_state_ struct, fire off our callback
  if (state_cb_) {
    state_cb_(&rx_state_);
  }
}


int ReflexHand::setupNetwork(const std::string &network_interface) {
  printf("Starting network interface %s.\n", network_interface.c_str());
  const char *mcast_addr_str = "224.0.0.124"; // parameterize someday !
  tx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  rx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if(tx_sock_ < 0 || rx_sock_ < 0) {
    printf("Couldn't create socket.\n");
    return -1;
  }
  memset(&mcast_addr_, 0, sizeof(mcast_addr_));
  mcast_addr_.sin_family = AF_INET;
  mcast_addr_.sin_addr.s_addr = inet_addr(mcast_addr_str);

  ifaddrs *ifaddr;
  if (getifaddrs(&ifaddr) == -1) {
    printf("couldn't get ipv4 address of interface %s\n", network_interface.c_str());
    return -1;
  }
  std::string tx_iface_addr;
  bool found_interface = false;
  for (ifaddrs *ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr)
      continue;
    int family = ifa->ifa_addr->sa_family;
    if (family != AF_INET)
      continue;
    char host[NI_MAXHOST];
    if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST))
      continue;
    printf("    found address %s on interface %s\n", host, ifa->ifa_name);
    if (std::string(ifa->ifa_name) == network_interface) {
      printf("using %s as the tx interface for IPv4 UDP multicast\n", host);
      tx_iface_addr = host;
      found_interface = true;
      break;
    }
  }
  freeifaddrs(ifaddr);
  int err = errno;
  if (!found_interface) {
    printf("Unable to find IPv4 address of interface %s %s. Perhaps it needs to be set to a static address?\n", strerror(err), network_interface.c_str());
    return -1;
  }

  in_addr local_addr;
  local_addr.s_addr = inet_addr(tx_iface_addr.c_str());
  int result = 0, loopback = 0;
  result = setsockopt(tx_sock_, IPPROTO_IP, IP_MULTICAST_IF, (char *)&local_addr, sizeof(local_addr));

  err = errno;
  if(result < 0) {
      printf("couldn't set local interface for udp tx sock, fails with errno: %s\n", strerror(err));
  }
  result = setsockopt(tx_sock_, IPPROTO_IP, IP_MULTICAST_LOOP, &loopback, sizeof(loopback));
  err = errno;
  if(result < 0) {
      printf("couldn't turn off outgoing multicast loopback, fails with errno: %s\n", strerror(err));
  }

  /////////////////////////////////////////////////////////////////////
  // set up the rx side of things
  int reuseaddr = 1;
  result = setsockopt(rx_sock_, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr));
  err = errno;
  if(result < 0) {
    printf("couldn't set SO_REUSEADDR on UDP RX socket, fails with errno: %s\n", strerror(err));
  }

  sockaddr_in rx_bind_addr;
  memset(&rx_bind_addr, 0, sizeof(rx_bind_addr));
  rx_bind_addr.sin_family = AF_INET;
  rx_bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  rx_bind_addr.sin_port = htons(PORT_BASE);
  result = bind(rx_sock_, (sockaddr *)&rx_bind_addr, sizeof(rx_bind_addr));
  err = errno;
  if(result < 0) {
    printf("couldn't bind rx socket to port %d, fails with errno: %s\n", PORT_BASE, strerror(err));
  }

  ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(mcast_addr_str);
  mreq.imr_interface.s_addr = inet_addr(tx_iface_addr.c_str());
  result = setsockopt(rx_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
  err = errno;
  if(result < 0) {
    printf("couldn't add to multicast group, fails with errno: %s\n", strerror(err));
  }
  printf("connected!\n\n");

  happy_ = true;

  return 0;
}


void ReflexHand::setStateCallback(StateCallback callback) {
  state_cb_ = callback;
}
