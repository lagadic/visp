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
 * MegaPose wrapper.
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_THREADS)

#include <visp3/dnn_tracker/vpMegaPose.h>

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#include <io.h>
#include <winsock2.h>
#include <ws2tcpip.h> // for inet_pton()
#endif
#include <stdexcept>
#include <mutex>
#include <thread>
using json = nlohmann::json; //! json namespace shortcut

BEGIN_VISP_NAMESPACE

//// Network message utils

/*Encode elements to a buffer of bytes*/

/*End of template recursion*/
void encode(std::vector<uint8_t> &)
{

}

/*
Append the byte representation of an object to the byte buffer.
By default a generic object cannot be encoded.
*/
template<typename T>
void encode(std::vector<uint8_t> &buffer, const T &object) = delete;


/*Single object specializations*/
template<>
void encode(std::vector<uint8_t> &buffer, const int &object)
{
  const uint32_t v = htonl(object);
  const uint8_t *varr = (uint8_t *)&v;
  buffer.insert(buffer.end(), varr, varr + 4);
}

template<>
void encode(std::vector<uint8_t> &buffer, const float &object)
{
  assert((sizeof(uint32_t) == sizeof(float)));
  const uint32_t *pointer = reinterpret_cast<const uint32_t *>(&object);
  const uint32_t v = htonl(*pointer);
  const uint8_t *varr = (uint8_t *)&v;
  buffer.insert(buffer.end(), varr, varr + 4);
}

template<>
void encode(std::vector<uint8_t> &buffer, const std::string &object)
{
  const int size = static_cast<int>(object.size());
  encode(buffer, size);
  const uint8_t *chars = (uint8_t *)&object[0];
  buffer.insert(buffer.end(), chars, chars + size);
}

template<typename T>
void encode(std::vector<uint8_t> &buffer, const std::vector<T> &object)
{
  const int size = static_cast<int>(object.size());
  encode(buffer, size);
  for (const T &value : object) {
    encode(buffer, value);
  }
}

/*Multiple arguments are processed one by one*/
template<typename T, typename ...Rest>
void encode(std::vector<uint8_t> &buffer, const T &object, const Rest& ...rest)
{
  encode(buffer, object);
  encode(buffer, rest...);
}

template<>
void encode(std::vector<uint8_t> &buffer, const vpImage<vpRGBa> &object)
{
  const int height = object.getHeight(), width = object.getWidth();
  encode(buffer, height, width, 4);
  const uint32_t sentSize = height * width * 4;

  buffer.reserve(buffer.size() + sentSize); // Avoid resizing multiple times as we iterate on pixels
  const uint8_t *const bitmap = (uint8_t *)object.bitmap;
  buffer.insert(buffer.end(), bitmap, bitmap + sentSize);
  //std::copy(bitmap, bitmap + sentSize, buffer.end());
}

template<>
void encode(std::vector<uint8_t> &buffer, const vpImage<uint16_t> &object)
{
  const int height = object.getHeight(), width = object.getWidth();
  encode(buffer, height, width);
  //test endianness
  const uint16_t hostTest = 1;
  const uint16_t netTest = htons(hostTest); // network is big endian
  const uint8_t endianness = hostTest == netTest ? '>' : '<';
  const uint32_t sentSize = height * width * 2;

  buffer.reserve(buffer.size() + sentSize + 1);
  buffer.push_back(endianness);
  const uint8_t *const bitmap = (uint8_t *)object.bitmap;
  buffer.insert(buffer.end(), bitmap, bitmap + sentSize);
}

template<>
void encode(std::vector<uint8_t> &buffer, const vpCameraParameters &object)
{
  encode(buffer, (float)object.get_px(), (float)object.get_py(),
    (float)object.get_u0(), (float)object.get_v0());
}

template<>
void encode(std::vector<uint8_t> &buffer, const vpHomogeneousMatrix &object)
{
  std::vector<float> array;
  array.reserve(16);
  const double *const data = object.data;
  for (unsigned i = 0; i < 16; ++i) {
    array.push_back((float)data[i]);
  }
  encode(buffer, array);
}

/*Decode elements (passed as references), given a buffer of bytes and an index (modified)*/

void decode(const std::vector<uint8_t> &, unsigned int &)
{ }

/*
  Modify an object, given a byte array and an index reading into the byte array.
  The byte array is not modified. But the index should be modified once the object is read.
  After calling this function, the index should indicate the position of the next object to be read.

  There is no default decoding behaviour. As such, specializations must be written.
*/
template<typename T>
void decode(const std::vector<uint8_t> &buffer, unsigned int &index, T &t) = delete;

template<>
void decode(const std::vector<uint8_t> &buffer, unsigned int &index, int &value)
{
  const uint8_t *ptr = &buffer[index];
  value = ntohl(*((uint32_t *)ptr)); // Convert from network (big endian) representation to this machine's representation.
  index += sizeof(int);
}
template<>
void decode(const std::vector<uint8_t> &buffer, unsigned int &index, float &value)
{
  const uint8_t *ptr = &buffer[index];
  const uint32_t v = ntohl(*((uint32_t *)ptr));
  memcpy(&value, &v, sizeof(uint32_t));
  index += sizeof(float);
}
template<>
void decode(const std::vector<uint8_t> &buffer, unsigned int &index, std::string &value)
{
  int size;
  decode(buffer, index, size);
  value.resize(size);
  value.replace(0, size, (char *)&buffer[index], size);
  index += size;
}

template<typename T>
void decode(const std::vector<uint8_t> &buffer, unsigned int &index, std::vector<T> &value)
{
  int size;
  decode(buffer, index, size);
  value.resize(size);
  for (int i = 0; i < size; ++i) {
    T t;
    decode(buffer, index, t);
    value[i] = t;
  }
}

template<>
void decode(const std::vector<uint8_t> &buffer, unsigned int &index, vpHomogeneousMatrix &value)
{
  std::vector<float> values;
  decode(buffer, index, values);
  assert(values.size() == 16);
  for (int i = 0; i < 16; ++i) {
    value.data[i] = values[i];
  }
}

/*
  Decode multiple objects from a byte array.
  These objects can have different types. They are read from the buffer in the order that they are given to the function.
*/
template<typename T, typename ...Rest>
void decode(const std::vector<uint8_t> &buffer, unsigned int &index, T &object, Rest& ...rest)
{
  decode(buffer, index, object);
  decode(buffer, index, rest...);
}

template<>
void decode(const std::vector<uint8_t> &buffer, unsigned int &index, vpImage<vpRGBa> &value)
{
  int height, width, channels;
  decode(buffer, index, height, width, channels);
  value.resize(height, width);
  if (channels == 3) {
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        value.bitmap[i * width + j] = vpRGBa(buffer[index], buffer[index + 1], buffer[index + 2], 255);
        index += 3;
      }
    }
  }
  else if (channels == 4) { // Despite having 4 channels, this is faster
    const unsigned copySize = height * width * channels;
    memcpy((uint8_t *)value.bitmap, &buffer[index], copySize);
    index += copySize;
  }
}


#define MEGAPOSE_CODE_SIZE 4
void handleWrongReturnMessage(const vpMegaPose::ServerMessage code, std::vector<uint8_t> &buffer)
{
  if (code != vpMegaPose::ServerMessage::ERR) {
    throw vpException(vpException::fatalError, "MegaPose: got an unexpected message from the server: " + std::to_string(code));
  }
  std::string message;
  unsigned index = 0;
  decode(buffer, index, message);
  throw vpException(vpException::badValue, "Server error : " + message);
}

const std::unordered_map<vpMegaPose::ServerMessage, std::string> vpMegaPose::m_codeMap =
{
  {ServerMessage::ERR, "RERR"},
  {ServerMessage::OK, "OKOK"},
  {ServerMessage::GET_POSE, "GETP"},
  {ServerMessage::RET_POSE, "RETP"},
  {ServerMessage::SET_INTR, "INTR"},
  {ServerMessage::GET_VIZ, "GETV"},
  {ServerMessage::RET_VIZ, "RETV"},
  {ServerMessage::GET_SCORE, "GSCO"},
  {ServerMessage::RET_SCORE, "RSCO"},
  {ServerMessage::SET_SO3_GRID_SIZE, "SO3G"},
  {ServerMessage::GET_LIST_OBJECTS, "GLSO"},
  {ServerMessage::RET_LIST_OBJECTS, "RLSO"},
  {ServerMessage::EXIT, "EXIT"},
};

std::string vpMegaPose::messageToString(const vpMegaPose::ServerMessage messageType)
{
  return m_codeMap.at(messageType);
}

vpMegaPose::ServerMessage vpMegaPose::stringToMessage(const std::string &s)
{
  for (auto it : m_codeMap) {
    if (it.second == s) {
      return it.first;
    }
  }
  return UNKNOWN;
}

void vpMegaPose::makeMessage(const vpMegaPose::ServerMessage messageType, std::vector<uint8_t> &data) const
{
  const uint32_t size = htonl(static_cast<uint32_t>(data.size()));
  const std::string code = messageToString(messageType);
  uint8_t arr[sizeof(size) + MEGAPOSE_CODE_SIZE];
  memcpy(arr, (uint8_t *)&size, sizeof(size));

  memcpy(arr + sizeof(size), (uint8_t *)code.c_str(), MEGAPOSE_CODE_SIZE);

  std::vector<uint8_t> header(arr, arr + sizeof(size) + MEGAPOSE_CODE_SIZE);
  data.insert(data.begin(), header.begin(), header.end());
}

std::pair<vpMegaPose::ServerMessage, std::vector<uint8_t>> vpMegaPose::readMessage() const
{
  uint32_t size;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  size_t readCount = read(m_serverSocket, &size, sizeof(uint32_t));
#else
  size_t readCount = recv(m_serverSocket, reinterpret_cast<char *>(&size), sizeof(uint32_t), 0);
#endif
  if (readCount != sizeof(uint32_t)) {
    throw vpException(vpException::ioError, "MegaPose: Error while reading data from socket");
  }
  size = ntohl(size);

  unsigned char code[MEGAPOSE_CODE_SIZE];
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  readCount = read(m_serverSocket, code, MEGAPOSE_CODE_SIZE);
#else
  readCount = recv(m_serverSocket, reinterpret_cast<char *>(code), MEGAPOSE_CODE_SIZE, 0);
#endif
  if (readCount != MEGAPOSE_CODE_SIZE) {
    throw vpException(vpException::ioError, "MegaPose: Error while reading data from socket");
  }

  std::vector<uint8_t> data;
  data.resize(size);
  unsigned read_size = 4096;
  unsigned read_total = 0;
  while (read_total < size) {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    int actually_read = read(m_serverSocket, &data[read_total], read_size);
#else
    int actually_read = recv(m_serverSocket, reinterpret_cast<char *>(&data[read_total]), read_size, 0);
#endif
    if (actually_read <= 0) {
      throw vpException(vpException::ioError, "MegaPose: Error while reading data from socket");
    }
    read_total += actually_read;
  }
  std::string codeStr(code, code + MEGAPOSE_CODE_SIZE);
  vpMegaPose::ServerMessage c = stringToMessage(codeStr);
  return std::make_pair(c, data);
}

vpMegaPose::vpMegaPose(const std::string &host, int port, const vpCameraParameters &cam, unsigned height, unsigned width)
{
#if defined(_WIN32)
  WSADATA WSAData;
  if (WSAStartup(MAKEWORD(2, 0), &WSAData) != 0) {
    throw vpException(vpException::ioError, "Could not perform WSAStartup");
  }
#endif
  struct sockaddr_in serv_addr;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  if ((m_serverSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
#else
  if ((m_serverSocket = static_cast<int>(socket(AF_INET, SOCK_STREAM, 0))) < 0) {
#endif
    throw vpException(vpException::ioError, "Could not create socket to connect to MegaPose server");
  }
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);
  // Convert string to a binary address representation
  if (inet_pton(AF_INET, host.c_str(), &serv_addr.sin_addr) <= 0) {
    throw vpException(vpException::badValue, "Invalid ip address: " + host);
  }
  //Initiate connection
  if ((m_fd = connect(m_serverSocket, (struct sockaddr *)&serv_addr, sizeof(serv_addr))) < 0) {
    throw vpException(vpException::ioError, "Could not connect to server at " + host + ":" + std::to_string(port));
  }
  setIntrinsics(cam, height, width);
}

vpMegaPose::~vpMegaPose()
{
  std::vector<uint8_t> data;
  makeMessage(ServerMessage::EXIT, data);
  send(m_serverSocket, reinterpret_cast<const char *>(data.data()), static_cast<int>(data.size()), 0);

#if defined(_WIN32)
  WSACleanup();
#else
  close(m_fd);
#endif
}

std::vector<vpMegaPoseEstimate>
vpMegaPose::estimatePoses(const vpImage<vpRGBa>&image, const std::vector<std::string>&labels,
                          const vpImage<uint16_t>*const depth, const double depth_to_m,
                          const std::vector<vpRect>*const detections, const std::vector<vpHomogeneousMatrix>*const initial_cTos,
                          int refinerIterations)
{
  const std::lock_guard<std::mutex> lock(m_mutex);
  std::vector<uint8_t> data;
  encode(data, image);
  json parametersJson;
  parametersJson["labels"] = labels;

  if (detections == nullptr && initial_cTos == nullptr) {
    throw vpException(vpException::badValue, "You must either provide detections (bounding boxes) or initial pose estimates for MegaPose to work.");
  }

  if (detections != nullptr) {
    if (detections->size() != labels.size()) {
      throw vpException(vpException::badValue, "Same number of bounding boxes and labels must be provided.");
    }
    json detectionsJson = json::array();
    for (const vpRect &bb : *detections) {
      json j;
      to_megapose_json(j, bb);
      detectionsJson.push_back(j);
    }
    parametersJson["detections"] = detectionsJson;
  }

  if (initial_cTos != nullptr) {
    if (initial_cTos->size() != labels.size()) {
      throw vpException(vpException::badValue, "An initial estimate should be given for each detected object in the image");
    }
    json cToJson = json::array();
    for (const vpHomogeneousMatrix &cTo : *initial_cTos) {
      json j;
      to_megapose_json(j, cTo);
      cToJson.push_back(j);
    }
    parametersJson["initial_cTos"] = cToJson;
  }
  if (refinerIterations >= 0) {
    parametersJson["refiner_iterations"] = refinerIterations;
  }
  if (depth != nullptr) {
    if (depth_to_m <= 0.0) {
      throw vpException(vpException::badValue, "When using depth, the scale factor should be specified.");
    }
    parametersJson["use_depth"] = true;
    parametersJson["depth_scale_to_m"] = depth_to_m;
  }
  else {
    parametersJson["use_depth"] = false;
  }
  encode(data, parametersJson.dump());
  if (depth != nullptr) {
    encode(data, *depth);
  }
  makeMessage(ServerMessage::GET_POSE, data);
  send(m_serverSocket, reinterpret_cast<const char *>(data.data()), static_cast<int>(data.size()), 0);
  //std::cout<< "Encoding time = " << (vpTime::measureTimeMs() - beforeEncoding) << std::endl;

  ServerMessage code;
  std::vector<uint8_t> data_result;
  std::tie(code, data_result) = readMessage();

  unsigned int index = 0;
  if (code != ServerMessage::RET_POSE) {
    handleWrongReturnMessage(code, data_result);
  }
  std::string jsonStr;
  decode(data_result, index, jsonStr);
  json jsonValue = json::parse(jsonStr);
  std::vector<vpMegaPoseEstimate> result = jsonValue;
  return result;
}

std::vector<double> vpMegaPose::scorePoses(const vpImage<vpRGBa>&image,
  const std::vector<std::string>&labels, const std::vector<vpHomogeneousMatrix>&cTos)
{
  const std::lock_guard<std::mutex> lock(m_mutex);
  std::vector<uint8_t> data;
  if (cTos.size() != labels.size()) {
    throw vpException(vpException::generalExceptionEnum::badValue, "The number of poses should be the same as the number of object labels");
  }
  encode(data, image);
  json parametersJson;
  json cToJson = json::array();
  for (const vpHomogeneousMatrix &cTo : cTos) {
    json j;
    to_megapose_json(j, cTo);
    cToJson.push_back(j);
  }
  parametersJson["cTos"] = cToJson;
  parametersJson["labels"] = labels;

  encode(data, parametersJson.dump());
  makeMessage(ServerMessage::GET_SCORE, data);
  send(m_serverSocket, reinterpret_cast<const char *>(data.data()), static_cast<int>(data.size()), 0);

  ServerMessage code;
  std::vector<uint8_t> data_result;
  std::tie(code, data_result) = readMessage();

  if (code != ServerMessage::RET_SCORE) {
    handleWrongReturnMessage(code, data_result);
  }
  unsigned int index = 0;
  std::string jsonStr;
  decode(data_result, index, jsonStr);
  json jsonValue = json::parse(jsonStr);
  std::vector<double> result = jsonValue;
  return result;
}


void vpMegaPose::setIntrinsics(const vpCameraParameters& cam, unsigned height, unsigned width)
{
  const std::lock_guard<std::mutex> lock(m_mutex);
  std::vector<uint8_t> data;

  json message;
  message["px"] = cam.get_px();
  message["py"] = cam.get_py();
  message["u0"] = cam.get_u0();
  message["v0"] = cam.get_v0();
  message["h"] = height;
  message["w"] = width;

  encode(data, message.dump());
  makeMessage(ServerMessage::SET_INTR, data);

  send(m_serverSocket, reinterpret_cast<const char *>(data.data()), static_cast<int>(data.size()), 0);
  ServerMessage code;
  std::vector<uint8_t> data_result;
  std::tie(code, data_result) = readMessage();
  if (code != ServerMessage::OK) {
    handleWrongReturnMessage(code, data_result);
  }
}

vpImage<vpRGBa> vpMegaPose::viewObjects(const std::vector<std::string>&objectNames,
                                        const std::vector<vpHomogeneousMatrix>&poses, const std::string& viewType)
{
  const std::lock_guard<std::mutex> lock(m_mutex);
  std::vector<uint8_t> data;
  json j;
  j["labels"] = objectNames;
  json cToJson = json::array();
  for (const vpHomogeneousMatrix &cTo : poses) {
    json j;
    to_megapose_json(j, cTo);
    cToJson.push_back(j);
  }
  j["poses"] = cToJson;
  j["type"] = viewType;
  encode(data, j.dump());
  makeMessage(ServerMessage::GET_VIZ, data);
  send(m_serverSocket, reinterpret_cast<const char *>(data.data()), static_cast<int>(data.size()), 0);
  ServerMessage code;
  std::vector<uint8_t> data_result;
  std::tie(code, data_result) = readMessage();

  if (code != ServerMessage::RET_VIZ) {
    handleWrongReturnMessage(code, data_result);
  }
  vpImage<vpRGBa> result;
  unsigned int index = 0;
  decode(data_result, index, result);
  return result;
}

void vpMegaPose::setCoarseNumSamples(const unsigned num)
{
  const std::lock_guard<std::mutex> lock(m_mutex);
  std::vector<uint8_t> data;
  json j;
  j["so3_grid_size"] = num;
  encode(data, j.dump());
  makeMessage(ServerMessage::SET_SO3_GRID_SIZE, data);
  send(m_serverSocket, reinterpret_cast<const char *>(data.data()), static_cast<int>(data.size()), 0);
  ServerMessage code;
  std::vector<uint8_t> data_result;
  std::tie(code, data_result) = readMessage();
  if (code != ServerMessage::OK) {
    handleWrongReturnMessage(code, data_result);
  }
}

std::vector<std::string> vpMegaPose::getObjectNames()
{
  const std::lock_guard<std::mutex> lock(m_mutex);
  std::vector<uint8_t> data;
  makeMessage(ServerMessage::GET_LIST_OBJECTS, data);
  send(m_serverSocket, reinterpret_cast<const char *>(data.data()), static_cast<int>(data.size()), 0);
  ServerMessage code;
  std::vector<uint8_t> data_result;
  std::tie(code, data_result) = readMessage();
  if (code != ServerMessage::RET_LIST_OBJECTS) {
    handleWrongReturnMessage(code, data_result);
  }
  unsigned int index = 0;
  std::string jsonStr;
  decode(data_result, index, jsonStr);
  json jsonValue = json::parse(jsonStr);
  std::vector<std::string> result = jsonValue;
  return result;
}
END_VISP_NAMESPACE
#else

// Work around to avoid libvisp_dnn_tracker library empty when threads are not used
class VISP_EXPORT dummy_vpMegaPose
{
public:
  dummy_vpMegaPose() { };
};

#if !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_dnn_tracker.a(vpMegaPose.cpp.o) has no symbols
void dummy_vpMegaPose_fct() { };
#endif

#endif
