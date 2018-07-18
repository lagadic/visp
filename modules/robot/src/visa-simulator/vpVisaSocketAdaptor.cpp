/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Visa simulator adpator.
 *
 * Authors:
 * Andrey Kudryavtsev (Femto-ST)
 *
 *****************************************************************************/

#ifdef HAVE_VISP
#  include <visp3/robot/vpVisaSocketAdaptor.h>
#else
#  include <vpVisaSocketAdaptor.h>
#endif

#include <base64.h>

// =============================================================================
// STRING MANIPULATIONS
// =============================================================================

std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r_ ")
{
  str.erase(str.find_last_not_of(chars) + 1);
  return str;
}

template<typename Out>
void split(const std::string &s, char delim, Out result)
{
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    *(result++) = item;
  }
}

std::vector<std::string> split(const std::string &s, char delim)
{
  std::vector<std::string> elems;
  split(s, delim, std::back_inserter(elems));
  return elems;
}

// =============================================================================
// FUNCTIONS
// =============================================================================

vpVisaSocketAdapter::vpVisaSocketAdapter()
  : m_connected(false), m_bufferImage(NULL)
{

}

vpVisaSocketAdapter::~vpVisaSocketAdapter()
{
  this->disconnect();
  if (m_bufferImage) {
    delete m_bufferImage;
    m_bufferImage = NULL;
  }
}


bool vpVisaSocketAdapter::connect(const std::string &host, const unsigned int port)
{
#ifdef _WIN32
  WSAStartup(MAKEWORD(2,0), &WSAData);
#endif

  m_server_socket.sin_addr.s_addr = inet_addr(host.c_str());
  m_server_socket.sin_family	    = AF_INET;
  m_server_socket.sin_port		    = htons(port);

  //sock = socket(AF_INET, SOCK_DGRAM , IPPROTO_UDP);
  m_sock = socket(AF_INET, SOCK_STREAM, 0);

#ifdef _WIN32
  m_connected = connected = (::connect(sock, (SOCKADDR*)&sin, sizeof(sin)) != SOCKET_ERROR);
#elif defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))
  int res = (::connect(m_sock, (struct sockaddr*)&m_server_socket, sizeof(m_server_socket))<0);
  m_connected = (res == 0);
  usleep(50*1000);
#endif

  return m_connected;
}

void vpVisaSocketAdapter::disconnect()
{
  if (m_connected){
    //dtor
#ifdef _WIN32
    closesocket(m_sock); // Fermeture du socket
    WSACleanup();
#elif defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))
    close(m_sock); // Fermeture du socket
#endif

    m_connected = false;
  }
}

bool vpVisaSocketAdapter::sendCmd(const std::string &cmd, const std::vector<double> &args) const
{
  std::string msg = cmd;
  for (size_t i = 0; i < args.size(); i++){
    msg.append(",");
    msg.append( std::to_string(args[i]) );
  }
  std::cout << msg << std::endl;

  char buffer[1024];
  strncpy(buffer, msg.c_str(), msg.size());
  ::send(m_sock, buffer, msg.size(), 0);

  char bufferResponse[500];
  ::recv(m_sock, bufferResponse, 500, 0);
  std::string str(bufferResponse);
  //std::cout << "response from visa" << str << std::endl;
  rtrim(str);
  if (str.compare(0,2,"OK") == 0){
    return true;
  }
  else{
    std::cerr << "ERROR: " << bufferResponse << std::endl;
    return false;
  }
}

bool vpVisaSocketAdapter::setJointPosAbs(const std::vector<double> &joints) const
{
  return sendCmd("SETJOINTPOSABS", joints);
}

bool vpVisaSocketAdapter::setJointPosRel(const std::vector<double> &joints) const
{
  return sendCmd("SETJOINTPOSREL", joints);
}

bool vpVisaSocketAdapter::setJointVel(const std::vector<double> &velocities) const
{
  return sendCmd("SETJOINTVEL", velocities);
}

bool vpVisaSocketAdapter::homing() const
{
  std::vector<double> nil;
  return sendCmd("HOMING", nil);
}

void vpVisaSocketAdapter::getCalibMatrix(std::vector<double> &matrix) const
{
  matrix.clear();
  char buffer[12] = "GETCALIBMAT";
  char bufferResponse[500]; //too large but sure to fit
  ::send(m_sock, buffer, sizeof(buffer)-1, 0);
  ::recv(m_sock, bufferResponse, 500, 0);
  std::string str(bufferResponse);
  rtrim(str);
  std::vector<std::string> valuesStr = split(str, ',');

  matrix.clear();
  matrix.resize(valuesStr.size());
  for (size_t i = 0; i < matrix.size(); i++){
    matrix[i] = std::stof(valuesStr[i]);
  }
}

void vpVisaSocketAdapter::getJointPos(std::vector<double> &values) const
{
  char buffer[12] = "GETJOINTPOS";
  char bufferResponse[500]; //too large but sure to fit
  ::send(m_sock, buffer, sizeof(buffer)-1, 0);
  ::recv(m_sock, bufferResponse, 500, 0);
  std::string str(bufferResponse);
  rtrim(str);
  std::vector<std::string> valuesStr = split(str, ',');

  values.clear();
  values.resize(valuesStr.size());
  for (size_t i = 0; i < values.size(); i++){
    values[i] = std::stof(valuesStr[i]);
  }
}

#ifdef HAVE_VISP
vpCameraParameters vpVisaSocketAdapter::getCameraParameters() const
{
  std::vector<double> calibMatrix;
  getCalibMatrix(calibMatrix);
  double px = calibMatrix[0];
  double py = calibMatrix[4];
  double u0 = calibMatrix[6];
  double v0 = calibMatrix[7];

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(px, py, u0, v0);

  return cam;
}

void vpVisaSocketAdapter::getJointPos(vpColVector &values) const
{
  std::vector<double> q;
  getJointPos(q);
  values = q;
}

bool vpVisaSocketAdapter::setJointVel(const vpColVector &velocities) const
{
  std::vector<double> qdot = velocities.toStdVector();
  return setJointVel(qdot);
}

bool vpVisaSocketAdapter::setJointPosAbs(const vpColVector &joints) const
{
  std::vector<double> q = joints.toStdVector();
  return setJointPosAbs(q);
}

bool vpVisaSocketAdapter::setJointPosRel(const vpColVector &joints) const
{
  std::vector<double> q = joints.toStdVector();
  return setJointPosRel(q);
}
#endif

void vpVisaSocketAdapter::getToolTransform(std::vector<double> &matrix) const
{
  char buffer[11] = "GETTOOLPOS";
  char bufferResponse[500]; //too large but sure to fit
  ::send(m_sock, buffer,sizeof(buffer)-1,0);
  ::recv(m_sock, bufferResponse, 500, 0);
  std::string str(bufferResponse);
  rtrim(str);
  std::vector<std::string> valuesStr = split(str, ',');

  matrix.clear();
  matrix.resize(valuesStr.size());
  for (size_t i = 0; i < matrix.size(); i++){
    matrix[i] = std::stof(valuesStr[i]);
  }
}

#ifdef VISP_HAVE_CPPBASE64
std::string vpVisaSocketAdapter::getImage()
{
  char buffer[9] = "GETIMAGE";
  char bufferResponse[500]; //UDP max package size
  std::string msgPrefix = "PACKAGE_LENGTH:";

  ::send(m_sock, buffer, 8, 0);
  ::recv(m_sock, bufferResponse, 500, 0);

  std::string message(bufferResponse);
  message = message.substr(msgPrefix.size(),10);
  message.erase(std::remove_if(message.begin(), message.end(),
                               [](char c) { return !std::isdigit(c); }),
                message.end());
  int imageSize = std::stoi(message); //parse int
  //std::cout << "imageSize: " << imageSize << std::endl;

  if (m_bufferImage == NULL) {
    m_bufferImage = new char[imageSize+2]; //allocate memory
  }
  //acquire the image
  ::recv(m_sock, m_bufferImage, imageSize+1, MSG_WAITALL);
  //delete prefix and decode
  std::string encodedImage(m_bufferImage);

  encodedImage = encodedImage.substr(0, imageSize);
  encodedImage.erase(0, 22);
  std::string res = base64_decode(encodedImage);

  return res;
}

#if defined(VISP_HAVE_OPENCV) || defined(HAVE_OPENCV)
cv::Mat vpVisaSocketAdapter::getImageOpenCV()
{
  std::string decodedImage = this->getImage();

  std::vector<uchar> vectordata(decodedImage.begin(), decodedImage.end());
  cv::Mat data_mat(vectordata,true);

  cv::Mat image(cv::imdecode(data_mat, 1)); //put 0 if you want greyscale
  return image;
}
#endif

#if defined(HAVE_VISP) && (defined(VISP_HAVE_OPENCV) || defined(HAVE_OPENCV))
vpImage<unsigned char> vpVisaSocketAdapter::getImageViSP()
{
  vpImage<unsigned char> I;
  vpImageConvert::convert(this->getImageOpenCV(), I);
  return I;
}
#endif
#endif // VISP_HAVE_CPPBASE64
