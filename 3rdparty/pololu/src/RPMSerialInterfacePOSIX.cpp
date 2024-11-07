/*
   The MIT License (MIT) (http://opensource.org/licenses/MIT)

   Copyright (c) 2015 Jacques Menuet

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/
#include "RPMSerialInterfacePOSIX.h"

#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <string.h>

#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

#include <errno.h>

RPMSerialInterfacePOSIX::RPMSerialInterfacePOSIX(const std::string &portName, std::string *errorMessage)
  : RPMSerialInterface(),
  mFileDescriptor(-1)
{
  mFileDescriptor = openPort(portName, errorMessage);
}

RPMSerialInterfacePOSIX::~RPMSerialInterfacePOSIX()
{
  if (isOpen()) {
    // Before destroying the interface, we "go home"
    goHomeCP();

    close(mFileDescriptor);
  }
  mFileDescriptor = -1;
}

bool RPMSerialInterfacePOSIX::isOpen() const
{
  return mFileDescriptor!=-1;
}

int RPMSerialInterfacePOSIX::openPort(const std::string &portName, std::string *errorMessage)
{
  int fd = open(portName.c_str(), O_RDWR | O_NOCTTY);
  if (fd == -1) {
    if (errorMessage) {
      std::stringstream stream;
      stream << "Failed to open serial port \"" << portName << "\". ";
      stream << strerror(errno);
      *errorMessage = stream.str();
    }
    return -1;
  }

#ifndef _WIN32
  struct termios options;
  tcgetattr(fd, &options);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_oflag &= ~(ONLCR | OCRNL);
  tcsetattr(fd, TCSANOW, &options);
#endif

  return fd;
}

bool RPMSerialInterfacePOSIX::writeBytes(const unsigned char *data, unsigned int numBytesToWrite)
{
  if (!isOpen())
    return false;

  // See http://linux.die.net/man/2/write
  ssize_t ret = write(mFileDescriptor, data, numBytesToWrite);
  if (ret==-1) {
    std::stringstream stream;
    stream << "Unable to write bytes to serial port. ";
    stream << "Error code " << errno;
    setErrorMessage(stream.str());
    return false;
  }
  else if (ret!=numBytesToWrite) {
    std::stringstream stream;
    stream << "Unable to write bytes to serial port. Wrote only " << ret << " out of " << numBytesToWrite;
    setErrorMessage(stream.str());
    return false;
  }

  return true;
}

bool RPMSerialInterfacePOSIX::readBytes(unsigned char *data, unsigned int numBytesToRead)
{
  if (!isOpen())
    return false;

  // See http://linux.die.net/man/2/read
  ssize_t ret = read(mFileDescriptor, data, numBytesToRead);
  if (ret==-1) {
    std::stringstream stream;
    stream << "Unable to read bytes from serial port. ";
    stream << "Error code " << errno;
    setErrorMessage(stream.str());
    return false;
  }
  else if (ret!=numBytesToRead) {
    std::stringstream stream;
    stream << "Unable to read bytes from serial port. Read only " << ret << " out of " << numBytesToRead;
    setErrorMessage(stream.str());
    return false;
  }
  return true;
}
