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
#include "RPMSerialInterfaceWindows.h"

#include <sstream>

RPMSerialInterfaceWindows::RPMSerialInterfaceWindows(const std::string &portName, unsigned int baudRate, std::string *errorMessage)
  : RPMSerialInterface(),
  mPortHandle(NULL)
{
  mPortHandle = openPort(portName, baudRate, errorMessage);
}

RPMSerialInterfaceWindows::~RPMSerialInterfaceWindows()
{
  if (isOpen()) {
    // Before destroying the interface, we "go home"
    goHomeCP();

    CloseHandle(mPortHandle);
  }
  mPortHandle = NULL;
}

bool RPMSerialInterfaceWindows::isOpen() const
{
  return mPortHandle!=INVALID_HANDLE_VALUE;
}

//  Open the port at given baud rate. Return the port handle when successful, otherwise return INVALID_HANDLE_VALUE
HANDLE RPMSerialInterfaceWindows::openPort(const std::string &portName, unsigned int baudRate, std::string *errorMessage)
{
  // Open the serial port
  HANDLE port = CreateFileA(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  if (port == INVALID_HANDLE_VALUE) {
    if (errorMessage) {
      std::stringstream stream;
      stream << "Failed to open serial port \"" << portName << "\". ";
      DWORD lastError = GetLastError();
      switch (lastError) {
      case ERROR_ACCESS_DENIED: stream << "Access denied. The device might be used by another program";
        break;
      case ERROR_FILE_NOT_FOUND: stream << "File not found. The port name might be wrong";
        break;
      default:
        stream << std::hex << "Error code 0x" << lastError;   // To test!
        break;
      }
      *errorMessage = stream.str();
    }
    return INVALID_HANDLE_VALUE;
  }

  // Set the timeouts
  COMMTIMEOUTS timeouts;
  BOOL success = GetCommTimeouts(port, &timeouts);
  if (!success) {
    if (errorMessage) {
      std::stringstream stream;
      stream << "Failed to open serial port \"" << portName << "\". ";
      stream << "Unable to get Comm Timeouts. " << std::hex << "Error code 0x" << GetLastError();
      *errorMessage = stream.str();
    }
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  timeouts.ReadIntervalTimeout = 1000;
  timeouts.ReadTotalTimeoutConstant = 1000;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 1000;
  timeouts.WriteTotalTimeoutMultiplier = 0;
  success = SetCommTimeouts(port, &timeouts);
  if (!success) {
    if (errorMessage) {
      std::stringstream stream;
      stream << "Failed to open serial port \"" << portName << "\". ";
      stream << "Unable to set Comm Timeouts. " << std::hex << "Error code 0x" << GetLastError();
      *errorMessage = stream.str();
    }
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  // Set the baud rate
  DCB commState;
  success = GetCommState(port, &commState);
  if (!success) {
    if (errorMessage) {
      std::stringstream stream;
      stream << "Failed to open serial port \"" << portName << "\". ";
      stream << "Unable to get Comm State. " << std::hex << "Error code 0x" << GetLastError();
      *errorMessage = stream.str();
    }
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  commState.BaudRate = baudRate;
  success = SetCommState(port, &commState);
  if (!success) {
    if (errorMessage) {
      std::stringstream stream;
      stream << "Failed to open serial port \"" << portName << "\". ";
      stream << "Unable to set Comm State. " << std::hex << "Error code 0x" << GetLastError();
      *errorMessage = stream.str();
    }
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  // Flush out any bytes received from the device earlier
  success = FlushFileBuffers(port);
  if (!success) {
    if (errorMessage) {
      std::stringstream stream;
      stream << "Failed to open serial port \"" << portName << "\". ";
      stream << "Unable to flush port buffers. " << std::hex << "Error code 0x" << GetLastError();
      *errorMessage = stream.str();
    }
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  return port;
}

bool RPMSerialInterfaceWindows::writeBytes(const unsigned char *data, unsigned int numBytesToWrite)
{
  if (!isOpen())
    return false;

  DWORD bytesTransferred = 0;
  BOOL success = WriteFile(mPortHandle, data, numBytesToWrite, &bytesTransferred, NULL);
  if (!success) {
    std::stringstream stream;
    stream << "Unable to write bytes to serial port. " << std::hex << "Error code 0x" << GetLastError();
    setErrorMessage(stream.str());
    return false;
  }

  if (numBytesToWrite!=bytesTransferred) {
    std::stringstream stream;
    stream << "Unable to write bytes to serial port. Wrote only " << bytesTransferred << " out of " << numBytesToWrite;
    setErrorMessage(stream.str());
    return false;
  }
  return true;
}

bool RPMSerialInterfaceWindows::readBytes(unsigned char *data, unsigned int numBytesToRead)
{
  if (!isOpen())
    return false;

  DWORD bytesTransferred = 0;
  BOOL success = ReadFile(mPortHandle, data, numBytesToRead, &bytesTransferred, NULL);
  if (!success) {
    std::stringstream stream;
    stream << "Unable to read bytes from serial port. " << std::hex << "Error code 0x" << GetLastError();
    setErrorMessage(stream.str());
    return false;
  }

  if (numBytesToRead!=bytesTransferred) {
    std::stringstream stream;
    stream << "Unable to read bytes from serial port. Read only " << bytesTransferred << " out of " << numBytesToRead;
    setErrorMessage(stream.str());
    return false;
  }
  return true;
}
