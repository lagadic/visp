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
#pragma once

#include "RPMSerialInterface.h"

class RPMSerialInterfacePOSIX : public RPMSerialInterface
{
public:
  // Creates a POSIX SerialInterface.
  // The syntax of the port name depends on the platform. For example:
  // - on Windows: "\\\\.\\USBSER000", "\\\\.\\COM6", etc...
  // - on Linux: "/dev/ttyACM0"
  // - on Mac OS: "/dev/cu.usbmodem00034567"
  RPMSerialInterfacePOSIX(const std::string &portName, std::string *errorMessage = NULL);

  virtual ~RPMSerialInterfacePOSIX();

  virtual bool isOpen() const;

private:
  int openPort(const std::string &portName, std::string *errorMessage = NULL);

  virtual bool writeBytes(const unsigned char *data, unsigned int dataSizeInBytes);
  virtual bool readBytes(unsigned char *data, unsigned int dataSizeInBytes);

  int	mFileDescriptor;
};
