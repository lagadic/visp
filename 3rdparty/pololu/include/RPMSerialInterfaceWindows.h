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

#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

class RPMSerialInterfaceWindows : public RPMSerialInterface
{
public:
  // Creates a Windows SerialInterface.
  // The port name corresponds to a file name as used by the CreateFile system function.
  // Such name can have various syntaxes, for example:
  // - "COM4",
  // - "\\\\.\\USBSER000"
  // - "USB#VID_1FFB&PID_0089&MI_04#6&3ad40bf600004#"
  RPMSerialInterfaceWindows(const std::string &portName, unsigned int baudRate, std::string *errorMessage = NULL);

  virtual ~RPMSerialInterfaceWindows();

  virtual bool isOpen() const;

private:
  static HANDLE openPort(const std::string &portName, unsigned int baudRate, std::string *errorMessage);

  virtual bool writeBytes(const unsigned char *data, unsigned int dataSizeInBytes);
  virtual bool readBytes(unsigned char *data, unsigned int dataSizeInBytes);

  HANDLE	mPortHandle;
};
