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

#include <string>

/*
  SerialInterface

  The SerialInterface implements the Pololu Maestro serial communication protocol.
  See http://www.pololu.com/docs/0J40/5.e for the details of the commands available.

  The SerialInterface methods of the SerialInterface come into different flavours,
  each uses a different protocol:
  - Compact protocol (CP)
  - Pololu protocol (PP)
  - Mini-SSC protocol (MSSCP)
  See http://www.pololu.com/docs/0J40/5.c for more information.

  Note: add the ability to specify min/max target value at construction time rather
  than having it hard-coded.
*/
class RPMSerialInterface
{
public:
  // Create a concrete platform-specific SerialInterface.
  // Return NULL if the interface couldn't be created and set the optional error message.
  static RPMSerialInterface *createSerialInterface(const std::string &portName, unsigned int baudRate, std::string *errorMessage = NULL);

  // Return the last error message. The message is set when a methods encounters a problem and returns false.
  // It is automatically cleared at the beginning of each method.
  const std::string &getErrorMessage() const { return mErrorMessage; }

  // Destructor
  virtual ~RPMSerialInterface();

  // Indicates whether the serial port has been successfully open at construction
  virtual bool isOpen() const = 0;

  // Return the minimum valid channel value in 0.25 microsecond units
  static unsigned short getMinChannelValue() { return mMinChannelValue; }

  // Return the maximum valid channel value in 0.25 microsecond units
  static unsigned short getMaxChannelValue() { return mMaxChannelValue; }

  // Set the target position of a channel to a given value in 0.25 microsecond units
  bool setTargetCP(unsigned char channelNumber, unsigned short target);
  bool setTargetPP(unsigned char deviceNumber, unsigned char channelNumber, unsigned short target);

  // In the Mini-SSC version of this method, the normalizedTarget value is between 0 and 255.
  // The value is converted to an actual 0.25 microsecond unit value based on pre-configured
  // settings (neutral and range parameters) stored on the Pololu Maestro device itself.
  // This is done using a tool such as the Maestro Control Center. Calibration can be therefore
  // done externally quite easily.
  // Additionally, the miniSCC channel number allows access to channels of chained devices.
  bool setTargetMSSCP(unsigned char miniSCCChannelNumber, unsigned char normalizedTarget);

  // On Mini Maestro 12, 18 and 24 only, so not supported here (as I only have a Mini Maestro 6)
  // bool setMultipleTargets(...)

  // Set the speed limit of a channel in units of (0.25 microsecond)/(10ms)
  bool setSpeedCP(unsigned char channelNumber, unsigned short speed);
  bool setSpeedPP(unsigned char deviceNumber, unsigned char channelNumber, unsigned short speed);

  // Set the acceleration limit of a channel in units of (0.25 microsecond)/(10ms)/(80ms)
  bool setAccelerationCP(unsigned char channelNumber, unsigned char acceleration);
  bool setAccelerationPP(unsigned char deviceNumber, unsigned char channelNumber, unsigned char acceleration);

  // Return the position of a channel.
  // For a servo channel, the position is the current pulse width in 0.25 microsecond units
  // For a digital output channel, a position less than 6000 means the line is low, and high when above 6000
  // For an input channel, the position represents the voltage measured on the channel.
  bool getPositionCP(unsigned char channelNumber, unsigned short &position);
  bool getPositionPP(unsigned char deviceNumber, unsigned char channelNumber, unsigned short &position);

  // Indicate whether the servo outputs have reached their targets or are still moving.
  bool getMovingStateCP(bool &servosAreMoving);
  bool getMovingStatePP(unsigned char deviceNumber, bool &servosAreMoving);

  // The return the error detected by the Pololu Maestro. This automatically clears the error flag.
  bool getErrorsCP(unsigned short &error);
  bool getErrorsPP(unsigned char deviceNumber, unsigned short &error);

  // Request the Pololu Maestro to "go home".
  // Going home sets the channels to their startup/error state.
  // This state is defined on a per-channel. It can either be:
  // - ignore: the value is unchanged when we do a "go home". PWM signal is continues to be generated
  // - go to: the channel is set to the specified value. Again PWM signal is generated
  // - off: the channel is turned off. There's no more PWM signal generated for the channel
  bool goHomeCP();
  bool goHomePP(unsigned char deviceNumber);

protected:
  RPMSerialInterface();
  void clearErrorMessage();
  void setErrorMessage(const std::string &message);

  bool checkPortIsOpen() const;                             // And update error message if not
  bool checkValidTargetValue(unsigned short target) const;  // Same here

private:
  static const unsigned short mMinChannelValue = 3968;
  static const unsigned short mMaxChannelValue = 8000;

  virtual bool writeBytes(const unsigned char *data, unsigned int dataSizeInBytes) = 0;
  virtual bool readBytes(unsigned char *data, unsigned int dataSizeInBytes) = 0;

  std::string mErrorMessage;
};
