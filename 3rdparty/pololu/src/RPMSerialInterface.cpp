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
#include "RPMSerialInterface.h"

#ifdef _WIN32
#include "RPMSerialInterfaceWindows.h"
#else
#include "RPMSerialInterfacePOSIX.h"
#endif

RPMSerialInterface *RPMSerialInterface::createSerialInterface(const std::string &portName, unsigned int baudRate, std::string *errorMessage)
{
  RPMSerialInterface *serialInterface = NULL;
#ifdef _WIN32
  serialInterface = new RPMSerialInterfaceWindows(portName, baudRate, errorMessage);
#else
  serialInterface = new RPMSerialInterfacePOSIX(portName, errorMessage);
  (void)baudRate;
#endif

  // If the interface couldn't be open properly, delete it
  if (!serialInterface->isOpen()) {
    delete serialInterface;
    serialInterface = NULL;
  }

  return serialInterface;
}

RPMSerialInterface::RPMSerialInterface()
  : mErrorMessage()
{ }

RPMSerialInterface::~RPMSerialInterface()
{ }

void RPMSerialInterface::clearErrorMessage()
{
  mErrorMessage.clear();
}

void RPMSerialInterface::setErrorMessage(const std::string &message)
{
  mErrorMessage = message;
}

bool RPMSerialInterface::setTargetCP(unsigned char channelNumber, unsigned short target)
{
  clearErrorMessage();
  if (target<getMinChannelValue() || target>getMaxChannelValue())
    return false;

  unsigned char command[4] = { 0x84, channelNumber, static_cast<unsigned char>(target & 0x7F), static_cast<unsigned char>((target >> 7) & 0x7F) };
  if (!writeBytes(command, sizeof(command)))
    return false;
  return true;
}

bool RPMSerialInterface::setTargetPP(unsigned char deviceNumber, unsigned char channelNumber, unsigned short target)
{
  clearErrorMessage();
  if (target<getMinChannelValue() || target>getMaxChannelValue())
    return false;
  unsigned char command[6] = { 0xAA, deviceNumber, static_cast<unsigned char>(0x84 & 0x7F), channelNumber, static_cast<unsigned char>(target & 0x7F), static_cast<unsigned char>((target >> 7) & 0x7F) };
  if (!writeBytes(command, sizeof(command)))
    return false;
  return true;
}

bool RPMSerialInterface::setTargetMSSCP(unsigned char miniSCCChannelNumber, unsigned char normalizedTarget)
{
  clearErrorMessage();
  if (normalizedTarget>254)
    return false;
  unsigned char command[3] = { 0xFF, miniSCCChannelNumber, normalizedTarget };
  if (!writeBytes(command, sizeof(command)))
    return false;
  return true;
}

bool RPMSerialInterface::setSpeedCP(unsigned char channelNumber, unsigned short speed)
{
  clearErrorMessage();
  unsigned char command[4] = { 0x87, channelNumber, static_cast<unsigned char>(speed & 0x7F), static_cast<unsigned char>((speed >> 7) & 0x7F) };
  if (!writeBytes(command, sizeof(command)))
    return false;
  return true;
}

bool RPMSerialInterface::setSpeedPP(unsigned char deviceNumber, unsigned char channelNumber, unsigned short speed)
{
  clearErrorMessage();
  unsigned char command[6] = { 0xAA, deviceNumber, static_cast<unsigned char>(0x87 & 0x7F), channelNumber, static_cast<unsigned char>(speed & 0x7F), static_cast<unsigned char>((speed >> 7) & 0x7F) };
  if (!writeBytes(command, sizeof(command)))
    return false;
  return true;
}

bool RPMSerialInterface::setAccelerationCP(unsigned char channelNumber, unsigned char acceleration)
{
  clearErrorMessage();
  unsigned short accelerationAsShort = acceleration;
  unsigned char command[4] = { 0x89, channelNumber, static_cast<unsigned char>(accelerationAsShort & 0x7F), static_cast<unsigned char>((accelerationAsShort >> 7) & 0x7F) };
  if (!writeBytes(command, sizeof(command)))
    return false;
  return true;
}

bool RPMSerialInterface::setAccelerationPP(unsigned char deviceNumber, unsigned char channelNumber, unsigned char acceleration)
{
  clearErrorMessage();
  unsigned short accelerationAsShort = acceleration;
  unsigned char command[6] = { 0xAA, deviceNumber, static_cast<unsigned char>(0x89 & 0x7F), channelNumber, static_cast<unsigned char>(accelerationAsShort & 0x7F), static_cast<unsigned char>((accelerationAsShort >> 7) & 0x7F) };
  if (!writeBytes(command, sizeof(command)))
    return false;
  return true;
}

bool RPMSerialInterface::getPositionCP(unsigned char channelNumber, unsigned short &position)
{
  clearErrorMessage();

  position = 0;

  unsigned char command[2] = { 0x90, channelNumber };
  if (!writeBytes(command, sizeof(command)))
    return false;

  unsigned char response[2] = { 0x00, 0x00 };
  if (!readBytes(response, sizeof(response)))
    return false;

  position = response[0] + 256*response[1];
  return true;
}

bool RPMSerialInterface::getPositionPP(unsigned char deviceNumber, unsigned char channelNumber, unsigned short &position)
{
  clearErrorMessage();

  position = 0;

  unsigned char command[4] = { 0xAA, deviceNumber, static_cast<unsigned char>(0x90 & 0x7F), channelNumber };
  if (!writeBytes(command, sizeof(command)))
    return false;

  unsigned char response[2] = { 0x00, 0x00 };
  if (!readBytes(response, sizeof(response)))
    return false;

  position = response[0] + 256*response[1];
  return true;
}

bool RPMSerialInterface::getMovingStateCP(bool &servosAreMoving)
{
  clearErrorMessage();

  servosAreMoving = false;
  unsigned char command = 0x93;
  if (!writeBytes(&command, sizeof(command)))
    return false;

  unsigned char response = 0x00;
  if (!readBytes(&response, sizeof(response)))
    return false;

  if (response!=0x00 && response!=0x01)
    return false;

  servosAreMoving = (response==0x01);
  return true;
}

bool RPMSerialInterface::getMovingStatePP(unsigned char deviceNumber, bool &servosAreMoving)
{
  clearErrorMessage();

  servosAreMoving = false;
  unsigned char command[3] = { 0xAA, deviceNumber, static_cast<unsigned char>(0x93 & 0x7F) };
  if (!writeBytes(command, sizeof(command)))
    return false;

  unsigned char response = 0x00;
  if (!readBytes(&response, sizeof(response)))
    return false;

  servosAreMoving = (response==0x01);
  return true;
}

bool RPMSerialInterface::getErrorsCP(unsigned short &errors)
{
  clearErrorMessage();

  unsigned char command = 0xA1;
  if (!writeBytes(&command, sizeof(command)))
    return false;

  unsigned char response[2] = { 0x00, 0x00 };
  if (!readBytes(response, sizeof(response)))
    return false;

  errors = (response[0] & 0x7F) + 256 * (response[1] & 0x7F);	// Need to check this code on real errors!
  return true;
}

bool RPMSerialInterface::getErrorsPP(unsigned char deviceNumber, unsigned short &errors)
{
  clearErrorMessage();

  unsigned char command[3] = { 0xAA, deviceNumber, 0xA1 & 0x7F };
  if (!writeBytes(command, sizeof(command)))
    return false;

  unsigned char response[2] = { 0x00, 0x00 };
  if (!readBytes(response, sizeof(response)))
    return false;

  errors = (response[0] & 0x7F) + 256 * (response[1] & 0x7F);	// Need to check this code on real errors!
  return true;
}

bool RPMSerialInterface::goHomeCP()
{
  clearErrorMessage();

  unsigned char command = 0xA2;
  if (!writeBytes(&command, sizeof(command)))
    return false;
  return true;
}

bool RPMSerialInterface::goHomePP(unsigned char deviceNumber)
{
  clearErrorMessage();

  unsigned char command[3] = { 0xAA, deviceNumber, 0xA2 & 0x7F };
  if (!writeBytes(command, sizeof(command)))
    return false;
  return true;
}
