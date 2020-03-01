/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Functions for correct endianness handling.
 *
 *****************************************************************************/

/*!
  \file vpEndian.cpp
  \brief Functions for correct endianness handling.
*/
#include <stdexcept>
#include <visp3/core/vpEndian.h>

namespace vpEndian
{
/*!
  Swap 16 bits by shifting to the right the first byte and by shifting to the
  left the second byte.
*/
uint16_t swap16bits(uint16_t val)
{
  return (((val >> 8) & 0x00FF) | ((val << 8) & 0xFF00));
}

/*!
  Swap 32 bits by shifting to the right the first 2 bytes and by shifting to
  the left the last 2 bytes.
*/
uint32_t swap32bits(uint32_t val)
{
  return (((val >> 24) & 0x000000FF) | ((val >> 8) & 0x0000FF00) | ((val << 8) & 0x00FF0000) |
          ((val << 24) & 0xFF000000));
}

/*!
  Swap a float, the union is necessary because of the representation of a
  float in memory in IEEE 754.
*/
float swapFloat(float f)
{
  union {
    float f;
    unsigned char b[4];
  } dat1, dat2;

  dat1.f = f;
  dat2.b[0] = dat1.b[3];
  dat2.b[1] = dat1.b[2];
  dat2.b[2] = dat1.b[1];
  dat2.b[3] = dat1.b[0];
  return dat2.f;
}

/*!
  Swap a double, the union is necessary because of the representation of a
  double in memory in IEEE 754.
*/
double swapDouble(double d)
{
  union {
    double d;
    unsigned char b[8];
  } dat1, dat2;

  dat1.d = d;
  dat2.b[0] = dat1.b[7];
  dat2.b[1] = dat1.b[6];
  dat2.b[2] = dat1.b[5];
  dat2.b[3] = dat1.b[4];
  dat2.b[4] = dat1.b[3];
  dat2.b[5] = dat1.b[2];
  dat2.b[6] = dat1.b[1];
  dat2.b[7] = dat1.b[0];
  return dat2.d;
}

/*!
  Reinterpret an array of unsigned char stored in a little-endian way into an uint16_t value.

  \warning Pointer must be valid and 16-bit must be correctly readable.
*/
uint16_t reinterpret_cast_uchar_to_uint16_LE(unsigned char * const ptr)
{
#ifdef VISP_LITTLE_ENDIAN
    return *reinterpret_cast<uint16_t *>(ptr);
#elif defined(VISP_BIG_ENDIAN)
    return swap16bits(*reinterpret_cast<uint16_t *>(ptr));
#else
    throw std::runtime_error("Not supported endianness for correct  custom reinterpret_cast() function.");
#endif
}
}
