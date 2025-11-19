/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Functions for endianness handling.
 */

/*!
  \file vpEndian.cpp
  \brief Functions for endianness handling.
*/
#include <stdexcept>
#include <visp3/core/vpEndian.h>

BEGIN_VISP_NAMESPACE
namespace vpEndian
{
/*!
  Swap 16 bits by shifting to the right the first byte and by shifting to the
  left the second byte.
*/
uint16_t swap16bits(uint16_t val)
{
  const unsigned int magic_8 = 8;
  const unsigned int magic_0x00FF = 0x00FF;
  const unsigned int magic_0xFF00 = 0xFF00;
  return (((val >> magic_8) & magic_0x00FF) | ((val << magic_8) & magic_0xFF00));
}

/*!
  Swap 32 bits by shifting to the right the first 2 bytes and by shifting to
  the left the last 2 bytes.
*/
uint32_t swap32bits(uint32_t val)
{
  const unsigned int magic_8 = 8;
  const unsigned int magic_24 = 24;
  const unsigned int magic_0x000000FF = 0x000000FFU;
  const unsigned int magic_0x0000FF00 = 0x0000FF00U;
  const unsigned int magic_0x00FF0000 = 0x00FF0000U;
  const unsigned int magic_0xFF000000 = 0xFF000000U;
  return (((val >> magic_24) & magic_0x000000FF) | ((val >> magic_8) & magic_0x0000FF00) | ((val << magic_8) & magic_0x00FF0000) |
          ((val << magic_24) & magic_0xFF000000));
}

/*!
  Swap 64 bits by shifting to the right the first 4 bytes and by shifting to
  the left the last 4 bytes.
*/
uint64_t swap64bits(uint64_t val)
{
  const unsigned int magic_8 = 8;
  const unsigned int magic_24 = 24;
  const unsigned int magic_40 = 40;
  const unsigned int magic_56 = 56;
  const uint64_t magic_0x000000000000FF00 = 0x000000000000FF00;
  const uint64_t magic_0x0000000000FF0000 = 0x0000000000FF0000;
  const uint64_t magic_0x00000000FF000000 = 0x00000000FF000000;
  const uint64_t magic_0x000000FF00000000 = 0x000000FF00000000;
  const uint64_t magic_0x0000FF0000000000 = 0x0000FF0000000000;
  const uint64_t magic_0x00FF000000000000 = 0x00FF000000000000;

  // https://stackoverflow.com/a/105342
  return (val >> magic_56) |
    ((val << magic_40) & magic_0x00FF000000000000) |
    ((val << magic_24) & magic_0x0000FF0000000000) |
    ((val << magic_8)  & magic_0x000000FF00000000) |
    ((val >> magic_8)  & magic_0x00000000FF000000) |
    ((val >> magic_24) & magic_0x0000000000FF0000) |
    ((val >> magic_40) & magic_0x000000000000FF00) |
    (val << magic_56);
}

/*!
  Swap a float, the union is necessary because of the representation of a
  float in memory in IEEE 754.
*/
float swapFloat(float f)
{
  union
  {
    float f;
    unsigned char b[4];
  } dat1, dat2;

  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  dat1.f = f;
  dat2.b[index_0] = dat1.b[index_3];
  dat2.b[index_1] = dat1.b[index_2];
  dat2.b[index_2] = dat1.b[index_1];
  dat2.b[index_3] = dat1.b[index_0];
  return dat2.f;
}

/*!
  Swap a double, the union is necessary because of the representation of a
  double in memory in IEEE 754.
*/
double swapDouble(double d)
{
  union
  {
    double d;
    unsigned char b[8];
  } dat1, dat2;

  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  const unsigned int index_6 = 6;
  const unsigned int index_7 = 7;
  dat1.d = d;
  dat2.b[index_0] = dat1.b[index_7];
  dat2.b[index_1] = dat1.b[index_6];
  dat2.b[index_2] = dat1.b[index_5];
  dat2.b[index_3] = dat1.b[index_4];
  dat2.b[index_4] = dat1.b[index_3];
  dat2.b[index_5] = dat1.b[index_2];
  dat2.b[index_6] = dat1.b[index_1];
  dat2.b[index_7] = dat1.b[index_0];
  return dat2.d;
}

/*!
  Reinterpret an array of unsigned char stored in a little-endian way into an uint16_t value.

  \warning Pointer must be valid and 16-bit must be correctly readable.
*/
uint16_t reinterpret_cast_uchar_to_uint16_LE(unsigned char *const ptr)
{
#ifdef VISP_LITTLE_ENDIAN
  return *reinterpret_cast<uint16_t *>(ptr);
#elif defined(VISP_BIG_ENDIAN)
  return swap16bits(*reinterpret_cast<uint16_t *>(ptr));
#else
  throw std::runtime_error("Not supported endianness for correct  custom reinterpret_cast() function.");
#endif
}

/*!
  Return true if the executed code runs on a big-endian platform.
*/
bool isBigEndian()
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  // https://github.com/mreininghaus/cnpypp/blob/c6cd4e2078e4f39e862720b66fb211c45577c510/src/cnpy%2B%2B.cpp#L29-L37
  // https://stackoverflow.com/questions/1001307/detecting-endianness-programmatically-in-a-c-program
  static_assert(sizeof(uint32_t) == 4);

  union
  {
    uint32_t i;
    char c[4];
  } constexpr test = { 0x01020304 };
#else
  union
  {
    uint32_t i;
    char c[4];
  } test = { 0x01020304 };
#endif

  return (test.c[0] == 0x01);
}
} // namespace vpEndian
END_VISP_NAMESPACE
