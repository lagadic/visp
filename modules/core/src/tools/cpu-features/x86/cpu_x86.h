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
 */

/* cpu_x86.h
 *
 * Author           : Alexander J. Yee
 * Date Created     : 04/12/2014
 * Last Modified    : 04/12/2014
 *
 */

#pragma once
#ifndef CPU_X86_H
#define CPU_X86_H
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  Dependencies
#include <stdint.h>
#include <string>
#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace FeatureDetector
{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
struct cpu_x86
{
//  Vendor
  bool Vendor_AMD;
  bool Vendor_Intel;

  //  OS Features
  bool OS_x64;
  bool OS_AVX;
  bool OS_AVX512;

  //  Misc.
  bool HW_MMX;
  bool HW_x64;
  bool HW_ABM;
  bool HW_RDRAND;
  bool HW_BMI1;
  bool HW_BMI2;
  bool HW_ADX;
  bool HW_PREFETCHWT1;
  bool HW_MPX;

  //  SIMD: 128-bit
  bool HW_SSE;
  bool HW_SSE2;
  bool HW_SSE3;
  bool HW_SSSE3;
  bool HW_SSE41;
  bool HW_SSE42;
  bool HW_SSE4a;
  bool HW_AES;
  bool HW_SHA;

  //  SIMD: 256-bit
  bool HW_AVX;
  bool HW_XOP;
  bool HW_FMA3;
  bool HW_FMA4;
  bool HW_AVX2;

  //  SIMD: 512-bit
  bool HW_AVX512_F;
  bool HW_AVX512_PF;
  bool HW_AVX512_ER;
  bool HW_AVX512_CD;
  bool HW_AVX512_VL;
  bool HW_AVX512_BW;
  bool HW_AVX512_DQ;
  bool HW_AVX512_IFMA;
  bool HW_AVX512_VBMI;

public:
  cpu_x86();

  void print() const;

  static void cpuid(uint32_t out[4], uint32_t x);
  static std::string get_vendor_string();

private:
  void detect_host();
  static void print(const char *label, bool yes);

  static bool detect_OS_x64();
  static bool detect_OS_AVX();
  static bool detect_OS_AVX512();
};
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
} // namespace FeatureDetector
#endif
#endif
