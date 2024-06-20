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

/* cpu_x86.cpp
 *
 * Author           : Alexander J. Yee
 * Date Created     : 04/12/2014
 * Last Modified    : 04/12/2014
 *
 * Modification for ViSP:
 *   - UNKNOWN_ARCH (ARM, ...)
 *   - ifndef _XCR_XFEATURE_ENABLED_MASK (MinGW)
 */

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  Dependencies
#include "cpu_x86.h"
#include <cstring>
#include <iostream>
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386) || defined(_M_IX86)
#if defined(_WIN32) && (!defined(__MINGW32__) || (!defined(__i386) && !defined(_M_IX86)))
#include "cpu_x86_Windows.ipp"
#elif defined(__GNUC__) || defined(__clang__)
#include "cpu_x86_Linux.ipp"
#else
//#       error "No cpuid intrinsic defined for compiler."
#define UNKNOWN_ARCH
#endif
#else
//#   error "No cpuid intrinsic defined for processor architecture."
#define UNKNOWN_ARCH
#endif

#ifndef _XCR_XFEATURE_ENABLED_MASK
#define _XCR_XFEATURE_ENABLED_MASK 0
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace FeatureDetector
{
using std::cout;
using std::endl;
using std::memcpy;
using std::memset;
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void cpu_x86::print(const char *label, bool yes)
{
  cout << label;
  cout << (yes ? "Yes" : "No") << endl;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
cpu_x86::cpu_x86()
{
  memset(this, 0, sizeof(*this));
  detect_host();
}
bool cpu_x86::detect_OS_AVX()
{
#ifndef UNKNOWN_ARCH
  //  Copied from: http://stackoverflow.com/a/22521619/922184

  bool avxSupported = false;

  uint32_t cpuInfo[4];
  cpuid(cpuInfo, 1);
  const unsigned int index_2 = 2;
  const unsigned int val_27 = 27;
  const unsigned int val_28 = 28;

  bool osUsesXSAVE_XRSTORE = (cpuInfo[index_2] & (1U << val_27)) != 0;
  bool cpuAVXSuport = (cpuInfo[index_2] & (1U << val_28)) != 0;

  if (osUsesXSAVE_XRSTORE && cpuAVXSuport) {
    uint64_t xcrFeatureMask = xgetbv(_XCR_XFEATURE_ENABLED_MASK);
    avxSupported = (xcrFeatureMask & 0x6U) == 0x6U;
  }

  return avxSupported;
#else
  return false;
#endif
}
bool cpu_x86::detect_OS_AVX512()
{
#ifndef UNKNOWN_ARCH
  if (!detect_OS_AVX()) {
    return false;
  }

  uint64_t xcrFeatureMask = xgetbv(_XCR_XFEATURE_ENABLED_MASK);
  return (xcrFeatureMask & 0xe6U) == 0xe6U;
#else
  return false;
#endif
}
std::string cpu_x86::get_vendor_string()
{
#ifndef UNKNOWN_ARCH
  uint32_t CPUInfo[4];
  char name[13];

  cpuid(CPUInfo, 0);
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int val_4 = 4;
  memcpy(name + 0, &CPUInfo[index_1], val_4);
  memcpy(name + 4, &CPUInfo[index_3], val_4);
  memcpy(name + 8, &CPUInfo[index_2], val_4);
  name[12] = '\0';

  return name;
#else
  return std::string();
#endif
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void cpu_x86::detect_host()
{
#ifndef UNKNOWN_ARCH
  //  OS Features
  OS_x64 = detect_OS_x64();
  OS_AVX = detect_OS_AVX();
  OS_AVX512 = detect_OS_AVX512();

  //  Vendor
  std::string vendor(get_vendor_string());
  if (vendor == "GenuineIntel") {
    Vendor_Intel = true;
  }
  else if (vendor == "AuthenticAMD") {
    Vendor_AMD = true;
  }

  uint32_t info[4];
  cpuid(info, 0);
  int nIds = info[0];

  cpuid(info, 0x80000000);
  uint32_t nExIds = info[0];
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int val_1 = 1;
  const unsigned int val_3 = 3;
  const unsigned int val_5 = 5;
  const unsigned int val_6 = 6;
  const unsigned int val_8 = 8;
  const unsigned int val_9 = 9;
  const unsigned int val_11 = 11;
  const unsigned int val_12 = 12;
  const unsigned int val_14 = 14;
  const unsigned int val_16 = 16;
  const unsigned int val_17 = 17;
  const unsigned int val_19 = 19;
  const unsigned int val_20 = 20;
  const unsigned int val_21 = 21;
  const unsigned int val_23 = 23;
  const unsigned int val_25 = 25;
  const unsigned int val_26 = 26;
  const unsigned int val_27 = 27;
  const unsigned int val_28 = 28;
  const unsigned int val_29 = 29;
  const unsigned int val_30 = 30;
  const unsigned int val_31 = 31;

  //  Detect Features
  if (nIds >= 0x00000001) {
    cpuid(info, 0x00000001);
    HW_MMX = (info[index_3] & (1U << val_23)) != 0U;
    HW_SSE = (info[index_3] & (1U << val_25)) != 0U;
    HW_SSE2 = (info[index_3] & (1U << val_26)) != 0U;
    HW_SSE3 = (info[index_2] & (1U << 0)) != 0U;

    HW_SSSE3 = (info[index_2] & (1U << val_9)) != 0U;
    HW_SSE41 = (info[index_2] & (1U << val_19)) != 0U;
    HW_SSE42 = (info[index_2] & (1U << val_20)) != 0U;
    HW_AES = (info[index_2] & (1U << val_25)) != 0U;

    HW_AVX = (info[index_2] & (1U << val_28)) != 0U;
    HW_FMA3 = (info[index_2] & (1U << val_12)) != 0U;

    HW_RDRAND = (info[index_2] & (1U << val_30)) != 0U;
  }
  if (nIds >= 0x00000007) {
    cpuid(info, 0x00000007);
    HW_AVX2 = (info[index_1] & (1U << val_5)) != 0U;

    HW_BMI1 = (info[index_1] & (1U << val_3)) != 0U;
    HW_BMI2 = (info[index_1] & (1U << val_8)) != 0U;
    HW_ADX = (info[index_1] & (1U << val_19)) != 0U;
    HW_MPX = (info[index_1] & (1U << val_14)) != 0U;
    HW_SHA = (info[index_1] & (1U << val_29)) != 0U;
    HW_PREFETCHWT1 = (info[index_2] & (1U << 0)) != 0U;

    HW_AVX512_F = (info[index_1] & (1U << val_16)) != 0U;
    HW_AVX512_CD = (info[index_1] & (1U << val_28)) != 0U;
    HW_AVX512_PF = (info[index_1] & (1U << val_26)) != 0U;
    HW_AVX512_ER = (info[index_1] & (1U << val_27)) != 0U;
    HW_AVX512_VL = (info[index_1] & (1U << val_31)) != 0U;
    HW_AVX512_BW = (info[index_1] & (1U << val_30)) != 0U;
    HW_AVX512_DQ = (info[index_1] & (1U << val_17)) != 0U;
    HW_AVX512_IFMA = (info[index_1] & (1U << val_21)) != 0U;
    HW_AVX512_VBMI = (info[index_2] & (1U << val_1)) != 0U;
  }
  if (nExIds >= 0x80000001) {
    cpuid(info, 0x80000001);
    HW_x64 = (info[index_3] & (1U << val_29)) != 0U;
    HW_ABM = (info[index_2] & (1U << val_5)) != 0U;
    HW_SSE4a = (info[index_2] & (1U << val_6)) != 0U;
    HW_FMA4 = (info[index_2] & (1U << val_16)) != 0U;
    HW_XOP = (info[index_2] & (1U << val_11)) != 0U;
  }
#endif
}
void cpu_x86::print() const
{
  cout << "CPU Vendor:" << endl;
  print("    AMD         = ", Vendor_AMD);
  print("    Intel       = ", Vendor_Intel);
  cout << endl;

  cout << "OS Features:" << endl;
#ifdef _WIN32
  print("    64-bit      = ", OS_x64);
#endif
  print("    OS AVX      = ", OS_AVX);
  print("    OS AVX512   = ", OS_AVX512);
  cout << endl;

  cout << "Hardware Features:" << endl;
  print("    MMX         = ", HW_MMX);
  print("    x64         = ", HW_x64);
  print("    ABM         = ", HW_ABM);
  print("    RDRAND      = ", HW_RDRAND);
  print("    BMI1        = ", HW_BMI1);
  print("    BMI2        = ", HW_BMI2);
  print("    ADX         = ", HW_ADX);
  print("    MPX         = ", HW_MPX);
  print("    PREFETCHWT1 = ", HW_PREFETCHWT1);
  cout << endl;

  cout << "SIMD: 128-bit" << endl;
  print("    SSE         = ", HW_SSE);
  print("    SSE2        = ", HW_SSE2);
  print("    SSE3        = ", HW_SSE3);
  print("    SSSE3       = ", HW_SSSE3);
  print("    SSE4a       = ", HW_SSE4a);
  print("    SSE4.1      = ", HW_SSE41);
  print("    SSE4.2      = ", HW_SSE42);
  print("    AES-NI      = ", HW_AES);
  print("    SHA         = ", HW_SHA);
  cout << endl;

  cout << "SIMD: 256-bit" << endl;
  print("    AVX         = ", HW_AVX);
  print("    XOP         = ", HW_XOP);
  print("    FMA3        = ", HW_FMA3);
  print("    FMA4        = ", HW_FMA4);
  print("    AVX2        = ", HW_AVX2);
  cout << endl;

  cout << "SIMD: 512-bit" << endl;
  print("    AVX512-F    = ", HW_AVX512_F);
  print("    AVX512-CD   = ", HW_AVX512_CD);
  print("    AVX512-PF   = ", HW_AVX512_PF);
  print("    AVX512-ER   = ", HW_AVX512_ER);
  print("    AVX512-VL   = ", HW_AVX512_VL);
  print("    AVX512-BW   = ", HW_AVX512_BW);
  print("    AVX512-DQ   = ", HW_AVX512_DQ);
  print("    AVX512-IFMA = ", HW_AVX512_IFMA);
  print("    AVX512-VBMI = ", HW_AVX512_VBMI);
  cout << endl;

  cout << "Summary:" << endl;
  print("    Safe to use AVX:     ", HW_AVX && OS_AVX);
  print("    Safe to use AVX512:  ", HW_AVX512_F && OS_AVX512);
  cout << endl;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
} // namespace FeatureDetector
#endif
