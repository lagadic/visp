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
 * CPU features (hardware capabilities).
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>
#include "x86/cpu_x86.h"
#if defined(VISP_HAVE_SIMDLIB)
#include <Simd/SimdLib.h>
#endif
#include <visp3/core/vpCPUFeatures.h>

BEGIN_VISP_NAMESPACE
namespace vpCPUFeatures
{
// TODO: try to refactor to keep only SimdCpuInfo code and remove cpu_x86 code?
static const FeatureDetector::cpu_x86 cpu_features;

bool checkSSE2() { return cpu_features.HW_SSE2; }

bool checkSSE3() { return cpu_features.HW_SSE3; }

bool checkSSSE3() { return cpu_features.HW_SSSE3; }

bool checkSSE41() { return cpu_features.HW_SSE41; }

bool checkSSE42() { return cpu_features.HW_SSE42; }

bool checkAVX() { return cpu_features.HW_AVX; }

bool checkAVX2() { return cpu_features.HW_AVX2; }

#if defined(VISP_HAVE_SIMDLIB)
size_t getCPUCacheL1() { return SimdCpuInfo(SimdCpuInfoCacheL1); }

size_t getCPUCacheL2() { return SimdCpuInfo(SimdCpuInfoCacheL2); }

size_t getCPUCacheL3() { return SimdCpuInfo(SimdCpuInfoCacheL3); }

bool checkNeon() { return SimdCpuInfo(SimdCpuInfoNeon) != 0; }
#endif

void printCPUInfo() { cpu_features.print(); }
} // namespace vpCPUFeatures
END_VISP_NAMESPACE
