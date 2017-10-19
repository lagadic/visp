/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * CPU features (hardware capabilities).
 *
 *****************************************************************************/

#include <visp3/core/vpCPUFeatures.h>
#include "x86/cpu_x86.h"

namespace vpCPUFeatures {
static const FeatureDetector::cpu_x86 cpu_features;

bool checkSSE2() {
  return cpu_features.HW_SSE2;
}

bool checkSSE3() {
  return cpu_features.HW_SSE3;
}

bool checkSSSE3() {
  return cpu_features.HW_SSSE3;
}

bool checkSSE41() {
  return cpu_features.HW_SSE41;
}

bool checkSSE42() {
  return cpu_features.HW_SSE42;
}

bool checkAVX() {
  return cpu_features.HW_AVX;
}

bool checkAVX2() {
  return cpu_features.HW_AVX2;
}

void printCPUInfo() {
  cpu_features.print();
}
} //namespace vpCPUFeatures
