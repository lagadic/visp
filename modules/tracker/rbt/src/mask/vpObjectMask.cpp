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

#include <visp3/rbt/vpObjectMask.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpDisplay.h>

BEGIN_VISP_NAMESPACE

void vpObjectMask::display(const vpImage<float> &mask, vpImage<unsigned char> &Imask) const
{
  if (mask.getSize() != Imask.getSize()) {
    throw vpException(vpException::dimensionError, "Cannot convert float mask to unsigned char mask as they do not have the same size");
  }

#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(mask.getSize()); ++i) {
    Imask.bitmap[i] = static_cast<unsigned char>(mask.bitmap[i] * 255.f);
  }

  vpDisplay::display(Imask);
}

END_VISP_NAMESPACE
