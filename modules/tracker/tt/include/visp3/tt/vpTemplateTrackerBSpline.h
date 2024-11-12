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
 * Template tracker.
 */
/*!
 \file vpTemplateTrackerBSpline.h
 \brief
*/

#ifndef VP_TEMPLATE_TRACKER_BSPLINE_H
#define VP_TEMPLATE_TRACKER_BSPLINE_H

#include <visp3/core/vpConfig.h>  // for BEGIN_VISP_NAMESPACE, END_VISP_NAME...

#ifndef DOXYGEN_SHOULD_SKIP_THIS
BEGIN_VISP_NAMESPACE

template <class Type> class vpImage;

class VISP_EXPORT vpTemplateTrackerBSpline
{
public:
  static double Bspline4(double diff);

  static double getSubPixBspline4(const vpImage<double> &I, double r, double t);
};
END_VISP_NAMESPACE
#endif
#endif
