/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
 \file vpTemplateTrackerHeader.h
 \brief
*/

#ifndef vpTemplateTrackerHeader_hh
#define vpTemplateTrackerHeader_hh

#include <stdio.h>

/*!
  \struct vpTemplateTrackerZPoint
  \ingroup group_tt_tools
*/
struct vpTemplateTrackerZPoint {
  int x, y;

  vpTemplateTrackerZPoint() : x(0), y(0) {}
};
/*!
  \struct vpTemplateTrackerDPoint
  \ingroup group_tt_tools
*/
struct vpTemplateTrackerDPoint {
  double x, y;

  vpTemplateTrackerDPoint() : x(0), y(0) {}
};
/*!
  \struct vpTemplateTrackerPoint
  \ingroup group_tt_tools
*/
struct vpTemplateTrackerPoint {
  int x, y;
  double dx, dy;
  double val;
  double *dW;
  double *HiG;

  vpTemplateTrackerPoint() : x(0), y(0), dx(0), dy(0), val(0), dW(NULL), HiG(NULL) {}
};
/*!
  \struct vpTemplateTrackerPointCompo
  \ingroup group_tt_tools
*/
struct vpTemplateTrackerPointCompo {
  double *dW;
  vpTemplateTrackerPointCompo() : dW(NULL) {}
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS
struct vpTemplateTrackerPointSuppMIInv {
  double et;
  int ct;
  double *BtInit;
  double *Bt;
  double *dBt;
  double *d2W;
  double *d2Wx;
  double *d2Wy;
  vpTemplateTrackerPointSuppMIInv() : et(0), ct(0), BtInit(NULL), Bt(NULL), dBt(NULL), d2W(NULL), d2Wx(NULL), d2Wy(NULL)
  {
  }
};
#endif // DOXYGEN_SHOULD_SKIP_THIS

#endif
