/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
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

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
#include <visp/vpSimulator.h>
#include <visp/vpColor.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpImageTools.h>
#include <visp/vpImageFilter.h>
#include <visp/vpTime.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPose.h>
#ifdef USE_DISPLAY
#include <visp/vpDisplayX.h>
#endif

#include <vector>

// #define USE_DISPLAY


struct vpTemplateTrackerZPoint {
    int x,y;
};
struct vpTemplateTrackerDPoint {
    double x,y;
};
struct vpTemplateTrackerPoint {
    int x,y;
    double dx,dy;
    double val;
    double *dW;
    double *HiG;

    vpTemplateTrackerPoint()
    {
      dW = NULL;
      HiG = NULL;
    }
};
struct vpTemplateTrackerPointCompo {
    double *dW;
    vpTemplateTrackerPointCompo()
    {
      dW = NULL;
    }
};
struct vpTemplateTrackerPointSuppMIInv {
    double et;
    int ct;
    double *Bt;
    double *dBt;
    double *d2W;
    double *d2Wx;
    double *d2Wy;
    vpTemplateTrackerPointSuppMIInv()
    {
      Bt = NULL;
      dBt = NULL;
      d2W = NULL;
      d2Wx = NULL;
      d2Wy = NULL;
    }
};


#endif
