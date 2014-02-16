/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 \file vpTemplateTrackerZNCC.h
 \brief
*/

#ifndef vpTemplateTrackerZNCC_hh
#define vpTemplateTrackerZNCC_hh

#include <math.h>

#include <visp/vpTemplateTracker.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpImageTools.h>
#include <visp/vpIoTools.h>
#include <visp/vpImageTools.h>
#include <visp/vpImageFilter.h>
#include <visp/vpMath.h>
#include <visp/vpList.h>
#include <visp/vpHomography.h>

#define APPROX_NCC

class VISP_EXPORT vpTemplateTrackerZNCC: public vpTemplateTracker
{
  protected:
    vpRowVector   DI;
    vpRowVector   temp;

  protected:
            double getCost(const vpImage<unsigned char> &I, vpColVector &tp) ;
            double getCost(const vpImage<unsigned char> &I) {vpColVector tp; return getCost(I,tp);}
    virtual void   initHessienDesired(const vpImage<unsigned char> &I)=0;
    virtual void   trackNoPyr(const vpImage<unsigned char> &I)=0;

  public:
    vpTemplateTrackerZNCC(vpTemplateTrackerWarp *warp);

    void   setGain(double _gain){gain=_gain;}
};
#endif

