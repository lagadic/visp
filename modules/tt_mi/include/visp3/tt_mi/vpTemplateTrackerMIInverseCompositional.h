/****************************************************************************
 *
 * $Id: templateTracker.cpp 4428 2013-09-06 13:51:34Z fspindle $
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
 * Example of template tracking.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpTemplateTrackerMIInverseCompositional_hh
#define vpTemplateTrackerMIInverseCompositional_hh

#include <visp3/core/vpConfig.h>

#include <visp3/tt/vpTemplateTracker.h>
#include <visp3/tt/vpTemplateTrackerHeader.h>
#include <visp3/core/vpImageFilter.h>

#include <visp3/tt_mi/vpTemplateTrackerMI.h>
#include <visp3/tt_mi/vpTemplateTrackerMIBSpline.h>

class VISP_EXPORT vpTemplateTrackerMIInverseCompositional: public vpTemplateTrackerMI
{
public:
  /*! Minimization method. */
  typedef enum {
    USE_NEWTON,
    USE_LMA,
    USE_GRADIENT,
    USE_QUASINEWTON
  } vpMinimizationTypeMIInverseCompositional;

private:
  vpMinimizationTypeMIInverseCompositional minimizationMethod;
  bool    CompoInitialised;
  bool    useTemplateSelect;//use only the strong gradient pixels to compute the Jabocian
  //pour eval evolRMS
  double  evolRMS;
  double  *x_pos;
  double  *y_pos;
  double  threshold_RMS;
  //valeur pour calculer Quasi_Newton
  vpColVector p_prec;
  vpColVector G_prec;
  vpMatrix    KQuasiNewton;

  bool    useAYOptim;
  bool    useOpenMP;

public: // AY Optimisation
  void initTemplateRefBspline(unsigned int ptIndex, double &et);

protected:
  void initCompInverse(const vpImage<unsigned char> &I);
  void initHessienDesired(const vpImage<unsigned char> &I);
  void trackNoPyr(const vpImage<unsigned char> &I);
  void deletePosEvalRMS();
  void computeEvalRMS(const vpColVector &p);
  void initPosEvalRMS(vpColVector &p);

public:
  vpTemplateTrackerMIInverseCompositional(vpTemplateTrackerWarp *_warp);

  /*! Use only the strong gradient pixels to compute the Jabobian. By default this feature is disabled. */
  void  setUseTemplateSelect(bool b) {useTemplateSelect = b;}
  void  setThresholdRMS(double threshold){threshold_RMS=threshold;}
  void  setMinimizationMethod(vpMinimizationTypeMIInverseCompositional method){minimizationMethod=method;}
};
#endif

