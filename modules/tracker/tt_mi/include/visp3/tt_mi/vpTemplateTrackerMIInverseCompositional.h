/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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

#include <visp3/core/vpImageFilter.h>
#include <visp3/tt/vpTemplateTracker.h>
#include <visp3/tt/vpTemplateTrackerHeader.h>

#include <visp3/tt_mi/vpTemplateTrackerMI.h>
#include <visp3/tt_mi/vpTemplateTrackerMIBSpline.h>

/*!
  \class vpTemplateTrackerMIInverseCompositional
  \ingroup group_tt_mi_tracker
  The algorithm implemented in this class is described in \cite Dame12a and
  \cite Marchand16a.
*/
class VISP_EXPORT vpTemplateTrackerMIInverseCompositional : public vpTemplateTrackerMI
{
public:
  /*! Minimization method. */
  typedef enum { USE_NEWTON, USE_LMA, USE_GRADIENT, USE_QUASINEWTON } vpMinimizationTypeMIInverseCompositional;

private:
  vpMinimizationTypeMIInverseCompositional minimizationMethod;
  bool CompoInitialised;
  bool useTemplateSelect; // use only the strong gradient pixels to compute
                          // the Jabocian
  // valeur pour calculer Quasi_Newton
  vpColVector p_prec;
  vpColVector G_prec;
  vpMatrix KQuasiNewton;

  // bool    useAYOptim;

public: // AY Optimisation
  void initTemplateRefBspline(unsigned int ptIndex, double &et);

protected:
  void initCompInverse(const vpImage<unsigned char> &I);
  void initHessienDesired(const vpImage<unsigned char> &I);
  void trackNoPyr(const vpImage<unsigned char> &I);

public:
  //! Default constructor.
  vpTemplateTrackerMIInverseCompositional()
    : vpTemplateTrackerMI(), minimizationMethod(USE_LMA), CompoInitialised(false), useTemplateSelect(false),
      p_prec(), G_prec(), KQuasiNewton()
  {
  }
  explicit vpTemplateTrackerMIInverseCompositional(vpTemplateTrackerWarp *_warp);

  /*! Use only the strong gradient pixels to compute the Jabobian. By default
   * this feature is disabled. */
  void setUseTemplateSelect(bool b) { useTemplateSelect = b; }
  void setMinimizationMethod(vpMinimizationTypeMIInverseCompositional method) { minimizationMethod = method; }
};
#endif
