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
 * Example of template tracking.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpTemplateTrackerMIESM_hh
#define vpTemplateTrackerMIESM_hh

#include <visp3/core/vpImageFilter.h>
#include <visp3/tt/vpTemplateTracker.h>
#include <visp3/tt/vpTemplateTrackerHeader.h>

#include <visp3/tt_mi/vpTemplateTrackerMI.h>
#include <visp3/tt_mi/vpTemplateTrackerMIBSpline.h>

/*!
  \class vpTemplateTrackerMIESM
  \ingroup group_tt_mi_tracker
*/
class VISP_EXPORT vpTemplateTrackerMIESM : public vpTemplateTrackerMI
{
  /*! Minimization method. */
  typedef enum {
    USE_NEWTON, // not used
    USE_LMA,    // not used
    USE_GRADIENT,
    USE_QUASINEWTON // not used => see default equivalence
  } vpMinimizationTypeMIESM;

protected:
  vpMinimizationTypeMIESM minimizationMethod;
  bool CompoInitialised;
  vpMatrix HDirect;
  vpMatrix HInverse;
  vpMatrix HdesireDirect;
  vpMatrix HdesireInverse;
  vpColVector GDirect;
  vpColVector GInverse;

protected:
  void initCompInverse();
  void initHessienDesired(const vpImage<unsigned char> &I);
  void trackNoPyr(const vpImage<unsigned char> &I);

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  vpTemplateTrackerMIESM(const vpTemplateTrackerMIESM &)
  //    : vpTemplateTrackerMI(), minimizationMethod(USE_NEWTON),
  //    CompoInitialised(false),
  //      HDirect(), HInverse(), HdesireDirect(), HdesireInverse(), GDirect(),
  //      GInverse()
  //  {
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!");
  //  }
  //  vpTemplateTrackerMIESM &operator=(const vpTemplateTrackerMIESM &){
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!"); return *this;
  //  }
  //#endif

public:
  //! Default constructor.
  vpTemplateTrackerMIESM()
    : vpTemplateTrackerMI(), minimizationMethod(USE_NEWTON), CompoInitialised(false), HDirect(), HInverse(),
      HdesireDirect(), HdesireInverse(), GDirect(), GInverse()
  {
  }
  explicit vpTemplateTrackerMIESM(vpTemplateTrackerWarp *_warp);

  void setMinimizationMethod(vpMinimizationTypeMIESM method) { minimizationMethod = method; }
};

#endif
