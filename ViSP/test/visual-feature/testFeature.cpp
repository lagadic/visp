/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
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
 *
 * Description:
 * Visual feature manipulation.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <iostream>


#include <visp/vpColVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpGenericFeature.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpDot2.h>
#include <visp/vpCameraParameters.h>

/*!

  \example testFeature.cpp

  Shows how to build a task with a \f$ \theta u \f$ visual feature.

*/
int main()
{
  try {
    for (int i=0; i < 3; i++) {
      vpServo task ;

      // Creation od a Theta U vector that represent the rotation
      // between the desired camera frame and the current one.
      vpThetaUVector tu_cdRc; // Current visual feature s
      tu_cdRc[0] =0.1;
      tu_cdRc[1] =0.2;
      tu_cdRc[2] =0.3;

      // Creation of the current feature s
      vpFeatureThetaU s(vpFeatureThetaU::cdRc);
      s.buildFrom(tu_cdRc);
      s.print();
      task.addFeature(s); // Add current ThetaU feature

      // Creation of the desired feature s^*
      vpFeatureThetaU s_star(vpFeatureThetaU::cdRc); // init to zero

      // Compute the interaction matrix for the ThetaU_z feature
      vpMatrix L_z =  s.interaction( vpFeatureThetaU::selectTUz() );
      // Compute the error vector (s-s^*) for the ThetaU_z feature
      s.error(s_star, vpFeatureThetaU::selectTUz());
      
      // A call to kill() is requested here to destroy properly the current
      // and desired feature lists.
      task.kill();

      vpTRACE("End, call destructors...");
    }
    return 0;
  }
  catch(vpServoException e) {
    std::cout << e << std::endl;
    return -1;
  }
}
