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
 * Visual feature manipulation.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpGenericFeature.h>
#include <visp3/vs/vpServo.h>

#include <iostream>

/*!

  \example testFeature.cpp

  Shows how to build a task with a \f$ \theta u \f$ visual feature.

*/
int main()
{
  try {
    for (int i = 0; i < 3; i++) {
      vpServo task;

      // Creation od a Theta U vector that represent the rotation
      // between the desired camera frame and the current one.
      vpThetaUVector tu_cdRc; // Current visual feature s
      tu_cdRc[0] = 0.1;
      tu_cdRc[1] = 0.2;
      tu_cdRc[2] = 0.3;

      // Creation of the current feature s
      vpFeatureThetaU s(vpFeatureThetaU::cdRc);
      s.buildFrom(tu_cdRc);
      s.print();
      task.addFeature(s); // Add current ThetaU feature

      // Creation of the desired feature s^*
      vpFeatureThetaU s_star(vpFeatureThetaU::cdRc); // init to zero

      // Compute the interaction matrix for the ThetaU_z feature
      vpMatrix L_z = s.interaction(vpFeatureThetaU::selectTUz());
      // Compute the error vector (s-s^*) for the ThetaU_z feature
      s.error(s_star, vpFeatureThetaU::selectTUz());

      // A call to kill() is requested here to destroy properly the current
      // and desired feature lists.
      task.kill();

      std::cout << "End, call vpServo destructors..." << std::endl;
    }
    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
