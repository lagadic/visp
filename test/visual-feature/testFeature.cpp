/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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
