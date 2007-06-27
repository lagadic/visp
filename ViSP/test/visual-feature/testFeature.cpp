/****************************************************************************
 *
 * $Id: testFeature.cpp,v 1.3 2007-06-27 14:39:37 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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

int main()
{
  try {
    for (int i=0; i < 3; i++) {
      vpServo task ;

      vpThetaUVector tuv;
      tuv[0] =0.1;
      tuv[1] =0.2;
      tuv[2] =0.3;

      vpFeatureThetaU tu;
      tu.buildFrom(tuv);

      task.addFeature(tu);

      task.kill();

      tu.print();
      vpTRACE("End, call destructors...");
    }
    return 0;
  }
  catch(vpServoException e) {
    std::cout << e << std::endl;
    return -1;
  }
}
