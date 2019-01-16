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
 * Tests some vpMatrix functionalities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testTwistMatrix.cpp

  \brief Test some vpMatrix functionalities.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

int main()
{
  try {
    vpTRACE("--------------------------");
    vpTRACE("--- TEST vpVelocityTwistMatrix ---");
    vpTRACE("--------------------------");

    // Set the translation
    vpTranslationVector cte;
    cte[0] = 1.;
    cte[1] = 0.5;
    cte[2] = -1.;

    // Set the rotation
    vpRxyzVector cre;
    cre[0] = M_PI / 2.;
    cre[1] = -M_PI / 2.;
    cre[2] = -M_PI / 4.;

    // Build rotation matrix
    vpRotationMatrix cRe(cre);

    // Build the twist matrix
    vpVelocityTwistMatrix cVe(cte, cRe);

    vpTRACE("cVe twist matrix:");
    cVe.print(std::cout, 6);

    // Set a speed skew
    vpColVector ev(6);

    ev[0] = 1.;
    ev[1] = 0.1;
    ev[2] = -0.5;
    ev[3] = M_PI / 180.;
    ev[4] = M_PI / 18.;
    ev[5] = M_PI / 10.;

    vpTRACE("ev colvector:");
    ev.print(std::cout, 6);

    // Set a speed skew
    vpColVector cv;

    cv = cVe * ev;

    vpTRACE("cv = cVe * ev:");
    cv.print(std::cout, 6);
    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
