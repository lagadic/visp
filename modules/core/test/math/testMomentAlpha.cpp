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
 * Test some vpMomentAlpha functionalities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testMomentAlpha.cpp

  Test for vpMomentAlpha class.
*/

#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentAlpha.h>
#include <visp3/io/vpImageIo.h>

int main()
{
  std::vector<int> vec_angle;
  vec_angle.push_back(0);
  vec_angle.push_back(45);
  vec_angle.push_back(90);
  vec_angle.push_back(135);
  vec_angle.push_back(180);
  vec_angle.push_back(225);
  vec_angle.push_back(270);
  vec_angle.push_back(315);

  vpImage<unsigned char> I;

  // Compure reference alpha for image arror-0deg.pgm
  vpImageIo::read(I, "arrow-0deg.pgm");
  vpMomentObject obj(3);
  obj.setType(vpMomentObject::DENSE_FULL_OBJECT);
  obj.fromImage(I, 127, vpCameraParameters());    // Init the dense object with the image
  vpMomentDatabase db;                            // Database
  vpMomentGravityCenter g;                        // Declaration of gravity center
  vpMomentCentered mc;                            // Centered moments
  vpMomentAlpha malpha_ref;                       // Alpha reference moments
  g.linkTo(db);                                   // Add gravity center to database
  mc.linkTo(db);                                  // Add centered moments
  malpha_ref.linkTo(db);                          // Add alpha moment
  db.updateAll(obj);                              // All of the moments must be updated, not just alpha
  g.compute();                                    // Compute the moment
  mc.compute();                                   // Compute centered moments AFTER gravity center
  malpha_ref.compute();                           // Compute centered moments AFTER gravity center

  std::vector<double> mu_ref = {mc.get(3,0), mc.get(2,1), mc.get(1,2), mc.get(0,3)};
  double alpha_ref = malpha_ref.get();

  for(size_t i = 0; i < vec_angle.size(); i++) {
    std::stringstream ss;
    ss << "arrow-" << vec_angle[i] << "deg.pgm";
    vpImage<unsigned char> I;
    //std::cout << "Process image " << ss.str() << std::endl;
    vpImageIo::read(I, ss.str());

    vpMomentObject obj(3);
    obj.setType(vpMomentObject::DENSE_FULL_OBJECT);
    obj.fromImage(I, 127, vpCameraParameters());  // Init the dense object with the image
    vpMomentDatabase db;                          // Database
    vpMomentGravityCenter g;                      // Declaration of gravity center
    vpMomentCentered mc;                          // Centered moments
    vpMomentAlpha malpha(mu_ref, alpha_ref);      // Alpha moment
    g.linkTo(db);                                 // Add gravity center to database
    mc.linkTo(db);                                // Add centered moments
    malpha.linkTo(db);                            // Add alpha depending on centered moments
    db.updateAll(obj);                            // All of the moments must be updated, not just alpha
    g.compute();                                  // Compute the moment
    mc.compute();                                 // Compute centered moments AFTER gravity center
    malpha.compute();                             // Compute alpha AFTER centered moments.

    // Tranform input angle from [0; 360] to [-180; +180] range
    double angle = vec_angle[i];
    if (angle > 180)
      angle -= 360;
    if (angle < -180)
      angle += 360;

    std::cout << "alpha expected " << angle << " computed " << vpMath::deg(malpha.get()) << " deg" << std::endl;

    double tolerance_deg = 0.5;
    if (! vpMath::equal(angle, vpMath::deg(malpha.get()), tolerance_deg)) { // 0.5 deg of tolerance
      std::cout << "Error: result is not in the tolerance: " << tolerance_deg << std::endl;
      return EXIT_FAILURE;
    }
  }
  std::cout << "Test succeed" << std::endl;
  return EXIT_SUCCESS;
}
