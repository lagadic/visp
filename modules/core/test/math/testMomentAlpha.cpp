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

#include <string>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentAlpha.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/io/vpImageIo.h>

int test_moment_alpha(const std::string &name, bool symmetry, const std::vector<int> &vec_angle, double tolerance_deg, double symmetry_threshold=1e-6)
{
  vpImage<unsigned char> I;

  std::cout << "** Test " << (symmetry == true ? "symmetric " : "non symmetric ") << name << " object" << std::endl;

  // ***************
  std::cout << "*** Test symmetry detection from mu 3rd order moments" << std::endl;
  // ***************
  std::vector<double> mu_ref;
  double alpha_ref = 0.;
  for(unsigned int i = (unsigned int)vec_angle.size(); i >= 1; --i) {
    // Compute reference alpha image <name>-<vec_angle>[i]deg.pgm
    std::stringstream ss;
    ss << name << "-" << vec_angle[i-1] << "deg.pgm";
    std::cout << "Process image " << ss.str() << std::endl;
    vpImageIo::read(I, ss.str());

    // Consider the case of a reference alpha
    {
      vpMomentObject obj(3);
      obj.setType(vpMomentObject::DENSE_FULL_OBJECT);
      obj.fromImage(I, 127, vpCameraParameters());                     // Init the dense object with the image and corresponding camera parameters
      vpMomentDatabase db;                            // Database
      vpMomentGravityCenter mg;                       // Declaration of gravity center moment
      vpMomentCentered mc;                            // Declaration of centered moments
      vpMomentAlpha malpha_ref;                       // Declaration of alpha reference moments
      mg.linkTo(db);                                  // Add gravity center moment to database
      mc.linkTo(db);                                  // Add centered moments
      malpha_ref.linkTo(db);                          // Add alpha moment
      db.updateAll(obj);                              // All of the moments must be updated, not just alpha
      mg.compute();                                   // Compute gravity center moment
      mc.compute();                                   // Compute centered moments AFTER gravity center
      malpha_ref.compute();                           // Compute alpha gravity center

      mu_ref.clear();
      mu_ref.push_back(mc.get(3,0));
      mu_ref.push_back(mc.get(2,1));
      mu_ref.push_back(mc.get(1,2));
      mu_ref.push_back(mc.get(0,3));
      alpha_ref = malpha_ref.get();
    }
    // Consider the case of a relative alpha
    {
      vpMomentObject obj(3);
      obj.setType(vpMomentObject::DENSE_FULL_OBJECT);
      obj.fromImage(I, 127, vpCameraParameters());                     // Init the dense object with the image and corresponding camera parameters
      vpMomentDatabase db;                            // Database
      vpMomentGravityCenter mg;                       // Declaration of gravity center moment
      vpMomentCentered mc;                            // Declaration of centered moments
      vpMomentAlpha malpha(mu_ref, alpha_ref, symmetry_threshold);        // Declaration of alpha relative moments
      mg.linkTo(db);                                  // Add gravity center moment to database
      mc.linkTo(db);                                  // Add centered moments
      malpha.linkTo(db);                              // Add alpha moment
      db.updateAll(obj);                              // All of the moments must be updated, not just alpha
      mg.compute();                                   // Compute gravity center moment
      mc.compute();                                   // Compute centered moments AFTER gravity center
      malpha.compute();                               // Compute alpha gravity center

      if (malpha.is_symmetric() != symmetry) {
        std::cout << "Error in symmety detection" << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  // ***************
  std::cout << "*** Compute angle in relative mode using the last reference from the previous test" << std::endl;
  // ***************
  for(size_t i = 0; i < vec_angle.size(); i++) {
    std::stringstream ss;
    ss << name << "-" << vec_angle[i] << "deg.pgm";
    vpImage<unsigned char> I;
    std::cout << "Process image " << ss.str() << std::endl;
    vpImageIo::read(I, ss.str());

    vpMomentObject obj(3);
    obj.setType(vpMomentObject::DENSE_FULL_OBJECT);
    obj.fromImage(I, 127, vpCameraParameters());      // Init the dense object with the image
    vpMomentDatabase db;                              // Database
    vpMomentGravityCenter g;                          // Declaration of gravity center
    vpMomentCentered mc;                              // Centered moments
    vpMomentAlpha malpha(mu_ref, alpha_ref, symmetry_threshold);          // Alpha moment relative to the reference alpha
    g.linkTo(db);                                     // Add gravity center to database
    mc.linkTo(db);                                    // Add centered moments
    malpha.linkTo(db);                                // Add alpha depending on centered moments
    db.updateAll(obj);                                // All of the moments must be updated, not just alpha
    g.compute();                                      // Compute the moment
    mc.compute();                                     // Compute centered moments AFTER gravity center
    malpha.compute();                                 // Compute alpha AFTER centered moments.

    if (! symmetry) {
      // Tranform input angle from [0; 360] to [-180; +180] range
      double angle = vec_angle[i];
      if (angle > 180)
        angle -= 360;
      if (angle < -180)
        angle += 360;

      std::cout << "alpha expected " << angle << " computed " << vpMath::deg(malpha.get()) << " deg" << std::endl;

      if (! vpMath::equal(angle, vpMath::deg(malpha.get()), tolerance_deg)) { // 0.5 deg of tolerance
        std::cout << "Error: result is not in the tolerance: " << tolerance_deg << std::endl;
        return EXIT_FAILURE;
      }
    }
    else {
      // Tranform input angle from [0; 360] to [0; 180] range
      double angle_des1 = vec_angle[i];
      double angle_des2 = vec_angle[i] - 180;

      // Tranform input angle from [0; 360] to [0; 180] range
      double alpha = vpMath::deg(malpha.get());

      std::cout << "alpha expected " << angle_des1 << " or " << angle_des2 << " computed " << alpha << " deg" << std::endl;

      if (! vpMath::equal(angle_des1, alpha, tolerance_deg) && ! vpMath::equal(angle_des2, alpha, tolerance_deg)) { // 0.5 deg of tolerance
        std::cout << "Error: result is not in the tolerance: " << tolerance_deg << std::endl;
        return EXIT_FAILURE;
      }
    }
  }
  std::cout << "Test succeed" << std::endl;
  return EXIT_SUCCESS;
}

int main()
{
  std::string name;
  bool symmetry;
  double tolerance_deg;
  std::vector<int> vec_angle;
  double symmetry_threshold;

  // *******************************
  // Test arrow
  // *******************************
  name = "arrow";
  symmetry = false;
  tolerance_deg = 0.5;
  vec_angle.clear();
  vec_angle.push_back(0);
  vec_angle.push_back(45);
  vec_angle.push_back(90);
  vec_angle.push_back(135);
  vec_angle.push_back(180);
  vec_angle.push_back(225);
  vec_angle.push_back(270);
  vec_angle.push_back(315);

  if (test_moment_alpha(name, symmetry, vec_angle, tolerance_deg) == EXIT_FAILURE) {
    return EXIT_FAILURE;
  }

  // *******************************
  // Test ellipse created with gimp
  // *******************************
  name = "ellipse";
  symmetry = true;
  tolerance_deg = 0.5;
  vec_angle.clear();
  vec_angle.push_back(0);
  vec_angle.push_back(45);
  vec_angle.push_back(90);
  vec_angle.push_back(135);

  if (test_moment_alpha(name, symmetry, vec_angle, tolerance_deg) == EXIT_FAILURE) {
    return EXIT_FAILURE;
  }

  // *******************************
  // Test ellipse created with xfig
  // *******************************
  name = "ellipse-xfig";
  symmetry = true;
  tolerance_deg = 2.5;
  symmetry_threshold = 1e-2; // Modify default value
  vec_angle.clear();
  vec_angle.push_back(0);
  vec_angle.push_back(45);
  vec_angle.push_back(90);
  vec_angle.push_back(135);

  if (test_moment_alpha(name, symmetry, vec_angle, tolerance_deg, symmetry_threshold) == EXIT_FAILURE) {
    return EXIT_FAILURE;
  }

  // *******************************
  // Test baleine created with gimp
  // *******************************
  name = "baleine";
  symmetry = false;
  tolerance_deg = 5.;
  vec_angle.clear();
  vec_angle.push_back(0);
  vec_angle.push_back(45);
  vec_angle.push_back(90);
  vec_angle.push_back(135);
  vec_angle.push_back(180);
  vec_angle.push_back(225);
  vec_angle.push_back(270);
  vec_angle.push_back(315);

  if (test_moment_alpha(name, symmetry, vec_angle, tolerance_deg) == EXIT_FAILURE) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
