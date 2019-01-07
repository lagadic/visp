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
 * Example of visual servoing with moments using a polygon as object container
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

/*!
  \example manServoMomentsSimple.cpp
  Minimalist example of moment-based visual servoing with polygon and a simple
  robot
*/

#include <visp3/core/vpPoint.h> //the basic tracker

#include <iostream> //some console output
#include <limits>
#include <vector> //store the polygon
#include <visp3/core/vpException.h>
#include <visp3/core/vpMomentCommon.h> //update the common database with the object
#include <visp3/core/vpMomentObject.h> //transmit the polygon to the object
#include <visp3/core/vpPlane.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureMomentCommon.h> //init the feature database using the information about moment dependencies
#include <visp3/vs/vpServo.h>                            //visual servoing task
// this function converts the plane defined by the cMo to 1/Z=Ax+By+C plane
// form

void cMoToABC(vpHomogeneousMatrix &cMo, double &A, double &B, double &C);

void cMoToABC(vpHomogeneousMatrix &cMo, double &A, double &B, double &C)
{
  vpPlane pl;
  pl.setABCD(0, 0, 1.0, 0);
  pl.changeFrame(cMo);

  if (fabs(pl.getD()) < std::numeric_limits<double>::epsilon()) {
    std::cout << "Invalid position:" << std::endl;
    std::cout << cMo << std::endl;
    std::cout << "Cannot put plane in the form 1/Z=Ax+By+C." << std::endl;
    throw vpException(vpException::divideByZeroError, "invalid position!");
  }
  A = -pl.getA() / pl.getD();
  B = -pl.getB() / pl.getD();
  C = -pl.getC() / pl.getD();
}

int main()
{
  try {
    double x[8] = {1, 3, 4, -1, -3, -2, -1, 1};
    double y[8] = {0, 1, 4, 4, -2, -2, 1, 0};
    double A, B, C, Ad, Bd, Cd;

    int nbpoints = 8;
    std::vector<vpPoint> vec_p,
        vec_p_d; // vectors that contain the vertices of the contour polygon

    vpHomogeneousMatrix cMo(0.1, 0.0, 1.0, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    vpHomogeneousMatrix cdMo(vpHomogeneousMatrix(0.0, 0.0, 1.0, vpMath::rad(0), vpMath::rad(0), -vpMath::rad(0)));
    vpHomogeneousMatrix wMo; // Set to identity
    vpHomogeneousMatrix wMc; // Camera position in the world frame

    cMoToABC(cMo, A, B, C);
    cMoToABC(cdMo, Ad, Bd, Cd);
    // Define source and destination polygons
    for (int i = 0; i < nbpoints; i++) {
      vpPoint p(x[i], y[i], 0.0);
      p.track(cMo);
      vec_p.push_back(p);
      p.track(cdMo);
      vec_p_d.push_back(p);
    }

    vpMomentObject cur(6);                      // Create a source moment object with 6 as maximum order
    cur.setType(vpMomentObject::DENSE_POLYGON); // The object is defined by a
                                                // countour polygon
    cur.fromVector(vec_p);                      // Init the dense object with the source polygon

    vpMomentObject dst(6);                      // Create a destination moment object with 6 as maximum order
    dst.setType(vpMomentObject::DENSE_POLYGON); // The object is defined by a
                                                // countour polygon
    dst.fromVector(vec_p_d);                    // Init the dense object with the destination polygon

    // init classic moment primitives (for source)
    vpMomentCommon mdb_cur(vpMomentCommon::getSurface(dst), vpMomentCommon::getMu3(dst),
                           vpMomentCommon::getAlpha(dst)); // Init classic features
    vpFeatureMomentCommon fmdb_cur(mdb_cur);

    ////init classic moment primitives (for destination)
    vpMomentCommon mdb_dst(vpMomentCommon::getSurface(dst), vpMomentCommon::getMu3(dst),
                           vpMomentCommon::getAlpha(dst)); // Init classic features
    vpFeatureMomentCommon fmdb_dst(mdb_dst);

    // update+compute moment primitives from object (for destination)
    mdb_dst.updateAll(dst);
    // update+compute features (+interaction matrixes) from plane
    fmdb_dst.updateAll(Ad, Bd, Cd);

    // define visual servoing task
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(1);

    task.addFeature(fmdb_cur.getFeatureGravityNormalized(), fmdb_dst.getFeatureGravityNormalized());
    task.addFeature(fmdb_cur.getFeatureAn(), fmdb_dst.getFeatureAn());
    // the object is NOT symmetric
    // select C4 and C6
    task.addFeature(fmdb_cur.getFeatureCInvariant(), fmdb_dst.getFeatureCInvariant(),
                    vpFeatureMomentCInvariant::selectC4() | vpFeatureMomentCInvariant::selectC6());
    task.addFeature(fmdb_cur.getFeatureAlpha(), fmdb_dst.getFeatureAlpha());

    vpBasicFeature *al = new vpFeatureMomentAlpha(mdb_dst, 0, 0, 1.);
    al->init();
    al->error(*al);
    // param robot
    vpSimulatorCamera robot;
    float sampling_time = 0.010f; // Sampling period in seconds
    robot.setSamplingTime(sampling_time);
    wMc = wMo * cMo.inverse();
    robot.setPosition(wMc);

    do {
      wMc = robot.getPosition();
      cMo = wMc.inverse() * wMo;
      vec_p.clear();

      for (int i = 0; i < nbpoints; i++) {
        vpPoint p(x[i], y[i], 0.0);
        p.track(cMo);
        vec_p.push_back(p);
      }
      cMoToABC(cMo, A, B, C);

      cur.fromVector(vec_p);
      // update+compute moment primitives from object (for source)
      mdb_cur.updateAll(cur);
      // update+compute features (+interaction matrixes) from plane
      fmdb_cur.updateAll(A, B, C);

      vpColVector v = task.computeControlLaw();
      task.print();
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);
      double t = vpTime::measureTimeMs();
      vpTime::wait(t, sampling_time * 1000); // Wait 10 ms
    } while ((task.getError()).sumSquare() > 0.005);
    std::cout << "final error=" << (task.getError()).sumSquare() << std::endl;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
