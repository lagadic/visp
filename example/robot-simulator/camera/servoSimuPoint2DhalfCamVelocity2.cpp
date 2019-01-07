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
 * Simulation of a 2 1/2 D visual servoing using theta U visual features.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example servoSimuPoint2DhalfCamVelocity2.cpp
  Simulation of a 2 1/2 D visual servoing (x,y,log Z, theta U)
  - (x,y,logZ, theta U) features
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - no display.
*/

#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpGenericFeature.h>
#include <visp3/vs/vpServo.h>

// List of allowed command line options
#define GETOPTARGS "h"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv);

/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Simulation of a 2 1/2 D visual servoing (x,y,log Z, theta U):\n\
- eye-in-hand control law,\n\
- velocity computed in the camera frame,\n\
- without display.\n\
          \n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
                  \n\
 -h\n\
    Print the help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'h':
      usage(argv[0], NULL);
      return false;
      break;

    default:
      usage(argv[0], optarg_);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    // Read the command line options
    if (getOptions(argc, argv) == false) {
      exit(-1);
    }

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " simulation of a 2 1/2 D visual servoing " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    // In this example we will simulate a visual servoing task.
    // In simulation, we have to define the scene frane Ro and the
    // camera frame Rc.
    // The camera location is given by an homogenous matrix cMo that
    // describes the position of the camera in the scene frame.

    vpServo task;

    // sets the initial camera location
    // we give the camera location as a size 6 vector (3 translations in meter
    // and 3 rotation (theta U representation)
    vpPoseVector c_r_o(0.1, 0.2, 2, vpMath::rad(20), vpMath::rad(10), vpMath::rad(50));

    // this pose vector is then transformed in a 4x4 homogeneous matrix
    vpHomogeneousMatrix cMo(c_r_o);

    // We define a robot
    // The vpSimulatorCamera implements a simple moving that is juste defined
    // by its location cMo
    vpSimulatorCamera robot;

    // Compute the position of the object in the world frame
    vpHomogeneousMatrix wMc, wMo;
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    // Now that the current camera position has been defined,
    // let us defined the defined camera location.
    // It is defined by cdMo
    // sets the desired camera location
    vpPoseVector cd_r_o(0, 0, 1, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    vpHomogeneousMatrix cdMo(cd_r_o);

    //----------------------------------------------------------------------
    // A 2 1/2 D visual servoing can be defined by
    // - the position of a point x,y
    // - the difference between this point depth and a desire depth
    //   modeled by log Z/Zd to be regulated to 0
    // - the rotation that the camera has to realized cdMc

    // Let us now defined the current value of these features

    // since we simulate we have to define a 3D point that will
    // forward-projected to define the current position x,y of the
    // reference point

    //------------------------------------------------------------------
    // First feature (x,y)
    //------------------------------------------------------------------
    // Let oP be this ... point,
    // a vpPoint class has three main member
    // .oP : 3D coordinates in scene frame
    // .cP : 3D coordinates in camera frame
    // .p : 2D

    //------------------------------------------------------------------
    // sets the point coordinates in the world frame
    vpPoint point(0, 0, 0);
    // computes  the point coordinates in the camera frame and its
    // 2D coordinates cP and then p
    // computes the point coordinates in the camera frame and its 2D
    // coordinates"  ) ;
    point.track(cMo);

    // We also defined (again by forward projection) the desired position
    // of this point according to the desired camera position
    vpPoint pointd(0, 0, 0);
    pointd.track(cdMo);

    // Nevertheless, a vpPoint is not a feature, this is just a "tracker"
    // from which the feature are built
    // a feature is juste defined by a vector s, a way to compute the
    // interaction matrix and the error, and if required a (or a vector of)
    // 3D information

    // for a point (x,y) Visp implements the vpFeaturePoint class.
    // we no defined a feature for x,y (and for (x*,y*))
    vpFeaturePoint p, pd;

    // and we initialized the vector s=(x,y) of p from the tracker P
    // Z coordinates in p is also initialized, it will be used to compute
    // the interaction matrix
    vpFeatureBuilder::create(p, point);
    vpFeatureBuilder::create(pd, pointd);

    //------------------------------------------------------------------
    // Second feature log (Z/Zd)
    // not necessary to project twice (reuse p)

    // This case in intersting since this visual feature has not
    // been predefined in VisP
    // In such case we have a generic feature class vpGenericFeature
    // We will have to defined
    // the vector s : .set_s(...)
    // the interaction matrix Ls : .setInteractionMatrix(...)

    // log(Z/Zd) is then a size 1 vector logZ
    vpGenericFeature logZ(1);
    // initialized to s = log(Z/Zd)
    // Let us note that here we use the point P and Pd, it's not necessary
    // to forward project twice (it's already done)
    logZ.set_s(log(point.get_Z() / pointd.get_Z()));

    // This visual has to be regulated to zero

    //------------------------------------------------------------------
    // 3rd feature ThetaU
    // The thetaU feature is defined, tu represents the rotation that the
    // camera has to realized. the complete displacement is then defined by:
    //------------------------------------------------------------------
    vpHomogeneousMatrix cdMc;
    // compute the rotation that the camera has to achieve
    cdMc = cdMo * cMo.inverse();

    // from this displacement, we extract the rotation cdRc represented by
    // the angle theta and the rotation axis u
    vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
    tu.buildFrom(cdMc);
    // This visual has to be regulated to zero

    // sets the desired rotation (always zero !)
    // since s is the rotation that the camera has to realize

    //------------------------------------------------------------------
    // Let us now the task itself
    //------------------------------------------------------------------

    // define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    //  we choose to control the robot in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA);
    // Interaction matrix is computed with the current value of s
    task.setInteractionMatrixType(vpServo::CURRENT);

    // we build the task by "stacking" the visual feature
    // previously defined
    task.addFeature(p, pd);
    task.addFeature(logZ);
    task.addFeature(tu);
    // addFeature(X,Xd) means X should be regulated to Xd
    // addFeature(X) means that X should be regulated to 0
    // some features such as vpFeatureThetaU MUST be regulated to zero
    // (otherwise, it will results in an error at exectution level)

    // set the gain
    task.setLambda(1);

    // Display task information
    task.print();
    //------------------------------------------------------------------
    // An now the closed loop

    unsigned int iter = 0;
    // loop
    while (iter++ < 200) {
      std::cout << "---------------------------------------------" << iter << std::endl;
      vpColVector v;

      // get the robot position
      robot.getPosition(wMc);
      // Compute the position of the camera wrt the object frame
      cMo = wMc.inverse() * wMo;

      // update the feature
      point.track(cMo);
      vpFeatureBuilder::create(p, point);

      cdMc = cdMo * cMo.inverse();
      tu.buildFrom(cdMc);

      // there is no feature for logZ, we explicitely build
      // the related interaction matrix") ;
      logZ.set_s(log(point.get_Z() / pointd.get_Z()));
      vpMatrix LlogZ(1, 6);
      LlogZ[0][0] = LlogZ[0][1] = LlogZ[0][5] = 0;
      LlogZ[0][2] = -1 / p.get_Z();
      LlogZ[0][3] = -p.get_y();
      LlogZ[0][4] = p.get_x();

      logZ.setInteractionMatrix(LlogZ);

      // compute the control law
      v = task.computeControlLaw();

      // send the camera velocity to the controller ") ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;
    }

    // Display task information
    task.print();
    task.kill();
    // Final camera location
    std::cout << cMo << std::endl;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
