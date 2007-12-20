/****************************************************************************
 *
 * $Id: servoAfma6FourPoints2DCamVelocityInteractionCurrent.cpp,v 1.1 2007-12-20 15:43:39 fspindle Exp $
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
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file servoAfma6FourPoints2DCamVelocityInteractionCurrent.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in the camera frame.  Visual features are the image coordinates of 4
  vpDot2 points. The interaction matrix is computed using the current visual
  features.

*/


/*!
  \example servoAfma6FourPoints2DCamVelocityInteractionCurrent.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame.  Visual features are the image coordinates of 4 vpDot2
  points. The interaction matrix is computed using the current visual
  features.

*/


#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_DC1394_2))

#include <visp/vp1394TwoGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>

#include <visp/vpMath.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpPoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpDot.h>
#include <visp/vpRobotAfma6.h>
#include <visp/vpServoDisplay.h>
#include <visp/vpPose.h>
#include <visp/vpIoTools.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

FILE *flagrange=NULL;
FILE *fdementhon=NULL;
FILE *flowe=NULL;

/*!

  Compute the pose \e cMo from the 3D coordinates of the points \e point and
  their corresponding 2D coordinates \e dot. The pose is computed using a Lowe
  non linear method.

  \param point : 3D coordinates of the points.

  \param dot : 2D coordinates of the points.

  \param ndot : Number of points or dots used for the pose estimation.

  \param cMo : Homogeneous matrix in output describing the transformation
  between the camera and object frame.

  \param cto : Translation in ouput extracted from \e cMo.

  \param cro : Rotation in ouput extracted from \e cMo.

  \param init : Indicates if the we have to estimate an initial pose with
  Lagrange or Dementhon methods.

*/
void compute_pose(vpPoint point[], vpDot2 dot[], int ndot,
		  vpCameraParameters cam, bool useDistortion,
		  vpHomogeneousMatrix &cMo,
		  vpTranslationVector &cto,
		  vpRxyzVector &cro, bool init)
{
  vpHomogeneousMatrix cMo_dementhon;  // computed pose with dementhon
  vpHomogeneousMatrix cMo_lagrange;  // computed pose with dementhon
  vpRotationMatrix cRo;
  vpPose pose;
  for (int i=0; i < ndot; i ++) {

    double x,y ;

//     std::cout << "Pixel " << i <<": " <<  dot[i].get_u()
// 	      << " " << dot[i].get_v() << std::endl;
    vpPixelMeterConversion::convertPoint(cam,
					 dot[i].get_u(), dot[i].get_v(),
					 x, y, useDistortion) ; //pixel to meter conversion
//     std::cout << "point cam: " << i << x << " " << y << std::endl;
    point[i].set_x(x) ;//projection perspective          p
    point[i].set_y(y) ;
    pose.addPoint(point[i]) ;
//     std::cout << "point " << i << std::endl;
//     point[i].print();

  }

  if (init == true) {
    pose.computePose(vpPose::DEMENTHON, cMo_dementhon) ;
    //compute the pose for a given method
//     cMo_dementhon.extract(cto);
//     cMo_dementhon.extract(cRo);
//     cro.buildFrom(cRo);
    // Compute and return the residual expressed in meter for the pose matrix
    // 'cMo'
    double residual_dementhon = pose.computeResidual(cMo_dementhon);

//     std::cout << "\nPose Dementhon "
// 	      << "(residual: " << residual_dementhon << ")\n "
// 	      << "cdto[0] = " << cto[0] << ";\n "
// 	      << "cdto[1] = " << cto[1] << ";\n "
// 	      << "cdto[2] = " << cto[2] << ";\n "
// 	      << "cdro[0] = vpMath::rad(" << vpMath::deg(cro[0]) << ");\n "
// 	      << "cdro[1] = vpMath::rad(" << vpMath::deg(cro[1]) << ");\n "
// 	      << "cdro[2] = vpMath::rad(" << vpMath::deg(cro[2]) << ");\n"
// 	      << std::endl;

    pose.computePose(vpPose::LAGRANGE, cMo_lagrange) ;
//     cMo_lagrange.extract(cto);
//     cMo_lagrange.extract(cRo);
//     cro.buildFrom(cRo);
    double residual_lagrange = pose.computeResidual(cMo_lagrange);

//     std::cout << "\nPose Lagrange "
// 	      << "(residual: " << residual_lagrange << ")\n "
// 	      << "cdto[0] = " << cto[0] << ";\n "
// 	      << "cdto[1] = " << cto[1] << ";\n "
// 	      << "cdto[2] = " << cto[2] << ";\n "
// 	      << "cdro[0] = vpMath::rad(" << vpMath::deg(cro[0]) << ");\n "
// 	      << "cdro[1] = vpMath::rad(" << vpMath::deg(cro[1]) << ");\n "
// 	      << "cdro[2] = vpMath::rad(" << vpMath::deg(cro[2]) << ");\n"
// 	      << std::endl;

    //   cout << "Lagrange residual term: " << residual_lagrange <<endl ;

    // Select the best pose to initialize the lowe pose computation
    if (residual_lagrange < residual_dementhon)                         //on garde le cMo
      cMo = cMo_lagrange;
    else
      cMo = cMo_dementhon;

  //   cout <<"------------------------------------------------------------"<<endl
  }
  else { // init = false; use of the previous pose to initialise LOWE
    cRo.buildFrom(cro);
    cMo.buildFrom(cRo, cto);
  }
  pose.computePose(vpPose::LOWE, cMo) ;
  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
//   double residual_lowe = pose.computeResidual(cMo);

//   std::cout << "\nPose LOWE "
// 	    << "(residual: " << residual_lowe << ")\n "
// 	    << "cdto[0] = " << cto[0] << ";\n "
// 	    << "cdto[1] = " << cto[1] << ";\n "
// 	    << "cdto[2] = " << cto[2] << ";\n "
// 	    << "cdro[0] = vpMath::rad(" << vpMath::deg(cro[0]) << ");\n "
// 	    << "cdro[1] = vpMath::rad(" << vpMath::deg(cro[1]) << ");\n "
// 	    << "cdro[2] = vpMath::rad(" << vpMath::deg(cro[2]) << ");\n"
// 	    << std::endl;

//   vpTRACE( "LOWE pose :" ) ;
//   std::cout <<  cMo << std::endl ;
}

int
main()
{
  try
    {
      std::string username;
      std::string outputdir; // for the results files
      // Set the default output path
#ifdef UNIX
      outputdir = "/tmp";
#elif WIN32
      outputdir = "C:/temp";
#endif

      // Get the user login name
      vpIoTools::getUserName(username);

      // Append to the output path string, the login name of the user
      outputdir = outputdir + "/" + username;

      // Test if the output path exist. If no try to create it
      if (vpIoTools::checkDirectory(outputdir) == false) {
	try {
	  // Create the dirname
	  vpIoTools::makeDirectory(outputdir);
	}
	catch (...) {
	  std::cerr << std::endl
		    << "ERROR:" << std::endl;
	  std::cerr << "  Cannot create " << outputdir << std::endl;
	  exit(-1);
	}
      }
      // Create the debug file: $outputdir/pose.txt
      char *filename = new char[FILENAME_MAX];
      sprintf(filename, "%s/pose.txt", outputdir.c_str());
      FILE *fdpose = fopen(filename, "w");

      bool useDistortion = true;
      vpRobotAfma6 robot;

      if (useDistortion) {
	// Load the end-effector to camera frame transformation obtained
	// using a camera intrinsic model with distortion
	robot.init(vpAfma6::CAMERA_DRAGONFLY2_8MM, useDistortion);
      }

      vpServo task ;

      vpImage<unsigned char> I ;
      int i ;

      vp1394TwoGrabber g;
      g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
      g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
      g.open(I) ;

      vpDisplayX display(I,100,100,"Current image") ;

      g.acquire(I) ;

      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;

      std::cout << std::endl ;
      std::cout << "-------------------------------------------------------" << std::endl ;
      std::cout << " Test program for vpServo "  <<std::endl ;
      std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl ;
      std::cout << " Simulation " << std::endl ;
      std::cout << " task : servo a point " << std::endl ;
      std::cout << "-------------------------------------------------------" << std::endl ;
      std::cout << std::endl ;


      vpDot2 dot[4] ;

      std::cout << "Click on the 4 dots clockwise starting from upper/left dot..."
	   << std::endl;
      for (i=0 ; i < 4 ; i++) {
	dot[i].initTracking(I) ;
	vpDisplay::displayCross(I,
				(unsigned int)dot[i].get_v(),
				(unsigned int)dot[i].get_u(),
				10, vpColor::blue) ;
	vpDisplay::flush(I);
      }

      vpCameraParameters cam ;
      // Update camera parameters
      robot.getCameraParameters (cam, I);

      vpTRACE("sets the current position of the visual feature ") ;
      vpFeaturePoint p[4] ;
      for (i=0 ; i < 4 ; i++)
	vpFeatureBuilder::create(p[i], cam, dot[i], useDistortion);  //retrieve x,y  of the vpFeaturePoint structure

      // Set the position of the square target in a frame which origin is
      // centered in the middle of the square
      vpPoint point[4] ;
#define L 0.05 // to deal with a 10cm by 10cm square
      point[0].setWorldCoordinates(-L, -L, 0) ;
      point[1].setWorldCoordinates( L, -L, 0) ;
      point[2].setWorldCoordinates( L,  L, 0) ;
      point[3].setWorldCoordinates(-L,  L, 0) ;

      // Initialise a desired pose to compute s*, the desired 2D point features
      vpHomogeneousMatrix cMo;
      vpTranslationVector cto(0, 0, 0.7); // tz = 1 meter
      vpRxyzVector cro(vpMath::rad(0), vpMath::rad(0), vpMath::rad(0)); // No rotations
      vpRotationMatrix cRo(cro); // Build the rotation matrix
      cMo.buildFrom(cRo, cto); // Build the homogeneous matrix

      vpTRACE("sets the desired position of the 2D visual feature ");
      vpFeaturePoint pd[4] ;
      // Compute the desired position of the features from the desired pose
      for (int i=0; i < 4; i ++) {
	vpColVector cP, p ;
	point[i].changeFrame(cMo, cP) ;
	point[i].projection(cP, p) ;

	pd[i].set_x(p[0]) ;
	pd[i].set_y(p[1]) ;
	pd[i].set_Z(cP[2]);
      }

      vpTRACE("define the task") ;
      vpTRACE("\t we want an eye-in-hand control law") ;
      vpTRACE("\t robot is controlled in the camera frame") ;
      vpTRACE("\t Interaction matrix is computed with the current visual features") ;
      task.setServo(vpServo::EYEINHAND_CAMERA) ;
      task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);

      vpTRACE("\t we want to see a point on a point..") ;
      std::cout << std::endl ;
      for (i=0 ; i < 4 ; i++)
	task.addFeature(p[i],pd[i]) ;

      vpTRACE("\t set the gain") ;
      task.setLambda(0.6) ;

      vpTRACE("Display task information " ) ;
      task.print() ;

      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

      int iter=0 ;
      double error = 1;
      vpTRACE("\t loop") ;
      while(error > 0.000001 ) {
	std::cout << "-------------------------------" << iter <<std::endl ;

	g.acquire(I) ;
	vpDisplay::display(I) ;

	for (i=0 ; i < 4 ; i++) {
	  dot[i].track(I) ;
	  vpDisplay::displayCross(I,
				  (unsigned int)dot[i].get_v(),
				  (unsigned int)dot[i].get_u(),
				  10, vpColor::green) ;
	}

	if (iter == 0) {
	  compute_pose(point, dot, 4, cam, useDistortion, cMo, cto, cro, true);
	  // At the first iteration we compute the pose using Lagrange and
	  // Dementhon methods, chose the best estimated pose (either Lagrange
	  // or Dementhon) and than compute the pose using LOWE method with
	  // Lagrange or Dementhon pose as initiaisation.
	}
	else {
	  // During the servo, we compute the pose using LOWE method. For the
	  // initial pose used in the non linear minimisation we use the pose
	  // computed at the previous iteration.
	  compute_pose(point, dot, 4, cam, useDistortion, cMo, cto, cro, false);
	}

	// Save the resulting pose: translations in meters, rotations (rx, ry,
	// rz) in degrees
	fprintf(fdpose, "%g %g %g %g %g %g\n",
		cto[0], cto[1], cto[2],
		vpMath::deg(cro[0]),
		vpMath::deg(cro[1]),
		vpMath::deg(cro[2]));

	for (i=0 ; i < 4 ; i++) {
	  vpFeatureBuilder::create(p[i], cam, dot[i], useDistortion);
	  // Set the feature Z coordinate from the pose
	  vpColVector cP;
	  point[i].changeFrame(cMo, cP) ;

	  p[i].set_Z(cP[2]);
	}

	task.print() ;

	vpColVector v ;
	v = task.computeControlLaw() ;

	vpServoDisplay::display(task, cam, I, useDistortion);
	//	v = 0;
	std::cout << "Velocity: " <<v.t() ;

	robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

	vpDisplay::flush(I) ;

	error = task.error.sumSquare();
	vpTRACE("\t\t || s - s* || = %g ", error) ;

	iter ++;
      }

      fclose(fdpose);

      vpTRACE("Display task information " ) ;
      task.print() ;
      task.kill();
    }
  catch (...) {
    vpERROR_TRACE(" Test failed") ;
    return 0;
  }
}

#else
int
main()
{
  vpERROR_TRACE("You do not have an afma6 robot or a firewire framegrabber connected to your computer...");

}

#endif
