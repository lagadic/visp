/****************************************************************************
 *
 * $Id: servoAfma6Points2DCamVelocityEyeToHand.cpp,v 1.3 2007-04-19 14:19:18 fspindle Exp $
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
 *   eye-to-hand control
 *   velocity computed in the camera frame
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/
/*!
  \file servoAfma6Points2DCamVelocityEyeToHand.cpp

  \brief Example of a eye-to-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The robot is controlled
  in the camera frame.

*/


/*!
  \example servoAfma6Points2DCamVelocityEyeToHand.cpp

  Example of a eye-to-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The robot is controlled
  in the camera frame.

*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_ITIFG8))

#define SAVE 0

#include <visp/vpItifg8Grabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpPoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>

#include <visp/vpRobotAfma6.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

#include <visp/vpDot.h>
#include <visp/vpPose.h>
#include <visp/vpImageIo.h>

#define L 0.006
#define D 0

int
main()
{
  try 
    {
      vpCameraParameters cam ;
      vpImage<unsigned char> I ;
      int i ;

      vpItifg8Grabber g(0) ;
      g.open(I) ;

      g.acquire(I) ;


      vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
      vpTRACE(" ") ;

      vpDisplay::display(I) ;



      cout << endl ;
      cout << "-------------------------------------------------------" << endl ;
      cout << " Test program for vpServo "  <<endl ;
      cout << " Eye-to-hand task control" << endl ;
      cout << " Simulation " << endl ;
      cout << " task : servo a point " << endl ;
      cout << "-------------------------------------------------------" << endl ;
      cout << endl ;

      int nbPoint =7 ;

      vpDot dot[nbPoint] ;

      for (i=0 ; i < nbPoint ; i++)
	{
	  dot[i].initTracking(I) ;
	  dot[i].setGraphics(true) ;
	  dot[i].track(I) ;
	  vpDisplay::flush(I) ;
	  dot[i].setGraphics(false) ;
	}

      // Compute the pose 3D model
      vpPoint point[nbPoint] ;
      point[0].setWorldCoordinates(-2*L,D, -3*L) ;
      point[1].setWorldCoordinates(0,D, -3*L) ;
      point[2].setWorldCoordinates(2*L,D, -3*L) ;

      point[3].setWorldCoordinates(-L,D,-L) ;
      point[4].setWorldCoordinates(L,D, -L) ;
      point[5].setWorldCoordinates(L,D, L) ;
      point[6].setWorldCoordinates(-L,D, L) ;


      vpHomogeneousMatrix cMo, cdMo ;
      vpPose pose ;
      pose.clearPoint() ;
      for (i=0 ; i < nbPoint ; i++)
	{
	  double x,y ;
	  vpPixelMeterConversion::convertPoint(cam,
					       dot[i].get_u(), dot[i].get_v(),
					       x,y)  ;
	  point[i].set_x(x) ;
	  point[i].set_y(y) ;
	  pose.addPoint(point[i]) ;
	}

      // compute the initial pose using Dementhon method followed by a non linear
      // minimisation method
      pose.computePose(vpPose::DEMENTHON_LOWE, cMo) ;


      cout << cMo << endl  ;
      cMo.print() ;

      /*------------------------------------------------------------------
	--  Learning the desired position
	--  or reading the desired position
	------------------------------------------------------------------
      */
      cout << " Learning 0/1 " <<endl ;
      char name[FILENAME_MAX] ;
      sprintf(name,"cdMo.dat") ;
      int learning ;
      cin >> learning ;
      if (learning ==1)
	{
	  // save the object position
	  vpTRACE("Save the location of the object in a file cdMo.dat") ;
	  ofstream f(name) ;
	  cMo.save(f) ;
	  f.close() ;
	  exit(1) ;
	}


      vpServo task ;
      vpRobotAfma6 robot ;

      {
	vpTRACE("Loading desired location from cdMo.dat") ;
	ifstream f("cdMo.dat") ;
	cdMo.load(f) ;
	f.close() ;
      }

      vpFeaturePoint p[nbPoint], pd[nbPoint] ;

      // set the desired position of the point by forward projection using
      // the pose cdMo
      for (i=0 ; i < nbPoint ; i++)
	{
	  vpColVector cP, p ;
	  point[i].changeFrame(cdMo, cP) ;
	  point[i].projection(cP, p) ;

	  pd[i].set_x(p[0]) ;
	  pd[i].set_y(p[1]) ;
	}



      //------------------------------------------------------------------

      vpTRACE("define the task") ;
      vpTRACE("\t we want an eye-in-hand control law") ;
      vpTRACE("\t robot is controlled in the camera frame") ;
      task.setServo(vpServo::EYETOHAND_L_cVe_eJe) ;
      task.setInteractionMatrixType(vpServo::CURRENT) ;


      for (i=0 ; i < nbPoint ; i++)
	{
	  task.addFeature(p[i],pd[i]) ;
	}


      vpTRACE("Display task information " ) ;
      task.print() ;


      //------------------------------------------------------------------


      double convergence_threshold = 0.00; //025 ;
      vpDisplay::getClick(I) ;

      //-------------------------------------------------------------
      double error =1 ;
      int iter=0 ;
      vpTRACE("\t loop") ;
      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;
      vpColVector v ; // computed robot velocity


      // position of the object in the effector frame
      vpHomogeneousMatrix oMcamrobot ;
      oMcamrobot[0][3] = -0.05 ;

      vpImage<vpRGBa> Ic ;
      int it = 0 ;

      double lambda_av =0.1;
      double alpha = 1 ; //1 ;
      double beta =3 ; //3 ;

      cout << "alpha 0.7" << endl;
      cin >> alpha ;
      cout << "beta 5" << endl;
      cin >> beta ;
      vpList<double> Lu, Lv ;
      while(error > convergence_threshold)
	{
	  cout << "---------------------------------------------" << iter++ <<endl ;

	  g.acquire(I) ;
	  vpDisplay::display(I) ;
	  vpDisplay::displayCharString(I,265,150,
				       "Eye-To-Hand Visual Servoing",
				       vpColor::green) ;
	  vpDisplay::displayCharString(I,280,150,
				       "IRISA-INRIA Rennes, Lagadic project",
				       vpColor::green) ;
	  try
	    {
	      for (i=0 ; i < nbPoint ; i++)
		{
		  dot[i].track(I) ;
		  Lu += dot[i].get_u() ;
		  Lv += dot[i].get_v() ;
		}
	    }
	  catch(...)
	    {
	      vpTRACE("Error detected while tracking visual features") ;
	      robot.stopMotion() ;
	      exit(1) ;
	    }

	  // compute the initial pose using  a non linear minimisation method
	  pose.clearPoint() ;

	  for (i=0 ; i < nbPoint ; i++)
	    {
	      double x,y ;
	      vpPixelMeterConversion::convertPoint(cam,
						   dot[i].get_u(), dot[i].get_v(),
						   x,y)  ;
	      point[i].set_x(x) ;
	      point[i].set_y(y) ;

	      vpColVector cP ;
	      point[i].changeFrame(cdMo, cP) ;

	      p[i].set_x(x) ;
	      p[i].set_y(y) ;
	      p[i].set_Z(cP[2]) ;

	      pose.addPoint(point[i]) ;

	      point[i].display(I,cMo,cam, vpColor::green) ;
	      point[i].display(I,cdMo,cam, vpColor::blue) ;
	    }
	  pose.computePose(vpPose::LOWE, cMo) ;
	  vpDisplay::flush(I) ;

	  //! set up the Jacobian
	  vpHomogeneousMatrix cMe, camrobotMe ;
	  robot.get_cMe(camrobotMe) ;
	  cMe = cMo *oMcamrobot * camrobotMe  ;


	  task.set_cVe(cMe) ;

	  vpMatrix eJe ;
	  robot.get_eJe(eJe) ;
	  task.set_eJe(eJe) ;


	  // Compute the adaptative gain (speed up the convergence)
	  double gain ;
	  if (iter>2)
	    {
	      if (alpha == 0) gain = lambda_av ;
	      else
		{
		  gain = alpha * exp (-beta * task.error.sumSquare() ) +  lambda_av;
		}
	    }
	  else gain = lambda_av ;
	  if (SAVE==1)
	    gain = gain/5 ;

	  vpTRACE("%f %f %f %f  %f",alpha, beta, lambda_av, task.error.sumSquare(),  gain) ;
	  task.setLambda(gain) ;


	  v = task.computeControlLaw() ;

	  // display points trajectory
	  Lu.front() ; Lv.front() ;
	  while (!Lu.outside())
	    {
	      double u = Lu.value() ;
	      double v = Lv.value() ;

	      vpDisplay::displayPoint(I,
				      vpMath::round(v), vpMath::round(u),
				      vpColor::red) ;
	      Lu.next() ; Lv.next() ;
	    }
	  vpServoDisplay::display(task,cam,I) ;
	  robot.setVelocity(vpRobot::ARTICULAR_FRAME, v) ;

	  error = task.error.sumSquare() ;
	  cout << "|| s - s* || = "<< error<<endl ;

	  if (error>7)
	    {
	      vpTRACE("Error detected while tracking visual features") ;
	      robot.stopMotion() ;
	      exit(1) ;
	    }

	  // display the pose
	  // pose.display(I,cMo,cam, 0.04, vpColor::red) ;
	  // display the pose
	  //   pose.display(I,cdMo,cam, 0.04, vpColor::blue) ;
	  if ((SAVE==1) && (iter %3==0))
	    {

	      vpDisplay::getImage(I,Ic) ;
	      sprintf(name,"/tmp/marchand/image.%04d.ppm",it++) ;
	      vpImageIo::writePPM(Ic,name) ;
	    }
	}
      v = 0 ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
      vpDisplay::getClick(I) ;
    }
  catch (...)
    {
      vpERROR_TRACE(" Test failed") ;
      return 0;
    }
}

#else
int
main()
{
  vpERROR_TRACE("You do not have an afma6 robot or an Itifg8 framegrabber connected to your computer...");
}

#endif
