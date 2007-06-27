/****************************************************************************
 *
 * $Id: servoBiclopsPoint2DArtVelocity.cpp,v 1.5 2007-06-27 14:44:06 fspindle Exp $
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
 *   velocity computed in articular
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file servoBiclopsPoint2DArtVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the biclops
  robot (pan-tilt head provided by Traclabs). The velocity is
  computed in articular. The visual feature is the center of gravity of a
  point.

*/


/*!
  \example servoBiclopsPoint2DArtVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the biclops
  robot (pan-tilt head provided by Traclabs). The velocity is
  computed in articular. The visual feature is the center of gravity of a
  point.

*/

#include <visp/vpTime.h>

#include <signal.h>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if ( defined (VISP_HAVE_BICLOPS) & (defined (VISP_HAVE_DC1394_2) | defined (VISP_HAVE_DC1394_1) | defined(VISP_HAVE_DIRECTSHOW)) )

#ifdef VISP_HAVE_PTHREAD
#  include <pthread.h>
#endif

#include <visp/vp1394Grabber.h>
#include <visp/vpDirectShowGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpPoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpRobotBiclops.h>
#include <visp/vpIoTools.h>
#include <visp/vpParseArgv.h>
#include <visp/vpServoDisplay.h>
#include <visp/vpDot.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>


#ifdef VISP_HAVE_PTHREAD
pthread_mutex_t mutexEndLoop = PTHREAD_MUTEX_INITIALIZER;
#endif

void signalCtrC( int signumber )
{
#ifdef VISP_HAVE_PTHREAD
  pthread_mutex_unlock( &mutexEndLoop );
#endif
  vpTime::wait(10);
  vpTRACE("Ctrl-C pressed...");
}


// List of allowed command line options
#define GETOPTARGS	"c:d:h"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param conf : Robot configuration file.
  \param debugdir : Debug file directory.
  \param user : Username.

 */
void usage(char *name, char *badparam, std::string& conf, std::string& debugdir, std::string& user)
{
  fprintf(stdout, "\n\
  Example of eye-in-hand control law. We control here a real robot, the biclops\n\
  robot (pan-tilt head provided by Traclabs). The velocity is\n\
  computed in articular. The visual feature is the center of gravity of a\n\
  point.\n\
\n\
SYNOPSIS\n\
  %s [-c <Biclops configuration file>] [-d <debug file directory>] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c <Biclops configuration file>                      %s\n\
     Sets the biclops robot configuration file.\n\n\
  -d <debug file directory>                            %s\n\
     Sets the debug file directory.\n\
     From this directory, creates the\"%s\"\n\
     subdirectory depending on the username, where\n\
     it writes biclops.txt file.\n", conf.c_str(), debugdir.c_str(), user.c_str());

}
/*!

Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param conf : Robot configuration file.
  \param debugdir : Debug file directory.
  \param user : Username.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, std::string& conf, std::string &debugdir, std::string& user)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': conf = optarg; break;
    case 'd': debugdir = optarg; break;
    case 'h': usage(argv[0], NULL, conf, debugdir, user); return false; break;

    default:
      usage(argv[0], optarg, conf, debugdir, user); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, conf, debugdir, user);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}



int
main(int argc, char ** argv)
{
  std::cout << std::endl ;
  std::cout << "-------------------------------------------------------" << std::endl ;
  std::cout << " Test program for vpServo "  <<std::endl ;
  std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl ;
  std::cout << " Simulation " << std::endl ;
  std::cout << " task : servo a point " << std::endl ;
  std::cout << "-------------------------------------------------------" << std::endl ;
  std::cout << std::endl ;

  try{

#ifdef VISP_HAVE_PTHREAD
    pthread_mutex_lock( &mutexEndLoop );
#endif
    signal( SIGINT,&signalCtrC );

    //default unix configuration file path
    std::string opt_conf = "/usr/share/BiclopsDefault.cfg";

    std::string username;
    std::string debugdir;
    std::string opt_debugdir;

    // Set the default output path
#ifdef UNIX
    opt_debugdir = "/tmp";
#elif WIN32
    opt_debugdir = "C:/temp";
#endif

    // Get the user login name
    vpIoTools::getUserName(username);

    // Read the command line options
    if (getOptions(argc, argv, opt_conf, opt_debugdir , username) == false) {
      exit (-1);
    }

    // Get the option value
    if (!opt_debugdir.empty())
      debugdir = opt_debugdir;

    // Append to the output path string, the login name of the user
    std::string dirname = debugdir + "/" + username;

    // Test if the output path exist. If no try to create it
    if (vpIoTools::checkDirectory(dirname) == false) {
      try {
	// Create the dirname
	vpIoTools::makeDirectory(dirname);
      }
      catch (...) {
	usage(argv[0], NULL, opt_conf, debugdir, username);
	std::cerr << std::endl
	     << "ERROR:" << std::endl;
	std::cerr << "  Cannot create " << dirname << std::endl;
	std::cerr << "  Check your -d " << debugdir << " option " << std::endl;
	exit(-1);
      }
    }

    // Create the debug file: debugdir/$user/biclops.txt
    char *filename = new char[FILENAME_MAX];
    sprintf(filename, "%s/biclops.txt", debugdir.c_str());
    FILE *fd = fopen(filename, "w");

    vpRobotBiclops robot(opt_conf.c_str()) ;
    {
      vpColVector q(2); q=0;
      robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
      robot.setPosition( vpRobot::ARTICULAR_FRAME,q );
    }

    vpImage<unsigned char> I ;

#if defined VISP_HAVE_DC1394_2
    vp1394TwoGrabber g;
#elif defined VISP_HAVE_DC1394_1
    vp1394Grabber g;
#elif defined VISP_HAVE_DIRECTSHOW
    vpDirectShowGrabber g;
#endif

    g.open(I) ;

    try{
      g.acquire(I) ;
    }
    catch(...)
      {
		vpERROR_TRACE(" Error caught") ;
		return(-1) ;
      }

    // We open a window using either X11 or GTK or GDI.
    // Its size is automatically defined by the image (I) size
#if defined VISP_HAVE_X11
    vpDisplayX display(I, 100, 100,"Display X...") ;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display(I, 100, 100,"Display GTK...") ;
#elif defined WIN32
    vpDisplayGDI display(I, 100, 100,"Display GDI...") ;
#endif

    
    try{
      vpDisplay::display(I) ;
    }
    catch(...)
      {
		vpERROR_TRACE(" Error caught") ;
		return(-1) ;
      }


    vpServo task ;

    vpDot dot ;

    try{
      std::cout << "Click on a dot to initialize the tracking..." << std::endl;
      dot.setGraphics(true);
      dot.initTracking(I) ;
      dot.track(I);
      vpERROR_TRACE("after dot.initTracking(I) ") ;
    }
    catch(...)
      {
		vpERROR_TRACE(" Error caught") ;
		return(-1) ;
      }

    vpCameraParameters cam ;

    vpTRACE("sets the current position of the visual feature ") ;
    vpFeaturePoint p ;
    vpFeatureBuilder::create(p,cam, dot)  ;  //retrieve x,y and Z of the vpPoint structure

    p.set_Z(1) ;
    vpTRACE("sets the desired position of the visual feature ") ;
    vpFeaturePoint pd ;
    pd.buildFrom(0,0,1) ;

    vpTRACE("define the task") ;
    vpTRACE("\t we want an eye-in-hand control law") ;
    vpTRACE("\t articular velocity are computed") ;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;


    vpTRACE("Set the position of the camera in the end-effector frame ") ;
    vpHomogeneousMatrix cMe ;
    //  robot.get_cMe(cMe) ;

    vpTwistMatrix cVe ;
    robot.get_cVe(cVe) ;
    std::cout << cVe <<std::endl ;
    task.set_cVe(cVe) ;

    std::cout << "Click in the image to start the servoing..." << std::endl;
    vpDisplay::getClick(I) ;
    vpTRACE("Set the Jacobian (expressed in the end-effector frame)") ;
    vpMatrix eJe ;
    robot.get_eJe(eJe) ;
    task.set_eJe(eJe) ;


    vpTRACE("\t we want to see a point on a point..") ;
    std::cout << std::endl ;
    task.addFeature(p,pd) ;

    vpTRACE("\t set the gain") ;
    task.setLambda(0.1) ;


    vpTRACE("Display task information " ) ;
    task.print() ;


    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

    int iter=0 ;
    vpTRACE("\t loop") ;
#ifdef VISP_HAVE_PTHREAD
    while( 0 != pthread_mutex_trylock( &mutexEndLoop ) )
#else
      while (1)
#endif
	{
	  std::cout << "---------------------------------------------" << iter <<std::endl ;

	  g.acquire(I) ;
	  vpDisplay::display(I) ;

	  dot.track(I) ;

	  //    vpDisplay::displayCross(I,(int)dot.I(), (int)dot.J(),
	  //			   10,vpColor::green) ;


	  vpFeatureBuilder::create(p,cam, dot);

	  // get the jacobian
	  robot.get_eJe(eJe) ;
	  task.set_eJe(eJe) ;

	  //  std::cout << (vpMatrix)cVe*eJe << std::endl ;

	  vpColVector v ;
	  v = task.computeControlLaw() ;

	  vpServoDisplay::display(task,cam,I) ;
	  std::cout << v.t() ;
	  robot.setVelocity(vpRobot::ARTICULAR_FRAME, v) ;

	  vpTRACE("\t\t || s - s* || = %f ", task.error.sumSquare()) ;

	  {
	    vpColVector s_minus_sStar(2);
	    s_minus_sStar = task.s - task.sStar;
	    fprintf(fd, "%f %f %f %f %f\n",
		    v[0], v[1],
		    s_minus_sStar[0], s_minus_sStar[1],
		    task.error.sumSquare());
	  }
	}

    vpTRACE("Display task information " ) ;
    task.print() ;
    task.kill();

    fclose(fd);

  } catch (...) { vpERROR_TRACE("Throw uncatched..."); }

}


#else
int
main()
{
  vpERROR_TRACE("You don't have a biclops head connected to your computer or 1394 framegrabbing capabilities...");
}
#endif
