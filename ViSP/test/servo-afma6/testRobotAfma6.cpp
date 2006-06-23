/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: testRobotAfma6.cpp,v $
 * Author:    Fabien Spindler
 *
 * Version control
 * ===============
 *
 *  $Id: testRobotAfma6.cpp,v 1.5 2006-06-23 14:45:07 brenier Exp $
 *
 * Description
 * ============
 *  Tests the control law
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \example testRobotAfma6.cpp

  Example of a real robot control, the Afma6 robot (cylindrical robot, with 4
  degrees of freedom).
*/

#include <iostream>
using namespace std;

#include <visp/vpConfig.h>
#include <visp/vpRobotAfma6.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_AFMA6
#ifdef VISP_HAVE_GSL
int gsl_warnings_off;
#endif

int main()
{
  cout << "a test..." << endl;

  vpAfma6 afma6;
  vpCTRACE << afma6;

  vpRobotAfma6 robotAfma6;

  vpCTRACE << "------- set camera 1394 -------------------" << endl;
  robotAfma6.init( vpAfma6::CAMERA_IEEE1394_12MM);

  vpCTRACE << robotAfma6;

  vpCTRACE << "------- set camera XC77 -------------------" << endl;
  robotAfma6.init( vpAfma6::CAMERA_XC77_12MM);
  vpCTRACE << robotAfma6;
  return 1;
}
#else
int main()
{
  cout << "a test..." << endl;
}

#endif
