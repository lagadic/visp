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
 *  $Id: testRobotAfma6.cpp,v 1.2 2005-11-09 15:34:38 marchand Exp $
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

#ifdef HAVE_ROBOT_AFMA6
#ifdef HAVE_LIBGSL
int gsl_warnings_off;
#endif

int main()
{
  cout << "a test..." << endl;

  vpAfma6 afma6;
  CTRACE << afma6;

  vpRobotAfma6 robotAfma6;
  CTRACE << robotAfma6;
  return 1;
}
#else
int main()
{
  cout << "a test..." << endl;
}

#endif
