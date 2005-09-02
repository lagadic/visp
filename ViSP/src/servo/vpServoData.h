
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpServoData.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpServoData.h,v 1.2 2005-09-02 14:37:25 marchand Exp $
 *
 * Description
 * ============
 *       save data during the task execution
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpServoData_H
#define vpServoData_H

/*!
  \file vpServoData.h
  \brief  save data during the task execution
*/

#include <iostream>

// Servo
#include <visp/vpServo.h>

/*!
  \class vpServoData
  \brief save data during the task execution
*/
class vpServoData
{

private:
  char baseDirectory[FILENAME_MAX] ;

  ofstream velocityFile ;
  ofstream errorFile ;
  ofstream errorNormFile ;
  ofstream sFile ;
  ofstream sStarFile ;
  ofstream vNormFile ;

  //! flag to known if velocity should be output in cm and degrees (true)
  //! or in m/rad
  bool cmDeg ;



public:


  ~vpServoData() ;

  //! velocity output in cm and deg
  void setCmDeg() ;
  //! velocity output in meter and deg (default)
  void setMeterRad() ;

  void save(const vpServo &task) ;
  void open(const char *baseDirectory) ;
  void close() ;

  void empty() ;
  void push() ;
  void display(vpImage<unsigned char> &I) ;

} ;



#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
