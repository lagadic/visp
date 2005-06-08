
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
 *  $Id: vpServoData.h,v 1.1.1.1 2005-06-08 07:08:09 fspindle Exp $
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



public:


  ~vpServoData() ;

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
