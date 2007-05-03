/****************************************************************************
 *
 * $Id: vpServoData.h,v 1.6 2007-05-03 11:36:34 fspindle Exp $
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
 * This file is part of the ViSP toolkit.
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
 * Save data during the task execution.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpServoData_H
#define vpServoData_H

/*!
  \file vpServoData.h
  \brief  save data during the task execution
*/

#include <iostream>

// Servo
#include <visp/vpConfig.h>
#include <visp/vpServo.h>

/*!
  \class vpServoData
  \brief save data during the task execution
*/
class VISP_EXPORT vpServoData
{

private:
  char baseDirectory[FILENAME_MAX] ;

  std::ofstream velocityFile ;
  std::ofstream errorFile ;
  std::ofstream errorNormFile ;
  std::ofstream sFile ;
  std::ofstream sStarFile ;
  std::ofstream vNormFile ;

  //! flag to known if velocity should be output in cm and degrees (true)
  //! or in m/rad
  bool cmDeg ;



public:


  virtual ~vpServoData() { ; }

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
