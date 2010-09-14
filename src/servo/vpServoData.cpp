/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Save data during the task execution.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpServoData.cpp
  \brief save data during the task execution
*/

// Servo
#include <visp/vpServo.h>

#include <visp/vpServoData.h>
#include <visp/vpIoException.h>
#include <visp/vpIoTools.h>

void
vpServoData::open(const char *directory)
{
  try
  {
    if (vpIoTools::checkDirectory(directory) == false)
      vpIoTools::makeDirectory(directory);

    char s[FILENAME_MAX] ;

    sprintf(s,"%s/vel.dat",directory) ;
    velocityFile.open(s)  ;
    sprintf(s,"%s/error.dat",directory) ;
    errorFile.open(s)  ;
    sprintf(s,"%s/errornorm.dat",directory) ;
    errorNormFile.open(s)  ;
    sprintf(s,"%s/s.dat",directory) ;
    sFile.open(s)  ;
    sprintf(s,"%s/sStar.dat",directory) ;
    sStarFile.open(s) ;

  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}

void vpServoData::setCmDeg()
{
  cmDeg = true ;
}
void vpServoData::setMeterRad()
{
  cmDeg = false ;
}
void vpServoData::save(const vpServo &task)
{
  if (cmDeg==false) velocityFile << task.q_dot.t() ;
  else
  {
    for (int i=0 ; i < 3 ; i++)
      velocityFile <<  task.q_dot[i]*100 <<" " ;
    for (int i=4 ; i < 6 ; i++)
      velocityFile <<  vpMath::deg(task.q_dot[i]) <<" " ;
    velocityFile << std::endl ;
  }
  errorFile << task.error.t() ;
  errorNormFile << task.error.sumSquare() << std::endl ;
  vNormFile << task.q_dot.sumSquare() << std::endl ;

  sFile <<task.s.t() ;
  sStarFile << task.sStar.t();
}



void vpServoData::close()
{
  velocityFile.close() ;
  errorFile.close() ;
  errorNormFile.close() ;
  sFile.close() ;
  sStarFile.close() ;
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
