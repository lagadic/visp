/****************************************************************************
 *
 * $Id: vpServoData.cpp,v 1.6 2007-04-18 16:14:28 asaunier Exp $
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
    try{
      vpIoTools::checkDirectory(directory);
    }
    catch(vpIoException me)
    {
      if (me.getCode() == vpIoException::ERRCantStatDirectory)
	vpIoTools::makeDirectory(directory) ;
    }

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
    velocityFile << endl ;
  }
  errorFile << task.error.t() ;
  errorNormFile << task.error.sumSquare() << endl ;
  vNormFile << task.q_dot.sumSquare() << endl ;

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
