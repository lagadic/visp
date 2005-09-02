
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpServoDisplay.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpServoData.cpp,v 1.3 2005-09-02 14:37:25 marchand Exp $
 *
 * Description
 * ============
 * save data during the task execution
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


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
    sStarFile.open(s)  ;

  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
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
