
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpForwardProjection.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpForwardProjection.cpp,v 1.2 2005-07-06 09:57:03 marchand Exp $
 *
 * Description
 * ============
 *     class that defines what is a
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpForwardProjection.h>
#include <visp/vpDebug.h>


/*!
  \file vpForwardProjection.cpp
  \brief   class that defines what is a point
*/






void
vpForwardProjection::print() const
{
  cout << "oP : " << oP.t() ;
  cout << "cP : " << cP.t() ;
  cout << "p : " << p.t() ;
}


void
vpForwardProjection::project()
{
  projection(cP, p) ;
}




//! change frame
void
vpForwardProjection::changeFrame(const vpHomogeneousMatrix &cMo)
{
  changeFrame(cMo,cP) ;
}

//! change frame and project
void
vpForwardProjection::project(const vpHomogeneousMatrix &cMo)
{
  try{
    changeFrame(cMo,cP) ;
    project() ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


void
vpForwardProjection::track(const vpHomogeneousMatrix &cMo)
{
  try{
    project(cMo) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
