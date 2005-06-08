
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpTracker.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpTracker.cpp,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what   is a generic tracker
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpTracker.h>
#include <visp/vpDebug.h>


/*!
  \file vpTracker.cpp
  \brief   class that defines what is a generic tracker
*/






void
vpTracker::init()
{
  cPAvailable = false ;
}



vpTracker::vpTracker()
{
  init() ;
}






/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
