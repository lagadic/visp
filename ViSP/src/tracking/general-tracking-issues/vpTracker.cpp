/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Generic tracker.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp/vpTracker.h>
#include <visp/vpDebug.h>


/*!
  \file vpTracker.cpp
  \brief Class that defines what is a generic tracker.
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

vpTracker::vpTracker(const vpTracker &tracker)
{
  *this = tracker;
}


vpTracker &vpTracker::operator=(const vpTracker &tracker)
{
  p = tracker.p;
  cP = tracker.cP;
  cPAvailable = tracker.cPAvailable;
  
  return *this;
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
