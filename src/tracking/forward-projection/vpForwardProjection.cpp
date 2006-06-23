/****************************************************************************
 *
 * $Id: vpForwardProjection.cpp,v 1.4 2006-06-23 14:45:06 brenier Exp $
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
 * Forward projection.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


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
    vpERROR_TRACE(" ") ;
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
    vpERROR_TRACE(" ") ;
    throw ;
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
