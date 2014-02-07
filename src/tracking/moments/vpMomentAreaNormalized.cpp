/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * 2D normalized surface moment descriptor (usually described as An)
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
#include <visp/vpMomentAreaNormalized.h>
#include <visp/vpMomentObject.h>
#include <visp/vpMomentCentered.h>
#include <visp/vpMomentDatabase.h>
#include <cmath>


/*!
  Computes the normalized area \f$ a_n=Z^* \sqrt{\frac{a^*}{a}} \f$.
  Depends on vpMomentCentered.
*/
void vpMomentAreaNormalized::compute(){
    bool found_moment_centered;        
    
    /* getMoments() returns a reference to a vpMomentDatabase. (a protected member inherited from vpMoment)
      .get() 		is a member function of vpMomentDatabase that returns a specific moment which is linked to it*/
    const vpMomentCentered& momentCentered = static_cast<const vpMomentCentered&>(getMoments().get("vpMomentCentered",found_moment_centered));

    if(!found_moment_centered) throw vpException(vpException::notInitialized,"vpMomentCentered not found");

    double a;
    /* getObject() returns a reference to the vpMomentObject from which
     * the moment values are calculated. (public member of vpMoment)*/
    if(getObject().getType()==vpMomentObject::DISCRETE)
        a = momentCentered.get(2,0)+momentCentered.get(0,2);
    else
        a = getObject().get(0,0);

    values[0] = desiredDepth*sqrt(desiredSurface/a);
}

/*!
  Default constructor.
  \param desired_surface : desired area \e a* when the visual servoing converges.
  \param desired_depth : desired depth \e Z* when the visual servoing converges.
*/
vpMomentAreaNormalized::vpMomentAreaNormalized(double desired_surface, double desired_depth)
  : vpMoment(),desiredSurface(desired_surface),desiredDepth(desired_depth)
{
    values.resize(1);
}

/*!
  Outputs the moment's values to a stream.
*/
VISP_EXPORT std::ostream & operator<<(std::ostream & os, const vpMomentAreaNormalized& m){
    os << "An:" << m.values[0] ;

    return os;    
}
