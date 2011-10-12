/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Descriptor for moments centered and normalized.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp/vpMomentCenteredNormalized.h>
#include <visp/vpMomentCentered.h>
#include <visp/vpMomentObject.h>
#include <exception>

/*!
  Computes centered and normalized moments of all available orders. 
  Depends on vpMomentCentered.
*/
void vpMomentCenteredNormalized::compute(){
    values.resize((getObject().getOrder()+1)*(getObject().getOrder()+1));
    bool found_moment_centered;    
    
    vpMomentCentered& momentCentered = static_cast<vpMomentCentered&>(getMoments().get("vpMomentCentered",found_moment_centered));
    if(!found_moment_centered) throw vpException(vpException::notInitialized,"vpMomentCentered not found");
    double a;
    unsigned int order = getObject().getOrder()+1;
    if(getObject().getType()==vpMomentObject::DISCRETE)
        a = momentCentered.get(2,0)+momentCentered.get(0,2);
    else
        a = getObject().get(0,0);

    
    for(register unsigned int i=0;i<order*order;i++)
	values[i]=momentCentered.get()[i]/a;
    
}

/*!
  Default constructor.
*/
vpMomentCenteredNormalized::vpMomentCenteredNormalized() : vpMomentCentered(){

}
