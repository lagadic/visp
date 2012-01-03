/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Basic moment descriptor
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp/vpMomentBasic.h>
#include <visp/vpMomentObject.h>
/*!
  Default constructor.
*/
vpMomentBasic::vpMomentBasic() : vpMoment(){

}

/*!
  Retrieve all moments of all orders computed. vpMomentBasic::get()[j*order+i] refers to moment \f$m_{ij}\f$.
  \return all computed moments.

  Same behaviour as vpMomentObject.
*/
std::vector<double>& vpMomentBasic::get(){
    return getObject().get();
}

/*!
  Gets the desired moment using indexes. 
  \param i : first index of the 2D moment.
  \param j : second index of the 2D moment.
  \return \f$m_{ij}\f$ moment.

  Same behaviour as vpMomentObject.
*/
double vpMomentBasic::get(unsigned int i,unsigned int j){
    return getObject().get(i,j);
}

/*!
  Dummy function. Everything is already done in object. 
*/
void vpMomentBasic::compute(){

}

/*!
  Outputs the moment's values to a stream.
  Same output as in vpMomentObject.
*/
std::ostream & operator<<(std::ostream & os, vpMomentBasic& m){

    os << m.getObject();
    
    return os;
}
