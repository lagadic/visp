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
 * Gray level histogram manipulation.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpHistogramValey.cpp
  \brief Declaration of the vpHistogramValey class.
  Class vpHistogramValey defines a gray level histogram valey.

*/

#include <visp/vpHistogramValey.h>

/*!

  Copy operator.
  \param v : Histogram valey to copy.

  \code
  vpHistogramValey v1(0, 255);
  vpHistogramValey v2 = v1; // Valey p2 is set to 0, 255
  \endcode
*/
vpHistogramValey &
vpHistogramValey::operator=(const vpHistogramValey &v)
{
  setLevel(v.level);
  setValue(v.value);

  return *this;
}

/*!

  Comparison operator.

  \param v : Gray level histogram valey to compar.

*/
bool
vpHistogramValey::operator==(const vpHistogramValey &v) const
{
  return ( (level == v.level) && (value == v.value) );
}

/*!
  \brief std::cout a valey
*/
std::ostream &operator <<(std::ostream &s,const vpHistogramValey &v)
{
  
  s << (int)v.getLevel() << " " << v.getValue();

  return s;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
