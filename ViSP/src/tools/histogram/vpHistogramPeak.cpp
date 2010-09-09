/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
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
  \file vpHistogramPeak.cpp
  \brief Declaration of the vpHistogramPeak class.
  Class vpHistogramPeak defines a gray level histogram peak.

*/

#include <visp/vpHistogramPeak.h>


/*!
  Defaut constructor for a gray level histogram peak.
*/
vpHistogramPeak::vpHistogramPeak()
{
  level = 0;
  value = 0;
}

/*!
  Defaut constructor for a gray level histogram peak.
*/
vpHistogramPeak::vpHistogramPeak(unsigned char level, unsigned value)
{
  setLevel(level);
  setValue(value);
}
/*!
  Copy constructor of a gray level histogram peak.
*/
vpHistogramPeak::vpHistogramPeak(const vpHistogramPeak &p)
{
  setLevel(p.level);
  setValue(p.value);
}

/*!

  Copy operator.
  \param p : Histogram peak to copy.

  \code
  vpHistogramPeak p1(0, 255);
  vpHistogramPeak p2 = p1; // Peak p2 is set to 0, 255
  \endcode
*/
vpHistogramPeak &
vpHistogramPeak::operator=(const vpHistogramPeak &p)
{
  setLevel(p.level);
  setValue(p.value);

  return *this;
}

/*!

  Comparison operator.

  \param p : Gray level histogram peak to compar.

*/
bool
vpHistogramPeak::operator==(const vpHistogramPeak &p) const
{
  return ( (level == p.level) && (value == p.value) );
}

/*!
  \brief std::cout a peak
*/
std::ostream &operator <<(std::ostream &s,const vpHistogramPeak &p)
{
  
  s << (int)p.getLevel() << " " << p.getValue();

  return s;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
