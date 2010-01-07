/****************************************************************************
 *
 * $Id: vpHistogramPeak.cpp,v 1.2 2007-09-17 09:15:57 fspindle Exp $
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
 * This file is part of the ViSP toolkit
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
