/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#include <visp3/core/vpHistogramPeak.h>

/*!
  Defaut constructor for a gray level histogram peak.
*/
vpHistogramPeak::vpHistogramPeak() : level(0), value(0) {}

/*!
  Defaut constructor for a gray level histogram peak.
*/
vpHistogramPeak::vpHistogramPeak(unsigned char lvl, unsigned val) : level(lvl), value(val) {}

/*!
  Copy constructor of a gray level histogram peak.
*/
vpHistogramPeak::vpHistogramPeak(const vpHistogramPeak &p) : level(0), value(0) { *this = p; }

/*!

  Copy operator.
  \param p : Histogram peak to copy.

  \code
  vpHistogramPeak p1(0, 255);
  vpHistogramPeak p2 = p1; // Peak p2 is set to 0, 255
  \endcode
*/
vpHistogramPeak &vpHistogramPeak::operator=(const vpHistogramPeak &p)
{
  setLevel(p.level);
  setValue(p.value);

  return *this;
}

/*!

  Comparison operator.

  \param p : Gray level histogram peak to compar.

*/
bool vpHistogramPeak::operator==(const vpHistogramPeak &p) const { return ((level == p.level) && (value == p.value)); }

/*!
  \relates vpHistogramPeak
  \brief std::cout a peak
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &s, const vpHistogramPeak &p)
{

  s << (int)p.getLevel() << " " << p.getValue();

  return s;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
