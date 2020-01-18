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
  \file vpHistogramValey.cpp
  \brief Declaration of the vpHistogramValey class.
  Class vpHistogramValey defines a gray level histogram valey.

*/

#include <visp3/core/vpHistogramValey.h>

/*!

  Copy operator.
  \param v : Histogram valey to copy.

  \code
  vpHistogramValey v1(0, 255);
  vpHistogramValey v2 = v1; // Valey p2 is set to 0, 255
  \endcode
*/
vpHistogramValey &vpHistogramValey::operator=(const vpHistogramValey &v)
{
  setLevel(v.level);
  setValue(v.value);

  return *this;
}

/*!

  Comparison operator.

  \param v : Gray level histogram valey to compar.

*/
bool vpHistogramValey::operator==(const vpHistogramValey &v) const
{
  return ((level == v.level) && (value == v.value));
}

/*!
  \relates vpHistogramValey
  \brief std::cout a valey
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &s, const vpHistogramValey &v)
{

  s << (int)v.getLevel() << " " << v.getValue();

  return s;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
