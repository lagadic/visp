/****************************************************************************
 *
 * $Id: vpHistogramValey.cpp,v 1.1 2007-03-02 13:05:11 fspindle Exp $
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


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
