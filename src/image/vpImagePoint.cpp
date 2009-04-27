/****************************************************************************
 *
 * $Id$
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
 * 2D point useful for image processing
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpImagePoint.h>
#include <visp/vpDebug.h>


/*!
  \file vpImagePoint.cpp
  \brief class that defines a 2D point in an image. This class is useful
  for image processing.
*/


void
vpImagePoint::init()
{
  iP.resize(2) ;
  iP = 0;
}

vpImagePoint::vpImagePoint()
{
  init();
}

vpImagePoint::vpImagePoint(const vpImagePoint &ip)
{
  init();

  this->iP = ip.iP;
}

/*!
  \relates vpImagePoint

  Returns true if ip1 and ip2 are equal; otherwire returns false.

*/
bool operator==( const vpImagePoint &ip1, const vpImagePoint &ip2 )
{
  return ( ( ip1.get_i() == ip2.get_j() ) && ( ip1.get_i() == ip2.get_j() ) );
}

/*!

  \relates vpImagePoint

  Returns true if ip1 and ip2 are different; otherwire returns true.

*/
bool operator!=( const vpImagePoint &ip1, const vpImagePoint &ip2 )
{
  return ( ( ip1.get_i() != ip2.get_j() ) || ( ip1.get_i() != ip2.get_j() ) );
}

/*!

  \relates vpImagePoint

  Writes the image point coordinates \e ip to the stream \e os, and
  returns a reference to the stream. Writes the first coordinate along
  the \e i axis and then the second one along the \e j axis. The
  coordinates are separated by a comma.

  The following code
  \code
#include <iostream>

#include <visp/vpImagePoint.h>
int main()
{
  vpImagePoint ip;

  ip.set_i(10);
  ip.set_j(11.1);

  std::cout << "Image point with coordinates: " << ip << std::endl;

  return 0;
}
  \endcode

  The previous sample code produces the output:
  \verbatim
Image point with coordinates: 10, 11.1
  \endverbatim
*/

std::ostream& operator<< (std::ostream &os, 
			  const vpImagePoint& ip)
{
  os << ip.get_i() << ", " << ip.get_j(); 
  return os;
}
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
