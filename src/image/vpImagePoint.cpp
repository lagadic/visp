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
 * 2D point useful for image processing
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/


#include <visp/vpConfig.h>
#include <visp/vpImagePoint.h>
#include <visp/vpRect.h>
#include <visp/vpHomography.h>

/*!

  Check if an image point belongs to a rectangle.
  
  \param rect : the rectangle.
  
  \return Returns true if the point belongs to the rectangle.

*/
bool vpImagePoint::inRectangle( const vpRect &rect ) const
{
  return ( this->i <= rect.getBottom() && 
	   this->i >= rect.getTop() &&
	   this->j <= rect.getRight() &&
	   this->j >= rect.getLeft());
}

/*!
  Project the current image point (in frame b) into the frame a using the
  homography aHb.

  \param aHb : Homography defining the relation between frame a and frame b.
  \return The projected image point in the frame a.
*/
vpImagePoint
vpImagePoint::projection(const vpHomography& aHb)
{
  vpImagePoint ap;

  double i_a = aHb[0][0] * i + aHb[0][1] * j + aHb[0][2];
  double j_a = aHb[1][0] * i + aHb[1][1] * j + aHb[1][2];
  double k_a = aHb[2][0] * i + aHb[2][1] * j + aHb[2][2];

  if(std::fabs(k_a) > std::numeric_limits<double>::epsilon()){
    ap.set_i(i_a / k_a);
    ap.set_j(j_a / k_a);
  }

  return ap;
}
