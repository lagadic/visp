/****************************************************************************
 *
 * $Id$
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
 * Single laser scanner point.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpScanPoint_h
#define vpScanPoint_h

#include <math.h>
#include "visp/vpConfig.h"

/*!
  \file vpScanPoint.h

  \brief Implements a single laser scanner point.
*/

/*!

  \class vpScanPoint

  \brief Class that defines a single laser scanner point.

  This class stores data of a single scan point as:

  - cartesian coordinates in the 3D space that are available throw
    getX(), getY() and getZ() methods.

  - polar coordinates that are the native data provided by a laser
    scanner. By polar coordinates we mean here the radial distance and the
    horizontal angle of a point in the scanner layer and an additional
    vertical angle that gives the orientation of the layer.

*/
class VISP_EXPORT vpScanPoint
{
 public:
  /*! Default constructor. */
  vpScanPoint() {
    this->rDist = 0;
    this->hAngle = 0;
    this->vAngle = 0;
  }
  /*! Copy constructor. */
  vpScanPoint(const vpScanPoint &scanpoint) {
    this->rDist = scanpoint.rDist;
    this->hAngle = scanpoint.hAngle;
    this->vAngle = scanpoint.vAngle;
  }
  /*! 
    Set the polar point coordinates. 
    \param rDist : Radial distance in meter.
    \param hAngle : Horizontal angle in radian.
    \param vAngle : Vertical angle in radian.   
  */
  vpScanPoint(double rDist, double hAngle, double vAngle) {
    this->rDist = rDist;
    this->hAngle = hAngle;
    this->vAngle = vAngle;
  }
  /*! Destructor that does nothing. */
  virtual ~vpScanPoint() {};
  /*! 
    Set the polar point coordinates. 
    \param rDist : Radial distance in meter.
    \param hAngle : Horizontal angle in radian.
    \param vAngle : Vertical angle in radian.   
  */
  inline void setPolar(double rDist, double hAngle, double vAngle) {
    this->rDist = rDist;
    this->hAngle = hAngle;
    this->vAngle = vAngle;
  }
  /*! 
    Return the radial distance in meter.
  */
  inline double getRadialDist() const {
    return ( this->rDist );
  }
  /*! 
    Returns the polar elevation (vertical) angle in radian.
  */
  inline double getVAngle() const {
    return ( this->vAngle );
  }
  /*! 
    Returns the polar elevation (vertical) angle in radian.
  */
  inline double getHAngle() const {
    return ( this->hAngle );
  }
  /*! 
    Returns the cartesian x coordinate.

    The x and y axis define an horizontal plane, where x is oriented
    positive in front of the laser while y on the left side.
    
  */
  inline double getX() const {
    return ( rDist * cos(this->hAngle) * cos(this->vAngle)  );
  }
  /*! 
    Returns the cartesian y coordinate.

    The x and y axis define an horizontal plane, where x is oriented
    positive in front of the laser while y on the left side.
    
  */
  inline double getY() const {
    return ( rDist * sin(this->hAngle) );
  }
  /*! 
    Returns the cartesian z coordinate.

    The z axis is vertical and oriented in direction of the sky.
    
  */
  inline double getZ() const {
    return ( rDist * cos(this->hAngle) * sin(this->vAngle) );
  }
   
  /*!

    Print the values of the scan point on the output stream. Data are
    separated by a white space. Data that are print are first the
    polar coordinates, than the cartesian coordinates:
    - the radial distance in meter
    - the horizontal angle in radian
    - the vertical angle in radian
    - the cartesian X coordinate
    - the cartesian Y coordinate
    - the cartesian Z coordinate
    
    The following code
    \code
#include <iostream>
#include "visp/vpScanPoint.h"

int main()
{
  vpScanPoint p;
  double radialDistance = 3; // 3 meters
  double horizontalAngle = 1.12; // 1.12 radian
  double verticalAngle = 0; // 0 radian for a horizontal layer
  
  p.setPolar(radialDistance, horizontalAngle, verticalAngle);

  std::cout << p << std::endl;
}
   \endcode
    whill produce the prints 
    \code
"3 1.12 0 1.307047339 2.700301327 0"
    \endcode
    
   */
     friend VISP_EXPORT std::ostream &operator << (std::ostream &s, 
						   const vpScanPoint &p) {
     s.precision(10);
     s << p.getRadialDist() << " " 
       << p.getHAngle() << " " 
       << p.getVAngle() << " "
       << p.getX() << " " 
       << p.getY() << " " << p.getZ();
     return s;
   }
   /*!
     
     Returns true if sp1 and sp2 are equal; otherwire returns false.

   */
   friend VISP_EXPORT inline bool operator==( const vpScanPoint &sp1, 
					      const vpScanPoint &sp2 ) {
     return ( ( sp1.getRadialDist() == sp2.getRadialDist() ) 
	      && ( sp1.getHAngle() == sp2.getHAngle() )
	      && ( sp1.getVAngle() == sp2.getVAngle() ) );
   }
   
   /*!
     
     Returns true if sp1 and sp2 are different; otherwire returns false.

   */
   friend VISP_EXPORT inline bool operator!=( const vpScanPoint &sp1, 
					      const vpScanPoint &sp2 ) {
     return ( ( sp1.getRadialDist() != sp2.getRadialDist() )
	      || ( sp1.getHAngle() != sp2.getHAngle() )  
	      || ( sp1.getVAngle() != sp2.getVAngle() ) );
 }

 private:
   double rDist;
   double hAngle;
   double vAngle;
};

#endif
