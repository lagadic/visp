/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Single laser scanner point.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpScanPoint_h
#define vpScanPoint_h

#include <visp3/core/vpMath.h>

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <math.h>
#include <ostream>
#include <sstream>

/*!
  \file vpScanPoint.h

  \brief Implements a single laser scanner point.
*/

/*!

  \class vpScanPoint
  \ingroup group_sensor_laserscanner

  \brief Class that defines a single laser scanner point.

  This class stores data of a single scan point as:

  - cartesian coordinates in the 3D space that are available throw
    getX(), getY() and getZ() methods.

  - polar coordinates that are the native data provided by a laser
    scanner. By polar coordinates we mean here the radial distance and the
    horizontal angle of a point in the scanner layer and an additional
    vertical angle that gives the orientation of the layer.

*/
class /* VISP_EXPORT */ vpScanPoint // Note that here VISP_EXPORT should not
                                    // be added since this class is complete
                                    // inline
{
public:
  /*! Default constructor. */
  inline vpScanPoint() : rDist(0), hAngle(0), vAngle(0) {}
  /*! Copy constructor. */
  inline vpScanPoint(const vpScanPoint &scanpoint) : rDist(0), hAngle(0), vAngle(0)
  {
    this->rDist = scanpoint.rDist;
    this->hAngle = scanpoint.hAngle;
    this->vAngle = scanpoint.vAngle;
  }
  /*!
    Set the polar point coordinates.
    \param r_dist : Radial distance in meter.
    \param h_angle : Horizontal angle in radian.
    \param v_angle : Vertical angle in radian.
  */
  inline vpScanPoint(double r_dist, double h_angle, double v_angle) : rDist(r_dist), hAngle(h_angle), vAngle(v_angle)
  {
    this->rDist = r_dist;
    this->hAngle = h_angle;
    this->vAngle = v_angle;
  }
  /*! Destructor that does nothing. */
  inline virtual ~vpScanPoint(){};
  /*!
    Set the polar point coordinates.
    \param r_dist : Radial distance in meter.
    \param h_angle : Horizontal angle in radian.
    \param v_angle : Vertical angle in radian.
  */
  inline void setPolar(double r_dist, double h_angle, double v_angle)
  {
    this->rDist = r_dist;
    this->hAngle = h_angle;
    this->vAngle = v_angle;
  }
  /*!
    Return the radial distance in meter.
  */
  inline double getRadialDist() const { return (this->rDist); }
  /*!
    Returns the polar elevation (vertical) angle in radian.
  */
  inline double getVAngle() const { return (this->vAngle); }
  /*!
    Returns the polar elevation (vertical) angle in radian.
  */
  inline double getHAngle() const { return (this->hAngle); }
  /*!
    Returns the cartesian x coordinate.

    The x and y axis define an horizontal plane, where x is oriented
    positive in front of the laser while y on the left side.

  */
  inline double getX() const { return (rDist * cos(this->hAngle) * cos(this->vAngle)); }
  /*!
    Returns the cartesian y coordinate.

    The x and y axis define an horizontal plane, where x is oriented
    positive in front of the laser while y on the left side.

  */
  inline double getY() const { return (rDist * sin(this->hAngle)); }
  /*!
    Returns the cartesian z coordinate.

    The z axis is vertical and oriented in direction of the sky.

  */
  inline double getZ() const { return (rDist * cos(this->hAngle) * sin(this->vAngle)); }

  friend inline std::ostream &operator<<(std::ostream &s, const vpScanPoint &p);

  /*!

    Returns true if sp1 and sp2 are equal; otherwire returns false.

  */
  friend inline bool operator==(const vpScanPoint &sp1, const vpScanPoint &sp2)
  {
    // return ( ( sp1.getRadialDist() == sp2.getRadialDist() )
    //	      && ( sp1.getHAngle() == sp2.getHAngle() )
    //	      && ( sp1.getVAngle() == sp2.getVAngle() ) );
    double rd1 = sp1.getRadialDist();
    double ha1 = sp1.getHAngle();
    double va1 = sp1.getVAngle();
    double rd2 = sp2.getRadialDist();
    double ha2 = sp2.getHAngle();
    double va2 = sp2.getVAngle();

    return ((std::fabs(rd1 - rd2) <= std::fabs(vpMath::maximum(rd1, rd2)) * std::numeric_limits<double>::epsilon()) &&
            (std::fabs(ha1 - ha2) <= std::fabs(vpMath::maximum(ha1, ha2)) * std::numeric_limits<double>::epsilon()) &&
            (std::fabs(va1 - va2) <= std::fabs(vpMath::maximum(va1, va2)) * std::numeric_limits<double>::epsilon()));
  }

  /*!

    Returns true if sp1 and sp2 are different; otherwire returns false.

  */
  friend inline bool operator!=(const vpScanPoint &sp1, const vpScanPoint &sp2)
  {
    // return ( ( sp1.getRadialDist() != sp2.getRadialDist() )
    //     || ( sp1.getHAngle() != sp2.getHAngle() )
    //     || ( sp1.getVAngle() != sp2.getVAngle() ) );
    double rd1 = sp1.getRadialDist();
    double ha1 = sp1.getHAngle();
    double va1 = sp1.getVAngle();
    double rd2 = sp2.getRadialDist();
    double ha2 = sp2.getHAngle();
    double va2 = sp2.getVAngle();
    return ((std::fabs(rd1 - rd2) > std::fabs(vpMath::maximum(rd1, rd2)) * std::numeric_limits<double>::epsilon()) ||
            (std::fabs(ha1 - ha2) <= std::fabs(vpMath::maximum(ha1, ha2)) * std::numeric_limits<double>::epsilon()) ||
            (std::fabs(va1 - va2) <= std::fabs(vpMath::maximum(va1, va2)) * std::numeric_limits<double>::epsilon()));
  }

private:
  double rDist;
  double hAngle;
  double vAngle;
};

/*!
  \relates vpScanPoint

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
#include <visp3/sensor/vpScanPoint.h>

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
  will produce the prints
  \code
"3 1.12 0 1.307047339 2.700301327 0"
  \endcode

 */
inline std::ostream &operator<<(std::ostream &s, const vpScanPoint &p)
{
  std::ios_base::fmtflags original_flags = s.flags();

  s.precision(10);
  s << p.getRadialDist() << " " << p.getHAngle() << " " << p.getVAngle() << " " << p.getX() << " " << p.getY() << " "
    << p.getZ();

  s.setf(original_flags); // restore s to standard state

  return s;
}

#endif
