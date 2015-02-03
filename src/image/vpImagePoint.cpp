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

/*!

  Operator +=.

  This operator can be used to compute the center of gravity of a set of image points.

  \code
#include <iostream>
#include <vector>
#include <visp/vpImagePoint.h>

int main()
{
std::vector<vpImagePoint> ip(2);

ip[0].set_ij(100, 200);
ip[1].set_ij(300, 400);

vpImagePoint cog(0,0);
for(unsigned int i=0; i<ip.size(); i++)
  cog += ip[i];
cog /= ip.size();
std::cout << "cog: " << cog << std::endl;
}
  \endcode

*/
vpImagePoint& vpImagePoint::operator+=(const vpImagePoint &ip) {
  this->i += ip.i;
  this->j += ip.j;
  return *this;
}

/*!

  Operator /=.

  This operator can be used to compute the center of gravity of a set of image points.
  \code
#include <iostream>
#include <vector>
#include <visp/vpImagePoint.h>

int main()
{
std::vector<vpImagePoint> ip(2);

ip[0].set_ij(100, 200);
ip[1].set_ij(300, 400);

vpImagePoint cog(0,0);
for(unsigned int i=0; i<ip.size(); i++)
  cog += ip[i];
cog /= ip.size();
std::cout << "cog: " << cog << std::endl;
}
  \endcode

*/
vpImagePoint& vpImagePoint::operator/=(const double scale) {
  this->i /= scale;
  this->j /= scale;
  return *this;
}

/*!
  \relates vpImagePoint

  Returns true if ip1 and ip2 are equal; otherwire returns false.

*/
VISP_EXPORT bool operator==( const vpImagePoint &ip1, const vpImagePoint &ip2 ) {
  //return ( ( ip1.get_i() == ip2.get_i() ) && ( ip1.get_j() == ip2.get_j() ) );

  double i1 = ip1.get_i();
  double j1 = ip1.get_j();
  double i2 = ip2.get_i();
  double j2 = ip2.get_j();

  return (
    ( std::fabs(i1-i2) <= std::fabs(vpMath::maximum(i1, i2))*std::numeric_limits<double>::epsilon() )
    &&
    ( std::fabs(j1-j2) <= std::fabs(vpMath::maximum(j1, j2))*std::numeric_limits<double>::epsilon() )
    );
}

/*!

  \relates vpImagePoint

  Returns true if ip1 and ip2 are different; otherwire returns true.

*/
VISP_EXPORT bool operator!=( const vpImagePoint &ip1, const vpImagePoint &ip2 ) {
  //return ( ( ip1.get_i() != ip2.get_i() ) || ( ip1.get_j() != ip2.get_j() ) );
  double i1 = ip1.get_i();
  double j1 = ip1.get_j();
  double i2 = ip2.get_i();
  double j2 = ip2.get_j();

  return (
    ( std::fabs(i1-i2) > std::fabs(vpMath::maximum(i1, i2))*std::numeric_limits<double>::epsilon() )
    ||
    ( std::fabs(j1-j2) > std::fabs(vpMath::maximum(j1, j2))*std::numeric_limits<double>::epsilon() )
    );
}

/*!

  \relates vpImagePoint

  Returns a vpImagePoint wich is the sum of \f$ ip1 \f$ and \f$ ip2 \f$.

*/
VISP_EXPORT vpImagePoint operator+( const vpImagePoint &ip1, const vpImagePoint &ip2 ) {
  return ( vpImagePoint(ip1.get_i()+ip2.get_i(), ip1.get_j()+ip2.get_j()));
}
/*!

  \relates vpImagePoint

  Returns a vpImagePoint wich is the sum of \f$ ip1 \f$ and \f$ ip2 \f$.

*/
VISP_EXPORT vpImagePoint operator+=( const vpImagePoint &ip1, const vpImagePoint &ip2 ) {
  return ( vpImagePoint(ip1.get_i()+ip2.get_i(), ip1.get_j()+ip2.get_j()));
}
/*!

  \relates vpImagePoint

  Returns a vpImagePoint with an offset added to the two coordinates.

  \code
#include <iostream>
#include <visp/vpImagePoint.h>

int main()
{
  vpImagePoint ip(100, 200); // Create an image point with coordinates i=100, j=200
  std::cout << "ip: " << ip << std::endl; // coordinates (100, 200)
  std::cout << "ip+10: " << ip+10 << std::endl; // new coordinates (110, 210)

  return 0;
}
  \endcode
*/
VISP_EXPORT vpImagePoint operator+( const vpImagePoint &ip1, const int offset ) {
  return ( vpImagePoint(ip1.get_i()+offset, ip1.get_j()+offset));
}
/*!

  \relates vpImagePoint

  Returns a vpImagePoint with an offset added to the two coordinates.

  \code
#include <iostream>
#include <visp/vpImagePoint.h>

int main()
{
  vpImagePoint ip(100, 200); // Create an image point with coordinates i=100, j=200
  std::cout << "ip: " << ip << std::endl; // coordinates (100, 200)
  std::cout << "ip+12.34: " << ip+12.34 << std::endl; // new coordinates (112.34, 212.34)

  return 0;
}
  \endcode
*/
VISP_EXPORT vpImagePoint operator+( const vpImagePoint &ip1, const double offset ) {
  return ( vpImagePoint(ip1.get_i()+offset, ip1.get_j()+offset));
}

/*!

  \relates vpImagePoint

  Returns a vpImagePoint wich is the difference between \f$ ip1 \f$ and \f$ ip2 \f$.

*/
VISP_EXPORT vpImagePoint operator-( const vpImagePoint &ip1, const vpImagePoint &ip2 ) {
  return ( vpImagePoint(ip1.get_i()-ip2.get_i(), ip1.get_j()-ip2.get_j()));
}
/*!

  \relates vpImagePoint

  Returns a vpImagePoint with an offset substracted to the two coordinates.

  \code
#include <iostream>
#include <visp/vpImagePoint.h>

int main()
{
  vpImagePoint ip(100, 200); // Create an image point with coordinates i=100, j=200
  std::cout << "ip: " << ip << std::endl; // coordinates (100, 200)
  std::cout << "ip-10: " << ip-10 << std::endl; // new coordinates (90, 190)

  return 0;
}
  \endcode
*/
VISP_EXPORT vpImagePoint operator-( const vpImagePoint &ip1, const int offset ) {
  return ( vpImagePoint(ip1.get_i()-offset, ip1.get_j()-offset));
}
/*!

  \relates vpImagePoint

  Returns a vpImagePoint with an offset substracted to the two coordinates.

  \code
#include <iostream>
#include <visp/vpImagePoint.h>

int main()
{
  vpImagePoint ip(100, 200); // Create an image point with coordinates i=100, j=200
  std::cout << "ip: " << ip << std::endl; // coordinates (100, 200)
  std::cout << "ip-12.34: " << ip-12.34 << std::endl; // new coordinates (87.66, 187.66)

  return 0;
}
  \endcode
*/
VISP_EXPORT vpImagePoint operator-( const vpImagePoint &ip1, const double offset ) {
  return ( vpImagePoint(ip1.get_i()-offset, ip1.get_j()-offset));
}
/*!

  \relates vpImagePoint

  Returns a vpImagePoint with coordinates multiplied by a scale factor.

  \code
#include <iostream>
#include <visp/vpImagePoint.h>

int main()
{
  vpImagePoint ip(100, 200); // Create an image point with coordinates i=100, j=200
  std::cout << "ip: " << ip << std::endl; // coordinates (100, 200)
  std::cout << "ip*2: " << ip*2 << std::endl; // new coordinates (200, 400)

  return 0;
}
  \endcode
*/
VISP_EXPORT vpImagePoint operator*( const vpImagePoint &ip1, const double scale ) {
  return ( vpImagePoint(ip1.get_i()*scale, ip1.get_j()*scale));
}
/*!

  \relates vpImagePoint

  Returns a vpImagePoint with coordinates divided by a scale factor.

  \code
#include <iostream>
#include <visp/vpImagePoint.h>

int main()
{
  vpImagePoint ip(100, 200); // Create an image point with coordinates i=100, j=200
  std::cout << "ip: " << ip << std::endl; // coordinates (100, 200)
  std::cout << "ip/2: " << ip/2 << std::endl; // new coordinates (50, 100)

  return 0;
}
  \endcode
*/
VISP_EXPORT vpImagePoint operator/( const vpImagePoint &ip1, const double scale ) {
  return ( vpImagePoint(ip1.get_i()/scale, ip1.get_j()/scale));
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
VISP_EXPORT std::ostream& operator<< (std::ostream &os, const vpImagePoint& ip)
{
  os << ip.get_i() << ", " << ip.get_j();
  return os;
}

/**
 * Computes and returns the bounding box.
 * @param ipVec : Vector of input image points.
 * @return Bounding box of the points.
 */
vpRect vpImagePoint::getBBox(const std::vector<vpImagePoint>& ipVec)
{
  vpRect rec(ipVec);

  return rec;
}
