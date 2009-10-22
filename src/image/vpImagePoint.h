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

#ifndef vpImagePoint_H
#define vpImagePoint_H

/*!
  \file vpImagePoint.h
  \brief Class that defines a 2D point in an image. This class is useful
  for image processing
*/

#include <ostream>

#include <visp/vpConfig.h>
#include <visp/vpMath.h>


/*!
  \class vpImagePoint
  \ingroup TrackingFeature GeometryFeature

  \brief Class that defines a 2D point in an image. This class is
  useful for image processing and stores only the <B>2D coordinates
  given in sub-pixel</B>.

  \warning If you want to define a point thanks to its coordinates
  given in meter in the object frame, the camera frame or the image
  plane, you have to use the class vpPoint. 

  In this class, the 2D coordinates are not necessary integer
  values. It is easy to manipulate the given coordinates in the two
  frames used in ViSP : the (i,j) coordinates and the (u,v)
  coordinates.  The two following images illustrate the two coordinate
  systems.

  \image html vpImagePoint.gif
  \image latex vpImagePoint.ps  width=10cm

  \warning <B>An instance of the vpImagePoint class corresponds to a
  particular point. Thus, if you change the point coordinate using the
  method set_i(const double i), it produces the same effect than if
  you used the method set_v(const double v). These two methods change
  the same private attribute. It is also true for the two methods
  set_j(const double j) and set_u(const double u).</B>
*/


class VISP_EXPORT vpImagePoint
{
 public:
  /*!
    Default constructor that initialize the coordinates of the image
    point to zero.
  */
  inline vpImagePoint() {
    i = 0;
    j = 0;
  }
  /*!
    Default constructor that initialize the coordinates of the image
    thanks to the parameters \f$ i \f$ and \f$ j \f$.
  */
  inline vpImagePoint(double i, double j) {
    this->i = i;
    this->j = j;
  }
  /*!
    Copy constructor.

    Initialize the coordinates of the image point with \e ip.

    \param ip : An image point.
  */
  inline vpImagePoint(const vpImagePoint &ip) {
    this->i = ip.i;
    this->j = ip.j;
  }
  //! Destructor.
  inline virtual ~vpImagePoint() { ; }

  /*!
  
    Copy operator.

  */
  inline const vpImagePoint& operator=(const vpImagePoint &ip) {
    this->i = ip.i;
    this->j = ip.j;
    return *this;
  }

  /*!

    Sets the point coordinate corresponding to the \f$ i \f$ axes in
    the frame (i,j).

    \param i : The desired value for the coordinate along the \f$ i \f$ axes.

    \sa set_j(), set_u(), set_v()
  */
  inline void set_i(const double i) {  this->i = i ; }

  /*!

    Sets the point coordinate corresponding to the \f$ j \f$ axes in
    the frame (i,j).

    \param j : The desired value for the coordinate along the \f$ j \f$ axes.

    \sa set_i(), set_u(), set_v()
  */
  inline void set_j(const double j) {  this->j = j ; }

  /*!

    Sets the point coordinates in the frame (i,j).

    \param i : The desired value for the coordinate along the \f$ i \f$ axes.
    \param j : The desired value for the coordinate along the \f$ j \f$ axes.

    \sa set_i(), set_j(), set_u(), set_v()
  */
  inline void set_ij(const double i, const double j) {
    this->i = i ;
    this->j = j ;
  }

  /*!

    Gets the point coordinate corresponding to the \f$ i \f$ axes in
    the frame (i,j).

    \return The value of the coordinate along the \f$ i \f$ axes.

    \sa get_j(), get_u(), get_v()
  */
  inline double get_i()  const { return i ; }

  /*!

    Gets the point coordinate corresponding to the \f$ j \f$ axes in
    the frame (i,j).

    \return The value of the coordinate along the \f$ j \f$ axes.

    \sa get_i(), get_u(), get_v()
  */
  inline double get_j()  const { return j ; }

  /*!

    Sets the point coordinate corresponding to the \f$ u \f$ axes in
    the frame (u,v).

    \param u : The desired value for the coordinate along the \f$ u \f$ axes.

    \sa set_i(), set_j(), set_v()
  */
  inline void set_u(const double u) {  j = u ; }

  /*!

    Sets the point coordinate corresponding to the \f$ v \f$ axes in
    the frame (u,v).

    \param v : The desired value for the coordinate along the \f$ v \f$ axes.

    \sa set_i(), set_j(), set_u()
  */
  inline void set_v(const double v) {  i = v ; }

  /*!

    Sets the point coordinates in the frame (u,v).

    \param u : The desired value for the coordinate along the \f$ u \f$ axes.
    \param v : The desired value for the coordinate along the \f$ v \f$ axes.

    \sa set_i(), set_j(), set_u(), set_v()
  */
  inline void set_uv(const double u, const double v) {
    this->i = v ;
    this->j = u ;
  }

  /*!

    Gets the point coordinate corresponding to the \f$ u \f$ axes in
    the frame (u,v).

    \return The value of the coordinate along the \f$ u \f$ axes.

    \sa get_i(), get_j(), get_v()
  */
  inline double get_u()  const { return j ; }

  /*!

    Gets the point coordinate corresponding to the \f$ v \f$ axes in
    the frame (u,v).

    \return The value of the coordinate along the \f$ v \f$ axes.

    \sa get_i(), get_j(), get_u()
  */
  inline double get_v()  const { return i; }

  /*!

    Compute the distance \f$ |iP1 - iP2| = \sqrt{(i_1-i_2)^2+(j_1-j_2)^2} \f$

    \param iP1 : First point
    \param iP2 : Second point

    \return the distance between the two points.
  */
  static double distance (const vpImagePoint iP1, const vpImagePoint iP2) {
    return(sqrt(vpMath::sqr(iP1.get_i()-iP2.get_i())+vpMath::sqr(iP1.get_j()-iP2.get_j())));}
    
  /*!

    Compute the distance \f$ |iP1 - iP2| = (i_1-i_2)^2+(j_1-j_2)^2 \f$

    \param iP1 : First point
    \param iP2 : Second point

    \return the distance between the two points.
  */
  static double sqrDistance (const vpImagePoint iP1, const vpImagePoint iP2) {
    return(vpMath::sqr(iP1.get_i()-iP2.get_i())+vpMath::sqr(iP1.get_j()-iP2.get_j()));}

 private:
  double i,j;
};

/*!
  \relates vpImagePoint

  Returns true if ip1 and ip2 are equal; otherwire returns false.

*/
VISP_EXPORT inline bool operator==( const vpImagePoint &ip1, 
				    const vpImagePoint &ip2 ) {
  return ( ( ip1.get_i() == ip2.get_i() ) && ( ip1.get_j() == ip2.get_j() ) );
}

/*!

  \relates vpImagePoint

  Returns true if ip1 and ip2 are different; otherwire returns true.

*/
VISP_EXPORT inline bool operator!=( const vpImagePoint &ip1, 
				    const vpImagePoint &ip2 ) {
  return ( ( ip1.get_i() != ip2.get_i() ) || ( ip1.get_j() != ip2.get_j() ) );
}

/*!

  \relates vpImagePoint

  Returns a vpImagePoint wich is the sum of \f$ ip1 \f$ and \f$ ip2 \f$.

*/
VISP_EXPORT inline vpImagePoint operator+( const vpImagePoint &ip1, 
				           const vpImagePoint &ip2 ) {
  return ( vpImagePoint(ip1.get_i()+ip2.get_i(), ip1.get_j()+ip2.get_j()));
}

/*!

  \relates vpImagePoint

  Returns a vpImagePoint wich is the difference between \f$ ip1 \f$ and \f$ ip2 \f$.

*/
VISP_EXPORT inline vpImagePoint operator-( const vpImagePoint &ip1, 
				           const vpImagePoint &ip2 ) {
  return ( vpImagePoint(ip1.get_i()-ip2.get_i(), ip1.get_j()-ip2.get_j()));
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
VISP_EXPORT inline std::ostream& operator<< (std::ostream &os,
					     const vpImagePoint& ip)
 {
  os << ip.get_i() << ", " << ip.get_j();
  return os;
}


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
