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


#include <visp/vpConfig.h>
#include <visp/vpColVector.h>


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
  vpImagePoint() ;
  /*!
    Copy constructor.

    Initialize the coordinates of the image point with \e ip.

    \param ip : An image point.
  */
  vpImagePoint(const vpImagePoint &ip);

  //! Destructor.
  virtual ~vpImagePoint() { ; }

  /*!
  
    Copy operator.

  */
  const vpImagePoint& operator=(const vpImagePoint &ip) {
    this->iP = ip.iP;
    return *this;
  }

  /*!
    Sets the point coordinate corresponding to the \f$ i \f$ axes in the frame (i,j).

    \param i : The desired value for the coordinate along the \f$ i \f$ axes.

    \sa set_j(), set_u(), set_v()
  */
  inline void set_i(const double i) {  iP[0] = i ; }

  /*!
    Sets the point coordinate corresponding to the \f$ j \f$ axes in the frame (i,j).

    \param j : The desired value for the coordinate along the \f$ j \f$ axes.

    \sa set_i(), set_u(), set_v()
  */
  inline void set_j(const double j) {  iP[1] = j ; }

  /*!
    Gets the point coordinate corresponding to the \f$ i \f$ axes in the frame (i,j).

    \return The value of the coordinate along the \f$ i \f$ axes.

    \sa get_j(), get_u(), get_v()
  */
  inline double get_i()  const { return iP[0] ; }

  /*!
    Gets the point coordinate corresponding to the \f$ j \f$ axes in the frame (i,j).

    \return The value of the coordinate along the \f$ j \f$ axes.

    \sa get_i(), get_u(), get_v()
  */
  inline double get_j()  const { return iP[1] ; }

  /*!
    Sets the point coordinate corresponding to the \f$ u \f$ axes in the frame (u,v).

    \param u : The desired value for the coordinate along the \f$ u \f$ axes.

    \sa set_i(), set_j(), set_v()
  */
  inline void set_u(const double u) {  iP[1] = u ; }

  /*!
    Sets the point coordinate corresponding to the \f$ v \f$ axes in the frame (u,v).

    \param v : The desired value for the coordinate along the \f$ v \f$ axes.

    \sa set_i(), set_j(), set_u()
  */
  inline void set_v(const double v) {  iP[0] = v ; }

  /*!
    Gets the point coordinate corresponding to the \f$ u \f$ axes in the frame (u,v).

    \return The value of the coordinate along the \f$ u \f$ axes.

    \sa get_i(), get_j(), get_v()
  */
  inline double get_u()  const { return iP[1] ; }

  /*!
    Gets the point coordinate corresponding to the \f$ v \f$ axes in the frame (u,v).

    \return The value of the coordinate along the \f$ v \f$ axes.

    \sa get_i(), get_j(), get_u()
  */
  inline double get_v()  const { return iP[0] ; }

 private:
  //! basic construction
  void init() ;


  vpColVector iP;


};

VISP_EXPORT bool operator==( const vpImagePoint &ip1, 
			     const vpImagePoint &ip2 );
VISP_EXPORT bool operator!=( const vpImagePoint &ip1, 
			     const vpImagePoint &ip2 );

VISP_EXPORT std::ostream& operator<< (std::ostream &os, 
				      const vpImagePoint& ip);

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
