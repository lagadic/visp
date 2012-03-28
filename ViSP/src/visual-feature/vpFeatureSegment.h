/****************************************************************************
 *
 * $Id: vpFeatureThetaU.h 3530 2012-01-03 10:52:12Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Segment visual feature.
 *
 * Authors:
 * Filip Novotny
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpFeatureSegment_H
#define vpFeatureSegment_H

/*!
  \file vpFeatureSegment.h
  \brief class that defines the Segment visual feature
*/

#include <visp/vpMatrix.h>
#include <visp/vpPoint.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpRGBa.h>
#include <visp/vpFeatureException.h>

/*!
  \class vpFeatureSegment
  \ingroup VsFeature2

  \brief Class that defines a 2D segment visual feature \f${\bf s} = (x_c, y_c, l, \alpha)\f$, where
  \f$(x_c,y_c)\f$ are the coordinates of the segment center, \f$ l \f$ the segment length
  and \f$ \alpha \f$ the orientation of the segment with respect to the \f$ x \f$ axis.

*/
class VISP_EXPORT vpFeatureSegment : public vpBasicFeature
{
public:
  //empty constructor
  vpFeatureSegment();

  //! Destructor. Does nothing.
  ~vpFeatureSegment() { if (flags != NULL) delete [] flags; }
  // change values of the segment
  void buildFrom(const double x1, const double y1, const double Z1, const double x2, const double y2, const double Z2);

  void display(const vpCameraParameters &cam,
               const vpImage<unsigned char> &I,
               const vpColor &color=vpColor::green,
               unsigned int thickness=1) const ;
  void display(const vpCameraParameters &cam,
               const vpImage<vpRGBa> &I,
               const vpColor &color=vpColor::green,
               unsigned int thickness=1) const ;
  //! Feature duplication.
  vpFeatureSegment *duplicate() const ;
  // compute the error between two visual features from a subset
  // a the possible features
  vpColVector error(const vpBasicFeature &s_star,
                    const unsigned int select = FEATURE_ALL)  ;

  /*!
      Get the value of \f$ x_c \f$ which represents the x coordinate of the segment center in the image plane.

      \return The value of \f$ x_c \f$.
    */
  inline double getXc() const { return s[0] ;}
  /*!
        Get the value of \f$ y_c \f$ which represents the y coordinate of the segment center in the image plane.

        \return The value of \f$ y_c \f$.
    */
  inline double getYc() const { return s[1] ;}
  /*!
        Get the value of \f$ l \f$ which represents the length of the segment.

        \return The value of \f$ l \f$.
    */
  inline double getL() const { return s[2] ;}
  /*!
        Get the value of \f$ \alpha \f$ which represents the orientation of the segment.

        \return The value of \f$ \alpha \f$.
    */
  inline double getAlpha() const { return s[3] ;}

  /*!
      Get the value of \f$ Z_1 \f$ which represents the Z coordinate in the camera frame
      of the 3D point that corresponds to the segment first point.

      \return The value of \f$ Z_1 \f$.
    */
  inline double getZ1() const { return Z1 ;}

  /*!
      Get the value of \f$ Z_2 \f$ which represents the Z coordinate in the camera frame
      of the 3D point that corresponds to the segment second point.

      \return The value of \f$ Z_2 \f$.
    */
  inline double getZ2() const { return Z2 ;}

  // Basic construction.
  void init() ;

  // compute the interaction matrix from a subset a the possible features
  vpMatrix  interaction(const unsigned int select = FEATURE_ALL);

  void print(const unsigned int select= FEATURE_ALL) const ;



  /*!

    Function used to select the \f$x_c\f$ subfeature.

    This function is to use in conjunction with interaction() in order
    to compute the interaction matrix associated to \f$x_c\f$ feature.

    See the interaction() method for an usage example.

    This function is also useful in the vpServo class to indicate that
    a subset of the visual feature is to use in the control law:

    \code
  vpFeatureSegment s, s_star; // Current and desired visual feature
  vpServo task;
  ...
  // Add only the xc subset feature from a segment to the task
  task.addFeature(s, s_star, vpFeatureSegment::selectXc());
    \endcode

    \sa selectYc(), selectL(), selectAlpha()
  */
  inline static unsigned int selectXc()  { return FEATURE_LINE[0] ; }

  /*!

    Function used to select the \f$y_c\f$ subfeature.

    This function is to use in conjunction with interaction() in order
    to compute the interaction matrix associated to \f$y_c\f$ feature.

    See the interaction() method for an usage example.

    This function is also useful in the vpServo class to indicate that
    a subset of the visual feature is to use in the control law:

    \code
  vpFeatureSegment s, s_star; // Current and desired visual feature
  vpServo task;
  ...
  // Add only the yc subset feature from a segment to the task
  task.addFeature(s, s_star, vpFeatureSegment::selectYc());
    \endcode

    \sa selectXc(), selectL(), selectAlpha()
  */

  inline static unsigned int selectYc()  { return FEATURE_LINE[1] ; }

  /*!

    Function used to select the \f$l\f$ subfeature.

    This function is to use in conjunction with interaction() in order
    to compute the interaction matrix associated to \f$l\f$ feature.

    See the interaction() method for an usage example.

    This function is also useful in the vpServo class to indicate that
    a subset of the visual feature is to use in the control law:

    \code
  vpFeatureSegment s, s_star; // Current and desired visual feature
  vpServo task;
  ...
  // Add only the l subset feature from a segment to the task
  task.addFeature(s, s_star, vpFeatureSegment::selectL());
    \endcode

    \sa selectXc(), selectYc(), selectAlpha()
  */

  inline static unsigned int selectL()  { return FEATURE_LINE[2] ; }

  /*!

    Function used to select the \f$\alpha\f$ subfeature.

    This function is to use in conjunction with interaction() in order
    to compute the interaction matrix associated to \f$\alpha\f$ feature.

    See the interaction() method for an usage example.

    This function is also useful in the vpServo class to indicate that
    a subset of the visual feature is to use in the control law:

    \code
  vpFeatureSegment s, s_star; // Current and desired visual feature
  vpServo task;
  ...
  // Add only the alpha subset feature from a segment to the task
  task.addFeature(s, s_star, vpFeatureSegment::selectAlpha());
    \endcode

    \sa selectXc(), selectYc(), selectL()
  */

  inline static unsigned int selectAlpha() { return FEATURE_LINE[3] ; }
  
  /*!

    Set the value of \f$ x_c \f$ which represents the x coordinate of the segment center
    in the image plane. It is one parameter of the visual feature \f$ s \f$.

    \param val : \f$ x_c \f$ value to set.
  */
  inline void setXc(const double val){
    s[0] = xc = val;
    flags[0] = true;
  }
  /*!

    Set the value of \f$ y_c \f$ which represents the y coordinate of the segment center
    in the image plane. It is one parameter of the visual feature \f$ s \f$.

    \param val : \f$ y_c \f$ value to set.
  */
  inline void setYc(const double val){
    s[1] = yc = val;
    flags[1] = true;
  }
  /*!

    Set the value of \f$ l \f$ which represents the length of the segment
    in the image plane. It is one parameter of the visual feature \f$ s \f$.

    \param val : \f$ l \f$ value to set.
  */
  inline void setL(const double val){
    s[2] = l = val;
    flags[2] = true;
  }
  /*!

    Set the value of \f$ \alpha \f$ which represents the orientation of the segment
    in the image plane. It is one parameter of the visual feature \f$ s \f$.

    \param val : \f$ \alpha \f$ value to set.
  */
  inline void setAlpha(const double val){
    s[3] = alpha = val;
    cos_a = cos(val);
    sin_a = sin(val);
    flags[3] = true;
  }

  /*!

    Set the value of \f$ Z_1 \f$ which represents the Z coordinate in the camera frame
    of the 3D point that corresponds to the segment first point.

    This value is requested to compute the interaction matrix.

    \param val : \f$ Z_1 \f$ value to set.

    \exception vpFeatureException::badInitializationError : If Z1 is behind the camera or equal to zero.
  */
  inline void setZ1(const double val)
  {
    Z1 = val;

    if (Z1 < 0)
    {
      vpERROR_TRACE("Point is behind the camera ") ;
      std::cout <<"Z1 = " << Z1 << std::endl ;

      throw(vpFeatureException(vpFeatureException::badInitializationError,
             "Point Z1 is behind the camera ")) ;
    }

    if (fabs(Z1) < 1e-6)
    {
      vpERROR_TRACE("Point Z1 coordinates is null ") ;
      std::cout <<"Z1 = " << Z1 << std::endl ;

      throw(vpFeatureException(vpFeatureException::badInitializationError,
             "Point Z1 coordinates is null")) ;
    }

    flags[4] = true;
  }

  /*!

    Set the value of \f$ Z_2 \f$ which represents the Z coordinate in the camera frame
    of the 3D point that corresponds to the segment second point.

    This value is requested to compute the interaction matrix.

    \param val : \f$ Z_2 \f$ value to set.

    \exception vpFeatureException::badInitializationError : If Z2 is behind the camera or equal to zero.
  */
  inline void setZ2(const double val)
  {
    Z2 = val;

    if (Z2 < 0)
    {
      vpERROR_TRACE("Point Z2 is behind the camera ") ;
      std::cout <<"Z2 = " << Z2 << std::endl ;

      throw(vpFeatureException(vpFeatureException::badInitializationError,
             "Point Z2 is behind the camera ")) ;
    }

    if (fabs(Z2) < 1e-6)
    {
      vpERROR_TRACE("Point Z2 coordinates is null ") ;
      std::cout <<"Z2 = " << Z2 << std::endl ;

      throw(vpFeatureException(vpFeatureException::badInitializationError,
             "Point Z2 coordinates is null")) ;
    }

    flags[5] = true;
  }


private:
  double xc;
  double yc;
  double l;
  double alpha;
  double Z1;
  double Z2;
  double cos_a;
  double sin_a;
} ;

#endif

