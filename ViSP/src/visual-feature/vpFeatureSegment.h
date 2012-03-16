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

/*!
  \class vpFeatureSegment
  \ingroup VsFeature3

  \brief Class that defines a Segment visual feature. The segment is 
  described as angle, length and center

*/
class VISP_EXPORT vpFeatureSegment : public vpBasicFeature
{
public:
  //empty constructor
  vpFeatureSegment();
  // Basic constructor.
  vpFeatureSegment(vpPoint& P1,vpPoint& P2);

  // Basic construction.
  void init() ;


  //! Destructor. Does nothing.
  ~vpFeatureSegment() { if (flags != NULL) delete [] flags; }
  //! change values of the sengment
  void buildFrom(vpPoint& P1,vpPoint& P2);

  //! change values of the sengment
  void buildFrom(const double x1, const double y1, const double Z1, const double x2, const double y2, const double Z2);

  void buildFrom(const double x1, const double y1, const double x2, const double y2);
  void display(const vpCameraParameters &cam,
               vpImage<unsigned char> &I,
               vpColor color=vpColor::green,
               unsigned int thickness=1) const ;
  void display(const vpCameraParameters &cam,
               vpImage<vpRGBa> &I,
               vpColor color=vpColor::green,
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
    inline double getXc(){ return Xc ;}
    /*!
        Get the value of \f$ y_c \f$ which represents the y coordinate of the segment center in the image plane.

        \return The value of \f$ y_c \f$.
    */
    inline double getYc(){ return Yc ;}
    /*!
        Get the value of \f$ l \f$ which represents the length of the segment.

        \return The value of \f$ l \f$.
    */
    inline double getL(){ return l ;}
    /*!
        Get the value of \f$ \alpha \f$ which represents the orientation of the segment.

        \return The value of \f$ \alpha \f$.
    */
    inline double getAlpha(){ return alpha ;}

    /*!
      Get the value of \f$ Z_1 \f$ which represents the z-coordinate of the first segment point.

      \return The value of \f$ Z_1 \f$.
    */
    inline double getZ1(){ return z1 ;}

    /*!
      Get the value of \f$ Z_2 \f$ which represents the z-coordinate of the first second point.

      \return The value of \f$ Z_2 \f$.
    */
    inline double getZ2(){ return z2 ;}

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
    vpPoint p1,p2;
    ...
    vpFeatureSegment seg(p2,p1);
    vpServo task;
    ...
    // Add only the rho subset coordinate feature from an image point to the task
    task.addFeature(seg, vpFeatureSegment::selectXc());
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
    vpPoint p1,p2;
    ...
    vpFeatureSegment seg(p2,p1);
    vpServo task;
    ...
    // Add only the rho subset coordinate feature from an image point to the task
    task.addFeature(seg, vpFeatureSegment::selectYc());
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
    vpPoint p1,p2;
    ...
    vpFeatureSegment seg(p2,p1);
    vpServo task;
    ...
    // Add only the rho subset coordinate feature from an image point to the task
    task.addFeature(seg, vpFeatureSegment::selectL());
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
    vpPoint p1,p2;
    ...
    vpFeatureSegment seg(p2,p1);
    vpServo task;
    ...
    // Add only the rho subset coordinate feature from an image point to the task
    task.addFeature(seg, vpFeatureSegment::selectAlpha());
    \endcode

    \sa selectXc(), selectYc(), selectL()
  */

  inline static unsigned int selectAlpha()  { return FEATURE_LINE[3] ; }
  

  inline void setXc(const double val);
  inline void setYc(const double val);
  inline void setL(const double val);
  inline void setAlpha(const double val);

  inline void setZ1(const double val);
  inline void setZ2(const double val);



private:
  double l;
  double Xc;
  double Yc;
  double alpha;
  double z1;
  double z2;
  double cos_a;
  double sin_a;
} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
