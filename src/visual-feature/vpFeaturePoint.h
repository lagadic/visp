/****************************************************************************
 *
 * $Id: vpFeaturePoint.h,v 1.11 2008-09-26 15:21:02 fspindle Exp $
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
 * 2D point visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpFeaturePoint_H
#define vpFeaturePoint_H

/*!
  \file vpFeaturePoint.h
  \brief Class that defines 2D point visual feature
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpPoint.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRGBa.h>


/*!
  \class vpFeaturePoint
  \ingroup VsFeature2

  \brief Class that defines a 2D point visual feature \f$ s\f$ which is composed by two parameters that are \f$ x \f$ and \f$ y \f$.

  In this class \f$ x \f$ and \f$ y \f$ are the 2D coordinates in the camera frame and are given in meter. \f$ Z \f$ which is the 3D coordinate representing the depth is also a parameter of the point. It is needed during the computation of the interaction matrix \f$ L \f$.

  The visual features can be set easily from an instance of the classes vpPoint, vpDot or vpDot2. For more precision see the class vpFeatureBuilder.

  Once the values of the visual features are set, the interaction() method allows to compute the interaction matrix \f$ L \f$ associated to the visual feature, while the error() method computes the error vector \f$(s - s^*)\f$ between the current visual feature and the desired one.

  The code below shows how to create a eye-in hand visual servoing task using a 2D point feature \f$(x,y)\f$ that correspond to the 2D coordinates of a point in the camera frame. To control six degrees of freedom, at least four other features must be considered like two other point features for example. First we create a current (\f$s\f$) 2D point feature. Then we set the task to use the interaction matrix associated to the current feature \f$L_s\f$. And finally we compute the camera velocity \f$v=-\lambda \; L_s^+ \; (s-s^*)\f$. The current feature \f$s\f$ is updated in the while() loop.

  \code
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>

int main()
{
  vpServo task; // Visual servoing task

  vpFeaturePoint sd; //The desired point feature.
  //Set the desired features x and y
  double xd = 0;
  double yd = 0;
  //Set the depth of the point in the camera frame.
  double Zd = 1;
  //Set the point feature thanks to the desired parameters.
  sd.buildfrom(xd, yd, Zd);

  vpFeaturePoint s; //The current point feature.
  //Set the current features x and y
  double x;  //You have to compute the value of x.
  double y;  //You have to compute the value of y.
  double Z;  //You have to compute the value of Z.
  //Set the point feature thanks to the current parameters.
  s.buildfrom(x, y, Z);
  //In this case the parameter Z is not necessary because the interaction matrix is computed
  //with the desired visual feature.

  // Set eye-in-hand control law. 
  // The computed velocities will be expressed in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA);
  // Interaction matrix is computed with the desired visual features sd
  task.setInteractionMatrixType(vpServo::DESIRED);

  // Add the 2D point feature to the task
  task.addFeature(s, sd);

  // Control loop
  while(1) {
    // The new parameters x and y must be computed here.
    
    // Update the current point visual feature
    s.buildfrom(x, y, Z);
    
    // compute the control law
    vpColVector v = task.computeControlLaw(); // camera velocity
  }
  return 0;
}
  \endcode

  If you want to build your own control law, this other example shows how to create a current (\f$s\f$) and desired (\f$s^*\f$) 2D point visual feature, compute the corresponding error vector \f$(s-s^*)\f$ and finally build the interaction matrix \f$L_s\f$.

  \code
#include <visp/vpFeaturePoint.h>
#include <visp/vpMatrix.h>

int main()
{
  vpFeaturePoint sd; //The desired point feature.
  //Set the desired features x and y
  double xd = 0;
  double yd = 0;
  //Set the depth of the point in the camera frame.
  double Zd = 1;
  //Set the point feature thanks to the desired parameters.
  sd.buildfrom(xd, yd, Zd);

  vpFeaturePoint s; //The current point feature.
  //Set the current features x and y
  double x;  //You have to compute the value of x.
  double y;  //You have to compute the value of y.
  double Z;  //You have to compute the value of Z.
  //Set the point feature thanks to the current parameters.
  s.buildfrom(x, y, Z);

  // Compute the interaction matrix L_s for the current point feature
  vpMatrix L = s.interaction();
  // You can also compute the interaction matrix L_s for the desired point feature
  // The corresponding line of code is : vpMatrix L = sd.interaction();

  // Compute the error vector (s-sd) for the point feature
  s.error(s_star);
}
  \endcode
*/


class VISP_EXPORT vpFeaturePoint : public vpBasicFeature
{
private:
  //! FeaturePoint depth (required to compute the interaction matrix)
  //! default Z = 1m
  double Z ;

public:

  void init() ;

  vpFeaturePoint() ;
  //! destructor
  virtual ~vpFeaturePoint() { ; }


  /*
    section Set coordinates
  */

  void buildFrom(const double x, const double y, const double Z) ;

  void set_x(const double x) ;

  void set_y(const double y) ;

  void set_Z(const double Z) ;

  void set_xyZ(const double x, const double y, const double Z) ;

  double get_x()  const ;

  double get_y()   const ;

  double get_Z() const  ;


  /*
    vpBasicFeature method instantiation
  */

  // feature selection
  /*! 

    Function used to select the \f$ x \f$ subset of the point visual feature.

    This function is to use in conjunction with interaction() in order to compute the interaction matrix associated to \f$ x \f$.

    This function is also useful in the vpServo class to indicate that a subset of the visual feature is to use in the control law:

    \code 
    vpFeaturePoint s;
    vpServo task;
    ...
    // Add the (x) subset features from the 2D point
    task.addFeature(s, vpFeaturePoint::selectX());
    \endcode
  */
  inline static int selectX()  { return FEATURE_LINE[0] ; }

  /*! 

    Function used to select the \f$ y \f$ subset of the point visual feature.

    This function is to use in conjunction with interaction() in order to compute the interaction matrix associated to \f$ y \f$.

    This function is also useful in the vpServo class to indicate that a subset of the visual feature is to use in the control law:

    \code 
    vpFeaturePoint s;
    vpServo task;
    ...
    // Add the (y) subset features from the 2D point
    task.addFeature(s, vpFeaturePoint::selectY());
    \endcode
  */
  inline static int selectY()  { return FEATURE_LINE[1] ; }

  vpMatrix  interaction(const int select = FEATURE_ALL) const;

  vpColVector error(const vpBasicFeature &s_star,
		    const int select = FEATURE_ALL)  ;

  void print(const int select = FEATURE_ALL ) const ;

  vpFeaturePoint *duplicate() const ;


  void display(const vpCameraParameters &cam,
	       vpImage<unsigned char> &I,
	       vpColor::vpColorType color=vpColor::green) const ;
  void display(const vpCameraParameters &cam,
               vpImage<vpRGBa> &I,
               vpColor::vpColorType color=vpColor::green) const ;


  /*!
    @name Deprecated functions
  */
  typedef enum
    {
      X,   // x coordinates
      Y    // y coordinates
    } vpFeaturePointType;
  //! Compute the error between a visual features and zero
  vpColVector error(const int select = FEATURE_ALL)  ;

} ;



#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
