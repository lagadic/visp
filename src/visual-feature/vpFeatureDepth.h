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
 * Nicolas Melchior
 *
 *****************************************************************************/


#ifndef vpFeatureDepth_H
#define vpFeatureDepth_H

/*!
  \file vpFeatureDepth.h
  \brief Class that defines 3D point visual feature
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRGBa.h>


/*!
  \class vpFeatureDepth
  \ingroup VsFeature3

  \brief Class that defines a 3D point visual feature \f$ s\f$ which is composed by one parameters that is \f$ log( \frac{Z}{Z^*}) \f$ that defines the current depth relative to the desired depth. Here \f$ Z \f$ represents the current depth and \f$ Z^* \f$ the desired depth.

  In this class \f$ x \f$ and \f$ y \f$ are the 2D coordinates in the camera frame and are given in meter. \f$ x \f$, \f$ y \f$ and \f$ Z \f$ are needed during the computation of the interaction matrix \f$ L \f$.

  The visual features can be set easily thanks to the buildFrom() method.

  As the visual feature \f$ s \f$ represents the current depth relative to the desired depth, the desired visual feature \f$ s^* \f$ is set to zero. Once the value of the visual feature is set, the interaction() method allows to compute the interaction matrix \f$ L \f$ associated to the visual feature, while the error() method computes the error vector \f$(s - s^*)\f$ between the current visual feature and the desired one which is here set to zero.

  The code below shows how to create a eye-in hand visual servoing task using a 3D depth feature \f$ log( \frac{Z}{Z^*}) \f$ that corresponds to the current depth relative to the desired depth. To control six degrees of freedom, at least five other features must be considered. First we create a current (\f$s\f$) 3D depth feature. Then we set the task to use the interaction matrix associated to the current feature \f$L_s\f$. And finally we compute the camera velocity \f$v=-\lambda \; L_s^+ \; (s-s^*)\f$. The current feature \f$s\f$ is updated in the while() loop.

  \code
#include <visp/vpFeatureDepth.h>
#include <visp/vpServo.h>

int main()
{
  vpServo task; // Visual servoing task

  vpFeatureDepth s; //The current point feature.
  //Set the current parameters x, y, Z and the desired depth Zs
  double x;  //You have to compute the value of x.
  double y;  //You have to compute the value of y.
  double Z;  //You have to compute the value of Z.
  double Zs;  //You have to define the desired depth Zs.
  //Set the point feature thanks to the current parameters.
  s.buildfrom(x, y, Z, log(Z/Zs));

  // Set eye-in-hand control law. 
  // The computed velocities will be expressed in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA);
  // Interaction matrix is computed with the desired visual features sd
  task.setInteractionMatrixType(vpServo::CURRENT);

  // Add the 3D depth feature to the task
  task.addFeature(s); // s* is here considered as zero

  // Control loop
  while(1) {
    // The new parameters x, y and Z must be computed here.
    
    // Update the current point visual feature
    s.buildfrom(x, y, Z, log(Z/Zs));
    
    // compute the control law
    vpColVector v = task.computeControlLaw(); // camera velocity
  }
  return 0;
}
  \endcode

  If you want to build your own control law, this other example shows how to create a current (\f$s\f$) and desired (\f$s^*\f$) 2D point visual feature, compute the corresponding error vector \f$(s-s^*)\f$ and finally build the interaction matrix \f$L_s\f$.

  \code
#include <visp/vpFeatureDepth.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

int main()
{
  vpFeatureDepth s; //The current point feature.
  //Set the current parameters x, y, Z and the desired depth Zs
  double x;  //You have to compute the value of x.
  double y;  //You have to compute the value of y.
  double Z;  //You have to compute the value of Z.
  double Zs;  //You have to define the desired depth Zs.
  //Set the point feature thanks to the current parameters.
  s.buildfrom(x, y, Z, log(Z/Zs));

  // Compute the interaction matrix L_s for the current point feature
  vpMatrix L = s.interaction();

  // Compute the error vector (s-s*) for the point feature with s* considered as 0.
  vpColVector s_star(1); //the dimension is 1.
  s_star(1) = 0; //The value of s* is 0.
  s.error(s_star);
}
  \endcode
*/


class VISP_EXPORT vpFeatureDepth : public vpBasicFeature
{

private:
  //! The \f$ x \f$ 2D coordinate of the point in the camera frame (required to compute the interaction matrix)
  double x;
  //! The \f$ y \f$ 2D coordinate of the point in the camera frame (required to compute the interaction matrix)
  double y;
  //! The \f$ Z \f$ 3D coordinate of the point in the camera frame (required to compute the interaction matrix)
  double Z;

public:

  void init() ;

  vpFeatureDepth() ;
  //! destructor
  virtual ~vpFeatureDepth() { ; }


  /*
    section Set coordinates
  */

  void buildFrom(const double x, const double y, const double Z, const double LogZoverZstar) ;

  void set_x(const double x) ;

  void set_y(const double y) ;

  void set_Z(const double Z) ;

  void set_LogZoverZstar(const double LogZoverZstar);

  void set_xyZLogZoverZstar(const double x, const double y, const double Z, const double logZZs) ;

  double get_x()  const ;

  double get_y()   const ;

  double get_Z() const  ;

  double get_LogZoverZstar() const  ;


  /*
    vpBasicFeature method instantiation
  */


  vpMatrix  interaction(const int select = FEATURE_ALL) const;

  vpColVector error(const vpBasicFeature &s_star,
		    const int select = FEATURE_ALL)  ;

  void print(const int select = FEATURE_ALL ) const ;

  vpFeatureDepth *duplicate() const ;


  void display(const vpCameraParameters &cam,
	       vpImage<unsigned char> &I,
	       vpColor::vpColorType color=vpColor::green) const ;
  void display(const vpCameraParameters &cam,
               vpImage<vpRGBa> &I,
               vpColor::vpColorType color=vpColor::green) const ;

} ;



#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
