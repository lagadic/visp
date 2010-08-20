/****************************************************************************
 *
 * $Id: vpFeaturePoint.h 2455 2010-01-07 10:24:57Z nmelchio $
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
 *   Luninance based feature .
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpFeatureLuminance_h
#define vpFeatureLuminance_h
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpImage.h>


/*!
  \file vpFeatureLuminance.h
  \brief Class that defines the image luminance visual feature

  for more details see
  C. Collewet, E. Marchand, F. Chaumette. Visual
  servoing set free from image processing. In IEEE Int. Conf. on
  Robotics and Automation, ICRA'08, Pages 81-86, Pasadena, Californie,
  Mai 2008.
*/


/*!
  \class vpLuminance.h
  \brief Class that defines the luminance and gradient of a point

  \sa vpFeatureLuminance
*/


class  vpLuminance
{
 public:
  double x, y;   // point coordinates (in meter)
  double I ; // pixel intensity
  double Ix,Iy ; // pixel gradient
  double Z; // pixel depth

};


/*!
  \class vpFeatureLuminance.h
  \brief Class that defines the image luminance visual feature

  for more details see
  C. Collewet, E. Marchand, F. Chaumette. Visual
  servoing set free from image processing. In IEEE Int. Conf. on
  Robotics and Automation, ICRA'08, Pages 81-86, Pasadena, Californie,
  Mai 2008.
*/

class vpFeatureLuminance : public vpBasicFeature
{
 protected:
  //! FeaturePoint depth (required to compute the interaction matrix)
  //! default Z = 1m
  double Z ;

  int nbr ;
  int nbc ;
  int bord ;
  
  //! Store the image (as a vector with intensity and gradient I, Ix, Iy) 
  vpLuminance *pixInfo ;
  int  firstTimeIn  ;

 public:
  void buildFrom(vpImage<unsigned char> &I) ;

public: 

  void init() ;
  void init(int _nbr, int _nbc, double _Z) ;

  vpFeatureLuminance() ;
 
  //! Destructor.
  virtual ~vpFeatureLuminance()  ;

 public:
  vpCameraParameters cam ;
  void setCameraParameters(vpCameraParameters &_cam)  ;
  /*
    section Set/get Z
  */


  void set_Z(const double Z) ;
  double get_Z() const  ;


  /*
    vpBasicFeature method instantiation
  */

 
  vpMatrix  interaction(const int select = FEATURE_ALL);
  void      interaction(vpMatrix &L);

  vpColVector error(const vpBasicFeature &s_star,
		    const int select = FEATURE_ALL)  ;
  void error(const vpBasicFeature &s_star,
	     vpColVector &e)  ;

  void print(const int select = FEATURE_ALL ) const ;

  vpFeatureLuminance *duplicate() const ;


  void display(const vpCameraParameters &cam,
	       vpImage<unsigned char> &I,
	       vpColor color=vpColor::green, unsigned int thickness=1) const ;
  void display(const vpCameraParameters &cam,
               vpImage<vpRGBa> &I,
               vpColor color=vpColor::green, unsigned int thickness=1) const ;


  //! Compute the error between a visual features and zero
  vpColVector error(const int select = FEATURE_ALL)  ;

} ;



#endif
