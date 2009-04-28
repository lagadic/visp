/****************************************************************************
 *
 * $Id: vpMeEllipse.h,v 1.7 2008-09-26 15:21:01 fspindle Exp $
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
 * Moving edges.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpMeEllipse_HH
#define vpMeEllipse_HH


#include <math.h>


#include <visp/vpConfig.h>

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

#include <visp/vpMeTracker.h>
#include <visp/vpMeSite.h>

#include <visp/vpImage.h>
#include <visp/vpColor.h>

/*!
  \class vpMeEllipse 
  \ingroup TrackingImageME

  \brief Class that tracks an ellipse moving edges.
						
*/
class VISP_EXPORT vpMeEllipse : public vpMeTracker
{
public:
  vpMeSite PExt[2] ;

  double theta ;
  //! vecteur de parametres de la quadrique
  //! i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4
  vpColVector K ;

  double ic, jc, e, a, b ;
  double ce, se ;
  int i1,j1, i2, j2 ;
  double alpha1 ;
  double alpha2 ;
private:
  //! seek extremities (in degree)
  double seek ;

public:
  int sample_step ;

public:
  vpMeEllipse() ;
  virtual ~vpMeEllipse() ;


  void setSeekExtremities(double seek) {this->seek = seek ; }
  void display(vpImage<unsigned char>&I, vpColor::vpColorType col) ;

private:
  void computeAngle(int ip1, int jp1,int ip2, int jp2) ;
  void computeAngle(int ip1, int jp1, double &alpha1,
	     int ip2, int jp2, double &alpha2) ;

  void sample(vpImage<unsigned char>&image);
  void reSample(vpImage<unsigned char> &I) ;
  void leastSquare() ;
  void updateTheta();
  void suppressPoints() ;
  void seekExtremities(vpImage<unsigned char> &I) ;

public:

  void getParameters() ;
  void printParameters() ;

  void track(vpImage<unsigned char>& Im);

  void initTracking(vpImage<unsigned char> &I) ;
  void initTracking(vpImage<unsigned char> &I, int n,
		    unsigned *i, unsigned *j) ;

private:
  bool circle ;
public:
  //! set to true if we are sure to track a circle and that this very
  //! unlikely to append in perspective projection, nevertherless for
  //! omnidirectional camera, this can be useful
  //!
  //! in that case
  //! i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4
  //! K0 = 1
  //! K1 = 0
  void setCircle(bool circle) { this->circle = circle ; }
};




#endif


