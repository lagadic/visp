/****************************************************************************
 *
 * $Id: vpMeLine.h,v 1.13 2008-12-15 15:11:27 nmelchio Exp $
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

/*!
\file vpMeLine.h
\brief Moving edges
*/

#ifndef vpMeLine_HH
#define vpMeLine_HH


#include <math.h>
#include <iostream>


#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpMeTracker.h>


/*!
  \class vpMeLine

  \ingroup TrackingImageME

  \brief Class that tracks a line moving edges.

  The line is defined by its equation ai + bj + c = 0.

  The code below shows how to use this class.
\code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpMeLine.h>


void main()
{
  vpImage<unsigned char> I;

  //Here the code to read or grab the gray scaled image.

  vpMeLine line;
  vpMe me;

  //Set the tracking parameters.
  me.setRange(25) ;
  me.setPointsToTrack(20) ;
  me.setThreshold(15000) ;
  me.setSampleStep(10);
  line.setMe(&me);

  //Initialize the tracking.
  line.initTracking(I);

  while ( 1 )
  {
    //Here the code to read or grab the next image.

    //Track the line.
    line.track(I);
  }
}
\endcode

  \note It is possible to display the line as an overlay. For that you must use the display function of the class vpMeLine.
*/

class VISP_EXPORT vpMeLine : public vpMeTracker
{
private:
  vpMeSite PExt[2] ;

  double rho, theta ;
  double delta ,delta_1;
  double angle, angle_1;
  int sign;

public:
  /*!
	Parameter a of the line equation a*i + b*j + c = 0
  */
  double a;
  /*!
	Parameter b of the line equation a*i + b*j + c = 0
  */
  double b;
  /*!
	Parameter c of the line equation a*i + b*j + c = 0
  */
  double c;


public:
  vpMeLine() ;
  virtual ~vpMeLine() ;

  void display(vpImage<unsigned char>& I, vpColor::vpColorType col) ;

  void track(vpImage<unsigned char>& Im);

  void sample(vpImage<unsigned char>&image);
  void reSample(vpImage<unsigned char> &I) ;
  void leastSquare() ;
  void updateDelta();
  void setExtremities() ;
  void seekExtremities(vpImage<unsigned char> &I) ;
  void suppressPoints() ;

  void initTracking(vpImage<unsigned char> &I) ;
  void initTracking(vpImage<unsigned char> &I,
		    unsigned i1,unsigned j1,
		    unsigned i2, unsigned j2) ;

  void computeRhoTheta(vpImage<unsigned char> &I) ;
  double getRho() const ;
  double getTheta() const ;
  void getExtremities(double &i1, double &j1, double &i2, double &j2) ;
};




#endif


