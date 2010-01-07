/****************************************************************************
 *
 * $Id$
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
 * Moving edges.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpMeEllipse.h
  \brief Moving edges on an ellipse
*/

#ifndef vpMeEllipse_HH
#define vpMeEllipse_HH


#include <math.h>


#include <visp/vpConfig.h>

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

#include <visp/vpMeTracker.h>
#include <visp/vpMeSite.h>
#include <visp/vpImagePoint.h>

#include <visp/vpImage.h>
#include <visp/vpColor.h>

/*!
  \class vpMeEllipse 
  \ingroup TrackingImageME

  \brief Class that tracks an ellipse moving edges.

  In this class, an ellipse is defined as the set of points \f$ (i,j) \f$ of the image frame (For more information about the image frame see the vpImagePoint documentation) that satisfy the implicit equation :

  \f[ i^2 + K_0j^2 + 2K_1ij + 2K_2i + 2K_3j + K4 = 0 \f]

  If \f$ K_0 \f$ is equal to 1 and \f$ K_1 \f$ is equal to 0 the the set of points \f$ (i,j) \f$ represents a circle.

  The five parameters are stored in the public attribute K.

  An ellipse is also defined thanks to three other parameter which are \f$ a \f$, \f$ b \f$ and \f$ e \f$. \f$ a \f$ represents the semiminor axis and \f$ b \f$ is the semimajor axis. Here \f$ e \f$ is the angle made by the
  major axis and the i axis of the image frame \f$ (i,j) \f$. The following figure shows better meaning of those parameters.

  \image html vpMeEllipse.gif
  \image latex vpMeEllipse.ps  width=10cm

  It is possible to compute the coordinates \f$ (i,j) \f$ of a point which belongs to the ellipse thanks to the following equations :

  \f[ i = i_c + b cos(e) cos(\alpha) - a sin(e) sin(\alpha) \f]
  \f[ j = j_c + b sin(e) cos(\alpha) + a cos(e) sin(\alpha) \f]

  Here the coordinates \f$ (i_c,j_c) \f$ are the coordinates of the ellipse center in the image frame and \f$ \alpha \f$ is an angle beetween \f$ [0,2\pi] \f$ and which enables to describe all the points of the ellipse.

  \image html vpMeEllipse2.gif
  \image latex vpMeEllipse2.ps  width=10cm

  The code below shows how to use this class.
\code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpMeEllipse.h>
#include <visp/vpImagePoint.h>

int main()
{
  vpImage<unsigned char> I;

  // I is the image containing the ellipse to track
    
  // Set the moving-edges tracker parameters
  vpMe me;
  me.setRange(25);
  me.setPointsToTrack(20);
  me.setThreshold(15000);
  me.setSampleStep(10);

  // Initialize the moving-edges ellipse tracker parameters
  vpMeEllipse ellipse;
  ellipse.setMe(&me);

  // Initialize the tracking. You have to click on five different points belonging to the ellipse.
  ellipse.initTracking(I);

  while ( 1 )
  {
    // ... Here the code to read or grab the next image.

    // Track the ellipse.
    ellipse.track(I);
  }
  return 0;
}
\endcode

  \note It is possible to display the ellipse as an overlay. For that you 
  must use the display function of the class vpMeEllipse.
*/

class VISP_EXPORT vpMeEllipse : public vpMeTracker
{
public:
  /*! Parameters of the ellipse to define the set of points that satisfy the implicit equation :
   \f[ i^2 + K_0j^2 + 2K_1ij + 2K_2i + 2K_3j + K4 = 0 \f]
  */
  vpColVector K ;
  //! The coordinates of the ellipse center.
  vpImagePoint iPc;
  //! \f$ a \f$ is the semiminor axis of the ellipse.
  double a;
  //! \f$ b \f$ is the semimajor axis of the ellipse.
  double b;
  //! \f$ e \f$ is the angle made by the major axis and the i axis of the image frame \f$ (i,j) \f$.
  double e;

  vpMeEllipse() ;
  virtual ~vpMeEllipse() ;

  void track(vpImage<unsigned char>& Im);

  void initTracking(vpImage<unsigned char> &I) ;
  void initTracking(vpImage<unsigned char> &I, int n,
		    vpImagePoint* iP) ;
  void display(vpImage<unsigned char>&I, vpColor col) ;
  void printParameters() ;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  //@{
  void initTracking(vpImage<unsigned char> &I, int n,
		    unsigned *i, unsigned *j) ;
  //@}
#endif //VISP_BUILD_DEPRECATED_FUNCTIONS

  /*!
      Set to true if you are sure to track a circle.

      \warning During all the tracking, the shape must be approximatively a circle and not an ellipse with a strong difference between the majoraxis and the minoraxis.

      In that case, the set of points \f$ (i,j) \f$ satisfy the implicit equation :

      \f[ i^2 + j^2 + 2K_2i + 2K_3j + K4 = 0 \f]

      Compared to the classical equation of an ellipse, \f$ K_0 \f$ is equal to 1 and \f$ K_1 \f$ is equal to 0.

      \param circle : Set to true if you want to track a circle.
  */
  void setCircle(bool circle) { this->circle = circle ; }

protected:
  //! The coordinates of the point corresponding to the smallest \f$ alpha \f$ angle. More things about the \f$ alpha \f$ are given at the beginning of the class description.
  vpImagePoint iP1;
  //! The coordinates of the point corresponding to the highest \f$ alpha \f$ angle. More things about the \f$ alpha \f$ are given at the beginning of the class description.
  vpImagePoint iP2;
  //! The smallest \f$ alpha \f$ angle.
  double alpha1 ;
  //! The highest \f$ alpha \f$ angle.
  double alpha2 ;
  //! Value of cos(e).
  double ce;
  //! Value of sin(e).
  double se;
  //!Stores the value of the \f$ alpha \f$ angle for each vpMeSite.
  vpList<double> angle;

private:
  //! True if the ellipse to track is a circle
  bool circle ;

  void computeAngle(vpImagePoint pt1, vpImagePoint pt2);
  void sample(vpImage<unsigned char>&image);
  void reSample(vpImage<unsigned char> &I) ;
  void leastSquare() ;
  void updateTheta();
  void suppressPoints() ;
  void seekExtremities(vpImage<unsigned char> &I) ;
  void setExtremities();
  void getParameters() ;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  //@{
  void computeAngle(int ip1, int jp1,int ip2, int jp2) ;
  void computeAngle(int ip1, int jp1, double &alpha1,
	     int ip2, int jp2, double &alpha2) ;
  //@}
#endif //VISP_BUILD_DEPRECATED_FUNCTIONS

};




#endif


