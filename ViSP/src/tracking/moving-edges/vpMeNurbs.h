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
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \file vpMeNurbs.h
  \brief Moving edges on a form reprsented by a NURBS (Non Uniform Rational B-Spline)
*/

#ifndef vpMeNurbs_HH
#define vpMeNurbs_HH

#include <math.h>
#include <iostream>

#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpMeTracker.h>
#include <visp/vpNurbs.h>

/*!
  \class vpMeNurbs

  \ingroup TrackingImageME

  \brief Class that tracks in an image a edge defined by a Nurbs.

  The advantage of this class is that it enables to track an edge whose equation is
  not known in advance. At each iteration, the Nurbs corresponding to the edge is computed.
  
  It is possible to have a direct access to the nurbs. It is indeed a public parameter.

  The code below shows how to use this class.
\code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpList.h>
#include <visp/vpMeNurbs.h>
#include <visp/vpImagePoint.h>

int main()
{
  vpImage<unsigned char> I(240, 320);

  // Fill the image with a black rectangle
  I = 0;
  for (int i = 100; i < 180; i ++) {
    for (int j = 0; j < 320; j ++) {
      I[i][j] = 255;
    }
  }
    
  // Set the moving-edges tracker parameters
  vpMe me;
  me.setRange(25);
  me.setPointsToTrack(20);
  me.setThreshold(15000);
  me.setSampleStep(10);

  // Initialize the moving-edges tracker parameters
  vpMeNurbs meNurbs;
  meNurbs.setNbControlPoints(4);
  meNurbs.setMe(&me);

  // Initialize the location of the edge to track (here a horizontal line
  vpList<vpImagePoint> ipList; //List of points belonginig to the edge
  ipList.addRight(vpImagePoint(110,119));
  ipList.addRight(vpImagePoint(140,119));
  ipList.addRight(vpImagePoint(160,119));
  ipList.addRight(vpImagePoint(170,119));

  meNurbs.initTracking(I, ipList);

  while ( 1 )
  {
    // ... Here the code to read or grab the next image.

    // Track the line.
    meNurbs.track(I);
  }
  return 0;
}
\endcode

  \note It is possible to display the nurbs as an overlay. For that you 
  must use the display function of the class vpMeNurbs.
  
  \note In case of an edge which is not smooth, it can be interesting to use the
  canny detection to find the extremities. In this case, use the method
  setEnableCannyDetection to enable it. Warning : This function requires OpenCV.
*/

class VISP_EXPORT vpMeNurbs : public vpMeTracker
{
  public:
    //! The Nurbs which represents the tracked edge.
    vpNurbs nurbs;
    
  private:
    //! Size of the nurbs
    double dist;
    //! Number of control points used to compute the Nurbs.
    int nbControlPoints;
    //! counter used to trigger the canny edge detection at the beginning of the Nurbs.
    int beginPtFound;
    //! counter used to trigger the canny edge detection at the end of the Nurbs.
    int endPtFound;
    //! True if the canny detection has to be used during the extremities search.
    bool enableCannyDetection;
    //! First canny threshold
    double cannyTh1;
    //! Second canny threshold
    double cannyTh2;

  public:
    vpMeNurbs();
    vpMeNurbs(vpMeNurbs &menurbs);
    virtual ~vpMeNurbs();
    
    /*!
      Sets the number of control points used to compute the Nurbs.

      \param nbControlPoints : The number of control points used to compute the Nurbs.
    */
    void setNbControlPoints(const int nbControlPoints) {this->nbControlPoints = nbControlPoints;}
    
    /*!
      Enables or disables the canny detection used during the extremities search.

      \param enableCannyDetection : if true it enables the canny detection.
    */
    void setEnableCannyDetection(const bool enableCannyDetection) {this->enableCannyDetection =enableCannyDetection;}
    
    /*!
      Enables to set the two thresholds use by the canny detection.
      
      \param th1 : The first threshold;
      \param th2 : The second threshold;
    */
    void setCannyThreshold(const double th1, const double th2)
	{this->cannyTh1 = th1;
	 this->cannyTh2 = th2;}
    
    void initTracking(vpImage<unsigned char> &I) ;
    void initTracking(vpImage<unsigned char> &I,
		      vpList<vpImagePoint> &ptList) ;

    void track(vpImage<unsigned char>& Im);

    void sample(vpImage<unsigned char>&image);
    void reSample(vpImage<unsigned char> &I) ;
    void updateDelta();
    void setExtremities() ;
    void seekExtremities(vpImage<unsigned char> &I) ;
    void seekExtremitiesCanny(vpImage<unsigned char> &I) ;
    void suppressPoints() ;

    void supressNearPoints();
    void localReSample(vpImage<unsigned char> &I);
    
    void display(vpImage<unsigned char>& I, vpColor col) ;
    
  private:
    bool computeFreemanChainElement(const vpImage<unsigned char> &I,
				   vpImagePoint &iP,
				   unsigned int &element);

    bool hasGoodLevel(const vpImage<unsigned char>& I,
			  const vpImagePoint iP) const;

    bool isInImage(const vpImage<unsigned char>& I, const vpImagePoint iP) const;
    
    void computeFreemanParameters( unsigned int element, vpImagePoint &diP);
    
    bool farFromImageEdge(const vpImage<unsigned char> I, const vpImagePoint iP);
};

#endif
