/*
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Moving edges.
 */

/*!
 * \file vpMeNurbs.h
 * \brief Moving edges on a form represented by a NURBS (Non Uniform Rational
 * B-Spline)
 */

#ifndef vpMeNurbs_HH
#define vpMeNurbs_HH

#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/me/vpMeTracker.h>
#include <visp3/me/vpNurbs.h>

#include <iostream>
#include <list>
#include <math.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpMeNurbs
 *
 * \ingroup module_me
 *
 * \brief Class that tracks in an image a edge defined by a Nurbs.
 *
 * The advantage of this class is that it enables to track an edge whose
 * equation is not known in advance. At each iteration, the Nurbs corresponding
 * to the edge is computed.
 *
 * It is possible to have a direct access to the nurbs. It is indeed a public
 * parameter.
 *
 * The code below shows how to use this class.
 * \code
 * #include <visp3/core/vpImage.h>
 * #include <visp3/core/vpImagePoint.h>
 * #include <visp3/me/vpMeNurbs.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpImage<unsigned char> I(240, 320);
 *
 *   // Fill the image with a black rectangle
 *   I = 0u;
 *   for (int i = 100; i < 180; i ++) {
 *     for (int j = 0; j < 320; j ++) {
 *       I[i][j] = 255;
 *     }
 *   }
 *
 *   // Set the moving-edges tracker parameters
 *   vpMe me;
 *   me.setRange(25);
 *   me.setPointsToTrack(20);
 *   me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
 *   me.setThreshold(20);
 *   me.setSampleStep(10);
 *
 *   // Initialize the moving-edges tracker parameters
 *   vpMeNurbs meNurbs;
 *   meNurbs.setNbControlPoints(4);
 *   meNurbs.setMe(&me);
 *
 *   // Initialize the location of the edge to track (here a horizontal line
 *   std::list<vpImagePoint> ipList; //List of points belonging to the edge
 *   ipList.push_back(vpImagePoint(110,119));
 *   ipList.push_back(vpImagePoint(140,119));
 *   ipList.push_back(vpImagePoint(160,119));
 *   ipList.push_back(vpImagePoint(170,119));
 *
 *   meNurbs.initTracking(I, ipList);
 *
 *   while ( 1 )
 *   {
 *     // ... Here the code to read or grab the next image.
 *
 *     // Track the line.
 *     meNurbs.track(I);
 *   }
 *   return 0;
 * }
 * \endcode
 *
 * \note It is possible to display the nurbs as an overlay. For that you
 * must use the display function of the class vpMeNurbs.
 *
 * \note In case of an edge which is not smooth, it can be interesting to use
 * the canny detection to find the extremities. In this case, use the method
 * setEnableCannyDetection to enable it.
 *
 * \warning : This function requires OpenCV.
*/
class VISP_EXPORT vpMeNurbs : public vpMeTracker
{
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
#else
private:
#endif
  //! The Nurbs which represents the tracked edge.
  vpNurbs nurbs;

private:
  //! Size of the nurbs
  double dist;
  //! Number of control points used to compute the Nurbs.
  unsigned int nbControlPoints;
  //! counter used to trigger the canny edge detection at the beginning of the
  //! Nurbs.
  int beginPtFound;
  //! counter used to trigger the canny edge detection at the end of the
  //! Nurbs.
  int endPtFound;
  //! True if the canny detection has to be used during the extremities
  //! search.
  bool enableCannyDetection;
  //! First canny threshold
  float cannyTh1;
  //! Second canny threshold
  float cannyTh2;

public:
  /*!
   * Basic constructor that calls the constructor of the class vpMeTracker.
   */
  vpMeNurbs();

  /*!
   * Copy constructor.
   */
  vpMeNurbs(const vpMeNurbs &menurbs);

  /*!
   * Sets the number of control points used to compute the Nurbs.
   *
   * \param nb_point : The number of control points used to compute the Nurbs.
   */
  void setNbControlPoints(unsigned int nb_point) { this->nbControlPoints = nb_point; }

  /*!
   * Enables or disables the canny detection used during the extremities
   * search.
   *
   * \param enable_canny : if true it enables the canny detection.
   */
  void setEnableCannyDetection(const bool enable_canny) { this->enableCannyDetection = enable_canny; }

  /*!
   * Enables to set the two thresholds use by the canny detection.
   *
   * \param th1 : The first threshold;
   * \param th2 : The second threshold;
   */
  void setCannyThreshold(float th1, float th2)
  {
    this->cannyTh1 = th1;
    this->cannyTh2 = th2;
  }

  /*!
   * Initialization of the tracking. Ask the user to click left on several points
   * along the edge to track and click right at the end.
   *
   * \param I : Image in which the edge appears.
   */
  void initTracking(const vpImage<unsigned char> &I);

  /*!
   * Initialization of the tracking. The Nurbs is initialized thanks to the
   * list of vpImagePoint.
   *
   * \param I : Image in which the edge appears.
   * \param ptList  : List of point to initialize the Nurbs.
   */
  void initTracking(const vpImage<unsigned char> &I, const std::list<vpImagePoint> &ptList);

  /*!
   * Track the edge in the image I.
   *
   * \param I : Image in which the edge appears.
   */
  void track(const vpImage<unsigned char> &I);

  /*!
   * Construct a list of vpMeSite moving edges at a particular sampling
   * step between the two extremities of the nurbs.
   *
   * \param I : Image in which the edge appears.
   * \param doNotTrack : Inherited parameter, not used.
   */
  virtual void sample(const vpImage<unsigned char> &I, bool doNotTrack = false);

  /*!
   * Resample the edge if the number of sample is less than 70% of the
   * expected value.
   *
   * \note The expected value is computed thanks to the length of the
   * nurbs and the parameter which indicates the number of pixel between
   * two points (vpMe::sample_step).
   *
   * \param I : Image in which the edge appears.
   */
  void reSample(const vpImage<unsigned char> &I);

  /*!
   * Set the alpha value (normal to the edge at this point)
   * of the different vpMeSite to a value computed thanks to the nurbs.
   */
  void updateDelta();

  /*!
   * Seek along the edge defined by the nurbs, the two extremities of
   * the edge. This function is useful in case of translation of the
   * edge.
   *
   * \param I : Image in which the edge appears.
   */
  void seekExtremities(const vpImage<unsigned char> &I);

  /*!
   * Seek the extremities of the edge thanks to a canny edge detection.
   * The edge detection enable to find the points belonging to the edge.
   * The any vpMeSite are initialized at this points.
   *
   * This method is useful when the edge is not smooth.
   *
   * \note To use the canny detection, OpenCV has to be installed.
   *
   * \param I : Image in which the edge appears.
   */
  void seekExtremitiesCanny(const vpImage<unsigned char> &I);

  /*!
   * Suppression of the points which:
   *
   * - belong no more to the edge.
   * - which are to closed to another point.
   */
  void suppressPoints();

  /*!
   * Suppress vpMeSites if they are too close to each other.
   *
   * The goal is to keep the order of the vpMeSites in the list.
   */
  void supressNearPoints();

  /*!
   * Resample a part of the edge if two vpMeSite are too far from each other.
   * In this case the method try to initialize any vpMeSite between the two
   * points.
   *
   * \param I : Image in which the edge appears.
   */
  void localReSample(const vpImage<unsigned char> &I);

  /*!
    Gets the nurbs;
  */
  inline vpNurbs getNurbs() const { return nurbs; }

  /*!
   * Display edge.
   *
   * \warning To effectively display the edge a call to
   * vpDisplay::flush() is needed.
   *
   * \param I : Image in which the edge appears.
   * \param color : Color of the displayed line.
   * \param thickness : Drawings thickness.
   */
  void display(const vpImage<unsigned char> &I, const vpColor &color, unsigned int thickness = 1);

private:
  bool computeFreemanChainElement(const vpImage<unsigned char> &I, vpImagePoint &iP, unsigned int &element);

  bool hasGoodLevel(const vpImage<unsigned char> &I, const vpImagePoint &iP) const;

  bool isInImage(const vpImage<unsigned char> &I, const vpImagePoint &iP) const;

  void computeFreemanParameters(unsigned int element, vpImagePoint &diP);

  bool farFromImageEdge(const vpImage<unsigned char> &I, const vpImagePoint &iP);

public:
  /*!
   * Display of a moving nurbs.
   *
   * \param I : The image used as background.
   * \param n : Nurbs to display
   * \param color : Color used to display the nurbs.
   * \param thickness : Drawings thickness.
   */
  static void display(const vpImage<unsigned char> &I, vpNurbs &n, const vpColor &color = vpColor::green, unsigned int thickness = 1);

  /*!
   * Display of a moving nurbs.
   *
   * \param I : The image used as background.
   * \param n : Nurbs to display
   * \param color : Color used to display the nurbs.
   * \param thickness : Drawings thickness.
   */
  static void display(const vpImage<vpRGBa> &I, vpNurbs &n, const vpColor &color = vpColor::green, unsigned int thickness = 1);
};
END_VISP_NAMESPACE
#endif
