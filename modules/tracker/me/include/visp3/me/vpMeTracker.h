/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
  \file vpMeTracker.h
  \brief Contains abstract elements for a Distance to Feature type feature.
*/

#ifndef _vpMeTracker_h_
#define _vpMeTracker_h_

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpTracker.h>
#include <visp3/me/vpMe.h>
#include <visp3/me/vpMeSite.h>

#include <iostream>
#include <list>
#include <math.h>

/*!
  \class vpMeTracker

  \ingroup module_me
  \brief Contains abstract elements for a Distance to Feature type feature.

  2D state = list of points, 3D state = feature
*/
class VISP_EXPORT vpMeTracker : public vpTracker
{
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
  /** @name Public Attributes Inherited from vpMeTracker */
  //@{
#else
protected:
  /** @name Protected Attributes Inherited from vpMeTracker */
  //@{
#endif
  //! Tracking dependent variables/functions
  //! List of tracked moving edges points.
  std::list<vpMeSite> list;
  //! Moving edges initialisation parameters
  vpMe *me;
  //! Initial range
  unsigned int init_range;
  //! Number of good moving-edges that are tracked
  int nGoodElement;
  //! Mask used to disable tracking on a part of image
  const vpImage<bool> *m_mask;
  //@}

protected:
  /** @name Protected Attributes Inherited from vpMeTracker */
  //@{
  vpMeSite::vpMeSiteDisplayType selectDisplay;
  //@}

public:
  /*!
   * Default constructor.
   */
  vpMeTracker();

  /*!
   * Copy constructor.
   */
  vpMeTracker(const vpMeTracker &meTracker);

  /*!
   * Destructor.
   */
  virtual ~vpMeTracker();

  /** @name Public Member Functions Inherited from vpMeTracker */
  //@{

  /*!
   * Display the moving edge sites with a color corresponding to their state.
   *
   * - If green : The vpMeSite is a good point.
   * - If blue : The point is removed because of the vpMeSite tracking phase (contrast problem).
   * - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
   * - If red : The point is removed because of the robust method in the virtual visual servoing (M-Estimator problem).
   * - If cyan : The point is removed because it's too close to another.
   * - Yellow otherwise.
   *
   * \param I : The image.
   */
  void display(const vpImage<unsigned char> &I);

  /*!
   * Display the moving edge sites with a color corresponding to their state.
   *
   * - If green : The vpMeSite is a good point.
   * - If blue : The point is removed because of the vpMeSite tracking phase (contrast problem).
   * - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
   * - If red : The point is removed because of the robust method in the virtual visual servoing (M-Estimator problem).
   * - If cyan : The point is removed because it's too close to another.
   * - Yellow otherwise.
   *
   * \param I : The image.
   */
  void display(const vpImage<vpRGBa> &I);

  /*!
   * Displays the status of moving edge sites
   *
   * \param I : The image.
   * \param w : vector
   * \param index_w : index
   */
  void display(const vpImage<unsigned char> &I, vpColVector &w, unsigned int &index_w);

  /*!
   * Test whether the pixel is inside the mask. Mask values that are set to true
   * are considered in the tracking.
   *
   * \param mask: Mask image or NULL if not wanted. Mask values that are set to true
   * are considered in the tracking. To disable a pixel, set false.
   * \param i : Pixel coordinate along the rows.
   * \param j : Pixel coordinate along the columns.
   */
  static bool inMask(const vpImage<bool> *mask, unsigned int i, unsigned int j);

  /*!
   * Return the initial range.
   *
   * \return Value of init_range.
   */
  inline unsigned int getInitRange() { return init_range; }

  /*!
   * Return the moving edges initialisation parameters.
   *
   * \return Moving Edges.
   */
  inline vpMe *getMe() { return me; }

  /*!
   * Return the list of moving edges
   *
   * \return List of Moving Edges.
   */
  inline std::list<vpMeSite> &getMeList() { return list; }

  /*!
   * Return the list of moving edges
   *
   * \return List of Moving Edges.
   */
  inline std::list<vpMeSite> getMeList() const { return list; }

  /*!
   * Return the number of points that has not been suppressed.
   *
   * \return Number of good points.
   */
  inline int getNbPoints() const { return nGoodElement; }

  /*!
   * Initialize the tracker.
   */
  void init();

  /*!
   * Virtual function that is called by lower classes vpMeEllipse, vpMeLine
   * and vpMeNurbs.
   *
   * \exception vpTrackingException::initializationError : Moving edges not
   * initialized.
   */
  void initTracking(const vpImage<unsigned char> &I);

  /*!
   * Return number of moving-edges that are tracked.
   */
  unsigned int numberOfSignal();

  /*!
   * Copy operator.
   */
  vpMeTracker &operator=(vpMeTracker &f);

  /*!
   * Check if a pixel i,j is out of the image.
   */
  int outOfImage(int i, int j, int half, int row, int cols);
  /*!
   * Check if a pixel i,j is out of the image.
   */
  int outOfImage(const vpImagePoint &iP, int half, int rows, int cols);

  /*!
   * Reset the tracker by removing all the moving edges.
   */
  void reset();

  /*!
   * Sample pixels at a given interval.
   */
  virtual void sample(const vpImage<unsigned char> &image, bool doNotTrack = false) = 0;

  /*!
   * Set type of moving-edges display.
   * @param select : Display type selector.
   */
  void setDisplay(vpMeSite::vpMeSiteDisplayType select) { selectDisplay = select; }

  /*!
   * Set the initial range.
   *
   * \param r : initial range.
   */
  void setInitRange(const unsigned int &r) { init_range = r; }

  /*!
   * Set the mask.
   *
   * \param mask : Mask.
   */
  virtual void setMask(const vpImage<bool> &mask) { m_mask = &mask; }

  /*!
   * Set the moving edges initialisation parameters.
   *
   * \param p_me : Moving Edges.
   */
  void setMe(vpMe *p_me) { this->me = p_me; }

  /*!
   * Set the list of moving edges.
   *
   * \param l : list of Moving Edges.
   */
  void setMeList(const std::list<vpMeSite> &l) { list = l; }

  /*!
   * Return the total number of moving-edges.
   */
  unsigned int totalNumberOfSignal();

  /*!
   * Track moving-edges.
   *
   * \param I : Image.
   *
   * \exception vpTrackingException::initializationError : Moving edges not initialized.
   */
  void track(const vpImage<unsigned char> &I);
  //@}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
  /** @name Public Attributes Inherited from vpMeTracker */
  //@{
  int query_range;
  bool display_point; // if 1 (TRUE) displays the line that is being tracked
  //@}
#endif
};

#endif
