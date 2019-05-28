/****************************************************************************
 *
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
 * See http://visp.inria.fr for more information.
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
 *
 * Authors:
 * Andrew Comport
 *
 *****************************************************************************/

/*!
\file vpMeTracker.cpp
\brief Contains abstract elements for a Distance to Feature type feature.
*/

#include <visp3/core/vpColor.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/me/vpMeTracker.h>

#include <algorithm>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpTrackingException.h>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0

void vpMeTracker::init()
{
  vpTracker::init();
  p.resize(2);
  selectDisplay = vpMeSite::NONE;
}

vpMeTracker::vpMeTracker()
  : list(), me(NULL), init_range(1), nGoodElement(0), m_mask(NULL), selectDisplay(vpMeSite::NONE)
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
    ,
    query_range(0), display_point(false)
#endif
{
  init();
}

vpMeTracker::vpMeTracker(const vpMeTracker &meTracker)
  : vpTracker(meTracker), list(), me(NULL), init_range(1), nGoodElement(0), m_mask(NULL), selectDisplay(vpMeSite::NONE)
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
    ,
    query_range(0), display_point(false)
#endif
{
  init();

  me = meTracker.me;
  list = meTracker.list;
  nGoodElement = meTracker.nGoodElement;
  init_range = meTracker.init_range;
  selectDisplay = meTracker.selectDisplay;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  display_point = meTracker.display_point;
  query_range = meTracker.query_range;
#endif
}

/*!
 Reset the tracker by removing all the moving edges.
 */
void vpMeTracker::reset()
{
  nGoodElement = 0;
  list.clear();
}

vpMeTracker::~vpMeTracker() { reset(); }

vpMeTracker &vpMeTracker::operator=(vpMeTracker &p_me)
{
  list = p_me.list;
  me = p_me.me;
  selectDisplay = p_me.selectDisplay;
  init_range = p_me.init_range;
  nGoodElement = p_me.nGoodElement;
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  query_range = p_me.query_range;
  display_point = p_me.display_point;
#endif
  return *this;
}

static bool isSuppressZero(const vpMeSite &P) { return (P.getState() == vpMeSite::NO_SUPPRESSION); }

unsigned int vpMeTracker::numberOfSignal()
{
  unsigned int number_signal = 0;

  // Loop through all the points tracked from the contour
  number_signal = static_cast<unsigned int>(std::count_if(list.begin(), list.end(), isSuppressZero));
  return number_signal;
}

unsigned int vpMeTracker::totalNumberOfSignal() { return (unsigned int)list.size(); }

/*!
  Test whether the pixel is inside the mask. Mask values that are set to true
  are considered in the tracking.

  \param mask: Mask image or NULL if not wanted. Mask values that are set to true
  are considered in the tracking. To disable a pixel, set false.
  \param i : Pixel coordinate along the rows.
  \param j : Pixel coordinate along the columns.
*/
bool vpMeTracker::inMask(const vpImage<bool> *mask, const unsigned int i, const unsigned int j)
{
  try {
    return (mask == NULL || mask->getValue(i, j));
  }
  catch (vpException &) {
    return false;
  }
}

int vpMeTracker::outOfImage(int i, int j, int half, int rows, int cols)
{
  return (!((i > half + 2) && (i < rows - (half + 2)) && (j > half + 2) && (j < cols - (half + 2))));
}

int vpMeTracker::outOfImage(const vpImagePoint &iP, int half, int rows, int cols)
{
  int i = vpMath::round(iP.get_i());
  int j = vpMath::round(iP.get_j());
  return (!((i > half + 2) && (i < rows - (half + 2)) && (j > half + 2) && (j < cols - (half + 2))));
}

/*!
  Virtual function that is called by lower classes vpMeEllipse, vpMeLine
  and vpMeNurbs.

  \exception vpTrackingException::initializationError : Moving edges not
  initialized.
*/
void vpMeTracker::initTracking(const vpImage<unsigned char> &I)
{
  if (!me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  // Must set range to 0
  unsigned int range_tmp = me->getRange();
  me->setRange(init_range);

  nGoodElement = 0;

  int d = 0;

  // Loop through list of sites to track
  for (std::list<vpMeSite>::iterator it = list.begin(); it != list.end(); ++it) {
    vpMeSite refp = *it; // current reference pixel

    d++;
    // If element hasn't been suppressed
    if (refp.getState() == vpMeSite::NO_SUPPRESSION) {
      try {
        refp.track(I, me, false);
      } catch (...) {
        // EM verifier quel signal est de sortie !!!
        vpERROR_TRACE("Error caught");
        throw;
      }
      if (refp.getState() == vpMeSite::NO_SUPPRESSION)
        nGoodElement++;
    }

#if (DEBUG_LEVEL2)
    {
      vpImagePoint ip1, ip2;
      double a, b;
      a = refp.i_1 - refp.i;
      b = refp.j_1 - refp.j;
      if (refp.getState() == vpMeSite::NO_SUPPRESSION) {
        ip1.set_i(refp.i);
        ip1.set_j(refp.j);
        ip2.set_i(refp.i + a);
        ip2.set_j(refp.j + b);
        vpDisplay::displayArrow(I, ip1, ip2, vpColor::green);
      }
    }
#endif
    *it = refp;
  }

  /*
  if (res != OK)
  {
  std::cout<< "In vpMeTracker::initTracking(): " ;
  switch (res)
  {
  case  ERR_TRACKING:
  std::cout << "vpMeTracker::initTracking:Track return ERR_TRACKING " <<
  std::endl ; break ; case fatalError: std::cout <<
  "vpMeTracker::initTracking:Track return fatalError" << std::endl ; break ;
  default:
  std::cout << "vpMeTracker::initTracking:Track return error " << res <<
  std::endl ;
  }
  return res ;
  }
  */

  me->setRange(range_tmp);
}

/*!
  Track moving-edges.

  \param I : Image.

  \exception vpTrackingException::initializationError : Moving edges not
  initialized.

*/
void vpMeTracker::track(const vpImage<unsigned char> &I)
{
  if (!me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  if (list.empty()) {
    vpDERROR_TRACE(2, "Tracking error: too few pixel to track");
    throw(vpTrackingException(vpTrackingException::notEnoughPointError, "too few pixel to track"));
  }

  nGoodElement = 0;

  // Loop through list of sites to track
  std::list<vpMeSite>::iterator it = list.begin();
  while (it != list.end()) {
    vpMeSite s = *it; // current reference pixel

    // If element hasn't been suppressed
    if (s.getState() == vpMeSite::NO_SUPPRESSION) {

      try {
        s.track(I, me, true);
      } catch (...) {
        s.setState(vpMeSite::THRESHOLD);
      }
      
      if (vpMeTracker::inMask(m_mask, s.i, s.j)) {
        if (s.getState() != vpMeSite::THRESHOLD) {
          nGoodElement++;

#if (DEBUG_LEVEL2)
          {
            double a, b;
            a = s.i_1 - s.i;
            b = s.j_1 - s.j;
            if (s.getState() == vpMeSite::NO_SUPPRESSION) {
              ip1.set_i(s.i);
              ip1.set_j(s.j);
              ip2.set_i(s.i + a * 5);
              ip2.set_j(s.j + b * 5);
              vpDisplay::displayArrow(I, ip1, ip2, vpColor::black);
            }
          }
#endif
        }
        *it = s;
        ++it;
      }
      else {
        // Site outside mask: it is no more tracked.
        it = list.erase(it);
      }
    }
    else {
      ++it;
    }
  }
}

/*!
  Display the moving edge sites with a color corresponding to their state.

  - If green : The vpMeSite is a good point.
  - If blue : The point is removed because of the vpMeSite tracking phase
  (constrast problem).
  - If purple : The point is removed because of the vpMeSite tracking phase
  (threshold problem).
  - If red : The point is removed because of the robust method in the virtual
  visual servoing (M-Estimator problem).
  - If cyan : The point is removed because it's too close to another.
  - Yellow otherwise

  \param I : The image.
*/
void vpMeTracker::display(const vpImage<unsigned char> &I)
{
#if (DEBUG_LEVEL1)
  {
    std::cout << "begin vpMeTracker::displayList() " << std::endl;
    std::cout << " There are " << list.size() << " sites in the list " << std::endl;
  }
#endif
  for (std::list<vpMeSite>::const_iterator it = list.begin(); it != list.end(); ++it) {
    vpMeSite p_me = *it;
    p_me.display(I);
  }
}

void vpMeTracker::display(const vpImage<vpRGBa> &I)
{
  for (std::list<vpMeSite>::const_iterator it = list.begin(); it != list.end(); ++it) {
    vpMeSite p_me = *it;
    p_me.display(I);
  }
}

/*! Displays the status of moving edge sites

  \param I : The image.
  \param w : vector
  \param index_w : index
*/
void vpMeTracker::display(const vpImage<unsigned char> &I, vpColVector &w, unsigned int &index_w)
{
  for (std::list<vpMeSite>::iterator it = list.begin(); it != list.end(); ++it) {
    vpMeSite P = *it;

    if (P.getState() == vpMeSite::NO_SUPPRESSION) {
      P.weight = w[index_w];
      index_w++;
    }

    *it = P;
  }
  display(I);
}

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
