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
  \file vpMeTracker.cpp
  \brief Contains abstract elements for a Distance to Feature type feature.
*/

#include <visp3/core/vpColor.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/me/vpMeTracker.h>

#include <algorithm>
#include <visp3/core/vpTrackingException.h>

BEGIN_VISP_NAMESPACE

void vpMeTracker::init()
{
  vpTracker::init();
  const unsigned int val_2 = 2;
  p.resize(val_2);
  m_selectDisplay = vpMeSite::NONE;
}

vpMeTracker::vpMeTracker()
  : m_meList(), m_me(nullptr), m_init_range(1), m_nGoodElement(0), m_mask(nullptr), m_maskCandidates(nullptr), m_selectDisplay(vpMeSite::NONE)
{
  init();
}

vpMeTracker::vpMeTracker(const vpMeTracker &meTracker)
  : vpTracker(meTracker), m_meList(), m_me(nullptr), m_init_range(1), m_nGoodElement(0), m_mask(nullptr), m_maskCandidates(nullptr), m_selectDisplay(vpMeSite::NONE)
{
  init();

  m_me = meTracker.m_me;
  m_meList = meTracker.m_meList;
  m_nGoodElement = meTracker.m_nGoodElement;
  m_init_range = meTracker.m_init_range;
  m_selectDisplay = meTracker.m_selectDisplay;
}

void vpMeTracker::reset()
{
  m_nGoodElement = 0;
  m_meList.clear();
}

vpMeTracker::~vpMeTracker() { reset(); }

vpMeTracker &vpMeTracker::operator=(vpMeTracker &meTracker)
{
  m_meList = meTracker.m_meList;
  m_me = meTracker.m_me;
  m_selectDisplay = meTracker.m_selectDisplay;
  m_init_range = meTracker.m_init_range;
  m_nGoodElement = meTracker.m_nGoodElement;
  return *this;
}

static bool isSuppressZero(const vpMeSite &P) { return (P.getState() == vpMeSite::NO_SUPPRESSION); }

unsigned int vpMeTracker::numberOfSignal()
{
  unsigned int number_signal = 0;

  // Loop through all the points tracked from the contour
  number_signal = static_cast<unsigned int>(std::count_if(m_meList.begin(), m_meList.end(), isSuppressZero));
  return number_signal;
}

unsigned int vpMeTracker::totalNumberOfSignal() { return static_cast<unsigned int>(m_meList.size()); }

bool vpMeTracker::inRoiMask(const vpImage<bool> *mask, unsigned int i, unsigned int j)
{
  try {
    return ((mask == nullptr) || (mask->getValue(i, j)));
  }
  catch (vpException &) {
    return false;
  }
}

bool vpMeTracker::inMeMaskCandidates(const vpImage<bool> *meMaskCandidates, unsigned int i, unsigned int j)
{
  if (meMaskCandidates == nullptr) {
    return true;
  }
  else {
    const unsigned int kernelSize = 3;
    const unsigned int halfKernelSize = (kernelSize - 1) / 2;
    const unsigned int nbRows = meMaskCandidates->getRows();
    const unsigned int nbCols = meMaskCandidates->getCols();

    if ((i >= nbRows) || (j >= nbCols)) {
      // The asked point is outside the mask
      return false;
    }
    if ((*meMaskCandidates)[i][j]) {
      // The asked point is a candidate
      return true;
    }
    unsigned int iStart = 0, jStart = 0;
    unsigned int iStop = nbRows - 1, jStop = nbCols - 1;
    // Ensuring we won't go outside the limits of the mask
    if (i >= halfKernelSize) {
      iStart = i - halfKernelSize;
    }
    if (j >= halfKernelSize) {
      jStart = j - halfKernelSize;
    }
    if ((i + halfKernelSize) < nbRows) {
      iStop = i + halfKernelSize;
    }
    if ((j + halfKernelSize) < nbCols) {
      jStop = j + halfKernelSize;
    }
    // Looking in its neighborhood
    bool isACandidate = false;
    unsigned int iter_i = iStart, iter_j = jStart;

    while ((!isACandidate) && (iter_i <= iStop)) {
      iter_j = jStart;
      while ((!isACandidate) && (iter_j <= jStop)) {
        isACandidate = (*meMaskCandidates)[iter_i][iter_j];
        ++iter_j;
      }
      ++iter_i;
    }

    return isACandidate;
  }
}

bool vpMeTracker::outOfImage(int i, int j, int border, int nrows, int ncols)
{
  int borderWith2SparedPixels = border + 2;
  return (!((i > borderWith2SparedPixels) && (i < (nrows - borderWith2SparedPixels))
            && (j > borderWith2SparedPixels) && (j < (ncols - borderWith2SparedPixels))
            ));
}

bool vpMeTracker::outOfImage(const vpImagePoint &iP, int border, int nrows, int ncols)
{
  const int borderPlus2 = border + 2;
  int i = vpMath::round(iP.get_i());
  int j = vpMath::round(iP.get_j());
  return (!((i > borderPlus2) && (i < (nrows - borderPlus2)) && (j > borderPlus2) && (j < (ncols - borderPlus2))));
}

void vpMeTracker::initTracking(const vpImage<unsigned char> &I)
{
  if (!m_me) {
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  // Must set range to 0
  unsigned int range_tmp = m_me->getRange();
  m_me->setRange(m_init_range);

  m_nGoodElement = 0;

  // Loop through list of sites to track
  std::list<vpMeSite>::iterator end = m_meList.end();
  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != end; ++it) {
    vpMeSite refp = *it; // current reference pixel

    // If element hasn't been suppressed
    if (refp.getState() == vpMeSite::NO_SUPPRESSION) {

      refp.track(I, m_me, false);

      if (refp.getState() == vpMeSite::NO_SUPPRESSION) {
        ++m_nGoodElement;
      }
    }

    *it = refp;
  }

  m_me->setRange(range_tmp);
}

void vpMeTracker::track(const vpImage<unsigned char> &I)
{
  if (!m_me) {
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  if (m_meList.empty()) {
    throw(vpTrackingException(vpTrackingException::notEnoughPointError, "Too few pixel to track"));
  }

  m_nGoodElement = 0;

  // Loop through list of sites to track
  std::list<vpMeSite>::iterator it = m_meList.begin();
  std::list<vpMeSite>::iterator end = m_meList.end();
  while (it != end) {
    vpMeSite s = *it; // current reference pixel

    // If element hasn't been suppressed
    if (s.getState() == vpMeSite::NO_SUPPRESSION) {
      s.track(I, m_me, true);


      if (vpMeTracker::inRoiMask(m_mask, s.get_i(), s.get_j())) {
        if (s.getState() == vpMeSite::NO_SUPPRESSION) {
          ++m_nGoodElement;
        }
      }
      else {
        // Site outside mask
        s.setState(vpMeSite::OUTSIDE_ROI_MASK);
      }
    }

    *it = s;
    ++it;
  }
}

void vpMeTracker::display(const vpImage<unsigned char> &I)
{
  std::list<vpMeSite>::const_iterator end = m_meList.end();
  for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != end; ++it) {
    vpMeSite p_me = *it;
    p_me.display(I);
  }
}

void vpMeTracker::display(const vpImage<vpRGBa> &I)
{
  std::list<vpMeSite>::const_iterator end = m_meList.end();
  for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != end; ++it) {
    vpMeSite p_me = *it;
    p_me.display(I);
  }
}

void vpMeTracker::display(const vpImage<unsigned char> &I, vpColVector &w, unsigned int &index_w)
{
  std::list<vpMeSite>::iterator end = m_meList.end();
  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != end; ++it) {
    vpMeSite P = *it;

    if (P.getState() == vpMeSite::NO_SUPPRESSION) {
      P.setWeight(w[index_w]);
      ++index_w;
    }

    *it = P;
  }
  display(I);
}

END_VISP_NAMESPACE
