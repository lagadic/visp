/*
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
  \file vpMeNurbs.cpp
  \brief Moving edges
*/

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <stdlib.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRect.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/me/vpMe.h>
#include <visp3/me/vpMeNurbs.h>
#include <visp3/me/vpMeSite.h>
#include <visp3/me/vpMeTracker.h>
#if defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#endif

BEGIN_VISP_NAMESPACE
double computeDelta(double deltai, double deltaj);
void findAngle(const vpImage<unsigned char> &I, const vpImagePoint &iP, vpMe *me, double &angle, double &convlt);
vpImagePoint findFirstBorder(const vpImage<unsigned char> &Isub, const vpImagePoint &iP);
bool findCenterPoint(std::list<vpImagePoint> *ip_edges_list);

// Compute the angle delta = arctan(deltai/deltaj)
// and normalize it between 0 and pi
double computeDelta(double deltai, double deltaj)
{
  double delta;
  delta = atan2(deltai, deltaj);
  delta -= M_PI / 2.0;
  while (delta > M_PI) {
    delta -= M_PI;
  }
  while (delta < 0) {
    delta += M_PI;
  }
  return (delta);
}

// Check if the image point is in the image and not to close to
// its edge to enable the computation of a convolution with a mask.
static bool outOfImage(const vpImagePoint &iP, int half, int rows, int cols)
{
  return ((iP.get_i() < half + 1) || (iP.get_i() > (rows - half - 3)) || (iP.get_j() < half + 1) ||
    (iP.get_j() > (cols - half - 3)));
}

// if iP is a edge point, it computes the angle corresponding to the
// highest convolution result. the angle is between 0 an 179.
// The result gives the angle in RADIAN + pi/2 (to deal with the moving edeg
// alpha angle)  and the corresponding convolution result.
void findAngle(const vpImage<unsigned char> &I, const vpImagePoint &iP, vpMe *me, double &angle, double &convlt)
{
  int Iheight = (int)I.getHeight();
  int Iwidth = (int)I.getWidth();
  angle = 0.0;
  convlt = 0.0;
  for (int i = 0; i < 180; i++) {
    double conv = 0.0;
    unsigned int half;
    half = (me->getMaskSize() - 1) >> 1;

    if (outOfImage(iP, (int)half + me->getStrip(), Iheight, Iwidth)) {
      conv = 0.0;
    }
    else {
      int index_mask;

      if (me->getAngleStep() != 0)
        index_mask = (int)(i / (double)me->getAngleStep());
      else
        throw(vpException(vpException::divideByZeroError, "angle step = 0"));

      unsigned int ihalf = (unsigned int)(iP.get_i() - half);
      unsigned int jhalf = (unsigned int)(iP.get_j() - half);
      unsigned int a;
      unsigned int b;
      for (a = 0; a < me->getMaskSize(); a++) {
        unsigned int ihalfa = ihalf + a;
        for (b = 0; b < me->getMaskSize(); b++) {
          conv += me->getMask()[index_mask][a][b] * I(ihalfa, jhalf + b);
        }
      }
    }
    conv = fabs(conv);
    if (conv > convlt) {
      convlt = conv;
      angle = vpMath::rad(i);
      angle += M_PI / 2;
      while (angle > M_PI) {
        angle -= M_PI;
      }
      while (angle < 0) {
        angle += M_PI;
      }
    }
  }
}

// Find the point belonging to the edge of the sub image which respects the
// following hypotheses:
//- the value of the pixel is upper than zero.
//- the distantce between the point and iP is less than 4 pixels.
// The function returns the nearest point of iP which respect the hypotheses
// If no point is found the returned point is (-1,-1)
vpImagePoint findFirstBorder(const vpImage<unsigned char> &Isub, const vpImagePoint &iP)
{
  double dist = 1e6;
  double dist_1 = 1e6;
  vpImagePoint index(-1, -1);
  for (unsigned int i = 0; i <= Isub.getHeight(); i++) {
    for (unsigned int j = 0; j <= Isub.getWidth(); j++) {
      if (i == 0 || i == Isub.getHeight() - 1 || j == 0 || j == Isub.getWidth() - 1) {
        if (Isub(i, j) > 0) {
          dist = vpImagePoint::sqrDistance(vpImagePoint(iP), vpImagePoint(i, j));
          if (dist <= 16 && dist < dist_1) {
            dist_1 = dist;
            index.set_ij(i, j);
          }
        }
      }
    }
  }
  return index;
}

// Check if the list of vpImagePoint contains a distant point of less tha 4
// pixels  from the center of the sub image (ie the point (15,15).
bool findCenterPoint(std::list<vpImagePoint> *ip_edges_list)
{
  for (std::list<vpImagePoint>::const_iterator it = ip_edges_list->begin(); it != ip_edges_list->end(); ++it) {
    vpImagePoint iP = *it;
    double dist = vpImagePoint::sqrDistance(iP, vpImagePoint(15, 15));
    if (dist <= 16) {
      return true;
    }
  }
  return false;
}

/***************************************/

vpMeNurbs::vpMeNurbs()
  : nurbs(), dist(0.), nbControlPoints(20), beginPtFound(0), endPtFound(0), enableCannyDetection(false), cannyTh1(100.),
  cannyTh2(200.)
{ }

vpMeNurbs::vpMeNurbs(const vpMeNurbs &menurbs)
  : vpMeTracker(menurbs), nurbs(menurbs.nurbs), dist(0.), nbControlPoints(20), beginPtFound(0), endPtFound(0),
  enableCannyDetection(false), cannyTh1(100.f), cannyTh2(200.f)
{
  dist = menurbs.dist;
  nbControlPoints = menurbs.nbControlPoints;
  beginPtFound = menurbs.beginPtFound;
  endPtFound = menurbs.endPtFound;
  enableCannyDetection = menurbs.enableCannyDetection;
  cannyTh1 = menurbs.cannyTh1;
  cannyTh2 = menurbs.cannyTh2;
}

void vpMeNurbs::initTracking(const vpImage<unsigned char> &I)
{
  std::list<vpImagePoint> ptList;
  vpImagePoint pt;
  vpMouseButton::vpMouseButtonType b;

  while (vpDisplay::getClick(I, pt, b)) {
    if (b == vpMouseButton::button1) {
      // std::cout<<pt<<std::endl;
      ptList.push_back(pt);
      vpDisplay::displayCross(I, pt, 10, vpColor::green);
      vpDisplay::flush(I);
    }
    if (b == vpMouseButton::button3)
      break;
  }
  if (ptList.size() > 3)
    initTracking(I, ptList);
  else
    throw(vpException(vpException::notInitialized, "Not enough points to initialize the Nurbs"));
}

void vpMeNurbs::initTracking(const vpImage<unsigned char> &I, const std::list<vpImagePoint> &ptList)
{
  nurbs.globalCurveInterp(ptList);

  sample(I);

  vpMeTracker::initTracking(I);
  track(I);
}

void vpMeNurbs::sample(const vpImage<unsigned char> &I, bool doNotTrack)
{
  (void)doNotTrack;
  int rows = (int)I.getHeight();
  int cols = (int)I.getWidth();
  double step = 1.0 / (double)m_me->getPointsToTrack();

  // Delete old list
  m_meList.clear();

  double u = 0.0;
  vpImagePoint *pt = nullptr;
  vpImagePoint pt_1(-rows, -cols);
  while (u <= 1.0) {
    if (pt != nullptr)
      delete[] pt;
    pt = nurbs.computeCurveDersPoint(u, 1);
    double delta = computeDelta(pt[1].get_i(), pt[1].get_j());

    // If point is in the image, add to the sample list
    if (!outOfImage(pt[0], 0, rows, cols) &&
      vpImagePoint::sqrDistance(pt[0], pt_1) >= vpMath::sqr(m_me->getSampleStep())) {
      vpMeSite pix;
      pix.init(pt[0].get_i(), pt[0].get_j(), delta);
      pix.setDisplay(m_selectDisplay);

      m_meList.push_back(pix);
      pt_1 = pt[0];
    }
    u = u + step;
  }
  if (pt != nullptr)
    delete[] pt;
}

void vpMeNurbs::suppressPoints()
{
  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != m_meList.end();) {
    vpMeSite s = *it; // current reference pixel

    if (s.getState() != vpMeSite::NO_SUPPRESSION) {
      it = m_meList.erase(it);
    }
    else
      ++it;
  }
}

void vpMeNurbs::updateDelta()
{
  double u = 0.0;
  double d = 1e6;
  double d_1 = 1e6;
  std::list<vpMeSite>::iterator it = m_meList.begin();

  vpImagePoint Cu;
  vpImagePoint *der = nullptr;
  double step = 0.01;
  while (u < 1 && it != m_meList.end()) {
    vpMeSite s = *it;
    vpImagePoint pt(s.get_i(), s.get_j());
    while (d <= d_1 && u < 1) {
      Cu = nurbs.computeCurvePoint(u);
      d_1 = d;
      d = vpImagePoint::distance(pt, Cu);
      u += step;
    }

    u -= step;
    if (der != nullptr)
      delete[] der;
    der = nurbs.computeCurveDersPoint(u, 1);
    // vpImagePoint toto(der[0].get_i(),der[0].get_j());
    // vpDisplay::displayCross(I,toto,4,vpColor::red);

    s.setAlpha(computeDelta(der[1].get_i(), der[1].get_j()));
    *it = s;
    ++it;
    d = 1e6;
    d_1 = 1.5e6;
  }
  if (der != nullptr)
    delete[] der;
}

void vpMeNurbs::seekExtremities(const vpImage<unsigned char> &I)
{
  int rows = (int)I.getHeight();
  int cols = (int)I.getWidth();

  vpImagePoint *begin = nullptr;
  vpImagePoint *end = nullptr;

  begin = nurbs.computeCurveDersPoint(0.0, 1);
  end = nurbs.computeCurveDersPoint(1.0, 1);

  // Check if the two extremities are not to close to eachother.
  double d = vpImagePoint::distance(begin[0], end[0]);
  double threshold = 3 * m_me->getSampleStep();
  double sample_step = m_me->getSampleStep();
  vpImagePoint pt;
  if (d > threshold /*|| (m_meList.firstValue()).m_mask_sign != (m_meList.lastValue()).m_mask_sign*/) {
    vpMeSite P;

    // Init vpMeSite
    P.init(begin[0].get_i(), begin[0].get_j(), (m_meList.front()).getAlpha(), 0, (m_meList.front()).m_mask_sign);
    P.setDisplay(m_selectDisplay);

    // Set the range
    unsigned int memory_range = m_me->getRange();
    m_me->setRange(2);

    // Point at the beginning of the list
    bool beginPtAdded = false;
    vpImagePoint pt_max = begin[0];
    double angle = atan2(begin[1].get_i(), begin[1].get_j());
    double co = vpMath::abs(cos(angle));
    co = co * vpMath::sign(begin[1].get_j());
    double si = vpMath::abs(sin(angle));
    si = si * vpMath::sign(begin[1].get_i());
    for (int i = 0; i < 3; i++) {
      P.m_ifloat = P.m_ifloat - si * sample_step;
      P.m_i = (int)P.m_ifloat;
      P.m_jfloat = P.m_jfloat - co * sample_step;
      P.m_j = (int)P.m_jfloat;
      pt.set_ij(P.m_ifloat, P.m_jfloat);
      if (vpImagePoint::distance(end[0], pt) < threshold)
        break;
      if (!outOfImage(P.get_i(), P.get_j(), 5, rows, cols)) {
        P.track(I, m_me, false);

        if (P.getState() == vpMeSite::NO_SUPPRESSION) {
          m_meList.push_front(P);
          beginPtAdded = true;
          pt_max = pt;
          if (vpDEBUG_ENABLE(3)) {
            vpDisplay::displayCross(I, pt, 5, vpColor::blue);
          }
        }
        else {
          if (vpDEBUG_ENABLE(3)) {
            vpDisplay::displayCross(I, pt, 10, vpColor::blue);
          }
        }
      }
    }

    if (!beginPtAdded)
      beginPtFound++;

    P.init(end[0].get_i(), end[0].get_j(), (m_meList.back()).getAlpha(), 0, (m_meList.back()).m_mask_sign);
    P.setDisplay(m_selectDisplay);

    bool endPtAdded = false;
    angle = atan2(end[1].get_i(), end[1].get_j());
    co = vpMath::abs(cos(angle));
    co = co * vpMath::sign(end[1].get_j());
    si = vpMath::abs(sin(angle));
    si = si * vpMath::sign(end[1].get_i());
    for (int i = 0; i < 3; i++) {
      P.m_ifloat = P.m_ifloat + si * sample_step;
      P.m_i = (int)P.m_ifloat;
      P.m_jfloat = P.m_jfloat + co * sample_step;
      P.m_j = (int)P.m_jfloat;
      pt.set_ij(P.m_ifloat, P.m_jfloat);
      if (vpImagePoint::distance(begin[0], pt) < threshold)
        break;
      if (!outOfImage(P.get_i(), P.get_j(), 5, rows, cols)) {
        P.track(I, m_me, false);

        if (P.getState() == vpMeSite::NO_SUPPRESSION) {
          m_meList.push_back(P);
          endPtAdded = true;
          if (vpDEBUG_ENABLE(3)) {
            vpDisplay::displayCross(I, pt, 5, vpColor::blue);
          }
        }
        else {
          if (vpDEBUG_ENABLE(3)) {
            vpDisplay::displayCross(I, pt, 10, vpColor::blue);
          }
        }
      }
    }
    if (!endPtAdded)
      endPtFound++;
    m_me->setRange(memory_range);
  }
  else {
    m_meList.pop_front();
  }
  /*if(begin != nullptr)*/ delete[] begin;
  /*if(end != nullptr)  */ delete[] end;
}

void vpMeNurbs::seekExtremitiesCanny(const vpImage<unsigned char> &I)
{
  vpMeSite pt = m_meList.front();
  vpImagePoint firstPoint(pt.m_ifloat, pt.m_jfloat);
  pt = m_meList.back();
  vpImagePoint lastPoint(pt.m_ifloat, pt.m_jfloat);
  if (beginPtFound >= 3 && farFromImageEdge(I, firstPoint)) {
    vpImagePoint *begin = nullptr;
    begin = nurbs.computeCurveDersPoint(0.0, 1);
    vpImage<unsigned char> Isub(32, 32); // Sub image.
    vpImagePoint topLeft(begin[0].get_i() - 15, begin[0].get_j() - 15);
    vpRect rect(topLeft, 32, 32);

    vpDisplay::displayRectangle(I, rect, vpColor::green);

    vpImageTools::crop(I, rect, Isub);

    vpImagePoint lastPtInSubIm(begin[0]);
    double u = 0.0;
    double step = 0.0001;
    // Find the point of the nurbs closest from the edge of the subImage and
    // in the subImage.
    while (inRectangle(lastPtInSubIm, rect) && u < 1) {
      u += step;
      lastPtInSubIm = nurbs.computeCurvePoint(u);
    }

    u -= step;
    if (u > 0)
      lastPtInSubIm = nurbs.computeCurvePoint(u);

    vpImageFilter::canny(Isub, Isub, 3, cannyTh1, 3);

    vpImagePoint firstBorder(-1, -1);

    firstBorder = findFirstBorder(Isub, lastPtInSubIm - topLeft);

    std::list<vpImagePoint> ip_edges_list;
    if (firstBorder != vpImagePoint(-1, -1)) {
      unsigned int dir;
      double fi = static_cast<double>(firstBorder.get_i());
      double fj = static_cast<double>(firstBorder.get_j());
      double w = Isub.getWidth() - 1;
      double h = Isub.getHeight() - 1;
      // if (firstBorder.get_i() == 0) dir = 4;
      if (std::fabs(fi) <= std::numeric_limits<double>::epsilon())
        dir = 4;
      // else if (firstBorder.get_i() == Isub.getHeight()-1) dir = 0;
      else if (std::fabs(fi - h) <= std::fabs(vpMath::maximum(fi, h)) * std::numeric_limits<double>::epsilon())
        dir = 0;
      // else if (firstBorder.get_j() == 0) dir = 2;
      else if (std::fabs(fj) <= std::numeric_limits<double>::epsilon())
        dir = 2;
      // else if (firstBorder.get_j() == Isub.getWidth()-1) dir = 6;
      else if (std::fabs(fj - w) <= std::fabs(vpMath::maximum(fj, w)) * std::numeric_limits<double>::epsilon())
        dir = 6;
      computeFreemanChainElement(Isub, firstBorder, dir);
      unsigned int firstDir = dir;
      ip_edges_list.push_back(firstBorder);
      vpImagePoint border(firstBorder);
      vpImagePoint dBorder;
      do {
        computeFreemanParameters(dir, dBorder);
        border = border + dBorder;
        vpDisplay::displayPoint(I, border + topLeft, vpColor::orange);

        ip_edges_list.push_back(border);

        computeFreemanChainElement(Isub, border, dir);
      } while ((border != firstBorder || dir != firstDir) && isInImage(Isub, border));
    }

    if (findCenterPoint(&ip_edges_list)) {
      for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != m_meList.end();
        /*++it*/) {
        vpMeSite s = *it;
        vpImagePoint iP(s.m_ifloat, s.m_jfloat);
        if (inRectangle(iP, rect))
          it = m_meList.erase(it);
        else
          break;
      }

      std::list<vpMeSite>::iterator itList = m_meList.begin();
      double convlt;
      double delta = 0;
      unsigned int nbr = 0;
      std::list<vpMeSite> addedPt;
      for (std::list<vpImagePoint>::const_iterator itEdges = ip_edges_list.begin(); itEdges != ip_edges_list.end();
        ++itEdges) {
        vpMeSite s = *itList;
        vpImagePoint iPtemp = *itEdges + topLeft;
        vpMeSite pix;
        pix.init(iPtemp.get_i(), iPtemp.get_j(), delta);
        dist = vpMeSite::sqrDistance(s, pix);
        if (dist >= vpMath::sqr(m_me->getSampleStep()) /*25*/) {
          bool exist = false;
          for (std::list<vpMeSite>::const_iterator itAdd = addedPt.begin(); itAdd != addedPt.end(); ++itAdd) {
            dist = vpMeSite::sqrDistance(pix, *itAdd);
            if (dist < vpMath::sqr(m_me->getSampleStep()) /*25*/)
              exist = true;
          }
          if (!exist) {
            findAngle(I, iPtemp, m_me, delta, convlt);
            pix.init(iPtemp.get_i(), iPtemp.get_j(), delta, convlt);
            pix.setDisplay(m_selectDisplay);
            --itList;
            m_meList.insert(itList, pix);
            ++itList;
            addedPt.push_front(pix);
            nbr++;
          }
        }
      }

      unsigned int memory_range = m_me->getRange();
      m_me->setRange(3);
      std::list<vpMeSite>::iterator itList2 = m_meList.begin();
      for (unsigned int j = 0; j < nbr; j++) {
        vpMeSite s = *itList2;
        s.track(I, m_me, false);
        *itList2 = s;
        ++itList2;
      }
      m_me->setRange(memory_range);
    }

    /* if (begin != nullptr) */ delete[] begin;
    beginPtFound = 0;
  }

  if (endPtFound >= 3 && farFromImageEdge(I, lastPoint)) {
    vpImagePoint *end = nullptr;
    end = nurbs.computeCurveDersPoint(1.0, 1);

    vpImage<unsigned char> Isub(32, 32); // Sub image.
    vpImagePoint topLeft(end[0].get_i() - 15, end[0].get_j() - 15);
    vpRect rect(topLeft, 32, 32);

    vpDisplay::displayRectangle(I, rect, vpColor::green);

    vpImageTools::crop(I, rect, Isub);

    vpImagePoint lastPtInSubIm(end[0]);
    double u = 1.0;
    double step = 0.0001;
    // Find the point of the nurbs closest from the edge of the subImage and
    // in the subImage.
    while (inRectangle(lastPtInSubIm, rect) && u > 0) {
      u -= step;
      lastPtInSubIm = nurbs.computeCurvePoint(u);
    }

    u += step;
    if (u < 1.0)
      lastPtInSubIm = nurbs.computeCurvePoint(u);

    vpImageFilter::canny(Isub, Isub, 3, cannyTh1, 3);

    vpImagePoint firstBorder(-1, -1);

    firstBorder = findFirstBorder(Isub, lastPtInSubIm - topLeft);

    std::list<vpImagePoint> ip_edges_list;
    if (firstBorder != vpImagePoint(-1, -1)) {
      unsigned int dir;
      double fi = firstBorder.get_i();
      double fj = firstBorder.get_j();
      double w = Isub.getWidth() - 1;
      double h = Isub.getHeight() - 1;
      // if (firstBorder.get_i() == 0) dir = 4;
      if (std::fabs(fi) <= std::numeric_limits<double>::epsilon())
        dir = 4;
      // else if (firstBorder.get_i() == Isub.getHeight()-1) dir = 0;
      else if (std::fabs(fi - h) <= std::fabs(vpMath::maximum(fi, h)) * std::numeric_limits<double>::epsilon())
        dir = 0;
      // else if (firstBorder.get_j() == 0) dir = 2;
      else if (std::fabs(fj) <= std::numeric_limits<double>::epsilon())
        dir = 2;
      // else if (firstBorder.get_j() == Isub.getWidth()-1) dir = 6;
      else if (std::fabs(fj - w) <= std::fabs(vpMath::maximum(fj, w)) * std::numeric_limits<double>::epsilon())
        dir = 6;

      computeFreemanChainElement(Isub, firstBorder, dir);
      unsigned int firstDir = dir;
      ip_edges_list.push_back(firstBorder);
      vpImagePoint border(firstBorder);
      vpImagePoint dBorder;
      do {
        computeFreemanParameters(dir, dBorder);
        border = border + dBorder;
        vpDisplay::displayPoint(I, border + topLeft, vpColor::orange);

        ip_edges_list.push_back(border);

        computeFreemanChainElement(Isub, border, dir);
      } while ((border != firstBorder || dir != firstDir) && isInImage(Isub, border));
    }

    if (findCenterPoint(&ip_edges_list)) {
      vpMeSite s;

      for (std::list<vpMeSite>::iterator it = m_meList.begin(); it!=m_meList.end(); ++it) {
        s = *it;
        vpImagePoint iP(s.m_ifloat, s.m_jfloat);
        if (inRectangle(iP, rect)) {
          m_meList.erase(it);
        }
        else
          break;
      }

      std::list<vpMeSite>::iterator itList = m_meList.end();
      --itList; // Move on the last element
      double convlt;
      double delta;
      unsigned int nbr = 0;
      std::list<vpMeSite> addedPt;
      for (std::list<vpImagePoint>::const_iterator itEdges = ip_edges_list.begin(); itEdges != ip_edges_list.end();
        ++itEdges) {
        s = *itList;
        vpImagePoint iPtemp = *itEdges + topLeft;
        vpMeSite pix;
        pix.init(iPtemp.get_i(), iPtemp.get_j(), 0);
        dist = vpMeSite::sqrDistance(s, pix);
        if (dist >= vpMath::sqr(m_me->getSampleStep())) {
          bool exist = false;
          for (std::list<vpMeSite>::const_iterator itAdd = addedPt.begin(); itAdd != addedPt.end(); ++itAdd) {
            dist = vpMeSite::sqrDistance(pix, *itAdd);
            if (dist < vpMath::sqr(m_me->getSampleStep()))
              exist = true;
          }
          if (!exist) {
            findAngle(I, iPtemp, m_me, delta, convlt);
            pix.init(iPtemp.get_i(), iPtemp.get_j(), delta, convlt);
            pix.setDisplay(m_selectDisplay);
            m_meList.push_back(pix);
            addedPt.push_back(pix);
            nbr++;
          }
        }
      }

      unsigned int memory_range = m_me->getRange();
      m_me->setRange(3);
      std::list<vpMeSite>::iterator itList2 = m_meList.end();
      --itList2; // Move to the last element
      for (unsigned int j = 0; j < nbr; j++) {
        vpMeSite me_s = *itList2;
        me_s.track(I, m_me, false);
        *itList2 = me_s;
        --itList2;
      }
      m_me->setRange(memory_range);
    }

    /* if (end != nullptr) */ delete[] end;
    endPtFound = 0;
  }
}

void vpMeNurbs::reSample(const vpImage<unsigned char> &I)
{
  unsigned int n = numberOfSignal();
  double nbPt = floor(dist / m_me->getSampleStep());

  if ((double)n < 0.7 * nbPt) {
    sample(I);
    vpMeTracker::initTracking(I);
  }
}

void vpMeNurbs::localReSample(const vpImage<unsigned char> &I)
{
  int rows = (int)I.getHeight();
  int cols = (int)I.getWidth();
  vpImagePoint *iP = nullptr;

  int n = (int)numberOfSignal();

  std::list<vpMeSite>::iterator it = m_meList.begin();
  std::list<vpMeSite>::iterator itNext = m_meList.begin();
  ++itNext;

  unsigned int range_tmp = m_me->getRange();
  m_me->setRange(2);

  while (itNext != m_meList.end() && n <= m_me->getPointsToTrack()) {
    vpMeSite s = *it;          // current reference pixel
    vpMeSite s_next = *itNext; // current reference pixel

    double d = vpMeSite::sqrDistance(s, s_next);
    if (d > 4 * vpMath::sqr(m_me->getSampleStep()) && d < 1600) {
      vpImagePoint iP0(s.m_ifloat, s.m_jfloat);
      vpImagePoint iPend(s_next.m_ifloat, s_next.m_jfloat);
      vpImagePoint iP_1(s.m_ifloat, s.m_jfloat);

      double u = 0.0;
      double ubegin = 0.0;
      double uend = 0.0;
      double dmin1_1 = 1e6;
      double dmin2_1 = 1e6;
      while (u < 1) {
        u += 0.01;
        double dmin1 = vpImagePoint::sqrDistance(nurbs.computeCurvePoint(u), iP0);
        double dmin2 = vpImagePoint::sqrDistance(nurbs.computeCurvePoint(u), iPend);

        if (dmin1 < dmin1_1) {
          dmin1_1 = dmin1;
          ubegin = u;
        }

        if (dmin2 < dmin2_1) {
          dmin2_1 = dmin2;
          uend = u;
        }
      }
      u = ubegin;

      // if(( u != 1.0 || uend != 1.0)
      if ((std::fabs(u - 1.0) > std::fabs(vpMath::maximum(u, 1.0)) * std::numeric_limits<double>::epsilon()) ||
        (std::fabs(uend - 1.0) > std::fabs(vpMath::maximum(uend, 1.0)) * std::numeric_limits<double>::epsilon())) {
        iP = nurbs.computeCurveDersPoint(u, 1);

        while (vpImagePoint::sqrDistance(iP[0], iPend) > vpMath::sqr(m_me->getSampleStep()) && u < uend) {
          u += 0.01;
          /*if (iP!=nullptr)*/ {
            delete[] iP;
            iP = nullptr;
          }
          iP = nurbs.computeCurveDersPoint(u, 1);
          if (vpImagePoint::sqrDistance(iP[0], iP_1) > vpMath::sqr(m_me->getSampleStep()) &&
            !outOfImage(iP[0], 0, rows, cols)) {
            double delta = computeDelta(iP[1].get_i(), iP[1].get_j());
            vpMeSite pix;
            pix.init(iP[0].get_i(), iP[0].get_j(), delta);
            pix.setDisplay(m_selectDisplay);
            pix.track(I, m_me, false);
            if (pix.getState() == vpMeSite::NO_SUPPRESSION) {
              m_meList.insert(it, pix);
              iP_1 = iP[0];
            }
          }
        }
        /*if (iP!=nullptr)*/ {
          delete[] iP;
          iP = nullptr;
        }
      }
    }
    ++it;
    ++itNext;
  }
  m_me->setRange(range_tmp);
}

void vpMeNurbs::supressNearPoints()
{
  std::list<vpMeSite>::const_iterator it = m_meList.begin();
  std::list<vpMeSite>::iterator itNext = m_meList.begin();
  ++itNext;
  for (; itNext != m_meList.end();) {
    vpMeSite s = *it;          // current reference pixel
    vpMeSite s_next = *itNext; // current reference pixel

    if (vpMeSite::sqrDistance(s, s_next) < vpMath::sqr(m_me->getSampleStep())) {
      s_next.setState(vpMeSite::TOO_NEAR);

      *itNext = s_next;
      ++it;
      ++itNext;
      if (itNext != m_meList.end()) {
        ++it;
        ++itNext;
      }
    }
    else {
      ++it;
      ++itNext;
    }
  }
}

void vpMeNurbs::track(const vpImage<unsigned char> &I)
{
  // Tracking des vpMeSites
  vpMeTracker::track(I);

  // Suppress points which are too close to each other
  supressNearPoints();

  // Suppressions des points ejectes par le tracking
  suppressPoints();

  if (m_meList.size() == 1)
    throw(vpTrackingException(vpTrackingException::notEnoughPointError, "Not enough valid me to track"));

  // Recalcule les parametres
  //  nurbs.globalCurveInterp(m_meList);
  nurbs.globalCurveApprox(m_meList, nbControlPoints);

  // On resample localement
  localReSample(I);

  seekExtremities(I);
  if (enableCannyDetection)
    seekExtremitiesCanny(I);

  //   nurbs.globalCurveInterp(m_meList);
  nurbs.globalCurveApprox(m_meList, nbControlPoints);

  double u = 0.0;
  vpImagePoint pt;
  vpImagePoint pt_1;
  dist = 0;
  while (u <= 1.0) {
    pt = nurbs.computeCurvePoint(u);
    // if(u!=0)
    if (std::fabs(u) > std::numeric_limits<double>::epsilon())
      dist = dist + vpImagePoint::distance(pt, pt_1);
    pt_1 = pt;
    u = u + 0.01;
  }

  updateDelta();

  reSample(I);
}

void vpMeNurbs::display(const vpImage<unsigned char> &I, const vpColor &color, unsigned int thickness)
{
  vpMeNurbs::display(I, nurbs, color, thickness);
}

/*!
  Considering a pixel iP compute the next element of the Freeman chain
  code.

  According to the gray level of pixel iP and his eight neighbors determine
  the next element of the chain in order to turn around the dot
  counterclockwise.

  \param I : The image we are working with.
  \param iP : The current pixel.
  \param element : The next freeman element chain code (0, 1, 2, 3, 4, 5, 6,
  7) with 0 for right moving, 2 for down, 4 for left and 6 for up moving.

  \return false if an element cannot be found. Occurs for example with an area
  constituted by a single pixel. Return true if success.
*/
bool vpMeNurbs::computeFreemanChainElement(const vpImage<unsigned char> &I, vpImagePoint &iP, unsigned int &element)
{
  vpImagePoint diP;
  vpImagePoint iPtemp;
  if (hasGoodLevel(I, iP)) {
    // get the point on the right of the point passed in
    computeFreemanParameters((element + 2) % 8, diP);
    iPtemp = iP + diP;
    if (hasGoodLevel(I, iPtemp)) {
      element = (element + 2) % 8; // turn right
    }
    else {
      computeFreemanParameters((element + 1) % 8, diP);
      iPtemp = iP + diP;

      if (hasGoodLevel(I, iPtemp)) {
        element = (element + 1) % 8; // turn diag right
      }
      else {
        computeFreemanParameters(element, diP);
        iPtemp = iP + diP;

        if (hasGoodLevel(I, iPtemp)) {
          // element = element;      // keep same dir
        }
        else {
          computeFreemanParameters((element + 7) % 8, diP);
          iPtemp = iP + diP;

          if (hasGoodLevel(I, iPtemp)) {
            element = (element + 7) % 8; // turn diag left
          }
          else {
            computeFreemanParameters((element + 6) % 8, diP);
            iPtemp = iP + diP;

            if (hasGoodLevel(I, iPtemp)) {
              element = (element + 6) % 8; // turn left
            }
            else {
              computeFreemanParameters((element + 5) % 8, diP);
              iPtemp = iP + diP;

              if (hasGoodLevel(I, iPtemp)) {
                element = (element + 5) % 8; // turn diag down
              }
              else {
                computeFreemanParameters((element + 4) % 8, diP);
                iPtemp = iP + diP;

                if (hasGoodLevel(I, iPtemp)) {
                  element = (element + 4) % 8; // turn down
                }
                else {
                  computeFreemanParameters((element + 3) % 8, diP);
                  iPtemp = iP + diP;

                  if (hasGoodLevel(I, iPtemp)) {
                    element = (element + 3) % 8; // turn diag right down
                  }
                  else {
                    // No neighbor with a good level
                    //
                    return false;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  else {
    return false;
  }
  return true;
}

/*!
  Check if the pixel iP is in the image and has
  a good level to belong to the edge.

  \param I : Image.
  \param iP : Pixel to test

  \return true : If the pixel iP is in the area and
  has a value greater than 0.

  \return false : Otherwise
*/
bool vpMeNurbs::hasGoodLevel(const vpImage<unsigned char> &I, const vpImagePoint &iP) const
{
  if (!isInImage(I, iP))
    return false;

  if (I((unsigned int)vpMath::round(iP.get_i()), (unsigned int)vpMath::round(iP.get_j())) > 0) {
    return true;
  }
  else {
    return false;
  }
}

/*!
  Test if a pixel is in the image. Points of the border are not considered to
  be in the image.

  \param I : The image.
  \param iP : An image point.

  \return true if the image point \e iP is in the image and false
  otherwise.
*/
bool vpMeNurbs::isInImage(const vpImage<unsigned char> &I, const vpImagePoint &iP) const
{
  return (iP.get_i() >= 0 && iP.get_j() >= 0 && iP.get_i() < I.getHeight() && iP.get_j() < I.getWidth());
}

/*!
  Gives the displacement corresponding to the value of the parameter \e
  element.

  - If element = 0 diP = (0,1).
  - If element = 1 diP = (1,1).
  - If element = 2 diP = (1,0).
  - If element = 3 diP = (1,-1).
  - If element = 4 diP = (0,-1).
  - If element = 5 diP = (-1,-1).
  - If element = 6 diP = (-1,0).
  - If element = 7 diP = (-1,1).

  \param element : the element value(typically given by the method
  computeFreemanChainElement). \param diP : the output parameter which
  contains the displacement cooresponding to the value of \e element.
*/
void vpMeNurbs::computeFreemanParameters(unsigned int element, vpImagePoint &diP)
{
  /*
           5  6  7
            \ | /
             \|/
         4 ------- 0
             /|\
            / | \
           3  2  1
  */
  switch (element) {
  case 0: // go right
    diP.set_ij(0, 1);
    break;

  case 1: // go right top
    diP.set_ij(1, 1);
    break;

  case 2: // go top
    diP.set_ij(1, 0);
    break;

  case 3:
    diP.set_ij(1, -1);
    break;

  case 4:
    diP.set_ij(0, -1);
    break;

  case 5:
    diP.set_ij(-1, -1);
    break;

  case 6:
    diP.set_ij(-1, 0);
    break;

  case 7:
    diP.set_ij(-1, 1);
    break;
  }
}

/*!
  Check if the point is far enough from the image edges

  \param I : The image.
  \param iP : An image point.

  \return true if the point iP is at least 20 pixels far from the image
  edeges.
*/
bool vpMeNurbs::farFromImageEdge(const vpImage<unsigned char> &I, const vpImagePoint &iP)
{
  unsigned int height = I.getHeight();
  unsigned int width = I.getWidth();
  return (iP.get_i() < height - 20 && iP.get_j() < width - 20 && iP.get_i() > 20 && iP.get_j() > 20);
}

void vpMeNurbs::display(const vpImage<unsigned char> &I, vpNurbs &n, const vpColor &color, unsigned int thickness)
{
  double u = 0.0;
  vpImagePoint pt;
  while (u <= 1) {
    pt = n.computeCurvePoint(u);
    vpDisplay::displayCross(I, pt, 4, color, thickness);
    u += 0.01;
  }
}

void vpMeNurbs::display(const vpImage<vpRGBa> &I, vpNurbs &n, const vpColor &color, unsigned int thickness)
{
  double u = 0.0;
  vpImagePoint pt;
  while (u <= 1) {
    pt = n.computeCurvePoint(u);
    vpDisplay::displayCross(I, pt, 4, color, thickness);
    u += 0.01;
  }
}
END_VISP_NAMESPACE
