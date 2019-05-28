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
 * Make the complete tracking of an object by using its CAD model
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined _MSC_VER && _MSC_VER >= 1200
#define NOMINMAX
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <utility>

#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/mbt/vpMbScanLine.h>

#if defined(DEBUG_DISP)
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS

vpMbScanLine::vpMbScanLine()
  : w(0), h(0), K(), maskBorder(0), mask(), primitive_ids(), visibility_samples(), depthTreshold(1e-06)
#if defined(DEBUG_DISP)
    ,
    dispMaskDebug(NULL), dispLineDebug(NULL), linedebugImg()
#endif
{
#if defined(VISP_HAVE_X11) && defined(DEBUG_DISP)
  dispLineDebug = new vpDisplayX();
  dispMaskDebug = new vpDisplayX();
#elif defined(VISP_HAVE_GDI) && defined(DEBUG_DISP)
  dispLineDebug = new vpDisplayGDI();
  dispMaskDebug = new vpDisplayGDI();
#endif
}

vpMbScanLine::~vpMbScanLine()
{
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(DEBUG_DISP)
  if (dispLineDebug != NULL)
    delete dispLineDebug;
  if (dispMaskDebug != NULL)
    delete dispMaskDebug;
#endif
}
/*!
  Compute the intersections between Y-axis scanlines and a given line (two
  points polygon).

  \param a : First point of the line.
  \param b : Second point of the line.
  \param edge : Pair with the two points of the line.
  \param ID : Id of the given line (has to be know when using queries).
  \param scanlines : Resulting intersections.
*/
void vpMbScanLine::drawLineY(const vpColVector &a, const vpColVector &b, const vpMbScanLineEdge &edge, const int ID,
                             std::vector<std::vector<vpMbScanLineSegment> > &scanlines)
{
  double x0 = a[0] / a[2];
  double y0 = a[1] / a[2];
  double z0 = a[2];
  double x1 = b[0] / b[2];
  double y1 = b[1] / b[2];
  double z1 = b[2];
  if (y0 > y1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
    std::swap(z0, z1);
  }

  // if (y0 >= h - 1 || y1 < 0 || y1 == y0)
  if (y0 >= h - 1 || y1 < 0 || std::fabs(y1 - y0) <= std::numeric_limits<double>::epsilon())
    return;

  const unsigned int _y0 = (std::max)((unsigned int)0, (unsigned int)(std::ceil(y0)));
  const double _y1 = (std::min)((double)h, (double)y1);

  const bool b_sample_Y = (std::fabs(y0 - y1) > std::fabs(x0 - x1));

  for (unsigned int y = _y0; y < _y1; ++y) {
    const double x = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    const double alpha = getAlpha(y, y0 * z0, z0, y1 * z1, z1);
    vpMbScanLineSegment s;
    s.p = x;
    s.type = POINT;
    s.Z2 = s.Z1 = mix(z0, z1, alpha);
    s.P2 = s.P1 = s.p * s.Z1;
    s.ID = ID;
    s.edge = edge;
    s.b_sample_Y = b_sample_Y;
    scanlines[y].push_back(s);
  }
}

/*!
  Compute the intersections between X-axis scanlines and a given line (two
  points polygon).

  \param a : First point of the line.
  \param b : Second point of the line.
  \param edge : Pair with the two points of the line.
  \param ID : Id of the given line (has to be know when using queries).
  \param scanlines : Resulting intersections.
*/
void vpMbScanLine::drawLineX(const vpColVector &a, const vpColVector &b, const vpMbScanLineEdge &edge, const int ID,
                             std::vector<std::vector<vpMbScanLineSegment> > &scanlines)
{
  double x0 = a[0] / a[2];
  double y0 = a[1] / a[2];
  double z0 = a[2];
  double x1 = b[0] / b[2];
  double y1 = b[1] / b[2];
  double z1 = b[2];
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
    std::swap(z0, z1);
  }

  // if (x0 >= w - 1 || x1 < 0 || x1 == x0)
  if (x0 >= w - 1 || x1 < 0 || std::fabs(x1 - x0) <= std::numeric_limits<double>::epsilon())
    return;

  const unsigned int _x0 = (std::max)((unsigned int)0, (unsigned int)(std::ceil(x0)));
  const double _x1 = (std::min)((double)w, (double)x1);

  const bool b_sample_Y = (std::fabs(y0 - y1) > std::fabs(x0 - x1));

  for (unsigned int x = _x0; x < _x1; ++x) {
    const double y = y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    const double alpha = getAlpha(x, x0 * z0, z0, x1 * z1, z1);
    vpMbScanLineSegment s;
    s.p = y;
    s.type = POINT;
    s.Z2 = s.Z1 = mix(z0, z1, alpha);
    s.P2 = s.P1 = s.p * s.Z1;
    s.ID = ID;
    s.edge = edge;
    s.b_sample_Y = b_sample_Y;
    scanlines[x].push_back(s);
  }
}

/*!
  Compute the Y-axis scanlines intersections of a polygon.

  \param polygon : Polygon composed by an array of lines.
  \param ID : ID of the polygon (has to be know when using queries).
  \param scanlines : Resulting intersections.
*/
void vpMbScanLine::drawPolygonY(const std::vector<std::pair<vpPoint, unsigned int> > &polygon, const int ID,
                                std::vector<std::vector<vpMbScanLineSegment> > &scanlines)
{
  if (polygon.size() < 2)
    return;

  if (polygon.size() == 2) {
    vpColVector p1, p2;
    createVectorFromPoint(polygon.front().first, p1, K);
    createVectorFromPoint(polygon.back().first, p2, K);

    drawLineY(p1, p2, makeMbScanLineEdge(polygon.front().first, polygon.back().first), ID, scanlines);
    return;
  }

  std::vector<std::vector<vpMbScanLineSegment> > local_scanlines;
  local_scanlines.resize(h);

  for (size_t i = 0; i < polygon.size(); ++i) {
    vpColVector p1, p2;
    createVectorFromPoint(polygon[i].first, p1, K);
    createVectorFromPoint(polygon[(i + 1) % polygon.size()].first, p2, K);

    drawLineY(p1, p2, makeMbScanLineEdge(polygon[i].first, polygon[(i + 1) % polygon.size()].first), ID,
              local_scanlines);
  }

  createScanLinesFromLocals(scanlines, local_scanlines, h);
}

/*!
  Compute the X-axis scanlines intersections of a polygon.

  \param polygon : Polygon composed by an array of lines.
  \param ID : ID of the polygon (has to be know when using queries).
  \param scanlines : Resulting intersections.
*/
void vpMbScanLine::drawPolygonX(const std::vector<std::pair<vpPoint, unsigned int> > &polygon, const int ID,
                                std::vector<std::vector<vpMbScanLineSegment> > &scanlines)
{
  if (polygon.size() < 2)
    return;

  if (polygon.size() == 2) {
    vpColVector p1, p2;
    createVectorFromPoint(polygon.front().first, p1, K);
    createVectorFromPoint(polygon.back().first, p2, K);

    drawLineX(p1, p2, makeMbScanLineEdge(polygon.front().first, polygon.back().first), ID, scanlines);
    return;
  }

  std::vector<std::vector<vpMbScanLineSegment> > local_scanlines;
  local_scanlines.resize(w);

  for (size_t i = 0; i < polygon.size(); ++i) {
    vpColVector p1, p2;
    createVectorFromPoint(polygon[i].first, p1, K);
    createVectorFromPoint(polygon[(i + 1) % polygon.size()].first, p2, K);

    drawLineX(p1, p2, makeMbScanLineEdge(polygon[i].first, polygon[(i + 1) % polygon.size()].first), ID,
              local_scanlines);
  }

  createScanLinesFromLocals(scanlines, local_scanlines, w);
}

/*!
  Organise local scanlines in a global scanline vector.
  It also marks the computed intersections as starting or ending points.
  This function will only be called by the drawPolygons functions.

  \param scanlines : Global scanline vector.
  \param localScanlines : Local scanline vector (X or Y-axis).
  \param size : Corresponding size (typically the width or the height).
*/
void vpMbScanLine::createScanLinesFromLocals(std::vector<std::vector<vpMbScanLineSegment> > &scanlines,
                                             std::vector<std::vector<vpMbScanLineSegment> > &localScanlines,
                                             const unsigned int &size)
{
  for (unsigned int j = 0; j < size; ++j) {
    std::vector<vpMbScanLineSegment> &scanline = localScanlines[j];
    sort(scanline.begin(), scanline.end(),
         vpMbScanLineSegmentComparator()); // Not sure its necessary

    bool b_start = true;
    for (size_t i = 0; i < scanline.size(); ++i) {
      vpMbScanLineSegment s = scanline[i];
      if (b_start) {
        s.type = START;
        s.P1 = s.p * s.Z1;
        b_start = false;
      } else {
        vpMbScanLineSegment &prev = scanlines[j].back();
        s.type = END;
        s.P1 = prev.P1;
        s.Z1 = prev.Z1;
        s.P2 = s.p * s.Z2;
        prev.P2 = s.P2;
        prev.Z2 = s.Z2;
        b_start = true;
      }
      scanlines[j].push_back(s);
    }
  }
}

/*!
  Render a scene of polygons and compute scanlines intersections in order to
  use queries.

  \param polygons : List of polygons composed by arrays of lines.
  \param listPolyIndices : List of polygons IDs (has to be know when using
  queries). \param cam : Camera parameters. \param width : Width of the image
  (render window). \param height : Height of the image (render window).
*/
void vpMbScanLine::drawScene(const std::vector<std::vector<std::pair<vpPoint, unsigned int> > *> &polygons,
                             std::vector<int> listPolyIndices, const vpCameraParameters &cam, unsigned int width,
                             unsigned int height)
{
  this->w = width;
  this->h = height;
  this->K = cam;

  visibility_samples.clear();

  std::vector<std::vector<vpMbScanLineSegment> > scanlinesY;
  scanlinesY.resize(h);
  std::vector<std::vector<vpMbScanLineSegment> > scanlinesX;
  scanlinesX.resize(w);

  mask.resize(h, w, 0);

  vpImage<unsigned char> maskY(h, w, 0);
  vpImage<unsigned char> maskX(h, w, 0);

  primitive_ids.resize(h, w, -1);

  for (unsigned int ID = 0; ID < polygons.size(); ++ID) {
    drawPolygonY(*(polygons[ID]), listPolyIndices[ID], scanlinesY);
    drawPolygonX(*(polygons[ID]), listPolyIndices[ID], scanlinesX);
  }

  // Y
  int last_ID = -1;
  vpMbScanLineSegment last_visible;
  for (unsigned int y = 0; y < scanlinesY.size(); ++y) {
    std::vector<vpMbScanLineSegment> &scanline = scanlinesY[y];
    sort(scanline.begin(), scanline.end(), vpMbScanLineSegmentComparator());

    std::vector<std::pair<double, vpMbScanLineSegment> > stack;
    for (size_t i = 0; i < scanline.size(); ++i) {
      const vpMbScanLineSegment &s = scanline[i];

      switch (s.type) {
      case START:
        stack.push_back(std::make_pair(s.Z1, s));
        break;
      case END:
        for (size_t j = 0; j < stack.size(); ++j)
          if (stack[j].second.ID == s.ID) {
            if (j != stack.size() - 1)
              stack[j] = stack.back();
            stack.pop_back();
            break;
          }
        break;
      case POINT:
        break;
      }

      for (size_t j = 0; j < stack.size(); ++j) {
        const vpMbScanLineSegment &s0 = stack[j].second;
        stack[j].first = mix(s0.Z1, s0.Z2, getAlpha(s.type == POINT ? s.p : (s.p + 0.5), s0.P1, s0.Z1, s0.P2, s0.Z2));
      }
      sort(stack.begin(), stack.end(), vpMbScanLineSegmentComparator());

      int new_ID = stack.empty() ? -1 : stack.front().second.ID;

      if (new_ID != last_ID || s.type == POINT) {
        if (s.b_sample_Y)
          switch (s.type) {
          case POINT:
            if (new_ID == -1 || s.Z1 - depthTreshold <= stack.front().first)
              visibility_samples[s.edge].insert((int)y);
            break;
          case START:
            if (new_ID == s.ID)
              visibility_samples[s.edge].insert((int)y);
            break;
          case END:
            if (last_ID == s.ID)
              visibility_samples[s.edge].insert((int)y);
            break;
          }

        // This part will only be used for MbKltTracking
        if (last_ID != -1) {
          const unsigned int x0 = (std::max)((unsigned int)0, (unsigned int)(std::ceil(last_visible.p)));
          const double x1 = (std::min)((double)w, (double)s.p);
          for (unsigned int x = x0 + maskBorder; x < x1 - maskBorder; ++x) {
            primitive_ids[(unsigned int)y][(unsigned int)x] = last_visible.ID;

            if (maskBorder != 0)
              maskY[(unsigned int)y][(unsigned int)x] = 255;
            else
              mask[(unsigned int)y][(unsigned int)x] = 255;
          }
        }

        last_ID = new_ID;
        if (!stack.empty()) {
          last_visible = stack.front().second;
          last_visible.p = s.p;
        }
      }
    }
  }

  // X
  last_ID = -1;
  for (unsigned int x = 0; x < scanlinesX.size(); ++x) {
    std::vector<vpMbScanLineSegment> &scanline = scanlinesX[x];
    sort(scanline.begin(), scanline.end(), vpMbScanLineSegmentComparator());

    std::vector<std::pair<double, vpMbScanLineSegment> > stack;
    for (size_t i = 0; i < scanline.size(); ++i) {
      const vpMbScanLineSegment &s = scanline[i];

      switch (s.type) {
      case START:
        stack.push_back(std::make_pair(s.Z1, s));
        break;
      case END:
        for (size_t j = 0; j < stack.size(); ++j)
          if (stack[j].second.ID == s.ID) {
            if (j != stack.size() - 1)
              stack[j] = stack.back();
            stack.pop_back();
            break;
          }
        break;
      case POINT:
        break;
      }

      for (size_t j = 0; j < stack.size(); ++j) {
        const vpMbScanLineSegment &s0 = stack[j].second;
        stack[j].first = mix(s0.Z1, s0.Z2, getAlpha(s.type == POINT ? s.p : (s.p + 0.5), s0.P1, s0.Z1, s0.P2, s0.Z2));
      }
      sort(stack.begin(), stack.end(), vpMbScanLineSegmentComparator());

      int new_ID = stack.empty() ? -1 : stack.front().second.ID;

      if (new_ID != last_ID || s.type == POINT) {
        if (!s.b_sample_Y)
          switch (s.type) {
          case POINT:
            if (new_ID == -1 || s.Z1 - depthTreshold <= stack.front().first)
              visibility_samples[s.edge].insert((int)x);
            break;
          case START:
            if (new_ID == s.ID)
              visibility_samples[s.edge].insert((int)x);
            break;
          case END:
            if (last_ID == s.ID)
              visibility_samples[s.edge].insert((int)x);
            break;
          }

        // This part will only be used for MbKltTracking
        if (maskBorder != 0 && last_ID != -1) {
          const unsigned int y0 = (std::max)((unsigned int)0, (unsigned int)(std::ceil(last_visible.p)));
          const double y1 = (std::min)((double)h, (double)s.p);
          for (unsigned int y = y0 + maskBorder; y < y1 - maskBorder; ++y) {
            // primitive_ids[(unsigned int)y][(unsigned int)x] =
            // last_visible.ID;
            maskX[(unsigned int)y][(unsigned int)x] = 255;
          }
        }

        last_ID = new_ID;
        if (!stack.empty()) {
          last_visible = stack.front().second;
          last_visible.p = s.p;
        }
      }
    }
  }

  if (maskBorder != 0)
    for (unsigned int i = 0; i < h; i++)
      for (unsigned int j = 0; j < w; j++)
        if (maskX[i][j] == 255 && maskY[i][j] == 255)
          mask[i][j] = 255;

#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(DEBUG_DISP)
  if (!dispMaskDebug->isInitialised()) {
    dispMaskDebug->init(mask, 800, 600);
  }

  vpDisplay::display(mask);

  for (unsigned int ID = 0; ID < polygons.size(); ++ID) {
    for (unsigned int i = 0; i < polygons[ID]->size(); i++) {
      vpPoint p1 = (*(polygons[ID]))[i].first;
      vpPoint p2 = (*(polygons[ID]))[(i + 1) % polygons[ID]->size()].first;
      double i1 = 0, j1 = 0, i2 = 0, j2 = 0;
      p1.project();
      p2.project();
      vpMeterPixelConversion::convertPoint(K, p1.get_x(), p1.get_y(), j1, i1);
      vpMeterPixelConversion::convertPoint(K, p2.get_x(), p2.get_y(), j2, i2);

      vpDisplay::displayLine(mask, i1, j1, i2, j2, vpColor::red, 3);
    }
  }

  vpDisplay::flush(mask);

  if (!dispLineDebug->isInitialised()) {
    linedebugImg.resize(h, w, 0);
    dispLineDebug->init(linedebugImg, 800, 100);
  }
  vpDisplay::display(linedebugImg);
#endif
}

/*!
  Test the visibility of a line. As a result, a subsampled line of the given
  one with all its visible parts.

  \param a : First point of the line.
  \param b : Second point of the line.
  \param lines : List of lines corresponding of the visible parts of the given
  line. \param displayResults : True if the results have to be displayed.
  False otherwise.
*/
void vpMbScanLine::queryLineVisibility(const vpPoint &a, const vpPoint &b,
                                       std::vector<std::pair<vpPoint, vpPoint> > &lines, const bool &displayResults)
{
  vpColVector _a, _b;
  createVectorFromPoint(a, _a, K);
  createVectorFromPoint(b, _b, K);

  double x0 = _a[0] / _a[2];
  double y0 = _a[1] / _a[2];
  double z0 = _a[2];
  double x1 = _b[0] / _b[2];
  double y1 = _b[1] / _b[2];
  double z1 = _b[2];

  vpMbScanLineEdge edge = makeMbScanLineEdge(a, b);
  lines.clear();

  if (displayResults) {
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(DEBUG_DISP)
    double i1(0.0), j1(0.0), i2(0.0), j2(0.0);
    vpPoint a_(a), b_(b);
    a_.project();
    b_.project();
    vpMeterPixelConversion::convertPoint(K, a_.get_x(), a_.get_y(), j1, i1);
    vpMeterPixelConversion::convertPoint(K, b_.get_x(), b_.get_y(), j2, i2);

    vpDisplay::displayLine(linedebugImg, i1, j1, i2, j2, vpColor::yellow, 3);
#endif
  }

  if (!visibility_samples.count(edge))
    return;

  // Initialized as the biggest difference between the two points is on the
  // X-axis
  double *v0(&x0), *w0(&z0);
  double *v1(&x1), *w1(&z1);
  unsigned int size(w);

  if (std::fabs(y0 - y1) > std::fabs(x0 - x1)) // Test if the biggest difference is on the Y-axis
  {
    v0 = &y0;
    v1 = &y1;
    size = h;
  }

  // Cannot call swap(a,b) since a and b are const
  // The fix consists in 2 new points that contain the right points
  vpPoint a_;
  vpPoint b_;

  if (*v0 > *v1) {
    std::swap(v0, v1);
    std::swap(w0, w1);
    // std::swap(a, b);
    // Cannot call swap(a,b) since a and b are const
    // Instead of swap we set the right address of the corresponding pointers
    a_ = b;
    b_ = a;
  } else {
    a_ = a;
    b_ = b;
  }

  // if (*v0 >= size - 1 || *v1 < 0 || *v1 == *v0)
  if (*v0 >= size - 1 || *v1 < 0 || std::fabs(*v1 - *v0) <= std::numeric_limits<double>::epsilon())
    return;

  const int _v0 = (std::max)(0, int(std::ceil(*v0)));
  const int _v1 = (std::min)((int)(size - 1), (int)(std::ceil(*v1) - 1));

  const std::set<int> &visible_samples = visibility_samples[edge];
  int last = _v0;
  vpPoint line_start;
  vpPoint line_end;
  bool b_line_started = false;
  for (std::set<int>::const_iterator it = visible_samples.begin(); it != visible_samples.end(); ++it) {
    const int v = *it;
    const double alpha = getAlpha(v, (*v0) * (*w0), (*w0), (*v1) * (*w1), (*w1));
    // const vpPoint p = mix(a, b, alpha);
    const vpPoint p = mix(a_, b_, alpha);
    if (last + 1 != v) {
      if (b_line_started)
        lines.push_back(std::make_pair(line_start, line_end));
      b_line_started = false;
    }
    if (v == _v0) {
      // line_start = a;
      line_start = a_;
      line_end = p;
      b_line_started = true;
    } else if (v == _v1) {
      // line_end = b;
      line_end = b_;
      if (!b_line_started)
        line_start = p;
      b_line_started = true;
    } else {
      line_end = p;
      if (!b_line_started)
        line_start = p;
      b_line_started = true;
    }
    last = v;
  }
  if (b_line_started)
    lines.push_back(std::make_pair(line_start, line_end));

  if (displayResults) {
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(DEBUG_DISP)
    double i1(0.0), j1(0.0), i2(0.0), j2(0.0);
    for (unsigned int i = 0; i < lines.size(); i++) {
      lines[i].first.project();
      lines[i].second.project();
      vpMeterPixelConversion::convertPoint(K, lines[i].first.get_x(), lines[i].first.get_y(), j1, i1);
      vpMeterPixelConversion::convertPoint(K, lines[i].second.get_x(), lines[i].second.get_y(), j2, i2);

      vpDisplay::displayLine(linedebugImg, i1, j1, i2, j2, vpColor::red, 3);
    }
    vpDisplay::flush(linedebugImg);
#endif
  }
}

/*!
  Create a vpMbScanLineEdge from two points while ordering them.

  \param a : First point of the line.
  \param b : Second point of the line.

  \return Resulting vpMbScanLineEdge.
*/
vpMbScanLine::vpMbScanLineEdge vpMbScanLine::makeMbScanLineEdge(const vpPoint &a, const vpPoint &b)
{
  vpColVector _a(3);
  vpColVector _b(3);

  _a[0] = std::ceil((a.get_X() * 1e8) * 1e-6);
  _a[1] = std::ceil((a.get_Y() * 1e8) * 1e-6);
  _a[2] = std::ceil((a.get_Z() * 1e8) * 1e-6);

  _b[0] = std::ceil((b.get_X() * 1e8) * 1e-6);
  _b[1] = std::ceil((b.get_Y() * 1e8) * 1e-6);
  _b[2] = std::ceil((b.get_Z() * 1e8) * 1e-6);

  bool b_comp = false;
  for (unsigned int i = 0; i < 3; ++i)
    if (_a[i] < _b[i]) {
      b_comp = true;
      break;
    } else if (_a[i] > _b[i])
      break;

  if (b_comp)
    return std::make_pair(_a, _b);

  return std::make_pair(_b, _a);
}

/*!
  Create a vpColVector of a projected point.

  \param p : Point to project.
  \param v : Resulting vector.
  \param K : Camera parameters.
*/
void vpMbScanLine::createVectorFromPoint(const vpPoint &p, vpColVector &v, const vpCameraParameters &K)
{
  v = vpColVector(3);

  v[0] = p.get_X() * K.get_px() + K.get_u0() * p.get_Z();
  v[1] = p.get_Y() * K.get_py() + K.get_v0() * p.get_Z();
  v[2] = p.get_Z();
}

/*!
  Compute the interpolation factor.

  \param x : Value used as basis to compute the interpolation factor
  \param X0 : First extremity.
  \param Z0 : First extremity.
  \param X1 : Second extremity.
  \param Z1 : Second extremity.
*/
double vpMbScanLine::getAlpha(double x, double X0, double Z0, double X1, double Z1)
{
  const double N = X0 - x * Z0;
  const double D = x * (Z1 - Z0) - (X1 - X0);
  double alpha = N / D;
  if (vpMath::isNaN(alpha) || vpMath::isInf(alpha))
    return 0.0;

  alpha = (std::min)(1.0, alpha);
  alpha = (std::max)(0.0, alpha);
  return alpha;
}

/*!
  Interpolate two values.

  \param a : first value.
  \param b : second value.
  \param alpha : interpolation factor.

  \return Interpolated value.
*/
double vpMbScanLine::mix(double a, double b, double alpha) { return a + (b - a) * alpha; }

/*!
  Interpolate two vpPoints.

  \param a : first point.
  \param b : second point.
  \param alpha : interpolation factor.

  \return Interpolated vpPoint.
*/
vpPoint vpMbScanLine::mix(const vpPoint &a, const vpPoint &b, double alpha)
{
  vpPoint res;
  res.set_X(a.get_X() + (b.get_X() - a.get_X()) * alpha);
  res.set_Y(a.get_Y() + (b.get_Y() - a.get_Y()) * alpha);
  res.set_Z(a.get_Z() + (b.get_Z() - a.get_Z()) * alpha);

  return res;
}

/*!
  Compute the norm of two vpPoints.

  \param a : first point.
  \param b : second point.

  \return Resulting norm.
*/
double vpMbScanLine::norm(const vpPoint &a, const vpPoint &b)
{
  return sqrt(vpMath::sqr(a.get_X() - b.get_X()) + vpMath::sqr(a.get_Y() - b.get_Y()) +
              vpMath::sqr(a.get_Z() - b.get_Z()));
}

#endif
