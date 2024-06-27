/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Defines a generic 2D polygon.
 */

// System
#include <limits>
#include <set>

// Core
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPolygon.h>
#include <visp3/core/vpUniRand.h>

// Local helper
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)

#include <opencv2/imgproc/imgproc.hpp>


/*!
 * Compute convex hull corners.
 *
 * \param[in] ips : List of 2D points.
 */
template <typename IpContainer> std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> convexHull(const IpContainer &ips)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  if (ips.size() == 0) {
    throw vpException(vpException::badValue,
                      "Convex Hull can not be computed as the input does not contain any image point.");
  }

  // Visp -> CV
  std::vector<cv::Point> cv_pts;
#if ((__cplusplus >= 201402L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201402L)))  // Check if cxx14 or higher
  std::transform(cbegin(ips), cend(ips), std::back_inserter(cv_pts), [](const vpImagePoint &ip) {
    return cv::Point(static_cast<int>(ip.get_u()), static_cast<int>(ip.get_v()));
                 });
#elif ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  std::transform(begin(ips), end(ips), std::back_inserter(cv_pts), [](const vpImagePoint &ip) {
    return cv::Point(static_cast<int>(ip.get_u()), static_cast<int>(ip.get_v()));
                 });
#else // cxx98
  for (typename IpContainer::const_iterator it = ips.begin(); it != ips.end(); ++it) {
    cv_pts.push_back(cv::Point(static_cast<int>(it->get_u()), static_cast<int>(it->get_v())));
  }
#endif

  // Get convex hull from OpenCV
  std::vector<cv::Point> cv_conv_hull_corners;
  cv::convexHull(cv_pts, cv_conv_hull_corners);

  // CV -> Visp
  std::vector<vpImagePoint> conv_hull_corners;
#if ((__cplusplus >= 201402L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201402L))) // Check if cxx14 or higher
  std::transform(cbegin(cv_conv_hull_corners), cend(cv_conv_hull_corners), std::back_inserter(conv_hull_corners),
                 [](const cv::Point &pt) {
                   return vpImagePoint { static_cast<double>(pt.y), static_cast<double>(pt.x) };
                 });
#elif ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  std::transform(begin(cv_conv_hull_corners), end(cv_conv_hull_corners), std::back_inserter(conv_hull_corners),
                 [](const cv::Point &pt) {
                   return vpImagePoint { static_cast<double>(pt.y), static_cast<double>(pt.x) };
                 });
#else // cxx98
  for (std::vector<cv::Point>::const_iterator it = cv_conv_hull_corners.begin(); it != cv_conv_hull_corners.end();
       ++it) {
    conv_hull_corners.push_back(vpImagePoint(static_cast<double>(it->y), static_cast<double>(it->x)));
  }
#endif

  return conv_hull_corners;
}

#endif

BEGIN_VISP_NAMESPACE
/*!
   Default constructor that creates an empty polygon.
*/
vpPolygon::vpPolygon()
  : _corners(), _center(), _area(0.), _goodPoly(true), _bbox(), m_PnPolyConstants(), m_PnPolyMultiples()
{ }

/*!
  Constructor which initialises the polygon thanks to the given corners.

  \warning the corners must be ordered (either clockwise or counter
  clockwise).

  \param corners : The Points defining the corners.
*/
vpPolygon::vpPolygon(const std::vector<vpImagePoint> &corners)
  : _corners(), _center(), _area(0.), _goodPoly(true), _bbox(), m_PnPolyConstants(), m_PnPolyMultiples()
{
  if (corners.size() < 3) {
    _goodPoly = false;
  }
  init(corners);
}

/*!
  Constructor which initialises the polygon thanks to the given corners.

  \warning the corners must be ordered (either clockwise or counter
  clockwise).

  \param corners : The Points defining the corners.
*/
vpPolygon::vpPolygon(const std::list<vpImagePoint> &corners)
  : _corners(), _center(), _area(0.), _goodPoly(true), _bbox(), m_PnPolyConstants(), m_PnPolyMultiples()
{
  if (corners.size() < 3) {
    _goodPoly = false;
  }
  init(corners);
}

/*!
  Copy constructor

  \param poly : The polygon used for the initialisation.
*/
vpPolygon::vpPolygon(const vpPolygon &poly)
  : _corners(poly._corners), _center(poly._center), _area(poly._area), _goodPoly(poly._goodPoly), _bbox(poly._bbox),
  m_PnPolyConstants(poly.m_PnPolyConstants), m_PnPolyMultiples(poly.m_PnPolyMultiples)
{ }

/*!
  Basic destructor
*/
vpPolygon::~vpPolygon() { }

/*!
  Equal operator.

  Assign \e poly to this polygon and return a reference to it.
*/
vpPolygon &vpPolygon::operator=(const vpPolygon &poly)
{
  _corners = poly._corners;
  _center = poly._center;
  _area = poly._area;
  _goodPoly = poly._goodPoly;
  _bbox = poly._bbox;
  m_PnPolyConstants = poly.m_PnPolyConstants;
  m_PnPolyMultiples = poly.m_PnPolyMultiples;
  return *this;
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \deprecated You should use build(const std::vector<vpImagePoint> &, const bool &) instead.
  Initialises the triangle thanks to the collection of 2D points (in pixel).

  \warning the corners must be ordered (either clockwise or counter
  clockwise).

  \param corners : The corners of the polygon.
  \param create_convex_hull: Create a convex hull from the given corners.
*/
void vpPolygon::buildFrom(const std::vector<vpImagePoint> &corners, const bool create_convex_hull)
{
  build(corners, create_convex_hull);
}

/*!
  \deprecated You should use build(const std::list<vpImagePoint> &, const bool &) instead.
  Initialises the polygon thanks to the collection of 2D points (in pixel).

  \warning the corners must be ordered (either clockwise or counter
  clockwise).

  \param corners : The corners of the polygon.
  \param create_convex_hull: Create a convex hull from the given corners.
*/
void vpPolygon::buildFrom(const std::list<vpImagePoint> &corners, const bool create_convex_hull)
{
  build(corners, create_convex_hull);
}

/*!
  \deprecated You should use build(const std::vector<vpPoint> &, const vpCameraParameters &, const bool &) instead.
  Initialises the triangle thanks to the collection of 2D points (in meter).
  The fields \e x and \e y are used to compute the corresponding coordinates
  in pixel thanks to the camera parameters \e cam.

  \warning the corners must be ordered (either clockwise or counter
  clockwise).

  \param corners : The corners of the polygon.
  \param cam : The camera parameters used to convert the coordinates from meter to pixel.
  \param create_convex_hull: Create a convex hull from the given corners.
*/
void vpPolygon::buildFrom(const std::vector<vpPoint> &corners, const vpCameraParameters &cam,
                          const bool create_convex_hull)
{
  build(corners, cam, create_convex_hull);
}
#endif

/*!
  Initialises the triangle thanks to the collection of 2D points (in pixel).

  \warning the corners must be ordered (either clockwise or counter
  clockwise).

  \param corners : The corners of the polygon.
  \param create_convex_hull: Create a convex hull from the given corners.
*/
vpPolygon &vpPolygon::build(const std::vector<vpImagePoint> &corners, const bool &create_convex_hull)
{
  if (create_convex_hull) {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
    init(convexHull(corners));
#else
    vpException(vpException::notImplementedError, "Cannot build a convex hull without OpenCV imgproc module");
#endif
  }
  else {
    init(corners);
  }
  return *this;
}

/*!
  Initialises the polygon thanks to the collection of 2D points (in pixel).

  \warning the corners must be ordered (either clockwise or counter
  clockwise).

  \param corners : The corners of the polygon.
  \param create_convex_hull: Create a convex hull from the given corners.
*/
vpPolygon &vpPolygon::build(const std::list<vpImagePoint> &corners, const bool &create_convex_hull)
{
  if (create_convex_hull) {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
    init(convexHull(corners));
#else
    vpException(vpException::notImplementedError, "Cannot build a convex hull without OpenCV imgproc module");
#endif
  }
  else {
    init(corners);
  }
  return *this;
}

/*!
  Initialises the triangle thanks to the collection of 2D points (in meter).
  The fields \e x and \e y are used to compute the corresponding coordinates
  in pixel thanks to the camera parameters \e cam.

  \warning the corners must be ordered (either clockwise or counter
  clockwise).

  \param corners : The corners of the polygon.
  \param cam : The camera parameters used to convert the coordinates from meter to pixel.
  \param create_convex_hull: Create a convex hull from the given corners.
*/
vpPolygon &vpPolygon::build(const std::vector<vpPoint> &corners, const vpCameraParameters &cam,
                          const bool &create_convex_hull)
{
  std::vector<vpImagePoint> ipCorners(corners.size());
  size_t corners_size = corners.size();
  for (size_t i = 0; i < corners_size; ++i) {
    vpMeterPixelConversion::convertPoint(cam, corners[i].get_x(), corners[i].get_y(), ipCorners[i]);
  }
  build(ipCorners, create_convex_hull);
  return *this;
}

/*!
  Initialises the polygon by (left-)clicking to add a corners to the polygon.
  A right click is used to stop the addition of new corners.

  \param I : The image where to click to initialise the corners.
  \param size : Cross size in terms of number of pixels that is displayed over a polygon corner.
  \param color : Color used to display the cross over the polygon corner.
  \param thickness : Thickness used to display the cross.
*/
void vpPolygon::initClick(const vpImage<unsigned char> &I, unsigned int size, const vpColor &color,
                          unsigned int thickness)
{
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
  vpImagePoint ip;

  std::vector<vpImagePoint> cornersClick;

  while (button == vpMouseButton::button1) {
    bool ret = vpDisplay::getClick(I, ip, button, true);
    if (ret && (button == vpMouseButton::button1)) {
      vpDisplay::displayCross(I, ip, size, color, thickness);
      cornersClick.push_back(ip);
      vpDisplay::flush(I);
    }
  }

  build(cornersClick);
}

/*!
  Initialises the polygon by (left-)clicking to add a corners to the polygon.
  A right click is used to stop the addition of new corners.

  \param I : The image where to click to initialise the corners.
  \param size : Size of the cross in terms of number of pixels that is displayed over a polygon corner.
  \param color : Color used to display the cross over the polygon corner.
  \param thickness : Thickness used to display the cross.
*/
void vpPolygon::initClick(const vpImage<vpRGBa> &I, unsigned int size, const vpColor &color, unsigned int thickness)
{
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
  vpImagePoint ip;

  std::vector<vpImagePoint> cornersClick;

  while (button == vpMouseButton::button1) {
    bool ret = vpDisplay::getClick(I, ip, button, true);
    if (ret && (button == vpMouseButton::button1)) {
      vpDisplay::displayCross(I, ip, size, color, thickness);
      cornersClick.push_back(ip);
      vpDisplay::flush(I);
    }
  }

  build(cornersClick);
}

/*!
  Initialises the polygon using the collection of image points. This method
  computes some internal variables such as center, area, ...

  \warning the corners must be ordered (either clockwise or counter
  clockwise).

  \param corners : The corners of the polygon.
*/
void vpPolygon::init(const std::vector<vpImagePoint> &corners)
{
  _corners = corners;

  updateBoundingBox();
  updateArea();
  updateCenter();

  precalcValuesPnPoly();
}

/*!
  Initialises the polygon using the collection of image points. This method
  computes some internal variables such as center, area, ...

  \warning the corners must be ordered (either clockwise or counter
  clockwise).

  \param corners : The corners of the polygon.
*/
void vpPolygon::init(const std::list<vpImagePoint> &corners)
{
  _corners.insert(_corners.begin(), corners.begin(), corners.end());

  updateBoundingBox();
  updateArea();
  updateCenter();

  precalcValuesPnPoly();
}

/*!
  Test if two segments are intersecting.

  \throw vpException::divideByZeroError if the two lines are aligned
  (denominator equal to zero).

  \param ip1 : The first image point of the first segment.
  \param ip2 : The second image point of the first segment.
  \param ip3 : The first image point of the second segment.
  \param ip4 : The second image point of the second segment.
*/
bool vpPolygon::testIntersectionSegments(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpImagePoint &ip3,
                                         const vpImagePoint &ip4) const
{
  double di1 = ip2.get_i() - ip1.get_i();
  double dj1 = ip2.get_j() - ip1.get_j();

  double di2 = ip4.get_i() - ip3.get_i();
  double dj2 = ip4.get_j() - ip3.get_j();

  double denominator = (di1 * dj2) - (dj1 * di2);

  if (fabs(denominator) < std::numeric_limits<double>::epsilon()) {
    throw vpException(vpException::divideByZeroError, "Denominator is null, lines are parallels");
  }

  double alpha = -(((ip1.get_i() - ip3.get_i()) * dj2) + (di2 * (ip3.get_j() - ip1.get_j()))) / denominator;
  if ((alpha < 0) || (alpha >= 1)) {
    return false;
  }

  double beta = -((di1 * (ip3.get_j() - ip1.get_j())) + (dj1 * (ip1.get_i() - ip3.get_i()))) / denominator;
  if ((beta < 0) || (beta >= 1)) {
    return false;
  }

  return true;
}

/*!
  Check if the 2D point \f$ ip \f$ is inside the polygon.

  \param ip : The point which have to be tested.
  \param method : Method to use for Point In Polygon test.

  \return Returns true if the point is inside the polygon, false otherwise.
*/
bool vpPolygon::isInside(const vpImagePoint &ip, const PointInPolygonMethod &method) const
{
  if (_corners.size() < 3) {
    return false;
  }

  bool test = false;
  // comment: previous implementation: switch method
  // case PnPolySegmentIntersection:
  if (method == PnPolySegmentIntersection) {
    vpImagePoint infPoint(100000, 100000); // take a point at 'infinity'
    // we add random since it appears that sometimes infPoint may cause a degenerated case (so relaunch and
    // hope that result will be different).
    vpUniRand generator;
    infPoint.set_i(infPoint.get_i() + (1000 * generator()));
    infPoint.set_j(infPoint.get_j() + (1000 * generator()));

    bool oddNbIntersections = false;
    size_t v_corners_size = _corners.size();
    for (size_t i = 0; i < v_corners_size; ++i) {
      vpImagePoint ip1 = _corners[i];
      vpImagePoint ip2 = _corners[(i + 1) % _corners.size()];
      bool intersection = false;

      // If the points are the same we continue without trying to found
      // an intersection
      if (ip1 == ip2) {
        // continue
      }
      else {
        try {
          intersection = testIntersectionSegments(ip1, ip2, ip, infPoint);
        }
        catch (...) {
          return isInside(ip);
        }

        if (intersection) {
          oddNbIntersections = !oddNbIntersections;
        }
      }
    }

    test = oddNbIntersections;
  }
  else {
    // Reference: http://alienryderflex.com/polygon/
    // comment: case PnPolyRayCasting
    // comment: default
    bool oddNodes = false;
    size_t v_corners_size = _corners.size();
    for (size_t i = 0, j = (v_corners_size - 1); i < v_corners_size; ++i) {
      if (((_corners[i].get_v() < ip.get_v()) && (_corners[j].get_v() >= ip.get_v())) ||
          ((_corners[j].get_v() < ip.get_v()) && (_corners[i].get_v() >= ip.get_v()))) {
        oddNodes ^= (ip.get_v() * m_PnPolyMultiples[i] + m_PnPolyConstants[i] < ip.get_u());
      }

      j = i;
    }

    test = oddNodes;
    // comment: }
    // comment: break
  }

  return test;
}

void vpPolygon::precalcValuesPnPoly()
{
  if (_corners.size() < 3) {
    return;
  }

  m_PnPolyConstants.resize(_corners.size());
  m_PnPolyMultiples.resize(_corners.size());

  size_t v_corners_size = _corners.size();
  for (size_t i = 0, j = (v_corners_size - 1); i < v_corners_size; ++i) {
    if (vpMath::equal(_corners[j].get_v(), _corners[i].get_v(), std::numeric_limits<double>::epsilon())) {
      m_PnPolyConstants[i] = _corners[i].get_u();
      m_PnPolyMultiples[i] = 0.0;
    }
    else {
      m_PnPolyConstants[i] = (_corners[i].get_u() -
        ((_corners[i].get_v() * _corners[j].get_u()) / (_corners[j].get_v() - _corners[i].get_v()))) +
        ((_corners[i].get_v() * _corners[i].get_u()) / (_corners[j].get_v() - _corners[i].get_v()));
      m_PnPolyMultiples[i] = (_corners[j].get_u() - _corners[i].get_u()) / (_corners[j].get_v() - _corners[i].get_v());
    }

    j = i;
  }
}

/*!
  Update the \c _area attribute of the polygon using the corners.

  The area is computed using the formula:
  \f[
  A = \frac{1}{2} \sum_{i=0}^{n-1} (x_1 y_{i+1} - x_{i+1} y_{i})
  \f]

*/
void vpPolygon::updateArea()
{
  if (_corners.size() == 0) {
    _area = 0;
    _goodPoly = false;
    return;
  }
  _area = 0;

  size_t v_corners_size = _corners.size();
  for (size_t i = 0; i < v_corners_size; ++i) {
    size_t i_p_1 = (i + 1) % _corners.size();
    _area += (_corners[i].get_j() * _corners[i_p_1].get_i()) - (_corners[i_p_1].get_j() * _corners[i].get_i());
  }

  _area /= 2;
  if (_area < 0) {
    _area = -_area;
  }
}

/*!
  Update the \c _center attribute of the polygon using the corners.

  The i coordinate is computed using:

  \f[
  i = \frac{1}{6{area}} \sum_{i=0}^{n-1} (i_i + i_{i+1})(i_{i+1} j_i - j_{i+1}
  i_i) \f]

  The computation of the j coordinate is similar.
*/
void vpPolygon::updateCenter()
{
  if (_corners.size() == 0) {
    _center = vpImagePoint(0, 0);
    _goodPoly = false;
    return;
  }
  double i_tmp = 0;
  double j_tmp = 0;

  size_t v_corners_size = _corners.size();
  for (size_t i = 0; i < v_corners_size; ++i) {
    size_t i_p_1 = (i + 1) % _corners.size();
    i_tmp += (_corners[i].get_i() + _corners[i_p_1].get_i()) *
      ((_corners[i_p_1].get_i() * _corners[i].get_j()) - (_corners[i_p_1].get_j() * _corners[i].get_i()));

    j_tmp += (_corners[i].get_j() + _corners[i_p_1].get_j()) *
      ((_corners[i_p_1].get_i() * _corners[i].get_j()) - (_corners[i_p_1].get_j() * _corners[i].get_i()));
  }

  if (_area > 0) {
    _center.set_i(fabs(i_tmp / (6 * _area)));
    _center.set_j(fabs(j_tmp / (6 * _area)));
  }
  else {
    _center = _corners[0];
    _goodPoly = false;
  }
}

/*!
  Update bounding box of the polygon.

  \sa getBoundingBox()
 */
void vpPolygon::updateBoundingBox()
{
  if (_corners.size() == 0) {
    _bbox.setBottomRight(vpImagePoint(0, 0));
    _bbox.setTopLeft(vpImagePoint(0, 0));
    _goodPoly = false;
    return;
  }

  std::set<double> setI;
  std::set<double> setJ;
  unsigned int v_corners_size = _corners.size();
  for (unsigned int i = 0; i < v_corners_size; ++i) {
    setI.insert(_corners[i].get_i());
    setJ.insert(_corners[i].get_j());
  }
  vpImagePoint tl(*(setI.begin()), *(setJ.begin()));
  std::set<double>::const_iterator iterI = setI.end();
  std::set<double>::const_iterator iterJ = setJ.end();
  --iterI;
  --iterJ;
  vpImagePoint br(*iterI, *iterJ);

  if (tl == br) {
    _goodPoly = false;
  }
  _bbox.setTopLeft(tl);
  _bbox.setBottomRight(br);
}

/*!
  Display the polygon in the image (overlay, so the image is not modified).
  A call to the flush() method is necessary.

  \param I : The image where the polygon is displayed.
  \param color : The color of the polygon's lines.
  \param thickness : The thickness of the polygon's lines.
*/
void vpPolygon::display(const vpImage<unsigned char> &I, const vpColor &color, unsigned int thickness) const
{
  const size_t N = _corners.size();
  for (size_t i = 0; i < N; ++i) {
    vpDisplay::displayLine(I, _corners[i], _corners[(i + 1) % N], color, thickness);
  }
}

/*!
  Test if an image point is inside a 2D polygon.

  \param roi : List of the polygon corners.
  \param i : i-coordinate of the image point to test.
  \param j : j-coordinate of the image point to test.
  \param method : Method to use for Point In Polygon test.

  \return True if the point defined by (i,j) is inside the polygon, False
  otherwise.
*/
bool vpPolygon::isInside(const std::vector<vpImagePoint> &roi, const double &i, const double &j,
                         const PointInPolygonMethod &method)
{
  vpPolygon poly(roi);
  return poly.isInside(vpImagePoint(i, j), method);
}

/*!
  Return number of corners belonging to the polygon.
 */
unsigned int vpPolygon::getSize() const
{
  return (static_cast<unsigned int>(_corners.size()));
}
END_VISP_NAMESPACE
