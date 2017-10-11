/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Defines a generic 2D polygon.
 *
 * Author:
 * Amaury Dame
 * Nicolas Melchior
 * Romain Tallonneau
 *
 *****************************************************************************/



#include <visp3/core/vpPolygon.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpUniRand.h>
#include <set>
#include <limits>
/*!
  Basic constructor.

  By default, it defines a triangle with the three 2D points coordinates :
  \f$ (0,0) \f$, \f$ (1,0) \f$ and \f$ (0,1) \f$.
*/
vpPolygon::vpPolygon()
  : _corners(), _center(), _area(0.), _goodPoly(true), _bbox(), m_PnPolyConstants(), m_PnPolyMultiples()
{
  std::vector<vpImagePoint> corners;
  corners.push_back(vpImagePoint(0,0));
  corners.push_back(vpImagePoint(1,0));
  corners.push_back(vpImagePoint(0,1));
  init(corners);
}

/*!
  Constructor which initialises the polygon thanks to the given corners.

  \warning the corners must be ordered (either clockwise or counter clockwise).

  \param corners : The Points defining the corners.
*/
vpPolygon::vpPolygon(const std::vector<vpImagePoint>& corners)
  : _corners(), _center(), _area(0.), _goodPoly(true), _bbox(), m_PnPolyConstants(), m_PnPolyMultiples()
{
  if(corners.size() < 3){
    _goodPoly = false;
  }
  init(corners);
}

/*!
  Constructor which initialises the polygon thanks to the given corners.

  \warning the corners must be ordered (either clockwise or counter clockwise).

  \param corners : The Points defining the corners.
*/
vpPolygon::vpPolygon(const std::list<vpImagePoint>& corners)
  : _corners(), _center(), _area(0.), _goodPoly(true), _bbox(), m_PnPolyConstants(), m_PnPolyMultiples()
{
  if(corners.size() < 3){
    _goodPoly = false;
  }
  init(corners);
}

/*!
  Copy constructor

  \param poly : The polygon used for the initialisation.
*/
vpPolygon::vpPolygon(const vpPolygon &poly)
  : _corners(poly._corners), _center(poly._center), _area(poly._area), _goodPoly(poly._goodPoly),
    _bbox(poly._bbox), m_PnPolyConstants(poly.m_PnPolyConstants), m_PnPolyMultiples(poly.m_PnPolyMultiples)
{
}

/*!
  Basic destructor
*/
vpPolygon::~vpPolygon()
{
}

/*!
  Equal operator.

  Assign \e poly to this polygon and return a reference to it.
*/
vpPolygon &
vpPolygon::operator=(const vpPolygon& poly)
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

/*!
  Initialises the triangle thanks to the collection of 2D points (in pixel).

  \warning the corners must be ordered (either clockwise or counter clockwise).

  \param corners : The corners of the polyon.
*/
void
vpPolygon::buildFrom(const std::vector<vpImagePoint>& corners)
{
  init(corners);
}

/*!
  Initialises the polygon thanks to the collection of 2D points (in pixel).

  \warning the corners must be ordered (either clockwise or counter clockwise).

  \param corners : The corners of the polyon.
*/
void
vpPolygon::buildFrom(const std::list<vpImagePoint>& corners)
{
  init(corners);
}

/*!
  Initialises the triangle thanks to the collection of 2D points (in meter). The
  fields \e x and \e y are used to compute the corresponding coordinates in
  pixel thanks to the camera parameters \e cam.

  \warning the corners must be ordered (either clockwise or counter clockwise).

  \param corners : The corners of the polyon.
  \param cam : The camera parameters used to convert the coordinates from meter
  to pixel.
*/
void
vpPolygon::buildFrom(const std::vector<vpPoint>& corners, const vpCameraParameters& cam)
{
  std::vector<vpImagePoint> ipCorners(corners.size());
  for(unsigned int i=0; i<corners.size(); ++i){
    vpMeterPixelConversion::convertPoint(cam, corners[i].get_x(), corners[i].get_y(), ipCorners[i]);
  }
  buildFrom(ipCorners);
}

/*!
  Initialises the polygon by (left-)clicking to add a corners to the polygon.
  A right click is used to stop the addition of new corners.

  \param I : The image where to click to initialise the corners.
  \param size : Cross size in terms of number of pixels that is displayed over a polygon corner.
  \param color : Color used to display the cross over the polygon corner.
  \param thickness : Thickness used to display the cross.
*/
void
vpPolygon::initClick(const vpImage<unsigned char>& I, unsigned int size, const vpColor &color, unsigned int thickness)
{
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
  vpImagePoint ip;

  std::vector<vpImagePoint> cornersClick;

  while(button == vpMouseButton::button1){
    bool ret = vpDisplay::getClick(I, ip, button, true);
    if(ret && button == vpMouseButton::button1){
      vpDisplay::displayCross(I, ip, size, color, thickness);
      cornersClick.push_back(ip);
      vpDisplay::flush(I);
    }
  }

  buildFrom(cornersClick);
}

/*!
  Initialises the polygon by (left-)clicking to add a corners to the polygon.
  A right click is used to stop the addition of new corners.

  \param I : The image where to click to initialise the corners.
  \param size : Size of the cross in terms of number of pixels that is displayed over a polygon corner.
  \param color : Color used to display the cross over the polygon corner.
  \param thickness : Thickness used to display the cross.
*/
void
vpPolygon::initClick(const vpImage<vpRGBa>& I, unsigned int size, const vpColor &color, unsigned int thickness)
{
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
  vpImagePoint ip;

  std::vector<vpImagePoint> cornersClick;

  while(button == vpMouseButton::button1){
    bool ret = vpDisplay::getClick(I, ip, button, true);
    if(ret && button == vpMouseButton::button1){
      vpDisplay::displayCross(I, ip, size, color, thickness);
      cornersClick.push_back(ip);
      vpDisplay::flush(I);
    }
  }

  buildFrom(cornersClick);
}

/*!
  Intialises the polygon using the collection of image points. This method
  computes some internal variables such as center, area, ...

  \warning the corners must be ordered (either clockwise or counter clockwise).

  \param corners : The corners of the polygon.
*/
void
vpPolygon::init(const std::vector<vpImagePoint>& corners)
{
  _corners = corners;

  updateBoundingBox();
  updateArea();
  updateCenter();

  precalcValuesPnPoly();
}


/*!
  Intialises the polygon using the collection of image points. This method
  computes some internal variables such as center, area, ...

  \warning the corners must be ordered (either clockwise or counter clockwise).

  \param corners : The corners of the polygon.
*/
void
vpPolygon::init(const std::list<vpImagePoint>& corners)
{
  _corners.insert(_corners.begin(), corners.begin(), corners.end());

  updateBoundingBox();
  updateArea();
  updateCenter();

  precalcValuesPnPoly();
}



/*!
  Test if two segments are intersecting.

  \throw vpException::divideByZeroError if the two lines are aligned (denominator equal to zero).

  \param ip1 : The first image point of the first segment.
  \param ip2 : The second image point of the first segment.
  \param ip3 : The first image point of the second segment.
  \param ip4 : The second image point of the second segment.
*/
bool
vpPolygon::testIntersectionSegments(const vpImagePoint& ip1, const vpImagePoint& ip2, const vpImagePoint& ip3, const vpImagePoint& ip4) const
{
  double di1 = ip2.get_i() - ip1.get_i();
  double dj1 = ip2.get_j() - ip1.get_j();

  double di2 = ip4.get_i() - ip3.get_i();
  double dj2 = ip4.get_j() - ip3.get_j();

  double denominator = di1 * dj2 - dj1 * di2;

  if(fabs(denominator) < std::numeric_limits<double>::epsilon()){
    throw vpException(vpException::divideByZeroError, "Denominator is null, lines are parallels");
  }

  double alpha = - ( ( ip1.get_i() - ip3.get_i() ) * dj2 + di2 * ( ip3.get_j() - ip1.get_j())) / denominator;
  if(alpha < 0  || alpha >= 1){
    return false;
  }

  double beta = - (di1 * (ip3.get_j() - ip1.get_j() ) + dj1 * (ip1.get_i() - ip3.get_i()) ) / denominator;
  if(beta < 0  || beta >= 1){
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
bool
vpPolygon::isInside(const vpImagePoint& ip, const PointInPolygonMethod &method) const
{
  if (_corners.size() < 3) {
    return false;
  }

  bool test = false;
  switch (method) {
    case PnPolySegmentIntersection:
      {
        vpImagePoint infPoint(100000, 100000); // take a point at 'inifinity'
        vpUniRand generator;
        infPoint.set_i( infPoint.get_i() + 1000 * generator());
        infPoint.set_j( infPoint.get_j() + 1000 * generator());// we add random since it appears that sometimes infPoint may cause a degenerated case (so realucnch and hope that result will be different).

        bool oddNbIntersections = false;
        for(unsigned int i=0; i<_corners.size(); ++i){
          vpImagePoint ip1 = _corners[i];
          vpImagePoint ip2 = _corners[(i+1)%_corners.size()];
          bool intersection = false;

          // If the points are the same we continue without trying to found
          // an intersection
          if (ip1 == ip2)
            continue;

          try{
            intersection = testIntersectionSegments(ip1, ip2, ip, infPoint );
          } catch(...){
            return isInside(ip);
          }

          if(intersection){
            oddNbIntersections = !oddNbIntersections;
          }
        }

        test = oddNbIntersections;
      }
      break;

    //Reference: http://alienryderflex.com/polygon/
    case PnPolyRayCasting:
    default:
      {
        bool oddNodes = false;
        for (size_t i = 0, j = _corners.size()-1; i < _corners.size(); i++) {
          if ((_corners[i].get_v() < ip.get_v() && _corners[j].get_v() >= ip.get_v()) || (_corners[j].get_v() < ip.get_v() && _corners[i].get_v() >= ip.get_v())) {
            oddNodes ^= (ip.get_v()*m_PnPolyMultiples[i] + m_PnPolyConstants[i] < ip.get_u());
          }

          j=i;
        }

        test = oddNodes;
      }
      break;
  }

  return test;
}

void
vpPolygon::precalcValuesPnPoly() {
  if (_corners.size() < 3) {
    return;
  }

  m_PnPolyConstants.resize(_corners.size());
  m_PnPolyMultiples.resize(_corners.size());

  for (size_t i = 0, j = _corners.size()-1; i < _corners.size(); i++) {
    if( vpMath::equal(_corners[j].get_v(), _corners[i].get_v(), std::numeric_limits<double>::epsilon())) {
      m_PnPolyConstants[i] = _corners[i].get_u();
      m_PnPolyMultiples[i] = 0.0;
    } else {
      m_PnPolyConstants[i] = _corners[i].get_u() - (_corners[i].get_v()*_corners[j].get_u()) / (_corners[j].get_v()-_corners[i].get_v()) + (_corners[i].get_v()*_corners[i].get_u())
                              / (_corners[j].get_v()-_corners[i].get_v());
      m_PnPolyMultiples[i] = (_corners[j].get_u()-_corners[i].get_u()) / (_corners[j].get_v()-_corners[i].get_v());
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
void
vpPolygon::updateArea()
{
  if(_corners.size() == 0){
    _area = 0;
    _goodPoly = false;
    return;
  }
  _area = 0;

  for(unsigned int i=0; i<_corners.size(); ++i){
    unsigned int i_p_1 = ( i+1 ) % _corners.size();
    _area += _corners[i].get_j() * _corners[i_p_1].get_i()
      - _corners[i_p_1].get_j() * _corners[i].get_i();
  }

  _area /= 2;
  if(_area < 0){
    _area = - _area;
  }
}


/*!
  Update the \c _center attribute of the polygon using the corners.

  The i coordinate is computed using:

  \f[
  i = \frac{1}{6{area}} \sum_{i=0}^{n-1} (i_i + i_{i+1})(i_{i+1} j_i - j_{i+1} i_i)
  \f]

  The computation of the j coordinate is similar.
*/
void
vpPolygon::updateCenter()
{
  if(_corners.size() == 0){
    _center = vpImagePoint(0, 0);
    _goodPoly = false;
    return;
  }
  double i_tmp = 0;
  double j_tmp = 0;
#if 0
  for(unsigned int i=0; i<(_corners.size()-1); ++i){
    i_tmp += (_corners[i].get_i() + _corners[i+1].get_i()) *
             (_corners[i+1].get_i() * _corners[i].get_j() - _corners[i+1].get_j() * _corners[i].get_i());

    j_tmp += (_corners[i].get_j() + _corners[i+1].get_j()) *
             (_corners[i+1].get_i() * _corners[i].get_j() - _corners[i+1].get_j() * _corners[i].get_i());
  }
#else
  for(unsigned int i=0; i<_corners.size(); ++i){
    unsigned int i_p_1 = ( i+1 ) % _corners.size();
    i_tmp += (_corners[i].get_i() + _corners[i_p_1].get_i()) *
             (_corners[i_p_1].get_i() * _corners[i].get_j()
        - _corners[i_p_1].get_j() * _corners[i].get_i());

    j_tmp += (_corners[i].get_j() + _corners[i_p_1].get_j()) *
             (_corners[i_p_1].get_i() * _corners[i].get_j()
        - _corners[i_p_1].get_j() * _corners[i].get_i());
  }
#endif

  if(_area > 0){
    _center.set_i(fabs(i_tmp / (6 * _area)));
    _center.set_j(fabs(j_tmp / (6 * _area)));
  }else{
    _center = _corners[0];
    _goodPoly = false;
  }
}

/*!
  Update bounding box of the polygon.

  \sa getBoundingBox()
 */
void
vpPolygon::updateBoundingBox()
{
  if(_corners.size() == 0){
    _bbox.setBottomRight(vpImagePoint(0, 0));
    _bbox.setTopLeft(vpImagePoint(0, 0));
    _goodPoly = false;
    return;
  }

  std::set<double> setI;
  std::set<double> setJ;
  for(unsigned int i=0; i<_corners.size(); ++i){
    setI.insert(_corners[i].get_i());
    setJ.insert(_corners[i].get_j());
  }
  vpImagePoint tl(*(setI.begin()), *(setJ.begin()));
  std::set<double>::const_iterator iterI = setI.end();
  std::set<double>::const_iterator iterJ = setJ.end();
  --iterI;
  --iterJ;
  vpImagePoint br(*iterI, *iterJ);

  if(tl == br){
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
void
vpPolygon::display(const vpImage<unsigned char>& I, const vpColor& color, unsigned int thickness) const
{
  const unsigned int N = (unsigned int)_corners.size();
  for(unsigned int i=0; i<N; ++i){
    vpDisplay::displayLine(I, _corners[i], _corners[(i+1)%N], color, thickness);
  }
}

/*!
  Test if an image point is inside a 2D polygon.

  \param roi : List of the polygon corners.
  \param i : i-coordinate of the image point to test.
  \param j : j-coordinate of the image point to test.
  \param method : Method to use for Point In Polygon test.

  \return True if the point defined by (i,j) is inside the polygon, False otherwise.
*/
bool
vpPolygon::isInside(const std::vector<vpImagePoint>& roi, const double &i, const double &j, const PointInPolygonMethod &method)
{
  vpPolygon poly(roi);
  return poly.isInside(vpImagePoint(i, j), method);
}

/*!
  Return number of corners belonging to the polygon.
 */
unsigned int vpPolygon::getSize() const
{
  return ((unsigned int) _corners.size());
}

