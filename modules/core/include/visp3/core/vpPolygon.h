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

#ifndef vpPolygon_h
#define vpPolygon_h

#include <vector>
#include <list>

#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRect.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpCameraParameters.h>

/*!
  \class vpPolygon
  \ingroup group_core_geometry
  \brief Defines a generic 2D polygon.

  A polygon is internally represented by N 2D points.

  By default three coordinates in the \f$ (i,j) \f$ frame (see vpImagePoint
  class documentation for more details about the frame) are used \f$ (0,0) \f$,
 \f$ (1,0) \f$ and \f$ (0,1) \f$.

  The code bellow shows how to manipulate a polygon.
\code
#include <iostream>

#include <visp3/core/vpPolygon.h>

int main()
{
  std::vector<vpImagePoint> corners;

  // Initialize the corners vector with 4 points
  corners.push_back( vpImagePoint( 50, 100) );
  corners.push_back( vpImagePoint( 50, 300) );
  corners.push_back( vpImagePoint(200, 300) );
  corners.push_back( vpImagePoint(200, 100) );

  // Initialize a polygon with the corners
  vpPolygon polygon(corners);

  // Get the polygon bounding box
  vpRect bbox = polygon.getBoundingBox();
  std::cout << "Bounding box: " << bbox.getTopLeft() << " to " << bbox.getBottomRight() << std::endl;

  // Get the polygon surface and center
  std::cout << "Area: " << polygon.getArea() << std::endl;
  std::cout << "Center: " << polygon.getCenter() << std::endl;

  // Check if a point is inside the polygon
  vpImagePoint ip(550, 200);
  std::cout << "The point " << ip << " is " << (polygon.isInside(ip) ? "inside":"outside") << " the polygon" << std::endl;

  return 0;
}
\endcode
*/
class VISP_EXPORT vpPolygon
{
  protected:
    //! Collection of image points containing the corners.
    std::vector<vpImagePoint> _corners;
    //! Center of the polygon. It is automatically computed when the corners are set.
    vpImagePoint _center;
    //! Area of the polygon.
    double _area;
    //! Flag set to true when the polygon is a good polygon (ie. it has more than two corners) or false otherwise.
    bool _goodPoly;
    //! Bounding box containing the polygon.
    vpRect _bbox;

  public:
    enum PointInPolygonMethod {
      PnPolySegmentIntersection,  /*!< Legacy Point In Polygon test. */
      PnPolyRayCasting            /*!< Point In Polygon test using ray casting method (faster). */
    };

    vpPolygon();
    explicit vpPolygon(const std::vector<vpImagePoint>& corners);
    explicit vpPolygon(const std::list<vpImagePoint>& corners);
    vpPolygon(const vpPolygon &poly);
    virtual ~vpPolygon();

    vpPolygon &operator=(const vpPolygon& poly);

    void buildFrom(const std::vector<vpImagePoint>& corners);
    void buildFrom(const std::list<vpImagePoint>& corners);
    void buildFrom(const std::vector<vpPoint>& corners, const vpCameraParameters& cam);

    unsigned int getSize() const;
    void initClick(const vpImage<unsigned char>& I, unsigned int size=5, const vpColor &color=vpColor::red, unsigned int thickness=1);
    void initClick(const vpImage<vpRGBa>& I, unsigned int size=5, const vpColor &color=vpColor::red, unsigned int thickness=1);

    bool isInside(const vpImagePoint &iP, const PointInPolygonMethod &method=PnPolyRayCasting) const;

    void display(const vpImage<unsigned char>& I, const vpColor& color, unsigned int thickness=1) const;

    /*!
      Get the corners of the polygon.

      \return A reference to the corners.
    */
    const std::vector<vpImagePoint>& getCorners() const {
      return _corners;
    }


    /*!
      Return the area of the polygon.
      The area is computed when the polygon is built from the corners.

      \return The area of the polygon.
    */
    inline double getArea() const{
      return this->_area;
    }

    /*!
      Return the center of the polygon.
      The center is computed when the polygon is built from the corners.

      \return The area of the polygon.
    */
    inline vpImagePoint getCenter() const{
      return this->_center;
    }

    /*!
      Return the bounding box. The bounding box is the smallest rectangle
      containing all the polygon.

      \return The bounding box of the polygon.
    */
    inline vpRect getBoundingBox() const {
      return _bbox;
    }

  protected:
    void init(const std::vector<vpImagePoint>& corners);
    void init(const std::list<vpImagePoint>& corners);
    void updateArea();
    void updateCenter();
    void updateBoundingBox();

  private:
    bool testIntersectionSegments(const vpImagePoint& ip1, const vpImagePoint& ip2, const vpImagePoint& ip3, const vpImagePoint& ip4) const;
    void precalcValuesPnPoly();

    std::vector<double> m_PnPolyConstants;
    std::vector<double> m_PnPolyMultiples;

  //###################
  // Static Functions
  //###################

  public:
    static bool isInside(const std::vector<vpImagePoint>& roi, const double &i, const double  &j, const PointInPolygonMethod &method=PnPolyRayCasting);
};

#endif

