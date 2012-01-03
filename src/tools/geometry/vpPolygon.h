/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

/*!
  \class vpPolygon
  \brief Defines a generic 2D polygon.
  
  A polygon is internally represented by N 2D points.
  
  By default three coordinates in the \f$ (i,j) \f$ frame (see vpImagePoint
  class documentation for more details about the frame.) are used \f$ (0,0) \f$,
 \f$ (1,0) \f$ and \f$ (0,1) \f$.
*/

#include <visp/vpImagePoint.h>
#include <visp/vpRect.h>
#include <visp/vpColor.h>
#include <visp/vpImage.h>
#include <visp/vpPoint.h>
#include <visp/vpCameraParameters.h>

#include <vector>

class VISP_EXPORT vpPolygon
{
  protected:
    //! Collection of image points containing the corners.
    std::vector<vpImagePoint> _corners;
    //! Center of the polygon. It is automatically computed when the corners are set.
    vpImagePoint _center;
    //! Area of the polygon.
    double _area;
    //! Flag to indicate whether the polygon is a good polygon (ie. it has more than two corners, ...)
    bool _goodPoly;
    //! Boumding box containing the polygon.
    vpRect _bbox;
    
  public:
    vpPolygon();
    vpPolygon(const std::vector<vpImagePoint>& corners);
    vpPolygon(const vpPolygon &poly);
    virtual ~vpPolygon();
    
    vpPolygon &operator=(const vpPolygon& poly);
    
    void buildFrom(const std::vector<vpImagePoint>& corners);
    void buildFrom(const std::vector<vpPoint>& corners, const vpCameraParameters& cam);
    void initClick(const vpImage<unsigned char>& I);
    
    bool isInside(const vpImagePoint &iP);

    void display(const vpImage<unsigned char>& I, const vpColor& color, unsigned int thickness=1);
    
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
    void updateArea();
    void updateCenter();
    void updateBoundingBox();
    
  private:
    bool testIntersectionSegments(const vpImagePoint& ip1, const vpImagePoint& ip2, const vpImagePoint& ip3, const vpImagePoint& ip4);
    
};

#endif

