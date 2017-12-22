/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpTemplateTrackerZone_hh
#define vpTemplateTrackerZone_hh

#include <vector>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRect.h>
#include <visp3/tt/vpTemplateTrackerHeader.h>
#include <visp3/tt/vpTemplateTrackerTriangle.h>

/*!
  \class vpTemplateTrackerZone
  \ingroup group_tt_tools
  A zone is defined by a set of triangles defined as
  vpTemplateTrackerTriangle.

  A zone can be initialized either by user interaction using mouse click in a
  display device throw initClick(), or by a list of points throw
  initFromPoints().
 */
class VISP_EXPORT vpTemplateTrackerZone
{
protected:
  std::vector<vpTemplateTrackerTriangle> Zone; //!< Vector of triangles that defines the zone.
  int min_x;                                   //!< Bounding box parameter
  int min_y;                                   //!< Bounding box parameter
  int max_x;                                   //!< Bounding box parameter
  int max_y;                                   //!< Bounding box parameter

public:
  vpTemplateTrackerZone();
  vpTemplateTrackerZone(const vpTemplateTrackerZone &z);
  ~vpTemplateTrackerZone();

  // add a triangle to the zone
  void add(const vpTemplateTrackerTriangle &t);
  void clear();
  void copy(const vpTemplateTrackerZone &z);

  // display the area on an image
  void display(const vpImage<unsigned char> &I, const vpColor &col = vpColor::green, const unsigned int thickness = 3);
  void display(const vpImage<vpRGBa> &I, const vpColor &col = vpColor::green, const unsigned int thickness = 3);

  // colorie le tieme triangle
  void fillTriangle(vpImage<unsigned char> &I, unsigned int id, unsigned char gray_level);

  double getArea() const;
  vpImagePoint getCenter() const;
  vpImagePoint getCenter(int borne_x, int borne_y) const;
  // get bounds of the area
  int getMaxx() const;
  int getMaxy() const;
  int getMinx() const;
  int getMiny() const;
  vpRect getBoundingBox() const;

  /*! Return the number of triangles that define the zone. \sa getTriangle()
   */
  unsigned int getNbTriangle() const { return (unsigned int)Zone.size(); }
  vpTemplateTrackerZone getPyramidDown() const;
  // renvoie le ieme triangle de la zone
  void getTriangle(unsigned int i, vpTemplateTrackerTriangle &T) const;
  vpTemplateTrackerTriangle getTriangle(unsigned int i) const;

  // create an area by clicking on an image
  void initClick(const vpImage<unsigned char> &I, bool delaunay = false);
  // create an area with a pointer of integer that describes a series of
  // triangles:
  // *pt= t0.S1.x,t0.S1.y,t0.S2.x,t0.S2.y,t0.S3.x,t0.S3.y, t1.S1.x ...
  void initFromPoints(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &ip, bool delaunay = false);

  // check if a point is in the area
  bool inZone(const int &i, const int &j) const;
  bool inZone(const double &i, const double &j) const;
  // check if a point is in the area and return the corresponding triangle
  // id_triangle where the point is.
  bool inZone(const int &i, const int &j, unsigned int &id_triangle) const;
  bool inZone(const double &i, const double &j, unsigned int &id_triangle) const;

  vpTemplateTrackerZone &operator=(const vpTemplateTrackerZone &z);
};
#endif
