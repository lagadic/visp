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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/

#include <limits> // numeric_limits

#include <visp3/core/vpConfig.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020300
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include <visp3/tt/vpTemplateTrackerZone.h>

/*!
   Default constructor.
 */
vpTemplateTrackerZone::vpTemplateTrackerZone() : Zone(), min_x(-1), min_y(-1), max_x(-1), max_y(-1) {}

/*!
   Copy constructor.
 */
vpTemplateTrackerZone::vpTemplateTrackerZone(const vpTemplateTrackerZone &z)
  : Zone(), min_x(-1), min_y(-1), max_x(-1), max_y(-1)
{
  *this = z;
}

/*!
   Remove all the triangles that define the zone.
 */
void vpTemplateTrackerZone::clear()
{
  min_x = -1;
  min_y = -1;
  max_x = -1;
  max_y = -1;

  Zone.clear();
}

/*!
   Copy operator.
 */
vpTemplateTrackerZone &vpTemplateTrackerZone::operator=(const vpTemplateTrackerZone &z)
{
  clear();

  this->copy(z);
  return (*this);
}

/*!
  Initialize a zone in image \e I using mouse click.

  \param I : Image used to select the zone.
  \param delaunay : Flag used to enable Delaunay triangulation.
  - If true, from the image points selected by the user, a Delaunay
  triangulation is performed to initialize the zone.
    - A left click select a image point;
    - A right click select the last image point and ends the initialisation
  stage. In that case at least 3 points need to be selected by the user.
  - If false, the user select directly points as successive triangle corners.
    Three successive points define a triangle. It is not mandatory
    that triangles have one edge in common; they can define a discontinued
  area.
    - A left click select a triangle corner;
    - A right click select the last triangle corner and ends the
  initialisation stage. The number of points that are selected by the user
  should be a multiple of 3. For example, to select a zone as two triangles,
  the user has to left click five times and finish the selection on the sixth
  corner with a right click.

 */
void vpTemplateTrackerZone::initClick(const vpImage<unsigned char> &I, bool delaunay)
{
  Zone.clear();

  std::vector<vpImagePoint> vip;

  bool end = false;

  do {
    vpImagePoint p;
    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I, p, button, false)) {
      vip.push_back(p);

      vpDisplay::displayCross(I, p, 7, vpColor::red);

      if (vip.size() > 1) {
        if (delaunay) {
          // Draw a line between the 2 last points
          vpDisplay::displayLine(I, p, vip[vip.size() - 2], vpColor::blue, 3);
        } else {
          if (vip.size() % 3 == 2)
            // draw line between point 2-1
            vpDisplay::displayLine(I, p, vip[vip.size() - 2], vpColor::blue, 3);
          else if (vip.size() % 3 == 0) {
            // draw line between point 3-2
            vpDisplay::displayLine(I, p, vip[vip.size() - 2], vpColor::blue, 3);
            // draw line between point 3-1
            vpDisplay::displayLine(I, p, vip[vip.size() - 3], vpColor::blue, 3);
          }
        }
      }

      if (button == vpMouseButton::button3)
        end = true;
    }

    vpTime::wait(20);
    vpDisplay::flush(I);
  } while (!end);

  initFromPoints(I, vip, delaunay);
}

/*!

  Initialize the zone using a vector of image points.

  \param I : Image to process.
  \param vip : Vector of image points used as initialization.
  \param delaunay :
  - If true, a Delaunay triangulation is perfomed on the vector of image
  points. This functionality is only available if ViSP is build with OpenCV
  >2.3 third-party.
  - If false, the vector of image points describe triangles. Its size is then
  a multiple of 3.
 */
void vpTemplateTrackerZone::initFromPoints(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
                                           bool delaunay)
{
  if (delaunay) {
    if (vip.size() == 3) {
      initFromPoints(I, vip, false);
    } else if (vip.size() == 4) {
      std::vector<vpImagePoint> vip_delaunay;
      vip_delaunay.push_back(vip[0]);
      vip_delaunay.push_back(vip[1]);
      vip_delaunay.push_back(vip[2]);
      vip_delaunay.push_back(vip[2]);
      vip_delaunay.push_back(vip[3]);
      vip_delaunay.push_back(vip[0]);
      initFromPoints(I, vip_delaunay, false);
    } else {
#if VISP_HAVE_OPENCV_VERSION >= 0x020300
      // Init Delaunay
      cv::Subdiv2D subdiv(cv::Rect(0, 0, (int)I.getWidth(), (int)I.getHeight()));
      for (size_t i = 0; i < vip.size(); i++) {
        cv::Point2f fp((float)vip[i].get_u(), (float)vip[i].get_v());
        // std::cout << "Click point: " << vip[i] << std::endl;
        subdiv.insert(fp);
      }

      // Compute Delaunay triangulation
      std::vector<cv::Vec6f> triangleList;
      subdiv.getTriangleList(triangleList);

      // Keep only the Delaunay points that are inside the area
      vpRect rect(0, 0, I.getWidth(), I.getHeight());

      std::vector<vpImagePoint> vip_delaunay;
      for (size_t i = 0; i < triangleList.size(); i++) {
        cv::Vec6f t = triangleList[i];
        std::vector<vpImagePoint> p(3);

        p[0].set_uv(t[0], t[1]);
        p[1].set_uv(t[2], t[3]);
        p[2].set_uv(t[4], t[5]);

        if (p[0].inRectangle(rect) && p[1].inRectangle(rect) && p[2].inRectangle(rect)) {
          vip_delaunay.push_back(p[0]);
          vip_delaunay.push_back(p[1]);
          vip_delaunay.push_back(p[2]);
        }
      }

      initFromPoints(I, vip_delaunay, false);
#else
      throw vpException(vpException::functionNotImplementedError, "Delaunay triangulation is not available!");
#endif
    }
  } else {
    Zone.clear();
    for (unsigned int i = 0; i < vip.size(); i += 3) {
      vpTemplateTrackerTriangle triangle(vip[i], vip[i + 1], vip[i + 2]);
      add(triangle);

      //      vpDisplay::displayLine(I, vip[i],   vip[i+1], vpColor::green,
      //      1); vpDisplay::displayLine(I, vip[i+1], vip[i+2],
      //      vpColor::green, 1); vpDisplay::displayLine(I, vip[i+2], vip[i],
      //      vpColor::green,1); vpDisplay::flush(I) ;

      // Update the bounding box
      if ((triangle.getMinx() < min_x) || (min_x == -1))
        min_x = (int)triangle.getMinx();
      if ((triangle.getMaxx() > max_x) || (max_x == -1))
        max_x = (int)triangle.getMaxx();
      if ((triangle.getMiny() < min_y) || (min_y == -1))
        min_y = (int)triangle.getMiny();
      if ((triangle.getMaxy() > max_y) || (max_y == -1))
        max_y = (int)triangle.getMaxy();
    }
  }
}

/*!
  Add a triangle to the zone and update the bounding box.
  \param t : Triangle to introduce in the zone.
  */
void vpTemplateTrackerZone::add(const vpTemplateTrackerTriangle &t)
{
  Zone.push_back(t);

  // Update the bounding box
  if ((t.getMinx() < min_x) || (min_x == -1))
    min_x = (int)t.getMinx();
  if ((t.getMaxx() > max_x) || (max_x == -1))
    max_x = (int)t.getMaxx();
  if ((t.getMiny() < min_y) || (min_y == -1))
    min_y = (int)t.getMiny();
  if ((t.getMaxy() > max_y) || (max_y == -1))
    max_y = (int)t.getMaxy();
}

/*!
  Test if a pixel with coordinates (i,j) is in the zone..
  \param i, j : Coordinates of the pixel to test.
  \return true if the pixel with coordinates (i,j) is in the zone defined by a
  set of triangles, false otherwise.
 */
bool vpTemplateTrackerZone::inZone(const int &i, const int &j) const
{
  std::vector<vpTemplateTrackerTriangle>::const_iterator Iterateurvecteur;
  for (Iterateurvecteur = Zone.begin(); Iterateurvecteur != Zone.end(); ++Iterateurvecteur) {
    if (Iterateurvecteur->inTriangle(i, j))
      return true;
  }
  return false;
}

/*!
  Test if a pixel with coordinates (i,j) is in the zone..
  \param i, j : Coordinates of the pixel to test.
  \return true if the pixel with coordinates (i,j) is in the zone defined by a
  set of triangles, false otherwise.
 */
bool vpTemplateTrackerZone::inZone(const double &i, const double &j) const
{
  std::vector<vpTemplateTrackerTriangle>::const_iterator Iterateurvecteur;
  for (Iterateurvecteur = Zone.begin(); Iterateurvecteur != Zone.end(); ++Iterateurvecteur) {
    if (Iterateurvecteur->inTriangle(i, j))
      return true;
  }
  return false;
}

/*!
  Test if a pixel with coordinates (i,j) is in the zone and returns also the
  index of the triangle that contains the pixel. \param i, j : Coordinates of
  the pixel to test. \param id_triangle : Index of the triangle that contains
  the pixel (i,j). \return true if the pixel with coordinates (i,j) is in the
  zone defined by a set of triangles, false otherwise.
 */
bool vpTemplateTrackerZone::inZone(const int &i, const int &j, unsigned int &id_triangle) const
{
  unsigned int id = 0;
  std::vector<vpTemplateTrackerTriangle>::const_iterator Iterateurvecteur;
  for (Iterateurvecteur = Zone.begin(); Iterateurvecteur != Zone.end(); ++Iterateurvecteur) {
    if (Iterateurvecteur->inTriangle(i, j)) {
      id_triangle = id;
      return true;
    }
    id++;
  }
  return false;
}

/*!
  Test if a pixel with coordinates (i,j) is in the zone and returns also the
  index of the triangle that contains the pixel. \param i, j : Coordinates of
  the pixel to test. \param id_triangle : Index of the triangle that contains
  the pixel (i,j). \return true if the pixel with coordinates (i,j) is in the
  zone defined by a set of triangles, false otherwise.
 */
bool vpTemplateTrackerZone::inZone(const double &i, const double &j, unsigned int &id_triangle) const
{
  unsigned int id = 0;
  std::vector<vpTemplateTrackerTriangle>::const_iterator Iterateurvecteur;
  for (Iterateurvecteur = Zone.begin(); Iterateurvecteur != Zone.end(); ++Iterateurvecteur) {
    if (Iterateurvecteur->inTriangle(i, j)) {
      id_triangle = id;
      return true;
    }
    id++;
  }
  return false;
}

/*!
  A zone is defined by a set of triangles. This function returns the ith
  triangle. \param i : Index of the triangle to return. \param T : The
  triangle corresponding to index i. \return true if the triangle with index i
  was found, false otherwise.

  The following sample code shows how to use this function:
  \code
    vpTemplateTrackerZone zone;
    ...
    for (unsigned int i=0; i < zone.getNbTriangle(); i++) {
      vpTemplateTrackerTriangle triangle;
      zone.getTriangle(i, triangle);
    }
  \endcode
 */
void vpTemplateTrackerZone::getTriangle(unsigned int i, vpTemplateTrackerTriangle &T) const
{
  if (i > getNbTriangle() - 1)
    throw(vpException(vpException::badValue, "Cannot get triangle with index %u", i));

  T = Zone[i];
}
/*!
  A zone is defined by a set of triangles. This function returns the ith
  triangle. \param i : Index of the triangle to return. \return The triangle
  corresponding to index i.

  The following sample code shows how to use this function:
  \code
    vpTemplateTrackerZone zone;
    ...
    for (unsigned int i=0; i < zone.getNbTriangle(); i++) {
      vpTemplateTrackerTriangle triangle = zone.getTriangle(i);
    }
  \endcode
 */
vpTemplateTrackerTriangle vpTemplateTrackerZone::getTriangle(unsigned int i) const
{
  if (i > getNbTriangle() - 1)
    throw(vpException(vpException::badValue, "Cannot get triangle with index %u", i));

  return Zone[i];
}
/*!
  Return the position of the center of gravity of the zone.
  \exception vpException::divideByZeroError: The size of the zone is null.
 */
vpImagePoint vpTemplateTrackerZone::getCenter() const
{
  double xc = 0;
  double yc = 0;
  int cpt = 0;
  for (int i = min_y; i < max_y; i++)
    for (int j = min_x; j < max_x; j++)
      if (inZone(i, j)) {
        xc += j;
        yc += i;
        cpt++;
      }
  if (!cpt) {
    throw(vpException(vpException::divideByZeroError, "Cannot compute the zone center: size = 0"));
  }
  xc = xc / cpt;
  yc = yc / cpt;
  vpImagePoint ip;
  ip.set_uv(xc, yc);
  return ip;
}

/*!
  \return The maximal x coordinate (along the columns of the image) of the
  points that are in the zone. \sa getMinx(), getBoundingBox()
 */
int vpTemplateTrackerZone::getMaxx() const { return max_x; }
/*!
  \return The maximal y coordinate (along the rows of the image) of the points
  that are in the zone. \sa getMiny(), getBoundingBox()
 */
int vpTemplateTrackerZone::getMaxy() const { return max_y; }
/*!
  \return The minimal x coordinate (along the columns of the image) of the
  points that are in the zone. \sa getMaxx(), getBoundingBox()
 */
int vpTemplateTrackerZone::getMinx() const { return min_x; }
/*!
  \return The minimal y coordinate (along the rows of the image) of the points
  that are in the zone. \sa getMaxy(), getBoundingBox()
 */
int vpTemplateTrackerZone::getMiny() const { return min_y; }

/*!
  Return a rectangle that defines the bounding box of the zone.
  \sa getMinx(), getMiny(), getMaxx(), getMaxy()
 */
vpRect vpTemplateTrackerZone::getBoundingBox() const
{
  vpRect bbox;
  bbox.setTopLeft(vpImagePoint(min_y, min_x));
  bbox.setBottomRight(vpImagePoint(max_y, max_x));
  return bbox;
}

/*!
  If a display device is associated to image \c I, display in overlay the
  triangles that define the zone. \param I : Image. \param col : Color used to
  display the triangles. \param thickness : Thickness of the triangle lines.
 */
void vpTemplateTrackerZone::display(const vpImage<unsigned char> &I, const vpColor &col, const unsigned int thickness)
{
  std::vector<vpImagePoint> ip;
  for (unsigned int i = 0; i < Zone.size(); i++) {
    vpTemplateTrackerTriangle triangle;
    Zone[i].getCorners(ip);
    vpDisplay::displayLine(I, ip[0], ip[1], col, thickness);
    vpDisplay::displayLine(I, ip[1], ip[2], col, thickness);
    vpDisplay::displayLine(I, ip[2], ip[0], col, thickness);
  }
}

/*!
  If a display device is associated to image \c I, display in overlay the
  triangles that define the zone. \param I : Image. \param col : Color used to
  display the triangles. \param thickness : Thickness of the triangle lines.
 */
void vpTemplateTrackerZone::display(const vpImage<vpRGBa> &I, const vpColor &col, const unsigned int thickness)
{
  std::vector<vpImagePoint> ip;
  for (unsigned int i = 0; i < Zone.size(); i++) {
    vpTemplateTrackerTriangle triangle;
    Zone[i].getCorners(ip);
    vpDisplay::displayLine(I, ip[0], ip[1], col, thickness);
    vpDisplay::displayLine(I, ip[1], ip[2], col, thickness);
    vpDisplay::displayLine(I, ip[2], ip[0], col, thickness);
  }
}

/*!
  Destructor.
 */
vpTemplateTrackerZone::~vpTemplateTrackerZone() { clear(); }

/*!
   Modify all the pixels inside a triangle with a given gray level.
   \param I: Output image.
   \param id: Triangle id. This value should be less than the number
   of triangles used to define the zone and available using getNbTriangle().
   \param gray_level: Color used to fill the triangle with.
 */
void vpTemplateTrackerZone::fillTriangle(vpImage<unsigned char> &I, unsigned int id, unsigned char gray_level)
{
  assert(id < getNbTriangle());
  vpTemplateTrackerTriangle triangle;
  getTriangle(id, triangle);
  for (int i = 0; i < (int)I.getHeight(); i++) {
    for (int j = 0; j < (int)I.getWidth(); j++) {
      if (triangle.inTriangle(i, j)) {
        I[i][j] = gray_level;
      }
    }
  }
}

/*!
  Return a zone with triangles that are down scaled by a factor 2.
  */
vpTemplateTrackerZone vpTemplateTrackerZone::getPyramidDown() const
{
  vpTemplateTrackerZone tempZone;
  vpTemplateTrackerTriangle Ttemp;
  vpTemplateTrackerTriangle TtempDown;
  for (unsigned int i = 0; i < getNbTriangle(); i++) {
    getTriangle(i, Ttemp);
    TtempDown = Ttemp.getPyramidDown();
    tempZone.add(TtempDown);
  }
  return tempZone;
}

/*!
  Copy all the triangles that define zone \c Z in the current zone (*this) and
  update the zone bounding box.
  \param z : Zone with a set of triangles provided as input.
 */
void vpTemplateTrackerZone::copy(const vpTemplateTrackerZone &z)
{
  vpTemplateTrackerTriangle triangle;
  for (unsigned int i = 0; i < z.getNbTriangle(); i++) {
    z.getTriangle(i, triangle);
    add(triangle);
    // Update the bounding box
    if ((triangle.getMinx() < min_x) || (min_x == -1))
      min_x = (int)triangle.getMinx();
    if ((triangle.getMaxx() > max_x) || (max_x == -1))
      max_x = (int)triangle.getMaxx();
    if ((triangle.getMiny() < min_y) || (min_y == -1))
      min_y = (int)triangle.getMiny();
    if ((triangle.getMaxy() > max_y) || (max_y == -1))
      max_y = (int)triangle.getMaxy();
  }
}

/*!
  Return the position of the center of gravity in a given area.
  \param borne_x : Right coordinate of the area to consider.
  \param borne_y : Bottom coordinate of the area to consider.
  \exception vpException::divideByZeroError: The size of the zone is null.
 */

vpImagePoint vpTemplateTrackerZone::getCenter(int borne_x, int borne_y) const
{
  int cpt_pt = 0;
  double x_center = 0, y_center = 0;
  for (int j = 0; j < borne_x; j++)
    for (int i = 0; i < borne_y; i++)
      if (inZone(i, j)) {
        x_center += j;
        y_center += i;
        cpt_pt++;
      }

  if (!cpt_pt) {
    throw(vpException(vpException::divideByZeroError, "Cannot compute the zone center: size = 0"));
  }

  x_center = x_center / cpt_pt;
  y_center = y_center / cpt_pt;
  vpImagePoint center;
  center.set_uv(x_center, y_center);
  return center;
}

/*!
 Return the area of the template zone.
 */
double vpTemplateTrackerZone::getArea() const
{
  double area = 0;
  vpTemplateTrackerTriangle triangle;
  for (unsigned int i = 0; i < getNbTriangle(); i++) {
    getTriangle(i, triangle);
    area += triangle.getArea();
  }
  return area;
}
