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
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 * Aurelien Yol
 *
 *****************************************************************************/

#include <limits.h>

#include <visp3/core/vpConfig.h>
/*!
 \file vpMbtPolygon.cpp
 \brief Implements a polygon of the model used by the model-based tracker.
*/

#include <visp3/core/vpPolygon.h>
#include <visp3/mbt/vpMbtPolygon.h>

/*!
  Basic constructor.
*/
vpMbtPolygon::vpMbtPolygon()
  : index(-1), isvisible(false), isappearing(false), useLod(false), minLineLengthThresh(50.0),
    minPolygonAreaThresh(2500.0), name(""), hasOrientation(true)
{
}

vpMbtPolygon::vpMbtPolygon(const vpMbtPolygon &mbtp)
  : vpPolygon3D(mbtp), index(mbtp.index), isvisible(mbtp.isvisible), isappearing(mbtp.isappearing), useLod(mbtp.useLod),
    minLineLengthThresh(mbtp.minLineLengthThresh), minPolygonAreaThresh(mbtp.minPolygonAreaThresh), name(mbtp.name),
    hasOrientation(mbtp.hasOrientation)
{
  //*this = mbtp; // Should not be called by copy contructor to avoid multiple
  // assignements.
}

vpMbtPolygon &vpMbtPolygon::operator=(const vpMbtPolygon &mbtp)
{
  vpPolygon3D::operator=(mbtp);
  index = mbtp.index;
  isvisible = mbtp.isvisible;
  isappearing = mbtp.isappearing;
  useLod = mbtp.useLod;
  minLineLengthThresh = mbtp.minLineLengthThresh;
  minPolygonAreaThresh = mbtp.minPolygonAreaThresh;
  name = mbtp.name;
  hasOrientation = mbtp.hasOrientation;

  return (*this);
}

/*!
  Basic destructor.
*/
vpMbtPolygon::~vpMbtPolygon() {}

/*!
  Check if the polygon is visible in the image and if the angle between the
  normal to the face and the line vector going from the optical center to the
  cog of the face is below the given threshold. To do that, the polygon is
  projected into the image thanks to the camera pose.

  \param cMo : The pose of the camera.
  \param alpha : Maximum angle to detect if the face is visible (in rad).
  \param modulo : Indicates if the test should also consider faces that are
  not oriented counter clockwise. If true, the orientation of the face is
  without importance.
  \param cam : Camera parameters (intrinsics parameters)
  \param width, height : Image size used to consider level of detail.

  \return Return true if the polygon is visible.
*/
bool vpMbtPolygon::isVisible(const vpHomogeneousMatrix &cMo, const double alpha, const bool &modulo,
                             const vpCameraParameters &cam, unsigned int width, unsigned int height)
{
  //   std::cout << "Computing angle from MBT Face (cMo, alpha)" << std::endl;

  changeFrame(cMo);

  if (nbpt <= 2) {
    // Level of detail (LOD)
    if (useLod) {
      vpCameraParameters c = cam;
      if (clippingFlag > 3) { // Contains at least one FOV constraint
        c.computeFov(width, height);
      }
      computePolygonClipped(c);
      std::vector<vpImagePoint> roiImagePoints;
      getRoiClipped(c, roiImagePoints);

      if (roiImagePoints.size() == 2) {
        double x1 = roiImagePoints[0].get_u();
        double y1 = roiImagePoints[0].get_v();
        double x2 = roiImagePoints[1].get_u();
        double y2 = roiImagePoints[1].get_v();
        double length = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        //          std::cout << "Index=" << index << " ; Line length=" <<
        //          length << " ; clippingFlag=" << clippingFlag << std::endl;
        //        vpTRACE("index=%d lenght=%f minLineLengthThresh=%f", index,
        //        length, minLineLengthThresh);

        if (length < minLineLengthThresh) {
          isvisible = false;
          isappearing = false;
          return false;
        }
      }
    }

    /* a line is always visible when LOD is not used */
    isvisible = true;
    isappearing = false;
    return true;
  }

  // If the polygon has no orientation, the angle check visibility is always
  // valid. Feature mainly used for cylinders.
  if (!hasOrientation) {
    isvisible = true;
    isappearing = false;
    return true;
  }

  // Check visibility from normal
  // Newell's Method for calculating the normal of an arbitrary 3D polygon
  // https://www.opengl.org/wiki/Calculating_a_Surface_Normal
  vpColVector faceNormal(3);
  vpColVector currentVertex, nextVertex;
  for (unsigned int i = 0; i < nbpt; i++) {
    currentVertex = p[i].cP;
    nextVertex = p[(i + 1) % nbpt].cP;

    faceNormal[0] += (currentVertex[1] - nextVertex[1]) * (currentVertex[2] + nextVertex[2]);
    faceNormal[1] += (currentVertex[2] - nextVertex[2]) * (currentVertex[0] + nextVertex[0]);
    faceNormal[2] += (currentVertex[0] - nextVertex[0]) * (currentVertex[1] + nextVertex[1]);
  }
  faceNormal.normalize();

  vpColVector e4(3);
  vpPoint pt;
  for (unsigned int i = 0; i < nbpt; i += 1) {
    pt.set_X(pt.get_X() + p[i].get_X());
    pt.set_Y(pt.get_Y() + p[i].get_Y());
    pt.set_Z(pt.get_Z() + p[i].get_Z());
  }
  e4[0] = -pt.get_X() / (double)nbpt;
  e4[1] = -pt.get_Y() / (double)nbpt;
  e4[2] = -pt.get_Z() / (double)nbpt;
  e4.normalize();

  double angle = acos(vpColVector::dotProd(e4, faceNormal));

  //  vpCTRACE << angle << "/" << vpMath::deg(angle) << "/" <<
  //  vpMath::deg(alpha) << std::endl;

  if (angle < alpha || (modulo && (M_PI - angle) < alpha)) {
    isvisible = true;
    isappearing = false;

    if (useLod) {
      vpCameraParameters c = cam;
      if (clippingFlag > 3) { // Contains at least one FOV constraint
        c.computeFov(width, height);
      }
      computePolygonClipped(c);
      std::vector<vpImagePoint> roiImagePoints;
      getRoiClipped(c, roiImagePoints);

      vpPolygon roiPolygon(roiImagePoints);
      double area = roiPolygon.getArea();
      //      std::cout << "After normal test ; Index=" << index << " ; area="
      //      << area << " ; clippingFlag="
      //          << clippingFlag << std::endl;
      if (area < minPolygonAreaThresh) {
        isappearing = false;
        isvisible = false;
        return false;
      }
    }

    return true;
  }

  if (angle < alpha + vpMath::rad(1)) {
    isappearing = true;
  } else if (modulo && (M_PI - angle) < alpha + vpMath::rad(1)) {
    isappearing = true;
  } else {
    isappearing = false;
  }

  isvisible = false;
  return false;
}

//###################################
//      Static functions
//###################################

/*!
  Set the flag to consider if the level of detail (LOD) is used or not.
  When activated, lines and faces of the 3D model are tracked if respectively
their projected lenght and area in the image are significative enough. By
significative, we mean:
  - if the lenght of the projected line in the image is greater that a
threshold set by setMinLineLengthThresh()
  - if the area of the projected face in the image is greater that a threshold
set by setMinPolygonAreaThresh().

  \param use_lod : true if level of detail must be used, false otherwise.

  The sample code below shows how to introduce this feature:
  \code
#include <visp/vpImageIo.h>
#include <visp/vpMbEdgeTracker.h>

int main()
{
  pImage<unsigned char> I;

  // Acquire an image
  vpImageIo::read(I, "my-image.pgm");

  std::string object = "my-object";
  vpMbEdgeTracker tracker;
  tracker.loadConfigFile( object+".xml" );
  tracker.loadModel( object+".cao" );

  tracker.setLod(true);
  tracker.setMinLineLengthThresh(20.);
  tracker.setMinPolygonAreaThresh(20.*20.);

  tracker.initClick(I, object+".init" );

  while (true) {
    // tracking loop
  }
  vpXmlParser::cleanup();

  return 0;
}
  \endcode

  \sa setMinLineLengthThresh(), setMinPolygonAreaThresh()
 */
void vpMbtPolygon::setLod(const bool use_lod) { this->useLod = use_lod; }
