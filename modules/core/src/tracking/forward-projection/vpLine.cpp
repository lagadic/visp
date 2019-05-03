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
 * Line feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/core/vpLine.h>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMath.h>

#include <visp3/core/vpFeatureDisplay.h>

/*!
  \file vpLine.cpp
  \brief   class that defines what is a line
*/

/*!

  Initialize the memory space requested for the 2D line parameters (\e
  p) in the image plane and for 3D line parameters (\e oP and \e cP)
  respectively in the object frame and the camera frame.
*/
void vpLine::init()
{
  oP.resize(8);
  cP.resize(8);
  p.resize(2);
}

/*!
  Default constructor.
*/
vpLine::vpLine() { init(); }

/*!

  Sets the parameters \e oP which define the line in the object
  frame. As said in the class description, the line is defined as the
  intersection of two planes. The different parameters here define the
  equations of the two planes in the object frame. They are used to set
  the vpForwardProjection::oP public attribute.

  \f[ A1 X + B1 Y + C1 Z +D1 = 0 \f]
  \f[ A2 X + B2 Y + C2 Z +D2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates in the object frame.

  \param A1, B1, C1, D1 : The parameters used to define the first plane.
  \param A2, B2, C2, D2 : The parameters used to define the second plane.

*/
void vpLine::setWorldCoordinates(const double &A1, const double &B1, const double &C1, const double &D1,
                                 const double &A2, const double &B2, const double &C2, const double &D2)
{
  oP[0] = A1;
  oP[1] = B1;
  oP[2] = C1;
  oP[3] = D1;

  oP[4] = A2;
  oP[5] = B2;
  oP[6] = C2;
  oP[7] = D2;
}

/*!

  Sets the parameters \e oP which define the line in the object frame. As
  said in the class description, the line is defined as the
  intersection of two planes. Eight parameters are required to define
  the equations of the two planes in the object frame. They are used to set
  the vpForwardProjection::oP public attribute.

  \f[ A1 X + B1 Y + C1 Z +D1 = 0 \f]
  \f[ A2 X + B2 Y + C2 Z +D2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates in the object frame.

  \param oP_ : The column vector which contains the eight parameters
  needed to define the equations of the two planes in the object
  frame. \f[ oP = \left[\begin{array}{c}A1 \\ B1 \\ C1 \\ D1 \\ A2 \\
  B2 \\ C2 \\ D2 \end{array}\right] \f]

*/
void vpLine::setWorldCoordinates(const vpColVector &oP_)
{
  if (oP_.getRows() != 8)
    throw vpException(vpException::dimensionError, "Size of oP is not equal to 8 as it should be");

  this->oP = oP_;
}

/*!

  Sets the parameters \e oP which define the line in the object frame. As
  said in the class description, the line is defined as the
  intersection of two planes. Eight parameters are required to define
  the equations of the two planes in the object frame. They are used to set
  the vpForwardProjection::oP public attribute.

  \f[ A1 X + B1 Y + C1 Z +D1 = 0 \f]
  \f[ A2 X + B2 Y + C2 Z +D2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates in the object frame.

  \param oP1 : The column vector which contains the four parameters
  needed to define the equations of the first plane. \f[ oP1 =
  \left[\begin{array}{c}A1 \\ B1 \\ C1 \\ D1 \end{array}\right] \f]

  \param oP2 : The column vector which contains the four parameters
  needed to define the equations of the second plane. \f[ oP2 =
  \left[\begin{array}{c} A2 \\ B2 \\ C2 \\ D2 \end{array}\right] \f]

*/
void vpLine::setWorldCoordinates(const vpColVector &oP1, const vpColVector &oP2)
{
  if (oP1.getRows() != 4)
    throw vpException(vpException::dimensionError, "Size of oP1 is not equal to 4 as it should be");

  if (oP2.getRows() != 4)
    throw vpException(vpException::dimensionError, "Size of oP2 is not equal to 4 as it should be");

  for (unsigned int i = 0; i < 4; i++) {
    oP[i] = oP1[i];
    oP[i + 4] = oP2[i];
  }
}

/*!

  Computes the 2D parameters \e p of the line in the image plane thanks to
  the 3D line parameters \e cP  in the camera frame located in the
  vpTracker::cP public attribute. The  parameters \f$p=(\rho,
  \theta)\f$ are updated in the vpTracker::p public attribute.

  \warning To compute these parameters \e p, the method exploit the feature
  parameters \e cP in the camera frame. Thus, vpTracker::cP need to be
  updated before the call of this method. For that, a call to
  changeFrame(const vpHomogeneousMatrix &) is requested.

  The code below shows how to use this method.
  \code
  //Create the line
  vpLine line;

  //Set the coordinates of the line in the object frame in meter.
  line.setWorldCoordinates( 1, 0, 0, -0.5, 0, 0, 1, 0.5)
  //Here the line is define by the intersection between the plane X = 0.5m and
  Z = 0.5m

  //Create the homogeneous matrix
  vpHomogeneousMatrix cMo;
  //Computes or set here the homogeneous matrix

  //Computes the equations of the two planes in the camera frame
  line.changeFrame(cMo);

  //Computes the line features in the camera frame( rho and theta)
  line.projection();
  \endcode
*/
void vpLine::projection()
{
  projection(cP, p);
}

/*!

  Computes the 2D parameters \e p of the line in the image plane thanks
  to the 3D line features \e cP expressed in the camera frame. The
  image plane parameters \f$p=(\rho , \theta)\f$ are updated in
  output.

  \param cP_ : The vector containing the line features relative to the
  camera frame. \f[ cP = \left[\begin{array}{c}A1 \\ B1 \\ C1 \\ D1
  \\ A2 \\ B2 \\ C2 \\ D2 \end{array}\right] \f]

  \param p_ : The vector which contains the 2D line features expressed
  in the image plane. \f[ p = \left[\begin{array}{c} \rho \\ \theta
  \end{array}\right] \f]

  \exception vpException::fatalError : Degenerate case, the image of the
  straight line is a point.
*/
void vpLine::projection(const vpColVector &cP_, vpColVector &p_)
{
  // projection

  if (cP.getRows() != 8)
    throw vpException(vpException::dimensionError, "Size of cP is not equal to 8 as it should be");

  double A1, A2, B1, B2, C1, C2, D1, D2;

  A1 = cP_[0];
  B1 = cP_[1];
  C1 = cP_[2];
  D1 = cP_[3];

  A2 = cP_[4];
  B2 = cP_[5];
  C2 = cP_[6];
  D2 = cP_[7];

  double a, b, c, s;
  a = A2 * D1 - A1 * D2;
  b = B2 * D1 - B1 * D2;
  c = C2 * D1 - C1 * D2;
  s = a * a + b * b;
  if (s <= 1e-8) // seuil pas terrible
  {
    printf("Degenerate case: the image of the straight line is a point!\n");
    throw vpException(vpException::fatalError, "Degenerate case: the image of the straight line is a point!");
  }
  s = 1.0 / sqrt(s);

  double rho = -c * s;
  double theta = atan2(b, a);

  if (p.getRows() != 2)
    p.resize(2);

  p_[0] = rho;
  p_[1] = theta;
}

/*!

  Computes the line parameters \e cP in the camera frame thanks to the
  line parameters  \e oP given in the object frame  and the homogeneous matrix
  relative to the pose \e cMo  between the object frame and the camera
  frame. Thus the computation gives the equations of the two planes
  needed to define the line in the camera frame thanks to the
  equations of the same two planes in the object frame.

  In input of this method, the line parameters \e oP in the object
  frame are those from the vpForwardProjection::oP public attribute.

  As a result of this method, line parameters \e cP in the camera
  frame are updated in the vpTracker::cP public attribute.

  \param cMo : The homogeneous matrix corresponding to the pose
  between the camera frame and the object frame.

  The code below shows how to use this method.
  \code
  //Create the line
  vpLine line;

  //Set the coordinates of the line in the object frame in meter.
  line.setWorldCoordinates( 1, 0, 0, -0.5, 0, 0, 1, 0.5)
  //The line is define by the intersection between the plane X = 0.5m and Z =
  0.5m

  //Create the homogeneous matrix
  vpHomogeneousMatrix cMo;
  //Computes or set here the homogeneous matrix

  //Computes the equations of the two planes in the camera frame
  line.changeFrame(cMo);
  \endcode
*/
void vpLine::changeFrame(const vpHomogeneousMatrix &cMo) { changeFrame(cMo, cP); }

/*!

  Computes the line parameters \e cP in the camera frame thanks to the
  line parameters \e oP given in the object frame and the homogeneous
  matrix relative to the pose between the camera frame  and the
  object frame. Thus the computation gives the equations of the two
  planes needed to define the line in the desired frame thanks to the
  equations of the same two planes in the object frame.

  In input of this method, the line parameters \e oP in the object
  frame are those from the vpForwardProjection::oP public attribute.

  \param cMo : The homogeneous matrix relative to the pose
  between the desired frame and the object frame.

  \param cP_ : The vector which will contain the parameters of the two
  planes in the camera frame. \f[ cP =
  \left[\begin{array}{c}A1 \\ B1 \\ C1 \\ D1 \\ A2 \\ B2 \\ C2 \\ D2
  \end{array}\right] \f]

  The code below shows how to use this method.
  \code
  //Create the line
  vpLine line;

  //Set the coordinates of the line in the object frame in meter.
  line.setWorldCoordinates( 1, 0, 0, -0.5, 0, 0, 1, 0.5)
  //The line is define by the intersection between the plane X = 0.5m and Z = 0.5m

  //Create the homogeneous matrix
  vpHomogeneousMatrix cMo;
  //Computes or set here the homogeneous matrix

  //Creates the vector which will contain the line features
  vpColVector cP(8);

  //Computes the equations of the two planes in the camera frame
  line.changeFrame(cMo, cP);
  \endcode
*/

void vpLine::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP_)
{

  double a1, a2, b1, b2, c1, c2, d1, d2;
  double A1, A2, B1, B2, C1, C2, D1, D2;

  // in case of verification
  // double x,y,z,ap1,ap2,bp1,bp2,cp1,cp2,dp1,dp2;

  if (cP.getRows() != 8)
    cP.resize(8);

  a1 = oP[0];
  b1 = oP[1];
  c1 = oP[2];
  d1 = oP[3];

  a2 = oP[4];
  b2 = oP[5];
  c2 = oP[6];
  d2 = oP[7];

  A1 = cMo[0][0] * a1 + cMo[0][1] * b1 + cMo[0][2] * c1;
  B1 = cMo[1][0] * a1 + cMo[1][1] * b1 + cMo[1][2] * c1;
  C1 = cMo[2][0] * a1 + cMo[2][1] * b1 + cMo[2][2] * c1;
  D1 = d1 - (cMo[0][3] * A1 + cMo[1][3] * B1 + cMo[2][3] * C1);

  A2 = cMo[0][0] * a2 + cMo[0][1] * b2 + cMo[0][2] * c2;
  B2 = cMo[1][0] * a2 + cMo[1][1] * b2 + cMo[1][2] * c2;
  C2 = cMo[2][0] * a2 + cMo[2][1] * b2 + cMo[2][2] * c2;
  D2 = d2 - (cMo[0][3] * A2 + cMo[1][3] * B2 + cMo[2][3] * C2);

  // in case of verification
  // ap1 = A1; bp1 = B1; cp1 = C1; dp1 = D1;
  // ap2 = A2; bp2 = B2; cp2 = C2; dp2 = D2;

  //  vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;
  //  vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;

  // Adding constraints on the straight line to have a unique representation

  // direction of the straight line = N1 x N2
  a2 = B1 * C2 - C1 * B2;
  b2 = C1 * A2 - A1 * C2;
  c2 = A1 * B2 - B1 * A2;

  // Constraint D1 = 0 (the origin belongs to P1)
  a1 = A2 * D1 - A1 * D2;
  b1 = B2 * D1 - B1 * D2;
  c1 = C2 * D1 - C1 * D2;

  if (fabs(D2) < fabs(D1)) // to be sure that D2 <> 0
  {
    A2 = A1;
    B2 = B1;
    C2 = C1;
    D2 = D1;
  }

  // Constraint A1^2 + B1^2 + C1^2 = 1
  d1 = 1.0 / sqrt(a1 * a1 + b1 * b1 + c1 * c1);
  cP_[0] = A1 = a1 * d1;
  cP_[1] = B1 = b1 * d1;
  cP_[2] = C1 = c1 * d1;
  cP_[3] = 0;

  // Constraint A1 A2 + B1 B2 + C1 C2 = 0 (P2 orthogonal to P1)
  // N2_new = (N1 x N2) x N1_new
  a1 = b2 * C1 - c2 * B1;
  b1 = c2 * A1 - a2 * C1;
  c1 = a2 * B1 - b2 * A1;

  // Constraint A2^2 + B2^2 + C2^2 = 1
  d1 = 1.0 / sqrt(a1 * a1 + b1 * b1 + c1 * c1);
  a1 *= d1;
  b1 *= d1;
  c1 *= d1;

  // D2_new = D2 / (N2^T . N2_new)
  D2 /= (A2 * a1 + B2 * b1 + C2 * c1);
  A2 = a1;
  B2 = b1;
  C2 = c1;

  // Constraint D2 < 0
  if (D2 > 0) {
    A2 = -A2;
    B2 = -B2;
    C2 = -C2;
    D2 = -D2;
  }
  //  vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;
  //  vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;

  cP_[4] = A2;
  cP_[5] = B2;
  cP_[6] = C2;
  cP_[7] = D2;

  // in case of verification
  /*
  x = -A2*D2;
  y = -B2*D2;
  z = -C2*D2;
  d1 = ap1*x+bp1*y+cp1*z+dp1;
  d2 = ap2*x+bp2*y+cp2*z+dp2;
  if ((fabs(d1) > 1e-8) || (fabs(d2) > 1e-8))
    {
      printf("PB in VPline: P1 : 0 = %lf, P2: 0 = %lf\n",d1,d2);
      exit(-1);
    }
  d1 = A1*x+B1*y+C1*z+D1;
  d2 = A2*x+B2*y+C2*z+D2;
  if ((fabs(d1) > 1e-8) || (fabs(d2) > 1e-8))
    {
      printf("PB in VPline: Pn1 : 0 = %lf, Pn2: 0 = %lf\n",d1,d2);
      exit(-1);
    }
  */
}

/*!

  Displays the line in the image \e I thanks to the 2D parameters of
  the line \e p in the image plane (vpTracker::p) and the camera
  parameters which enable to convert the parameters from meter to
  pixel.

  \param I : The image where the line must be displayed.

  \param cam : The camera parameters to enable the conversion from
  meter to pixel.

  \param color : The desired color to display the line in the image.

  \param thickness : Thickness of the feature representation.
*/
void vpLine::display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color,
                     const unsigned int thickness)
{
  vpFeatureDisplay::displayLine(p[0], p[1], cam, I, color, thickness);
}

/*!

  Displays the line in the image \e I thanks to the parameters in the
  object frame (vpForwardProjection::oP), the homogeneous matrix
  relative to the pose between the camera frame and the object frame
  and the camera parameters which enable to convert the features from
  meter to pixel.

  \param I : The image where the line must be displayed in overlay.

  \param cMo : The homogeneous matrix corresponding to the pose
  between the camera frame and the object frame.

  \param cam : The camera parameters to enable the conversion from
  meter to pixel.

  \param color : The desired color to display the line in the image.

  \param thickness : Thickness of the feature representation.
*/
// non destructive wrt. cP and p
void vpLine::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                     const vpColor &color, const unsigned int thickness)
{
  vpColVector _cP, _p;
  changeFrame(cMo, _cP);
  try {
    projection(_cP, _p);
    vpFeatureDisplay::displayLine(_p[0], _p[1], cam, I, color, thickness);
  }
  catch(...) {
    // Skip potential exception: due to a degenerate case: the image of the straight line is a point!
  }
}

/*!
  Create an object with the same type.

  \code
  vpForwardProjection *fp;
  vpLine line;
  fp = line.duplicate(); // fp is now a vpLine
  \endcode

*/
vpLine *vpLine::duplicate() const
{
  vpLine *feature = new vpLine(*this);
  return feature;
}
