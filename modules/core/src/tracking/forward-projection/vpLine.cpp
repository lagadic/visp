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
 * Line feature.
 *
*****************************************************************************/

/*!
  \file vpLine.cpp
  \brief   class that defines what is a line
*/

#include <visp3/core/vpLine.h>

#include <visp3/core/vpMath.h>

#include <visp3/core/vpFeatureDisplay.h>

BEGIN_VISP_NAMESPACE
/*!

  Initialize the memory space requested for the 2D line parameters (\e
  p) in the image plane and for 3D line parameters (\e oP and \e cP)
  respectively in the object frame and the camera frame.
*/
void vpLine::init()
{
  const unsigned int val_2 = 2;
  const unsigned int val_8 = 8;
  oP.resize(val_8);
  cP.resize(val_8);
  p.resize(val_2);
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

  \f[ oA1 X + oB1 Y + oC1 Z + oD1 = 0 \f]
  \f[ oA2 X + oB2 Y + oC2 Z + oD2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates of a point in the object frame.

  \param oA1, oB1, oC1, oD1 : The parameters used to define the first plane in the object frame.
  \param oA2, oB2, oC2, oD2 : The parameters used to define the second plane in the object frame.

*/
void vpLine::setWorldCoordinates(const double &oA1, const double &oB1, const double &oC1, const double &oD1,
                                 const double &oA2, const double &oB2, const double &oC2, const double &oD2)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  const unsigned int index_6 = 6;
  const unsigned int index_7 = 7;
  oP[index_0] = oA1;
  oP[index_1] = oB1;
  oP[index_2] = oC1;
  oP[index_3] = oD1;

  oP[index_4] = oA2;
  oP[index_5] = oB2;
  oP[index_6] = oC2;
  oP[index_7] = oD2;
}

/*!

  Sets the parameters \e oP which define the line in the object frame. As
  said in the class description, the line is defined as the
  intersection of two planes. Eight parameters are required to define
  the equations of the two planes in the object frame. They are used to set
  the vpForwardProjection::oP public attribute.

  \f[ oA1 X + oB1 Y + oC1 Z + oD1 = 0 \f]
  \f[ oA2 X + oB2 Y + oC2 Z + oD2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates of a point in the object frame.

  \param oP_ : The column vector which contains the eight parameters
  needed to define the equations of the two planes in the object
  frame. \f[ oP = \left[\begin{array}{c}oA1 \\ oB1 \\ oC1 \\ oD1 \\ oA2 \\
  oB2 \\ oC2 \\ oD2 \end{array}\right] \f]

*/
void vpLine::setWorldCoordinates(const vpColVector &oP_)
{
  if (oP_.getRows() != 8) {
    throw vpException(vpException::dimensionError, "Size of oP is not equal to 8 as it should be");
  }
  this->oP = oP_;
}

/*!

  Sets the parameters \e oP which define the line in the object frame. As
  said in the class description, the line is defined as the
  intersection of two planes. Eight parameters are required to define
  the equations of the two planes in the object frame. They are used to set
  the vpForwardProjection::oP public attribute.

  \f[ oA1 X + oB1 Y + oC1 Z + oD1 = 0 \f]
  \f[ oA2 X + oB2 Y + oC2 Z + oD2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates of a point in the object frame.

  \param oP1 : The column vector which contains the four parameters
  needed to define the equations of the first plane in the object frame. \f[ oP1 =
  \left[\begin{array}{c}oA1 \\ oB1 \\ oC1 \\ oD1 \end{array}\right] \f]

  \param oP2 : The column vector which contains the four parameters
  needed to define the equations of the second plane in the object frame. \f[ oP2 =
  \left[\begin{array}{c} oA2 \\ oB2 \\ oC2 \\ oD2 \end{array}\right] \f]

*/
void vpLine::setWorldCoordinates(const vpColVector &oP1, const vpColVector &oP2)
{
  if (oP1.getRows() != 4) {
    throw vpException(vpException::dimensionError, "Size of oP1 is not equal to 4 as it should be");
  }
  if (oP2.getRows() != 4) {
    throw vpException(vpException::dimensionError, "Size of oP2 is not equal to 4 as it should be");
  }
  const unsigned int val_4 = 4;
  for (unsigned int i = 0; i < val_4; ++i) {
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
void vpLine::projection() { projection(cP, p); }

/*!

  Computes the 2D parameters \e p of the line in the image plane thanks
  to the 3D line parameters \e cP expressed in the camera frame. The
  image plane parameters \f$p=(\rho , \theta)\f$ are updated in
  output.

  \param[in] cP_ : The vector containing the line parameters relative to the
  camera frame. \f[ cP = \left[\begin{array}{c}cA1 \\ cB1 \\ cC1 \\ cD1
  \\ cA2 \\ cB2 \\ cC2 \\ cD2 \end{array}\right] \f]

  \param[out] p_ : The vector which contains the 2D line features expressed
  in the image plane. \f[ p = \left[\begin{array}{c} \rho \\ \theta
  \end{array}\right] \f]

  \exception vpException::fatalError : Degenerate case, the image of the
  straight line is a point.
*/
void vpLine::projection(const vpColVector &cP_, vpColVector &p_) const
{
  p_.resize(2, false);
  // projection

  if (cP.getRows() != 8) {
    throw vpException(vpException::dimensionError, "Size of cP is not equal to 8 as it should be");
  }
  double A1, A2, B1, B2, C1, C2, D1, D2;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  const unsigned int index_6 = 6;
  const unsigned int index_7 = 7;

  A1 = cP_[index_0];
  B1 = cP_[index_1];
  C1 = cP_[index_2];
  D1 = cP_[index_3];

  A2 = cP_[index_4];
  B2 = cP_[index_5];
  C2 = cP_[index_6];
  D2 = cP_[index_7];

  double a, b, c, s;
  a = (A2 * D1) - (A1 * D2);
  b = (B2 * D1) - (B1 * D2);
  c = (C2 * D1) - (C1 * D2);
  s = (a * a) + (b * b);
  if (s <= 1e-8) // seuil pas terrible
  {
    printf("Degenerate case: the image of the straight line is a point!\n");
    throw vpException(vpException::fatalError, "Degenerate case: the image of the straight line is a point!");
  }
  s = 1.0 / sqrt(s);

  double rho = -c * s;
  double theta = atan2(b, a);

  p_[0] = rho;
  p_[1] = theta;
}

/*!

  Computes the line parameters \e cP in the camera frame thanks to the
  line parameters \e oP given in the object frame and the homogeneous matrix
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
  \left[\begin{array}{c}cA1 \\ cB1 \\ cC1 \\ cD1 \\ cA2 \\ cB2 \\ cC2 \\ cD2
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
void vpLine::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP_) const
{
  cP_.resize(8, false);

  double a1, a2, b1, b2, c1, c2, d1, d2;
  double A1, A2, B1, B2, C1, C2, D1, D2;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  const unsigned int index_6 = 6;
  const unsigned int index_7 = 7;

  // in case of verification
  // double x,y,z,ap1,ap2,bp1,bp2,cp1,cp2,dp1,dp2;

  a1 = oP[index_0];
  b1 = oP[index_1];
  c1 = oP[index_2];
  d1 = oP[index_3];

  a2 = oP[index_4];
  b2 = oP[index_5];
  c2 = oP[index_6];
  d2 = oP[index_7];

  A1 = (cMo[index_0][0] * a1) + (cMo[index_0][1] * b1) + (cMo[index_0][index_2] * c1);
  B1 = (cMo[index_1][0] * a1) + (cMo[index_1][1] * b1) + (cMo[index_1][index_2] * c1);
  C1 = (cMo[index_2][0] * a1) + (cMo[index_2][1] * b1) + (cMo[index_2][index_2] * c1);
  D1 = d1 - ((cMo[index_0][index_3] * A1) + (cMo[index_1][index_3] * B1) + (cMo[index_2][index_3] * C1));

  A2 = (cMo[index_0][0] * a2) + (cMo[index_0][1] * b2) + (cMo[index_0][index_2] * c2);
  B2 = (cMo[index_1][0] * a2) + (cMo[index_1][1] * b2) + (cMo[index_1][index_2] * c2);
  C2 = (cMo[index_2][0] * a2) + (cMo[index_2][1] * b2) + (cMo[index_2][index_2] * c2);
  D2 = d2 - ((cMo[index_0][index_3] * A2) + (cMo[index_1][index_3] * B2) + (cMo[index_2][index_3] * C2));

  // Adding constraints on the straight line to have a unique representation

  // direction of the straight line = N1 x N2
  a2 = (B1 * C2) - (C1 * B2);
  b2 = (C1 * A2) - (A1 * C2);
  c2 = (A1 * B2) - (B1 * A2);

  // Constraint D1 = 0 (the origin belongs to P1)
  a1 = (A2 * D1) - (A1 * D2);
  b1 = (B2 * D1) - (B1 * D2);
  c1 = (C2 * D1) - (C1 * D2);

  if (fabs(D2) < fabs(D1)) // to be sure that D2 <> 0
  {
    A2 = A1;
    B2 = B1;
    C2 = C1;
    D2 = D1;
  }

  // Constraint A1^2 + B1^2 + C1^2 = 1
  d1 = 1.0 / sqrt((a1 * a1) + (b1 * b1) + (c1 * c1));
  A1 = a1 * d1;
  B1 = b1 * d1;
  C1 = c1 * d1;
  cP_[index_0] = A1;
  cP_[index_1] = B1;
  cP_[index_2] = C1;

  cP_[index_3] = 0;

  // Constraint A1 A2 + B1 B2 + C1 C2 = 0 (P2 orthogonal to P1)
  // N2_new = (N1 x N2) x N1_new
  a1 = (b2 * C1) - (c2 * B1);
  b1 = (c2 * A1) - (a2 * C1);
  c1 = (a2 * B1) - (b2 * A1);

  // Constraint A2^2 + B2^2 + C2^2 = 1
  d1 = 1.0 / sqrt((a1 * a1) + (b1 * b1) + (c1 * c1));
  a1 *= d1;
  b1 *= d1;
  c1 *= d1;

  // D2_new = D2 / (N2^T . N2_new)
  D2 /= ((A2 * a1) + (B2 * b1) + (C2 * c1));
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

  cP_[4] = A2;
  cP_[5] = B2;
  cP_[6] = C2;
  cP_[7] = D2;
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
                     unsigned int thickness)
{
  vpFeatureDisplay::displayLine(p[0], p[1], cam, I, color, thickness);
}

/*!

  Displays the line in the image \e I thanks to the 2D parameters of
  the line \e p in the image plane (vpTracker::p) and the camera
  parameters which enable to convert the parameters from meter to pixel.

  \param I : The image where the line must be displayed.

  \param cam : The camera parameters to enable the conversion from
                meter to pixel.

  \param color : The desired color to display the line in the image.

  \param thickness : Thickness of the feature representation.
                          */
void vpLine::display(const vpImage<vpRGBa> &I, const vpCameraParameters &cam, const vpColor &color,
                     unsigned int thickness)
{
  vpFeatureDisplay::displayLine(p[0], p[1], cam, I, color, thickness);
}

/*!

  Displays the line in the image \e I thanks to the parameters in the
  object frame (vpForwardProjection::oP), the homogeneous matrix
  relative to the pose between the camera frame and the object frame
  and the camera parameters which enable to convert the features from
  meter to pixel.

  This method is non destructive wrt. cP and p internal line parameters.

  \param I : The image where the line must be displayed in overlay.

  \param cMo : The homogeneous matrix corresponding to the pose
  between the camera frame and the object frame.

  \param cam : The camera parameters to enable the conversion from
  meter to pixel.

  \param color : The desired color to display the line in the image.

  \param thickness : Thickness of the feature representation.
*/
void vpLine::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                     const vpColor &color, unsigned int thickness)
{
  vpColVector v_cP, v_p;
  changeFrame(cMo, v_cP);
  try {
    projection(v_cP, v_p);
    vpFeatureDisplay::displayLine(v_p[0], v_p[1], cam, I, color, thickness);
  }
  catch (...) {
 // Skip potential exception: due to a degenerate case: the image of the straight line is a point!
  }
}

/*!

  Displays the line in the image \e I thanks to the parameters in the
  object frame (vpForwardProjection::oP), the homogeneous matrix
  relative to the pose between the camera frame and the object frame
  and the camera parameters which enable to convert the features from
  meter to pixel.

  This method is non destructive wrt. cP and p internal line parameters.

  \param I : The image where the line must be displayed in overlay.

  \param cMo : The homogeneous matrix corresponding to the pose
                between the camera frame and the object frame.

  \param cam : The camera parameters to enable the conversion from
                meter to pixel.

  \param color : The desired color to display the line in the image.

  \param thickness : Thickness of the feature representation.
                          */
void vpLine::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                     const vpColor &color, unsigned int thickness)
{
  vpColVector v_cP, v_p;
  changeFrame(cMo, v_cP);
  try {
    projection(v_cP, v_p);
    vpFeatureDisplay::displayLine(v_p[0], v_p[1], cam, I, color, thickness);
  }
  catch (...) {
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
END_VISP_NAMESPACE
