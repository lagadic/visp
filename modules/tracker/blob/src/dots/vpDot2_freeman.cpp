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
 * Freeman chain dedicated functions.
 */

#include <visp3/core/vpConfig.h>
#include <visp3/blob/vpDot2.h>

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace
{
/*!
* Given the previous position of a pixel (u_p, v_p) on the dot border, the
* direction to reach the next pixel on the border being top right and
* because with use the moment, compute Freeman parameters.
* \param u_p : Previous value of the row coordinate of a pixel on a border.
* \param v_p : Previous value of the column coordinate of a pixel on a border.
* \param dMuv : Moment increases. Cumulated values of dMuv gives m11.
* \param dMu2 : Second order moment along v axis increases. Cumulated values
* of dMu2 gives m20.
* \param dMv2 : Second order moment along u axis increases. Cumulated values
* of dMv2 gives m02.
*/
void computeTopRightWithMoment(const int &u_p, const int &v_p, float &dMuv, float &dMu2, float &dMv2)
{
  float half_u_p = static_cast<float>(0.5 * u_p);
  dMuv = static_cast<float>((v_p * v_p * (0.25 + half_u_p)) + (v_p * ((1. / 3.) + half_u_p)) + ((1. / 6.) * u_p) + 0.125);
  dMu2 = static_cast<float>(((-1. / 3.) * u_p * ((u_p * u_p) + (1.5 * u_p) + 1.)) - (1. / 12.0));
  dMv2 = static_cast<float>(((1. / 3.) * v_p * ((v_p * v_p) + (1.5 * v_p) + 1.)) + (1. / 12.0));
}

/*!
* Given the previous position of a pixel (u_p, v_p) on the dot border, the
* direction to reach the next pixel on the border being top left and
* because with use the moment, compute Freeman parameters.
* \param u_p : Previous value of the row coordinate of a pixel on a border.
* \param v_p : Previous value of the column coordinate of a pixel on a border.
* \param dMuv : Moment increases. Cumulated values of dMuv gives m11.
* \param dMu2 : Second order moment along v axis increases. Cumulated values
* of dMu2 gives m20.
* \param dMv2 : Second order moment along u axis increases. Cumulated values
* of dMv2 gives m02.
*/
void computeTopLeftWithMoment(const int &u_p, const int &v_p, float &dMuv, float &dMu2, float &dMv2)
{
  float half_u_p = static_cast<float>(0.5 * u_p);
  dMuv = static_cast<float>((((v_p * v_p * (0.25 - half_u_p)) + (v_p * ((1. / 3.) - half_u_p))) - ((1. / 6.) * u_p)) + 0.125);
  dMu2 = static_cast<float>(((-1. / 3.) * u_p * (((u_p * u_p) - (1.5 * u_p)) + 1.)) - (1. / 12.0));
  dMv2 = static_cast<float>(((-1. / 3.) * v_p * ((v_p * v_p) + (1.5 * v_p) + 1.)) - (1. / 12.0));
}

/*!
* Given the previous position of a pixel (u_p, v_p) on the dot border, the
* direction to reach the next pixel on the border being down right and
* because with use the moment, compute Freeman parameters.
* \param u_p : Previous value of the row coordinate of a pixel on a border.
* \param v_p : Previous value of the column coordinate of a pixel on a border.
* \param dMuv : Moment increases. Cumulated values of dMuv gives m11.
* \param dMu2 : Second order moment along v axis increases. Cumulated values
* of dMu2 gives m20.
* \param dMv2 : Second order moment along u axis increases. Cumulated values
* of dMv2 gives m02.
*/
void computeDownRightWithMoment(const int &u_p, const int &v_p, float &dMuv, float &dMu2, float &dMv2)
{
  float half_u_p = static_cast<float>(0.5 * u_p);
  dMuv = static_cast<float>(((v_p * v_p * (0.25 + half_u_p)) - (v_p * ((1. / 3.) + half_u_p))) + ((1. / 6.) * u_p) + 0.125);
  dMu2 = static_cast<float>(((1. / 3.) * u_p * ((u_p * u_p) + (1.5 * u_p) + 1.)) + (1. / 12.0));
  dMv2 = static_cast<float>(((1. / 3.) * v_p * (((v_p * v_p) - (1.5 * v_p)) + 1.)) - (1. / 12.0));
}

/*!
* Given the previous position of a pixel (u_p, v_p) on the dot border, the
* direction to reach the next pixel on the border being down left and
* because with use the moment, compute Freeman parameters.
* \param u_p : Previous value of the row coordinate of a pixel on a border.
* \param v_p : Previous value of the column coordinate of a pixel on a border.
* \param dMuv : Moment increases. Cumulated values of dMuv gives m11.
* \param dMu2 : Second order moment along v axis increases. Cumulated values
* of dMu2 gives m20.
* \param dMv2 : Second order moment along u axis increases. Cumulated values
* of dMv2 gives m02.
*/
void computeDownLeftWithMoment(const int &u_p, const int &v_p, float &dMuv, float &dMu2, float &dMv2)
{
  float half_u_p = static_cast<float>(0.5 * u_p);
  dMuv = static_cast<float>((((v_p * v_p * (0.25 - half_u_p)) - (v_p * ((1. / 3.) - half_u_p))) - ((1. / 6.) * u_p)) + 0.125);
  dMu2 = static_cast<float>(((1. / 3.) * u_p * (((u_p * u_p) - (1.5 * u_p)) + 1.)) - (1. / 12.0));
  dMv2 = static_cast<float>(((-1. / 3.) * v_p * (((v_p * v_p) - (1.5 * v_p)) + 1.)) - (1. / 12.0));
}
}

#endif // DOXYGEN_SHOULD_SKIP_THIS
/*!

  Gets the list of Freeman chain code used to turn around the dot
  counterclockwise.

  \param[out] freeman_chain : List of Freeman chain list [0, ..., 7]
  - 0 : right
  - 1 : top right
  - 2 : top
  - 3 : top left
  - 4 : left
  - 5 : down left
  - 6 : down
  - 7 : down right
*/
void vpDot2::getFreemanChain(std::list<unsigned int> &freeman_chain) const { freeman_chain = m_direction_list; }

/*!

  Considering a pixel (u, v) compute the next element of the Freeman chain
  code.

  According to the gray level of pixel (u, v) and his eight neighbors
  determine the next element of the chain in order to turn around the dot
  counterclockwise.

  \param I : The image we are working with.
  \param v : The row coordinate of a pixel on a border.
  \param u : The column coordinate of the pixel on a border.
  \param element : The next freeman element chain code (0, 1, 2, 3, 4, 5, 6,
  7) with 0 for right moving, 2 for down, 4 for left and 6 for up moving.

  \return false if an element cannot be found. Occurs for example with an area
  constituted by a single pixel. Return true if success.
*/
bool vpDot2::computeFreemanChainElement(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v,
                                        unsigned int &element)
{
  if (hasGoodLevel(I, u, v)) {
    unsigned int v_u = u;
    unsigned int v_v = v;
    const unsigned int val_1 = 1;
    const unsigned int val_2 = 2;
    const unsigned int val_3 = 3;
    const unsigned int val_4 = 4;
    const unsigned int val_5 = 5;
    const unsigned int val_6 = 6;
    const unsigned int val_7 = 7;
    const unsigned int val_8 = 8;
    // get the point on the right of the point passed in
    updateFreemanPosition(v_u, v_v, (element + val_2) % val_8);
    if (hasGoodLevel(I, v_u, v_v)) {
      element = (element + val_2) % val_8; // turn right
    }
    else {
      unsigned int v_u1 = u;
      unsigned int v_v1 = v;
      updateFreemanPosition(v_u1, v_v1, (element + val_1) % val_8);

      if (hasGoodLevel(I, v_u1, v_v1)) {
        element = (element + val_1) % val_8; // turn diag right
      }
      else {
        unsigned int v_u2 = u;
        unsigned int v_v2 = v;
        updateFreemanPosition(v_u2, v_v2, element); // same direction

        if (hasGoodLevel(I, v_u2, v_v2)) {
          // element = element;      // keep same dir
        }
        else {
          unsigned int v_u3 = u;
          unsigned int v_v3 = v;
          updateFreemanPosition(v_u3, v_v3, (element + val_7) % val_8); // diag left

          if (hasGoodLevel(I, v_u3, v_v3)) {
            element = (element + val_7) % val_8; // turn diag left
          }
          else {
            unsigned int v_u4 = u;
            unsigned int v_v4 = v;
            updateFreemanPosition(v_u4, v_v4, (element + val_6) % val_8); // left

            if (hasGoodLevel(I, v_u4, v_v4)) {
              element = (element + val_6) % val_8; // turn left
            }
            else {
              unsigned int v_u5 = u;
              unsigned int v_v5 = v;
              updateFreemanPosition(v_u5, v_v5, (element + val_5) % val_8); // left

              if (hasGoodLevel(I, v_u5, v_v5)) {
                element = (element + val_5) % val_8; // turn diag down
              }
              else {
                unsigned int v_u6 = u;
                unsigned int v_v6 = v;
                updateFreemanPosition(v_u6, v_v6, (element + val_4) % val_8); // left

                if (hasGoodLevel(I, v_u6, v_v6)) {
                  element = (element + val_4) % val_8; // turn down
                }
                else {
                  unsigned int v_u7 = u;
                  unsigned int v_v7 = v;
                  updateFreemanPosition(v_u7, v_v7, (element + val_3) % val_8); // diag

                  if (hasGoodLevel(I, v_u7, v_v7)) {
                    element = (element + val_3) % val_8; // turn diag right down
                  }
                  else {
                    // No neighbor with a good level
                    //
                    return false;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  else {
    return false;
  }

  return true;
}

/*!

  Given the previous position of a pixel (u_p, v_p) on the dot border and the
  direction to reach the next pixel on the border, compute Freeman parameters.

  \param u_p : Previous value of the row coordinate of a pixel on a border.
  \param v_p : Previous value of the column coordinate of a pixel on a border.
  \param du : Increment to go from previous to next pixel on the dot border.
  \param dv : Increment to go from previous to next pixel on the dot border.

  \param dS : Enclosed area increases. Cumulated values of dS gives m00.

  \param dMu : First order moment along v axis increases. Cumulated values of
  dMu gives m10.

  \param dMv : First order moment along u axis increases. Cumulated values of
  dMv gives m01.

  \param dMuv : Moment increases. Cumulated values of dMuv gives m11.

  \param dMu2 : Second order moment along v axis increases. Cumulated values
  of dMu2 gives m20.

  \param dMv2 : Second order moment along u axis increases. Cumulated values
  of dMv2 gives m02.

  Considering the previous coordinates (u_p, v_p) of a pixel on a border, the
  next coordinates (u, v) are given by: u = u_p + du and v = v_p + dv
*/
void vpDot2::computeFreemanParameters(const int &u_p, const int &v_p, unsigned int &element, int &du, int &dv,
                                      float &dS, float &dMu, float &dMv, float &dMuv, float &dMu2, float &dMv2)
{
  /*
      3  2  1
       \ | /
        \|/
    4 ------- 0
        /|\
       / | \
      5  6  7
  */
  const unsigned int go_right = 0;
  const unsigned int go_right_top = 1;
  const unsigned int go_top = 2;
  const unsigned int go_top_left = 3;
  const unsigned int go_left = 4;
  const unsigned int go_left_down = 5;
  const unsigned int go_down = 6;
  const unsigned int go_down_right = 7;
  du = 0;
  dv = 0;
  dMuv = 0;
  dMu2 = 0;
  dMv2 = 0;
  const unsigned int val_2 = 2;
  switch (element) {
  case go_right: // go right
    du = 1;
    dS = static_cast<float>(v_p);
    dMu = 0.0;
    dMv = static_cast<float>(0.5 * v_p * v_p);
    if (m_compute_moment) {
      dMuv = static_cast<float>(0.25 * v_p * v_p * ((val_2 * u_p) + 1));
      dMu2 = 0;
      dMv2 = static_cast<float>((1.0 / 3.) * v_p * v_p * v_p);
    }
    break;

  case go_right_top: // go right top
    du = 1;
    dv = 1;
    dS = static_cast<float>(v_p + 0.5);
    dMu = -static_cast<float>((0.5 * u_p * (u_p + 1)) + (1.0 / 6.0));
    dMv = static_cast<float>((0.5 * v_p * (v_p + 1)) + (1.0 / 6.0));
    if (m_compute_moment) {
      computeTopRightWithMoment(u_p, v_p, dMuv, dMu2, dMv2);
    }
    break;

  case go_top: // go top
    dv = 1;
    dS = 0.0;
    dMu = static_cast<float>(-0.5 * u_p * u_p);
    dMv = 0.0;
    if (m_compute_moment) {
      dMuv = 0;
      dMu2 = static_cast<float>((-1.0 / 3.) * u_p * u_p * u_p);
      dMv2 = 0;
    }
    break;

  case go_top_left:
    du = -1;
    dv = 1;
    dS = static_cast<float>(-v_p - 0.5);
    dMu = -static_cast<float>((0.5 * u_p * (u_p - 1)) + (1.0 / 6.0));
    dMv = -static_cast<float>((0.5 * v_p * (v_p + 1)) + (1.0 / 6.0));
    if (m_compute_moment) {
      computeTopLeftWithMoment(u_p, v_p, dMuv, dMu2, dMv2);
    }
    break;

  case go_left:
    du = -1;
    dS = static_cast<float>(-v_p);
    dMv = static_cast<float>(-0.5 * v_p * v_p);
    dMu = 0.0;
    if (m_compute_moment) {
      dMuv = static_cast<float>(-0.25 * v_p * v_p * ((val_2 * u_p) - 1));
      dMu2 = 0;
      dMv2 = static_cast<float>((-1.0 / 3.) * v_p * v_p * v_p);
    }
    break;

  case go_left_down:
    du = -1;
    dv = -1;
    dS = static_cast<float>(-v_p + 0.5);
    dMu = static_cast<float>((0.5 * u_p * (u_p - 1)) + (1.0 / 6.0));
    dMv = static_cast<float>(-((0.5 * v_p * (v_p - 1)) + (1.0 / 6.0)));
    if (m_compute_moment) {
      computeDownLeftWithMoment(u_p, v_p, dMuv, dMu2, dMv2);
    }
    break;

  case go_down:
    dv = -1;
    dS = 0.0;
    dMu = static_cast<float>(0.5 * u_p * u_p);
    dMv = 0.0;
    if (m_compute_moment) {
      dMuv = 0;
      dMu2 = static_cast<float>((1.0 / 3.) * u_p * u_p * u_p);
      dMv2 = 0;
    }
    break;

  case go_down_right:
    du = 1;
    dv = -1;
    dS = static_cast<float>(v_p - 0.5);
    dMu = static_cast<float>((0.5 * u_p * (u_p + 1)) + (1.0 / 6.0));
    dMv = static_cast<float>((0.5 * v_p * (v_p - 1)) + (1.0 / 6.0));
    if (m_compute_moment) {
      computeDownRightWithMoment(u_p, v_p, dMuv, dMu2, dMv2);
    }
    break;

  default:
    std::cout << "to complete the default" << std::endl;
  }
}

/*!

  From a pixel coordinate and a direction, update the pixel coordinates after
  moving forward.

  \param v : The row coordinate of the pixel, updated by this method.

  \param u : The column coordinate of the pixel, updated by this method.

  \param dir : The direction in the image, 0=right, 1, 2=down, 3, 4=left, 5,
  6=up and 7.

*/
void vpDot2::updateFreemanPosition(unsigned int &u, unsigned int &v, const unsigned int &dir)
{
  /*
      3  2  1
       \ | /
        \|/
    4 ------- 0
        /|\
       / | \
      5  6  7
  */
  const unsigned int go_right = 0;
  const unsigned int go_right_top = 1;
  const unsigned int go_top = 2;
  const unsigned int go_top_left = 3;
  const unsigned int go_left = 4;
  const unsigned int go_left_down = 5;
  const unsigned int go_down = 6;
  const unsigned int go_down_right = 7;
  switch (dir) {
  case go_right:
    u += 1;
    break;
  case go_right_top:
    u += 1;
    v += 1;
    break;
  case go_top:
    v += 1;
    break;
  case go_top_left:
    u -= 1;
    v += 1;
    break;
  case go_left:
    u -= 1;
    break;
  case go_left_down:
    u -= 1;
    v -= 1;
    break;
  case go_down:
    v -= 1;
    break;
  case go_down_right:
    u += 1;
    v -= 1;
    break;
  default:
    std::cout << "In vpDot2::updateFreemanPosition dir not identified" << std::endl;
  }
}

END_VISP_NAMESPACE
