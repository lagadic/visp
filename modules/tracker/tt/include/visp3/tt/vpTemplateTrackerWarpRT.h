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
/*!
 \file vpTemplateTrackerWarpRT.h
 \brief sRt warping function
*/

#ifndef vpTemplateTrackerWarpRT_hh
#define vpTemplateTrackerWarpRT_hh

#include <visp3/tt/vpTemplateTrackerWarp.h>

/*!
  \class vpTemplateTrackerWarpRT
  \ingroup group_tt_warp
*/
class VISP_EXPORT vpTemplateTrackerWarpRT : public vpTemplateTrackerWarp
{
public:
  // constructor;
  vpTemplateTrackerWarpRT();

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  void computeCoeff(const vpColVector & /*p*/) {}
  void computeDenom(vpColVector & /*vX*/, const vpColVector & /*ParamM*/) {}
#endif

  /*!
      Compute the derivative of the warping function according to its
     parameters.

      \param X1 : Point to consider in the derivative computation.
      \param X2 : Point to consider in the derivative computation.
      \param ParamM : Parameters of the warping function.
      \param dW : Resulting derivative matrix.
    */
  void dWarp(const vpColVector &X1, const vpColVector &X2, const vpColVector &ParamM, vpMatrix &dW);

  /*!
      Compute the compositionnal derivative of the warping function according
     to its parameters.

      \param X1 : Point to consider in the derivative computation.
      \param X2 : Point to consider in the derivative computation.
      \param ParamM : Parameters of the warping function.
      \param dwdp0 : Derivative matrix of the warping function according to
     the initial warping function parameters (p=0). \param dW : Resulting
     compositionnal derivative matrix.
    */
  void dWarpCompo(const vpColVector &X1, const vpColVector &X2, const vpColVector &ParamM, const double *dwdp0,
                  vpMatrix &dW);

  /*!
      Compute the derivative of the image with relation to the warping
     function parameters.

      \param i : i coordinate (along the rows) of the point to consider in the
     image. \param j : j coordinate (along the columns) of the point to
     consider in the image. \param dy : Derivative on the y-axis (along the
     rows) of the point (i,j). \param dx : Derivative on the x-axis (along the
     columns) of the point (i,j). \param dIdW : Resulting derivative matrix
     (Image according to the warping function).
    */
  void getdW0(const int &i, const int &j, const double &dy, const double &dx, double *dIdW);

  /*!
      Compute the derivative of the warping function according to the initial
     parameters.

      \param i : i coordinate (along the rows) of the point to consider in the
     image. \param j : j coordinate (along the columns) of the point to
     consider in the image. \param dIdW : Resulting derivative matrix (Image
     according to the warping function).
    */
  void getdWdp0(const int &i, const int &j, double *dIdW);

  /*!
      Get the inverse of the warping function parameters.

      \param ParamM : Parameters of the warping function.
      \param ParamMinv : Inverse parameters.
    */
  void getParamInverse(const vpColVector &ParamM, vpColVector &ParamMinv) const;

  /*!
      Get the parameters of the warping function one level down.

      \param p : Current parameters of the warping function.
      \param pdown : Resulting parameters on level down.
    */
  void getParamPyramidDown(const vpColVector &p, vpColVector &pdown);

  /*!
      Get the parameters of the warping function one level up.

      \param p : Current parameters of the warping function.
      \param pup : Resulting parameters one level up.
    */
  void getParamPyramidUp(const vpColVector &p, vpColVector &pup);

  /*!
      Tells if the warping function is ESM compatible.

      \return True if it is ESM compatible, False otherwise.
    */
  bool isESMcompatible() const { return false; }

  /*!
      Get the displacement resulting from the composition of two other
     displacements.

      \param p1 : First displacement.
      \param p2 : Second displacement.
      \param pres : Displacement resulting from the composition of p1 and p2.
    */
  void pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &pres) const;

  /*!
      Warp a point.

      \param vX : Coordinates of the point to warp.
      \param vXres : Coordinates of the warped point.
      \param ParamM : Parameters of the warping function.
    */
  void warpX(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM);

  /*!
      Warp a point.

      \param i : i coordinate (along the rows) of the point to warp.
      \param j : j coordinate (along the columns) of the point to warp.
      \param i2 : i coordinate (along the rows) of the warped point.
      \param j2 : j coordinate (along the columns) of the warped point.
      \param ParamM : Parameters of the warping function.
    */
  void warpX(const int &i, const int &j, double &i2, double &j2, const vpColVector &ParamM);

  /*!
      Inverse Warp a point.

      \param vX : Coordinates of the point to warp.
      \param vXres : Coordinates of the warped point.
      \param ParamM : Parameters of the warping function.
    */
  void warpXInv(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM);
};
#endif
