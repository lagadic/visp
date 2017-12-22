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
 \file vpTemplateTrackerWarp.h
 \brief
*/

#ifndef vpTemplateTrackerWarp_hh
#define vpTemplateTrackerWarp_hh

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/tt/vpTemplateTrackerHeader.h>
#include <visp3/tt/vpTemplateTrackerTriangle.h>
#include <visp3/tt/vpTemplateTrackerZone.h>

/*!
  \class vpTemplateTrackerWarp
  \ingroup group_tt_warp
*/
class VISP_EXPORT vpTemplateTrackerWarp
{
protected:
  double denom;
  vpMatrix dW;
  unsigned int nbParam;

public:
  // constructor;
  vpTemplateTrackerWarp() : denom(1.), dW(), nbParam(0) {}
  virtual ~vpTemplateTrackerWarp() {}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  virtual void computeCoeff(const vpColVector &p) = 0;
  virtual void computeDenom(vpColVector &vX, const vpColVector &ParamM) = 0;
#endif

  /*!
    Compute the derivative of the warping function according to its
    parameters.

    \param X1 : Point to consider in the derivative computation.
    \param X2 : Point to consider in the derivative computation.
    \param ParamM : Parameters of the warping function.
    \param dW : Resulting derivative matrix.
  */
  virtual void dWarp(const vpColVector &X1, const vpColVector &X2, const vpColVector &ParamM, vpMatrix &dW) = 0;

  /*!
    Compute the compositionnal derivative of the warping function according to
    its parameters.

    \param X1 : Point to consider in the derivative computation.
    \param X2 : Point to consider in the derivative computation.
    \param ParamM : Parameters of the warping function.
    \param dwdp0 : Derivative matrix of the warping function according to the
    initial warping function parameters (p=0). \param dW : Resulting
    compositionnal derivative matrix.
  */
  virtual void dWarpCompo(const vpColVector &X1, const vpColVector &X2, const vpColVector &ParamM, const double *dwdp0,
                          vpMatrix &dW) = 0;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  void findWarp(const double *ut0, const double *vt0, const double *u, const double *v, int nb_pt, vpColVector &p);
#endif

  /*!
    Compute the derivative of the image with relation to the warping function
    parameters.

    \param i : i coordinate (along the rows) of the point to consider in the
    image. \param j : j coordinate (along the columns) of the point to
    consider in the image. \param dy : Derivative on the y-axis (along the
    rows) of the point (i,j). \param dx : Derivative on the x-axis (along the
    columns) of the point (i,j). \param dIdW : Resulting derivative matrix
    (Image according to the warping function).
  */
  virtual void getdW0(const int &i, const int &j, const double &dy, const double &dx, double *dIdW) = 0;

  /*!
    Compute the derivative of the warping function according to the initial
    parameters.

    \param i : i coordinate (along the rows) of the point to consider in the
    image. \param j : j coordinate (along the columns) of the point to
    consider in the image. \param dIdW : Resulting derivative matrix (Image
    according to the warping function).
  */
  virtual void getdWdp0(const int &i, const int &j, double *dIdW) = 0;

  /*!
    Compute the distance between a zone and its associated warped zone.

    \param Z : Zone to consider.
    \param p : Parameters of the warping function.
  */
  double getDistanceBetweenZoneAndWarpedZone(const vpTemplateTrackerZone &Z, const vpColVector &p);

  /*!
    Get the number of parameters of the warping function.

    \return Number of parameters.
  */
  unsigned int getNbParam() const { return nbParam; }

  /*!
    Get the inverse of the warping function parameters.

    \param ParamM : Parameters of the warping function.
    \param ParamMinv : Inverse parameters.
  */
  virtual void getParamInverse(const vpColVector &ParamM, vpColVector &ParamMinv) const = 0;

  /*!
    Get the parameters of the warping function one level down.

    \param p : Current parameters of the warping function.
    \param pdown : Resulting parameters on level down.
  */
  virtual void getParamPyramidDown(const vpColVector &p, vpColVector &pdown) = 0;

  /*!
    Get the parameters of the warping function one level up.

    \param p : Current parameters of the warping function.
    \param pup : Resulting parameters one level up.
  */
  virtual void getParamPyramidUp(const vpColVector &p, vpColVector &pup) = 0;

  /*!
    Tells if the warping function is ESM compatible.

    \return True if it is ESM compatible, False otherwise.
  */
  virtual bool isESMcompatible() const = 0;

  /*!
    Get the displacement resulting from the composition of two other
    displacements.

    \param p1 : First displacement.
    \param p2 : Second displacement.
    \param pres : Displacement resulting from the composition of p1 and p2.
  */
  virtual void pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &pres) const = 0;

  /*!
    Set the number of parameters of the warping function.

    \param nb : New number of parameters.
  */
  void setNbParam(unsigned int nb)
  {
    nbParam = nb;
    dW.resize(2, nbParam);
  }

  /*!
    Warp a list of points.

    \param ut0 : List of u coordinates of the points.
    \param vt0 : List of v coordinates of the points.
    \param nb_pt : Number of points to consider.
    \param p : Parameters of the warp.
    \param u : Resulting u coordinates.
    \param v : resulting v coordinates.
  */
  void warp(const double *ut0, const double *vt0, int nb_pt, const vpColVector &p, double *u, double *v);

  /*!
    Warp a point.

    \param i : i coordinate (along the rows) of the point to warp.
    \param j : j coordinate (along the columns) of the point to warp.
    \param i2 : i coordinate (along the rows) of the warped point.
    \param j2 : j coordinate (along the columns) of the warped point.
    \param ParamM : Parameters of the warp.
  */
  virtual void warpX(const int &i, const int &j, double &i2, double &j2, const vpColVector &ParamM) = 0;

  /*!
    Warp a point.

    \param vX : Coordinates of the point to warp.
    \param vXres : Coordinates of the warped point.
    \param ParamM : Parameters of the warping function.
  */
  virtual void warpX(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM) = 0;

  /*!
    Inverse Warp a point.

    \param vX : Coordinates of the point to warp.
    \param vXres : Coordinates of the warped point.
    \param ParamM : Parameters of the warping function.
  */
  virtual void warpXInv(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM) = 0;

  /*!
    Warp a triangle and store the result in a new zone.

    \param in : Triangle to warp.
    \param p : Parameters of the warping function. These parameters are
    estimated by the template tracker and returned using
    vpTemplateTracker::getp(). \param out : Resulting triangle.
  */
  void warpTriangle(const vpTemplateTrackerTriangle &in, const vpColVector &p, vpTemplateTrackerTriangle &out);

  /*!
    Warp a zone and store the result in a new zone.

    \param in : Zone to warp.
    \param p : Parameters of the warping function. These parameters are
    estimated by the template tracker and returned using
    vpTemplateTracker::getp(). \param out : Resulting zone.
  */
  void warpZone(const vpTemplateTrackerZone &in, const vpColVector &p, vpTemplateTrackerZone &out);
};

#endif
