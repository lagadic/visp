/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 *
*****************************************************************************/
/*!
 \file vpTemplateTrackerWarp.h
 \brief
*/

#ifndef vpTemplateTrackerWarp_hh
#define vpTemplateTrackerWarp_hh

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/tt/vpTemplateTrackerHeader.h>
#include <visp3/tt/vpTemplateTrackerTriangle.h>
#include <visp3/tt/vpTemplateTrackerZone.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpTemplateTrackerWarp
  \ingroup group_tt_warp
*/
class VISP_EXPORT vpTemplateTrackerWarp
{
protected:
  double denom;         //!< Internal value used by homography warp model.
  unsigned int nbParam; //!< Number of parameters used to model warp transformation.

public:
  /*!
   * Default constructor.
   */
  vpTemplateTrackerWarp() : denom(1.), nbParam(0) { }
  /*!
   * Destructor.
   */
  virtual ~vpTemplateTrackerWarp() { }

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  virtual void computeCoeff(const vpColVector &p) = 0;
  virtual void computeDenom(vpColVector &vX, const vpColVector &ParamM) = 0;
#endif

  /*!
   * Compute the derivative matrix of the warping function at point \f$X=(u,v)\f$ according to the model parameters:
   * \f[
   * \frac{\partial M}{\partial p}(X_1, X_2, p)
   * \f]
   * \param X1 : Vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to
   * consider in the derivative computation.
   * \param X2 : Vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to
   * consider in the derivative computation.
   * \param p : Vector that contains the parameters of the warping function.
   * \param dM : Resulting warping model derivative returned as a matrix.
   */
  virtual void dWarp(const vpColVector &X1, const vpColVector &X2, const vpColVector &p, vpMatrix &dM) = 0;

  /*!
   * Compute the compositionnal derivative matrix of the warping function according to the model parameters.
   * \param X1 : Point to consider in the derivative computation.
   * \param X2 : Point to consider in the derivative computation.
   * \param p : Vector that contains the parameters of the warping function.
   * \param dwdp0 : Derivative matrix of the warping function according to
   * the initial warping function parameters (p=0).
   * \param dM : Resulting warping model compositionnal derivative returned as a 2-by-3 matrix.
   */
  virtual void dWarpCompo(const vpColVector &X1, const vpColVector &X2, const vpColVector &p, const double *dwdp0,
                          vpMatrix &dM) = 0;

  /*!
   * Compute the derivative of the image with relation to the warping function parameters.
   * \param v : Coordinate (along the image rows axis) of the point to consider in the image.
   * \param u : Coordinate (along the image columns axis) of the point to consider in the image.
   * \param dv : Derivative on the v-axis (along the rows) of the point (u,v).
   * \param du : Derivative on the u-axis (along the columns) of the point (u,v).
   * \param dIdW : Resulting derivative matrix (image according to the warping function).
   */
  virtual void getdW0(const int &v, const int &u, const double &dv, const double &du, double *dIdW) = 0;

  /*!
   * Compute the derivative of the warping model \f$M\f$ according to the initial parameters \f$p_0\f$
   * at point \f$X=(u,v)\f$:
   * \f[
   * \frac{\partial M}{\partial p}(X, p_0)
   * \f]
   *
   * \param v : Coordinate (along the image rows axis) of the point X(u,v) to consider in the image.
   * \param u : Coordinate (along the image columns axis) of the point X(u,v) to consider in the image.
   * \param dIdW : Resulting 2-by-3 derivative matrix.
   */
  virtual void getdWdp0(const int &v, const int &u, double *dIdW) = 0;

  /*!
   * Compute inverse of the RT warping transformation.
   * \param p : Vector that contains the parameters corresponding
   * to the transformation to inverse.
   * \param p_inv : Vector that contains the parameters of the inverse transformation \f$ {M(p)}^{-1}\f$.
   */
  virtual void getParamInverse(const vpColVector &p, vpColVector &p_inv) const = 0;

  /*!
   * Get the parameters of the warping function one level down
   * where image size is divided by two along the lines and the columns.
   * \param p : Vector that contains the current parameters of the warping function.
   * \param p_down : Vector that contains the resulting parameters one level down.
   */
  virtual void getParamPyramidDown(const vpColVector &p, vpColVector &p_down) = 0;

  /*!
   * Get the parameters of the warping function one level up
   * where image size is multiplied by two along the lines and the columns.
   * \param p : Vector that contains the current parameters of the warping function.
   * \param p_up : Vector that contains the resulting parameters one level up.
   */
  virtual void getParamPyramidUp(const vpColVector &p, vpColVector &p_up) = 0;

  /*!
    Tells if the warping function is ESM compatible.

    \return true if it is ESM compatible, false otherwise.
  */
  virtual bool isESMcompatible() const = 0;

  /*!
   * Compute the RT transformation resulting from the composition of two other RT transformations.
   * \param p1 : Vector that contains the parameters corresponding
   * to first transformation.
   * \param p2 : Vector that contains the parameters corresponding
   * to second transformation.
   * \param p12 : Vector that contains the resulting transformation \f$ p_{12} = p_1 \circ p_2\f$.
   */
  virtual void pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &p12) const = 0;

  /*!
   * Warp point \f$X_1=(u_1,v_1)\f$ using the transformation model with parameters \f$p\f$.
   * \f[X_2 = {^2}M_1(p) * X_1\f]
   * \param v1 : Coordinate (along the image rows axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
   * \param u1 : Coordinate (along the image columns axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
   * \param v2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image rows axis.
   * \param u2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image column axis.
   * \param p : Vector that contains the parameters of the transformation.
   */
  virtual void warpX(const int &v1, const int &u1, double &v2, double &u2, const vpColVector &p) = 0;

  /*!
   * Warp point \f$X_1=(u_1,v_1)\f$ using the RT transformation model.
   * \f[X_1 = M(p) * X_2\f]
   * \param X1 : Vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to warp.
   * \param X2 : Vector corresponding to the coordinates \f$(u_2, v_2)\f$ of the warped point.
   * \param p : Vector that contains the parameters of the RT transformation.
   */
  virtual void warpX(const vpColVector &X1, vpColVector &X2, const vpColVector &p) = 0;

  /*!
   * Warp a point X1 with the inverse transformation \f$M\f$.
   * \f[ X_2 = {\left( {^1}M_2 \right) }^{-1} \; X_1\f]
   * \param X1 : Vector corresponding to the coordinates (u,v) of the point to warp.
   * \param X2 : Vector corresponding to the coordinates (u,v) of the warped point.
   * \param p : Parameters corresponding to the warping RT model \f${^1}M_2\f$.
   */
  virtual void warpXInv(const vpColVector &X1, vpColVector &X2, const vpColVector &p) = 0;

  /** @name Inherited functionalities from vpTemplateTrackerWarp */
  //@{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  void findWarp(const double *ut0, const double *vt0, const double *u, const double *v, int nb_pt, vpColVector &p);
#endif
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
    Set the number of parameters of the warping function.

    \param nb : New number of parameters.
  */
  void setNbParam(unsigned int nb) { nbParam = nb; }

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
    vpTemplateTracker::getp().
    \param out : Resulting zone.
  */
  void warpZone(const vpTemplateTrackerZone &in, const vpColVector &p, vpTemplateTrackerZone &out);
  //@}
};
END_VISP_NAMESPACE
#endif
