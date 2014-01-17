/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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

#include <visp/vpDisplay.h>
#include <visp/vpTemplateTrackerHeader.h>
#include <visp/vpTemplateTrackerTriangle.h>
#include <visp/vpTemplateTrackerZone.h>
#include <visp/vpTrackingException.h>

class VISP_EXPORT vpTemplateTrackerWarp
{
  protected:
    double Denom;
    vpMatrix dW;
    unsigned int NbParam ;
    
  public:
    //constructor;
    vpTemplateTrackerWarp(){ NbParam = 0; Denom = 1.;}
    virtual ~vpTemplateTrackerWarp(){}

    virtual void computeCoeff(const vpColVector &p)=0;
    virtual void computeDenom(vpColVector &vX, const vpColVector &ParamM)=0;
    
    /*!
      Compute the derivative of the warp according to the its parameters.

      \param X1 : Point to consider in the derivative computation.
      \param X2 : Point to consider in the derivative computation.
      \param ParamM : Parameters of the warp.
      \param dW : Resulting derivative matrix.
    */
    virtual void dWarp(const vpColVector &X1,const vpColVector &X2,const vpColVector &ParamM,vpMatrix &dW) = 0;

    /*!
      Compute the compositionnal derivative of the warp according to its parameters.

      \param X1 : Point to consider in the derivative computation.
      \param X2 : Point to consider in the derivative computation.
      \param ParamM : Parameters of the warp.
      \param dwdp0 : Derivative matrix of the warp according to the warp parameters (p=0).
      \param dW : Resulting derivative matrix.
    */
    virtual void dWarpCompo(const vpColVector &X1,const vpColVector &X2,const vpColVector &ParamM,const double *dwdp0,vpMatrix &dW) = 0;

    /*!
      Find the displacement parameters from a list of points.

      \warning Only used in vpTemplateTrackerWarpHomographySL3.
    */
    void find_warp(const double *ut0,const double *vt0,const double *u,const double *v,int nb_pt,vpColVector& p);
    
    /*!
      Compute the derivative of the image according to the warp.

      \param i : i coordinate of the point to consider in the image.
      \param j : j coordinate of the point to consider in the image.
      \param dy : Derivative on the y-axis of the point (i,j).
      \param dx : Derivative on the x-axis of the point (i,j).
      \param dIdW : Resulting derivative matrix.
    */
    virtual void getdW0(const int &i,const int &j,const double &dy,const double &dx,double *dIdW) = 0;

    /*!
      Compute the derivative of the warp according to the parameters (p=0).

      \param i : i coordinate of the point to consider in the image.
      \param j : j coordinate of the point to consider in the image.
      \param dIdW : Resulting derivative matrix.
    */
    virtual void getdWdp0(const int &i,const int &j,double *dIdW) = 0;

    /*!
      Compute the distance between a zone and its associated warped zone.

      \param Z : Zone to consider.
      \param p : Parameters of the warp.
    */
    double getDistanceBetweenZoneAndWarpedZone(const vpTemplateTrackerZone &Z,const vpColVector &p);

    /*!
      Get the number of parameters of the warp.

      \return Number of parameters.
    */
    unsigned int getNbParam() const {return NbParam;}

    /*!
      Get the parameters of the warp one level down.

      \param p : Current parameters of the warp.
      \param pdown : Resulting parameters on level down.
    */
    virtual void getp_PyramidDown(const vpColVector &p,vpColVector &pdown) =0;

    /*!
      Get the parameters of the warp one level up.

      \param p : Current parameters of the warp.
      \param pup : Resulting parameters one level up.
    */
    virtual void getp_PyramidUp(const vpColVector &p,vpColVector &pup) =0;

    /*!
      Tells if the warp is ESM compatible.

      \return True if it is ESM compatible, False otherwise.
    */
    virtual bool isESMcompatible() const =0;

    /*!
      Get the inverse of the warp parameters.

      \param ParamM : Parameters of the warp.
      \param ParamMinv : Inverse parameters.
    */
    virtual void Param_inv(const vpColVector &ParamM,vpColVector &ParamMinv) = 0;

    /*!
      Get the displacement resulting from the composition of two other displacements.

      \param p1 : First displacement.
      \param p2 : Second displacement.
      \param pres : Displacement resulting from the composition of p1 and p2.
    */
    virtual void pRondp(const vpColVector &p1, const vpColVector &p2,vpColVector &pres) const = 0;

    /*!
      Set the number of parameters of the warp.

      \param nb : New number of parameters.
    */
    void setNbParam(unsigned int nb){NbParam=nb;dW.resize(2,NbParam);}

    /*!
      Warp a list of points.

      \param ut0 : List of u coordinates of the points.
      \param vt0 : List of v coordinates of the points.
      \param nb_pt : Number of points to consider.
      \param p : Parameters of the warp.
      \param u : Resulting u coordinates.
      \param v : resulting v coordinates.
    */
    void warp(const double *ut0,const double *vt0,int nb_pt,const vpColVector& p,double *u,double *v);

    /*!
      Warp a point.

      \param i : i coordinate of the point to warp.
      \param j : j coordinate of the point to warp.
      \param i2 : i coordinate of the warped point.
      \param j2 : j coordinate of the warped point.
      \param ParamM : Parameters of the warp.
    */
    virtual void warpX(const int &i, const int &j,double &i2,double &j2, const vpColVector &ParamM) = 0;

    /*!
      Warp a point.

      \param vX : Coordinates of the point to warp.
      \param vXres : Coordinates of the warped point.
      \param ParamM : Parameters of the warp.
    */
    virtual void warpX(const vpColVector &vX,vpColVector &vXres,const vpColVector &ParamM) = 0;

    /*!
      Inverse Warp a point.

      \param vX : Coordinates of the point to warp.
      \param vXres : Coordinates of the warped point.
      \param ParamM : Parameters of the warp.
    */
    virtual void warpXInv(const vpColVector &vX,vpColVector &vXres,const vpColVector &ParamM) = 0;

    /*!
      Warp a triangle and store the result in a new zone.

      \param in : Triangle to warp.
      \param p : Parameters of the warping function. These parameters are estimated by the template
      tracker and returned using vpTemplateTracker::getp().
      \param out : Resulting triangle.
    */
    void warpTriangle(const vpTemplateTrackerTriangle &in,const vpColVector &p, vpTemplateTrackerTriangle &out);

    /*!
      Warp a zone and store the result in a new zone.

      \param in : Zone to warp.
      \param p : Parameters of the warping function. These parameters are estimated by the template
      tracker and returned using vpTemplateTracker::getp().
      \param out : Resulting zone.
    */
    void warpZone(const vpTemplateTrackerZone &in,const vpColVector &p, vpTemplateTrackerZone &out);

};

#endif

