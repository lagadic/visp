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
 \file vpTemplateTrackerWarpAffine.h
 \brief Affine warping function: w(X)=AX+b with: A=[[1+p0, p2], [p1, 1+p3]] and b= [p4, p5]]
*/


#ifndef vpTemplateTrackerWarpAffine_hh
#define vpTemplateTrackerWarpAffine_hh

#include <visp/vpTemplateTrackerWarp.h>


class VISP_EXPORT vpTemplateTrackerWarpAffine: public vpTemplateTrackerWarp
{
  public:
    //constructor;
    vpTemplateTrackerWarpAffine();
    
    void computeCoeff(const vpColVector &/*p*/){}
    void computeDenom(vpColVector &/*vX*/, const vpColVector &/*ParamM*/){}
    //fonctions renvoyant la matrice dw/dv au point i,j
    void dWarp(const vpColVector &X1,const vpColVector &X2,const vpColVector &ParamM,vpMatrix &dW);
    void dWarpCompo(const vpColVector &X1,const vpColVector &X2,const vpColVector &ParamM,const double *dwdp0,vpMatrix &dW);
    /*calcul de di*dw(x,p0)/dp pour compositionnel*/
    void getdW0(const int &i,const int &j,const double &dy,const double &dx,double *dIdW);
    void getdWdp0(const int &i,const int &j,double *dIdW);
    void getp_PyramidDown(const vpColVector &p,vpColVector &pdown);
    void getp_PyramidUp(const vpColVector &p,vpColVector &pup);
    bool isESMcompatible() const {return false;}
    //fonction renvoyant le resultat le vecteur de deplacement inverse au deplacement par ParamM
    void Param_inv(const vpColVector &ParamM,vpColVector &ParamMinv);
    //fonction renvoyant le vecteur deplacement resultant de la composition du deplacement p1 avec p2
    void pRondp(const vpColVector &p1, const vpColVector &p2,vpColVector &pres) const;
    //fonction renvoyant le resultat du deplacement de vX par le vecteur  ParamM
    void warpX(const vpColVector &vX,vpColVector &vXres,const vpColVector &ParamM);
    void warpX(const int &i,const int &j,double &i2,double &j2,const vpColVector &ParamM);
    //fonction renvoyant le resultat du deplacement inverse de vX par le vecteur  ParamM
    void warpXInv(const vpColVector &vX,vpColVector &vXres,const vpColVector &ParamM);
};
#endif
