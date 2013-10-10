/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
    vpTemplateTrackerWarp(){}
    virtual ~vpTemplateTrackerWarp(){}

    virtual void computeCoeff(const vpColVector &p)=0;
    virtual void computeDenom(vpColVector &vX, const vpColVector &ParamM)=0;
    
    // dérivée du warp par rapport aux paramètres
    virtual void dWarp(const vpColVector &X1,const vpColVector &X2,const vpColVector &ParamM,vpMatrix &dW) = 0;
    // Similaire à dWarp en compositionnel
    virtual void dWarpCompo(const vpColVector &X1,const vpColVector &X2,const vpColVector &ParamM,const double *dwdp0,vpMatrix &dW) = 0;

    //retrouve le parametre de deplacement a partir d'une liste de point
    void find_warp(const double *ut0,const double *vt0,const double *u,const double *v,int nb_pt,vpColVector& p);
    
    // dérivée de l'image par rapport au warp
    virtual void getdW0(const int &i,const int &j,const double &dy,const double &dx,double *dIdW) = 0;
    // dérivée du warp par rapport aux paramètres avec param = 0
    virtual void getdWdp0(const int &i,const int &j,double *dIdW) = 0;
    //calcule la somme des distances entre les points d'origines et points warpes
    double getDistanceBetweenZoneAndWarpedZone(const vpTemplateTrackerZone &Z,const vpColVector &p);
    //retourne nombre de parametre du deplacement
    unsigned int getNbParam() const {return NbParam;}
    virtual void getp_PyramidDown(const vpColVector &p,vpColVector &pdown) =0;
    virtual void getp_PyramidUp(const vpColVector &p,vpColVector &pup) =0;
    virtual bool isESMcompatible() const =0;
    virtual void Param_inv(const vpColVector &ParamM,vpColVector &ParamMinv) = 0;
    //fonction renvoyant le vecteur deplacement resultant de la composition du deplacement p1 avec p2
    virtual void pRondp(const vpColVector &p1, const vpColVector &p2,vpColVector &pres) const = 0;

    void setNbParam(unsigned int nb){NbParam=nb;dW.resize(2,NbParam);}

    //Deplacement liste de point
    void warp(const double *ut0,const double *vt0,int nb_pt,const vpColVector& p,double *u,double *v);
    virtual void warpX(const int &i, const int &j,double &i2,double &j2, const vpColVector &ParamM) = 0;
    virtual void warpX(const vpColVector &vX,vpColVector &vXres,const vpColVector &ParamM) = 0;
    //fonction renvoyant le resultat du deplacement inverse de vX par le vecteur  ParamM
    virtual void warpXInv(const vpColVector &vX,vpColVector &vXres,const vpColVector &ParamM) = 0;
    //transforme triangle TR en TT
    void warpTriangle(const vpTemplateTrackerTriangle &TR,const vpColVector &p, vpTemplateTrackerTriangle &TT);
    //transforme zone ZR en ZT
    void warpZone(const vpTemplateTrackerZone &ZR,const vpColVector &p, vpTemplateTrackerZone &ZT);
    //warp les sommets d'une zone pour affichage
    void warpZone(const vpTemplateTrackerZone &Z,const vpColVector &p);

    static void get_p_by_click(const vpImage<unsigned char> &I0,const vpImage<unsigned char> &I,vpTemplateTrackerWarp *warp,vpColVector &p,int nb_pt);
    static void get_p_by_click(const vpImage<vpRGBa> &I0,const vpImage<unsigned char> &I,vpTemplateTrackerWarp *warp,vpColVector &p,int nb_pt);
};

#endif

