/****************************************************************************
 *
 * $Id: vpFeatureMomentImpl.cpp 3317 2011-09-06 14:14:47Z fnovotny $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * INRIA Rennes - Brngne Atlantique
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
 *
 * Description:
 * Implementation for all supported moment features.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
#include <visp/vpMomentObject.h>
#include <visp/vpMomentCentered.h>
#ifdef VISP_MOMENTS_COMBINE_MATRICES
#include <visp/vpFeatureMomentGravityCenterNormalized.h>
#include <visp/vpMomentAreaNormalized.h>
#include <visp/vpMomentGravityCenter.h>
#include <visp/vpFeatureMomentGravityCenter.h>
#include <visp/vpFeatureMomentAreaNormalized.h>

#include <visp/vpFeatureMomentDatabase.h>
#include <vector>
#include <limits>

/*!
  Computes interaction matrix for centered and normalized moment. Called internally.
  The moment primitives must be computed before calling this.
  This feature depends on:
  - vpFeatureMomentGravityCenter
  - vpMomentGravityCenter
  - vpMomentAreaNormalized
  - vpFeatureMomentAreaNormalized
*/
void vpFeatureMomentGravityCenterNormalized::compute_interaction(){
    bool found_moment_gravity;
    bool found_moment_surface_normalized;

    bool found_featuremoment_gravity;
    bool found_featuremoment_surfacenormalized;

    vpMomentAreaNormalized& momentSurfaceNormalized = static_cast<vpMomentAreaNormalized&>(moments.get("vpMomentAreaNormalized",found_moment_surface_normalized));
    vpMomentGravityCenter& momentGravity = static_cast<vpMomentGravityCenter&>(moments.get("vpMomentGravityCenter",found_moment_gravity));
    vpFeatureMomentGravityCenter& featureMomentGravity = (static_cast<vpFeatureMomentGravityCenter&>(featureMoments->get("vpFeatureMomentGravityCenter",found_featuremoment_gravity)));
    vpFeatureMomentAreaNormalized featureMomentAreaNormalized = (static_cast<vpFeatureMomentAreaNormalized&>(featureMoments->get("vpFeatureMomentAreaNormalized",found_featuremoment_surfacenormalized)));

    if(!found_moment_surface_normalized) throw vpException(vpException::notInitialized,"vpMomentAreaNormalized not found");
    if(!found_moment_gravity) throw vpException(vpException::notInitialized,"vpMomentGravityCenter not found");

    if(!found_featuremoment_gravity) throw vpException(vpException::notInitialized,"vpFeatureMomentGravityCenter not found");
    if(!found_featuremoment_surfacenormalized) throw vpException(vpException::notInitialized,"vpFeatureMomentAreaNormalized not found");


    interaction_matrices[0].resize(1,6);
    interaction_matrices[1].resize(1,6);

    interaction_matrices[0] = momentGravity.get()[0]*featureMomentAreaNormalized.interaction(1) + momentSurfaceNormalized.get()[0]*featureMomentGravity.interaction(1);
    interaction_matrices[1] = momentGravity.get()[1]*featureMomentAreaNormalized.interaction(1) + momentSurfaceNormalized.get()[0]*featureMomentGravity.interaction(2);
}

#else
#include <visp/vpFeatureMomentGravityCenterNormalized.h>
#include <visp/vpMomentAreaNormalized.h>
#include <visp/vpMomentGravityCenter.h>

#include <visp/vpFeatureMomentDatabase.h>
#include <vector>
#include <limits>
#define VX 0
#define VY 1
#define VZ 2
#define WX 3
#define WY 4
#define WZ 5
/*!
  Computes interaction matrix for centered and normalized moment. Called internally.
  The moment primitives must be computed before calling this.
  This feature depends on:
  - vpMomentCentered
  - vpMomentAreaNormalized
  - vpMomentGravityCenter
*/
void vpFeatureMomentGravityCenterNormalized::compute_interaction(){

    bool found_moment_surface_normalized;
    bool found_moment_gravity;
    bool found_moment_centered;

    vpMomentCentered& momentCentered = static_cast<vpMomentCentered&>(moments.get("vpMomentCentered",found_moment_centered));
    vpMomentGravityCenter& momentGravity = static_cast<vpMomentGravityCenter&>(moments.get("vpMomentGravityCenter",found_moment_gravity));
    vpMomentAreaNormalized& momentSurfaceNormalized = static_cast<vpMomentAreaNormalized&>(moments.get("vpMomentAreaNormalized",found_moment_surface_normalized));

    if(!found_moment_surface_normalized) throw vpException(vpException::notInitialized,"vpMomentAreaNormalized not found");
    if(!found_moment_gravity) throw vpException(vpException::notInitialized,"vpMomentGravityCenter not found");
    if(!found_moment_centered) throw vpException(vpException::notInitialized,"vpMomentCentered not found");

    vpMomentObject& momentObject = moment->getObject();
    interaction_matrices[0].resize(1,6);
    interaction_matrices[1].resize(1,6);


    double n11 = momentCentered.get(1,1)/momentObject.get(0,0);
    double n20 = momentCentered.get(2,0)/momentObject.get(0,0);
    double n02 = momentCentered.get(0,2)/momentObject.get(0,0);
    double n10 = momentCentered.get(1,0)/momentObject.get(0,0);
    double n01 = momentCentered.get(0,1)/momentObject.get(0,0);
    double n03 = momentCentered.get(0,3)/momentObject.get(0,0);


    double Xg = momentGravity.getXg();
    double Yg = momentGravity.getYg();

    double An=momentSurfaceNormalized.get()[0];

    double Xn = An*Xg;
    double Yn = An*Yg;


    double Xnvx,Xnvy,Xnvz,Xnwx,Xnwy;
    double Ynvx,Ynvy,Ynvz,Ynwx,Ynwy;

    if(momentObject.getType()==vpMomentObject::DISCRETE){
        double a = momentCentered.get(2,0)+momentCentered.get(0,2);

        double e11 = momentCentered.get(1,1)/a;
        double e12 = momentCentered.get(1,2)/a;
        double e21 = momentCentered.get(2,1)/a;
        double e30 = momentCentered.get(3,0)/a;

        double NA = n20+n02;


        Xnvx = B*Xn*e11-Yn*B-An*C-n02*A*Xn/NA;
        Xnvy = A*Xn*e11+n02*B*Xn/NA;

        Xnwx = An*e11*NA+Yn*n10-Xn*Xg*e11+Xn*n01+Xn*n10*e11-Xn*e21+(-Xn*n03+(Xn*n01-Yn*Xg)*n02)/NA;
        Xnwy = -An*NA+Xn*e12+Xn*Xg-An+e11*Xg*Yn-Xn*n01*e11-2*Xn*n10+Xn*e30+n02*An+(-Xn*Xg+Xn*n10)*n02/NA;



        Ynvx = (Yn-n02*Yn/NA)*A+Yn*e11*B;
        Ynvy = (-Xn+e11*Yn)*A+(-Yn+n02*Yn/NA)*B-An*C;

        Ynwx = n02*An+Yn*n10*e11-e11*Xg*Yn+An-Yn*e21+Yn*n01+(-Yn*n03+(Yn*n01-Yn*Yg)*n02)/NA;
        Ynwy = -An*e11*NA+Yn*e11*Yg-Yn*n01*e11+Yn*Xg+Yn*e12+Yn*e30-Xn*n01-2*Yn*n10+(Yn*n10-Yn*Xg)*n02/NA;


    }else{
        Xnvx = -An*C-A*Xn-Yn*B;
        Xnvy = (0.5)*B*Xn;

        Xnwx = (0.5*(8.*n10-Xg))*Yn+4.*An*n11+4.*n01*Xn;
        Xnwy = (0.5*(-2.-8.*n20))*An+(0.5)*Xn*(-8.*n10+Xg);

        Ynvx = (0.5)*A*Yn;
        Ynvy = -(0.5)*B*Yn-C*An-A*Xn;

        Ynwx = (0.5)*Yn*(8.*n01-Yg)+(.5*(2.+8.*n02))*An;
        Ynwy = (0.5*(-8.*n10+Xg))*Yn-4.*An*n11-4.*n01*Xn;

    }

    Ynvz = -A*Ynwy+(-An+Ynwx)*B;
    Xnvz = -A*An-A*Xnwy+B*Xnwx;

    interaction_matrices[0][0][VX] = Xnvx;
    interaction_matrices[0][0][VY] = Xnvy;
    interaction_matrices[0][0][VZ] = Xnvz;

    interaction_matrices[0][0][WX] = Xnwx;
    interaction_matrices[0][0][WY] = Xnwy;
    interaction_matrices[0][0][WZ] = Yn;

    interaction_matrices[1][0][VX] = Ynvx;
    interaction_matrices[1][0][VY] = Ynvy;
    interaction_matrices[1][0][VZ] = Ynvz;

    interaction_matrices[1][0][WX] = Ynwx;
    interaction_matrices[1][0][WY] = Ynwy;
    interaction_matrices[1][0][WZ] = -Xn;

}
#endif
