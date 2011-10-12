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
 *
 * Description:
 * Implementation for all supported moment features.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
#include <visp/vpMomentObject.h>
#ifdef VISP_MOMENTS_COMBINE_MATRICES
#include <visp/vpFeatureMomentCentered.h>
#include <visp/vpFeatureMomentBasic.h>
#include <visp/vpFeatureMomentGravityCenter.h>
#include <visp/vpMomentGravityCenter.h>
#include <visp/vpFeatureMomentDatabase.h>
#include <vector>
#include <limits>
/*!
  Default constructor
  \param moments : Database of moment primitives.
  \param A : First plane coefficient for a plane equation of the following type Ax+By+C=1/Z.
  \param B : Second plane coefficient for a plane equation of the following type Ax+By+C=1/Z.
  \param C : Third plane coefficient for a plane equation of the following type Ax+By+C=1/Z.
  \param FeatureMoments : Database of features.
*/
vpFeatureMomentCentered::vpFeatureMomentCentered(vpMomentDatabase& moments,double A, double B, double C,vpFeatureMomentDatabase* FeatureMoments) :
    vpFeatureMoment(moments,A,B,C,FeatureMoments)

{
}

/*!
Interaction matrix corresponding to \f$ \mu_{ij} \f$ moment
\param select_one : first index (i)
\param select_two : second index (j)
\return Interaction matrix corresponding to the moment
*/
vpMatrix 	vpFeatureMomentCentered::interaction (unsigned int select_one,unsigned int select_two){
    if(select_one+select_two>moment->getObject().getOrder()) throw vpException(vpException::badValue,"The requested value has not been computed, you should specify a higher order.");
    return interaction_matrices[select_two*order+select_one];
}


/*!
  Computes interaction matrix for centered moment. Called internally.
  The moment primitives must be computed before calling this.
  This feature depends on:
  - vpFeatureMomentBasic
  - vpFeatureMomentGravityCenter
  - vpMomentGravityCenter
*/
void vpFeatureMomentCentered::compute_interaction(){
    bool found_featuremoment_basic;
    bool found_feature_gravity_center;
    bool found_moment_gravity;

    vpMomentObject& momentObject = moment->getObject();
    order = momentObject.getOrder()+1;
    interaction_matrices.resize(order*order);
    for(std::vector< vpMatrix >::iterator i=interaction_matrices.begin();i!=interaction_matrices.end();i++)
        i->resize(1,6);

    vpFeatureMomentBasic& featureMomentBasic= (static_cast<vpFeatureMomentBasic&>(featureMoments->get("vpFeatureMomentBasic",found_featuremoment_basic)));
    vpFeatureMomentGravityCenter& featureMomentGravityCenter= (static_cast<vpFeatureMomentGravityCenter&>(featureMoments->get("vpFeatureMomentGravityCenter",found_feature_gravity_center)));
    vpMomentGravityCenter& momentGravity = static_cast<vpMomentGravityCenter&>(moments.get("vpMomentGravityCenter",found_moment_gravity));
    vpMatrix zeros(1,6);
    for(int i=0;i<6;i++) zeros[0][i]=0;

    if(!found_featuremoment_basic) throw vpException(vpException::notInitialized,"vpFeatureMomentBasic not found");
    if(!found_feature_gravity_center) throw vpException(vpException::notInitialized,"vpFeatureMomentGravityCenter not found");
    if(!found_moment_gravity) throw vpException(vpException::notInitialized,"vpMomentGravityCenter not found");



    vpMatrix LXg = featureMomentGravityCenter.interaction(1<<0);
    vpMatrix LYg = featureMomentGravityCenter.interaction(1<<1);

        //compute centered moment features as a combination of basic features
    for(int i=0;i<(int)order;i++){
        for(int j=0;j<(int)order-i;j++){
            interaction_matrices[(unsigned int)j*order+(unsigned int)i] = zeros;
            vpMatrix mat1 = zeros;
            vpMatrix mat2 = zeros;
            vpMatrix mat3 = zeros;
            for(int k=0;k<=i;k++){
                for(int l=0;l<=j;l++){
                    mat1+= std::abs(momentGravity.get()[0])<std::numeric_limits<double>::epsilon()?zeros:vpMath::comb((unsigned int)i, (unsigned int)k) * vpMath::comb((unsigned int)j, (unsigned int)l) * pow(-momentGravity.get()[0], i - k) * (i - k) * LXg / momentGravity.get()[0] * pow(-momentGravity.get()[1], j - l) * momentObject.get((unsigned int)k, (unsigned int)l);
                    mat2+= std::abs(momentGravity.get()[1])<std::numeric_limits<double>::epsilon()?zeros:vpMath::comb((unsigned int)i, (unsigned int)k) * vpMath::comb((unsigned int)j, (unsigned int)l) * pow(-momentGravity.get()[0], i - k) * pow(-momentGravity.get()[1], j - l) * (j - l) * LYg / momentGravity.get()[1] * momentObject.get((unsigned int)k, (unsigned int)l);
                    mat3+= vpMath::comb((unsigned int)i, (unsigned int)k) * vpMath::comb((unsigned int)j, (unsigned int)l) * pow(-momentGravity.get()[0], i - k) * pow(-momentGravity.get()[1], j - l) * featureMomentBasic.interaction((unsigned int)k, (unsigned int)l);
                }
            }
            interaction_matrices[(unsigned int)j*order+(unsigned int)i]=(mat1+mat2+mat3);
        }
    }
}

#else
#include <visp/vpMomentGravityCenter.h>
#include <visp/vpMomentCentered.h>
#include <visp/vpFeatureMomentCentered.h>

#include <visp/vpFeatureMomentDatabase.h>
#include <vector>
#include <limits>

#define VX 0
#define VY 1
#define VZ 2
#define WX 3
#define WY 4
#define WZ 5
#define MU(i,j) (momentCentered.get((unsigned int)i,(unsigned int)j))

/*!
  Default constructor
  \param moments : Database of moment primitives.
  \param A : First plane coefficient for a plane equation of the following type Ax+By+C=1/Z.
  \param B : Second plane coefficient for a plane equation of the following type Ax+By+C=1/Z.
  \param C : Third plane coefficient for a plane equation of the following type Ax+By+C=1/Z.
  \param FeatureMoments : Database of features.
*/
vpFeatureMomentCentered::vpFeatureMomentCentered(vpMomentDatabase& moments,double A, double B, double C,vpFeatureMomentDatabase* FeatureMoments) :
    vpFeatureMoment(moments,A,B,C,FeatureMoments)

{
}

/*!
Interaction matrix corresponding to \f$ \mu_{ij} \f$ moment
\param select_one : first index (i)
\param select_two : second index (j)
\return Interaction matrix corresponding to the moment
*/
vpMatrix 	vpFeatureMomentCentered::interaction (unsigned int select_one,unsigned int select_two){
    if(select_one+select_two>moment->getObject().getOrder()) throw vpException(vpException::badValue,"The requested value has not been computed, you should specify a higher order.");
    return interaction_matrices[select_two*order+select_one];
}


/*!
  Computes interaction matrix for centered moment. Called internally.
  The moment primitives must be computed before calling this.
  This feature depends on:
  - vpMomentGravityCenter
  - vpMomentCentered
*/
void vpFeatureMomentCentered::compute_interaction(){

    bool found_moment_centered;
    bool found_moment_gravity;



    vpMomentCentered& momentCentered= (static_cast<vpMomentCentered&>(moments.get("vpMomentCentered",found_moment_centered)));
    vpMomentGravityCenter& momentGravity = static_cast<vpMomentGravityCenter&>(moments.get("vpMomentGravityCenter",found_moment_gravity));

    if(!found_moment_centered) throw vpException(vpException::notInitialized,"vpMomentCentered not found");
    if(!found_moment_gravity) throw vpException(vpException::notInitialized,"vpMomentGravityCenter not found");

    int delta;
    int epsilon;
    vpMomentObject& momentObject = moment->getObject();
    order = momentObject.getOrder()+1;
    interaction_matrices.resize(order*order);
    for(std::vector< vpMatrix >::iterator i=interaction_matrices.begin();i!=interaction_matrices.end();i++)
        i->resize(1,6);
    if(momentObject.getType()==vpMomentObject::DISCRETE){
        delta=0;
        epsilon=1;
    }else{
        delta=1;
        epsilon=4;
    }
    double n11 = momentCentered.get(1,1)/momentObject.get(0,0);
    double n20 = momentCentered.get(2,0)/momentObject.get(0,0);
    double n02 = momentCentered.get(0,2)/momentObject.get(0,0);
    double Xg = momentGravity.getXg();
    double Yg = momentGravity.getYg();


    interaction_matrices[0][0][VX] = -(delta)*A*MU(0,0);
    interaction_matrices[0][0][VY] = -(delta)*B*MU(0,0);

    interaction_matrices[0][0][WX] = (3*delta)*MU(0,1)+(3*delta)*Yg*MU(0,0);
    interaction_matrices[0][0][WY] = -(3*delta)*MU(1,0)-(3*delta)*Xg*MU(0,0);
    interaction_matrices[0][0][VZ] = -A*interaction_matrices[0][0][WY]+B*interaction_matrices[0][0][WX]+(2*delta)*C*MU(0,0);
    interaction_matrices[0][0][WZ] = 0.;


    for(int i=1;i<(int)order-1;i++){
        interaction_matrices[(unsigned int)i][0][VX] = -(i+delta)*A*MU(i,0)-i*B*MU(i-1,1);
        interaction_matrices[(unsigned int)i][0][VY] = -(delta)*B*MU(i,0);

        interaction_matrices[(unsigned int)i][0][WX] = (i+3*delta)*MU(i,1)+(i+3*delta)*Yg*MU(i,0)+i*Xg*MU(i-1,1)-i*epsilon*n11*MU(i-1,0);
        interaction_matrices[(unsigned int)i][0][WY] = -(i+3*delta)*MU(i+1,0)-(2*i+3*delta)*Xg*MU(i,0)+i*epsilon*n20*MU(i-1,0);
        interaction_matrices[(unsigned int)i][0][VZ] = -A*interaction_matrices[(unsigned int)i][0][WY]+B*interaction_matrices[(unsigned int)i][0][WX]+(i+2*delta)*C*MU(i,0);
        interaction_matrices[(unsigned int)i][0][WZ] = i*MU(i-1,1);
    }

    for(int j=1;j<(int)order-1;j++){
        interaction_matrices[(unsigned int)j*order][0][VX] = -(delta)*A*MU(0,j);
        interaction_matrices[(unsigned int)j*order][0][VY] = -j*A*MU(1,j-1)-(j+delta)*B*MU(0,j);

        interaction_matrices[(unsigned int)j*order][0][WX] = (j+3*delta)*MU(0,j+1)+(2*j+3*delta)*Yg*MU(0,j)-j*epsilon*n02*MU(0,j-1);
        interaction_matrices[(unsigned int)j*order][0][WY] = -(j+3*delta)*MU(1,j)-(j+3*delta)*Xg*MU(0,j)-j*Yg*MU(1,j-1)+j*epsilon*n11*MU(0,j-1);
        interaction_matrices[(unsigned int)j*order][0][VZ] = -A*interaction_matrices[(unsigned int)j*order][0][WY]+B*interaction_matrices[(unsigned int)j*order][0][WX]+(j+2*delta)*C*MU(0,j);
        interaction_matrices[(unsigned int)j*order][0][WZ] = -j*MU(1,j-1);
    }

    for(int j=1;j<(int)order-1;j++){
        for(int i=1;i<(int)order-j-1;i++){
            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VX] = -(i+delta)*A*MU(i,j)-i*B*MU(i-1,j+1);
            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VY] = -j*A*MU(i+1,j-1)-(j+delta)*B*MU(i,j);

            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WX] = (i+j+3*delta)*MU(i,j+1)+(i+2*j+3*delta)*Yg*MU(i,j)
                                                        +i*Xg*MU(i-1,j+1)-i*epsilon*n11*MU(i-1,j)-j*epsilon*n02*MU(i,j-1);
            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WY] = -(i+j+3*delta)*MU(i+1,j)-(2*i+j+3*delta)*Xg*MU(i,j)
                                                        -j*Yg*MU(i+1,j-1)+i*epsilon*n20*MU(i-1,j)+j*epsilon*n11*MU(i,j-1);
            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VZ] = -A*interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WY]+B*interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WX]+(i+j+2*delta)*C*MU(i,j);
            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WZ] = i*MU(i-1,j+1)-j*MU(i+1,j-1);
        }
    }
}
#endif
