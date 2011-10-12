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
#include <visp/vpFeatureMomentBasic.h>
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
  Default constructor.
  \param moments : Database of moment primitives.
  \param A : First plane coefficient for a plane equation of the following type Ax+By+C=1/Z.
  \param B : Second plane coefficient for a plane equation of the following type Ax+By+C=1/Z.
  \param C : Third plane coefficient for a plane equation of the following type Ax+By+C=1/Z.
  \param FeatureMoments : Database of features.
*/
vpFeatureMomentBasic::vpFeatureMomentBasic(vpMomentDatabase& moments,double A, double B, double C,vpFeatureMomentDatabase* FeatureMoments) :
    vpFeatureMoment(moments,A,B,C,FeatureMoments)
{
}

/*!
  Computes interaction matrix for basic moment. Called internally.
  The moment primitives must be computed before calling this.
*/
void vpFeatureMomentBasic::compute_interaction(){
    int delta;
    vpMomentObject& momentObject = moment->getObject();
    order = momentObject.getOrder()+1;
    interaction_matrices.resize(order*order);
    for(std::vector< vpMatrix >::iterator i=interaction_matrices.begin();i!=interaction_matrices.end();i++)
        i->resize(1,6);
    if(momentObject.getType()==vpMomentObject::DISCRETE){
        delta=0;
    }else{
        delta=1;
    }

    //i=0;j=0
    interaction_matrices[0][0][VX] = -delta*A*momentObject.get(0, 0);
    interaction_matrices[0][0][VY] = -delta*B*momentObject.get(0, 0);
    interaction_matrices[0][0][VZ] = 3*delta*(A*momentObject.get(1, 0)+B*momentObject.get(0, 1)+C*momentObject.get(0, 0))-delta*C*momentObject.get(0, 0);

    interaction_matrices[0][0][WX] = 3*delta*momentObject.get(0, 1);
    interaction_matrices[0][0][WY] = -3*delta*momentObject.get(1, 0);
    interaction_matrices[0][0][WZ] = 0;

    int i=0;
    for(int j=1;j<(int)order-1;j++){
        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VX] = -delta*A*momentObject.get(0, (unsigned int)j);
        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VY] = -j*(A*momentObject.get(1, (unsigned int)(j-1))+B*momentObject.get(0, (unsigned int)j)+C*momentObject.get(0,(unsigned int)(j-1)))-delta*B*momentObject.get(0, (unsigned int)j);
        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VZ] = (j+3*delta)*(A*momentObject.get(1, (unsigned int)j)+B*momentObject.get(0, (unsigned int)(j+1))+C*momentObject.get(0, (unsigned int)j))-delta*C*momentObject.get(0, (unsigned int)j);

        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WX] = (j+3*delta)*momentObject.get(0,(unsigned int)(j+1))+j*momentObject.get(0, (unsigned int)(j-1));
        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WY] = -(j+3*delta)*momentObject.get(1, (unsigned int)j);
        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WZ] = -j*momentObject.get(1, (unsigned int)(j-1));
    }

    int j=0;
    for(int i=1;i<(int)order-1;i++){
        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VX] = -i*(A*momentObject.get((unsigned int)i, 0)+B*momentObject.get((unsigned int)(i-1), 1)+C*momentObject.get((unsigned int)(i-1), 0))-delta*A*momentObject.get((unsigned int)i, 0);
        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VY] = -delta*B*momentObject.get((unsigned int)i, 0);
        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VZ] = (i+3*delta)*(A*momentObject.get((unsigned int)(i+1), 0)+B*momentObject.get((unsigned int)i, 1)+C*momentObject.get((unsigned int)i, 0))-delta*C*momentObject.get((unsigned int)i, 0);

        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WX] = (i+3*delta)*momentObject.get((unsigned int)i, 1);
        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WY] = -(i+3*delta)*momentObject.get((unsigned int)(i+1), 0)-i*momentObject.get((unsigned int)(i-1), 0);
        interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WZ] = i*momentObject.get((unsigned int)(i-1), 1);
    }

    for(int j=1;j<(int)order-1;j++){
        for(int i=1;i<(int)order-j-1;i++){
            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VX] = -i*(A*momentObject.get((unsigned int)i, (unsigned int)j)+B*momentObject.get((unsigned int)(i-1), (unsigned int)(j+1))+C*momentObject.get((unsigned int)(i-1), (unsigned int)j))-delta*A*momentObject.get((unsigned int)i, (unsigned int)j);
            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VY] = -j*(A*momentObject.get((unsigned int)(i+1), (unsigned int)(j-1))+B*momentObject.get((unsigned int)i, (unsigned int)j)+C*momentObject.get((unsigned int)i, (unsigned int)(j-1)))-delta*B*momentObject.get((unsigned int)i, (unsigned int)j);
            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][VZ] = (i+j+3*delta)*(A*momentObject.get((unsigned int)(i+1), (unsigned int)j)+B*momentObject.get((unsigned int)(i), (unsigned int)(j+1))+C*momentObject.get((unsigned int)i, (unsigned int)j))-delta*C*momentObject.get((unsigned int)i, (unsigned int)j);

            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WX] = (i+j+3*delta)*momentObject.get((unsigned int)i, (unsigned int)(j+1))+j*momentObject.get((unsigned int)i, (unsigned int)(j-1));
            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WY] = -(i+j+3*delta)*momentObject.get((unsigned int)(i+1), (unsigned int)j)-i*momentObject.get((unsigned int)(i-1), (unsigned int)j);
            interaction_matrices[(unsigned int)j*order+(unsigned int)i][0][WZ] = i*momentObject.get((unsigned int)(i-1), (unsigned int)(j+1))-j*momentObject.get((unsigned int)(i+1), (unsigned int)(j-1));
        }
    }
}

/*!
Interaction matrix corresponding to \f$ m_{ij} \f$ moment.
\param select_one : first index (i).
\param select_two : second index (j).
\return Interaction matrix \f$ L_{m_{ij}} \f$ corresponding to the moment.
*/
vpMatrix vpFeatureMomentBasic::interaction (unsigned int select_one,unsigned int select_two){
    if(select_one+select_two>moment->getObject().getOrder()) throw vpException(vpException::badValue,"The requested value has not been computed, you should specify a higher order.");
    return interaction_matrices[select_two*order+select_one];
}
