/****************************************************************************
 *
 * $Id$
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
 * Base for all moment features
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp/vpFeatureMoment.h>
#include <visp/vpMoment.h>
#include <visp/vpFeatureMomentDatabase.h>
#include <visp/vpMomentDatabase.h>
#include <visp/vpMath.h>

#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpFeatureException.h>

#include <visp/vpDebug.h>
#include <vector>

class vpBasicFeature;

/*!
  Initialize common parameters for moment features.
*/
void vpFeatureMoment::init (){
//feature dimension
    if(this->moment!=NULL)
        dim_s = this->moment->get().size();
    else
        dim_s = 0;

    nbParameters = 1;
    
    // memory allocation    
    s.resize(dim_s) ;
    for(unsigned int i=0;i<dim_s;i++)
        s[i] = 0;

    if (flags == NULL)
	flags = new bool[nbParameters];
    for (int i = 0; i < nbParameters; i++) 
	flags[i] = false;
}


/*!
  Feature's dimension according to selection.
*/
int vpFeatureMoment::getDimension (unsigned int select) const{
    int dim=0;

    for(unsigned int i=0;i<dim_s;++i)
        if(vpBasicFeature::FEATURE_LINE[i] & select)
            dim++;

    return dim;
}


/*!
  Error between two moment features. The error is only computed on selected parts of the feature.
*/
vpColVector vpFeatureMoment::error (const vpBasicFeature &s_star, unsigned int select) const{
    vpColVector e(0),stateLine(1);

    for(unsigned int i=0;i<dim_s;++i){
        if(vpBasicFeature::FEATURE_LINE[i] & select){
            stateLine[0] = s_star.get_s()[i]-s[i];
            e.stackMatrices(stateLine);
        }
    }

    return e;
}

/*!
  Outputs the content of the feature: it's corresponding selected moments.
*/
void vpFeatureMoment::print (unsigned int select) const{
    for(unsigned int i=0;i<dim_s;++i){
        if(vpBasicFeature::FEATURE_LINE[i] & select){
            std::cout << s[i] << ",";
        }
    }

    std::cout << std::endl;
}

/*!
  Not implemented since visual representation of a moment doesn't often make sense.
*/
void vpFeatureMoment::display (const vpCameraParameters &cam, vpImage< unsigned char > &I, vpColor color, unsigned int thickness) const{
	//visual representation of a moment doesn't often make sense
    (void)cam;
    (void)I;
    (void)color;
    (void)thickness;
}

/*!
  Not implemented since visual representation of a moment doesn't often make sense.
*/    
void vpFeatureMoment::display (const vpCameraParameters &cam, vpImage< vpRGBa > &I, vpColor color, unsigned int thickness) const    {
    (void)cam;
    (void)I;
    (void)color;
    (void)thickness;
}

/*!
  Updates the interaction matrices with the image plane the camera is facing. The plane must be in the format: \f$ \frac{1}{Z}=Ax+By+C \f$ .
  The moment primitives MUST be updated before calling this function.

  This method also computes the interaction matrix. Therefore, you must call vpFeatureMoment::update before calling vpFeatureMoment::interaction.

  \attention The behaviour of this method is not the same as vpMoment::update which only acknowledges the new object. This method also computes the interaction matrices.

  \param A : A coefficient of the plane.
  \param B : B coefficient of the plane.
  \param C : C coefficient of the plane.
*/
void vpFeatureMoment::update (double A, double B, double C){
    this->A = A;
    this->B = B;
    this->C = C;

    if(moment==NULL){
        bool found;        
        this->moment = &(moments.get(momentName(),found));
        if(!found) throw ("Moment not found for feature");

    }
    nbParameters = 1;
    if(this->moment!=NULL){
        dim_s = this->moment->get().size();

        s.resize(dim_s);

        for(unsigned int i=0;i<dim_s;i++)
            s[i] = this->moment->get()[i];

        if (flags == NULL)
            flags = new bool[nbParameters];
        for (int i = 0; i < nbParameters; i++)
            flags[i] = false;
    }else
        dim_s = 0;

    compute_interaction();
}

/*!
  Retrieves the interaction matrix. No computation is done.
  
  \param select : Feature selector. 
  
  \return The corresponding interaction matrix.

  There is no rule about the format of the feature selector. It may be
  different for different features.  For example, for
  vpFeatureMomentBasic or vpFeatureMomentCentered features, select may
  refer to the \f$ (i,j) \f$ couple in the \f$ j \times order + i \f$
  format, but for vpFeatureMomentCInvariant the selector allows to
  select couples \f$ (i,j,k,l...) \f$ in the following format: 1 << i
  + 1 << j + 1 << k + 1 << l.
*/
vpMatrix vpFeatureMoment::interaction (unsigned int select){
    vpMatrix L(0,0);

    for(unsigned int i=0;i<dim_s;++i){
        if(vpBasicFeature::FEATURE_LINE[i] & select){
            L.stackMatrices(interaction_matrices[i]);
        }
    }

    return L;
}

/*!  Duplicates the feature into a vpGenericFeature harbouring the
  same properties.  The resulting feature is of vpMomentGenericFeature
  type. While it still can compute interaction matrices and has acces
  to it's moment primitive, it has lost all precise information about
  its precise type and therefore cannot be used in a feature database.
  
  \return The corresponding feature.
*/
vpBasicFeature* vpFeatureMoment::duplicate () const {
    vpFeatureMoment* feat = new vpMomentGenericFeature(moments,A,B,C,featureMoments,moment);
    feat->interaction_matrices = interaction_matrices;
    feat->dim_s = dim_s;
    feat->nbParameters = nbParameters;    
    // memory allocation
    feat->s.resize(dim_s) ;
    for(unsigned int i=0;i<dim_s;i++)
        feat->s[i] = this->s[i];

    feat->flags = new bool[(unsigned int)nbParameters];
    for (unsigned int i = 0; i < (unsigned int)nbParameters; i++)
        feat->flags[i] = flags[i];

    return feat;
}

/*!
  Links the feature to the feature's database. NB: The feature's database is different from the moment's database.
  \param FeatureMoments : database in which the moment features are stored.
  
*/
void vpFeatureMoment::linkTo(vpFeatureMomentDatabase& FeatureMoments){
    std::strcpy(_name,name());
    this->featureMoments=&FeatureMoments;

    FeatureMoments.add(*this,_name);
}


void vpFeatureMoment::compute_interaction (){

}

vpFeatureMoment::~vpFeatureMoment (){

}
