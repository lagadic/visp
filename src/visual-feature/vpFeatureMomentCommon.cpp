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
 *
 * Description:
 * Pre-filled pseudo-database used to handle dependencies between common moment features.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp/vpFeatureMomentCommon.h>
#include <visp/vpMomentDatabase.h>

/*!
  Constructor which initializes and links all common features in the database
  \param moments : database for moment primitives
  \param A : first plane coefficient for a plane equation of the following type Ax+By+C=1/Z
  \param B : second plane coefficient for a plane equation of the following type Ax+By+C=1/Z
  \param C : third plane coefficient for a plane equation of the following type Ax+By+C=1/Z  
*/
vpFeatureMomentCommon::vpFeatureMomentCommon(vpMomentDatabase& moments,double A,double B,double C) :
    featureGravity(moments,A,B,C),
    featureGravityNormalized(moments,A,B,C),        
    featureAn(moments,A,B,C),
    featureCInvariant(moments,A,B,C),
    featureAlpha(moments,A,B,C),
    featureCentered(moments,A,B,C),
    featureMomentBasic(moments,A,B,C),
	feature_moment_area(moments,A,B,C)

{
    featureGravity.linkTo(*this);
    featureGravityNormalized.linkTo(*this);
    featureAn.linkTo(*this);
    featureCInvariant.linkTo(*this);    
    featureAlpha.linkTo(*this);
    featureMomentBasic.linkTo(*this);
    featureCentered.linkTo(*this);
    feature_moment_area.linkTo(*this);
}

/*!
  Update all moment features in the database with plane coefficients  
  \param A : first plane coefficient for a plane equation of the following type Ax+By+C=1/Z
  \param B : second plane coefficient for a plane equation of the following type Ax+By+C=1/Z
  \param C : third plane coefficient for a plane equation of the following type Ax+By+C=1/Z  
*/
void vpFeatureMomentCommon::updateAll(double A,double B,double C){
    featureMomentBasic.update(A,B,C);
    featureGravity.update(A,B,C);
    featureCentered.update(A,B,C);
    featureAn.update(A,B,C);    
    featureGravityNormalized.update(A,B,C);
    featureCInvariant.update(A,B,C);    
    featureAlpha.update(A,B,C);
    feature_moment_area.update(A,B,C);
}

