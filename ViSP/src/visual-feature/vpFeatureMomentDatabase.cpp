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
 * Pseudo-database used to handle dependencies between moment features.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp/vpFeatureMomentDatabase.h>
#include <visp/vpFeatureMoment.h>
#include <typeinfo>
#include <iostream>

/*!
  Add a moment and it's corresponding name to the database
  \param FeatureMoment : database for moment features
  \param name : the feature's name, usually the string naming it's class. Each name must be unique
*/
void vpFeatureMomentDatabase::add(vpFeatureMoment& FeatureMoment,char* name){
    FeatureMoments.insert(std::pair<const char*,vpFeatureMoment*>((const char*)name,&FeatureMoment));
}

/*!
  Retrieves a moment feature from the database
  \param type : the name of the feature, the one specified when using add
  \param found : true if the type string is found inside the database, false otherwise

  \return the moment feature corresponding to the type string
*/
vpFeatureMoment& vpFeatureMomentDatabase::get(const char* type, bool& found){
  std::map<const char*,vpFeatureMoment*,vpFeatureMomentDatabase::cmp_str>::const_iterator it = FeatureMoments.find(type);

    found = (it!=FeatureMoments.end());
    return *(it->second);
}

/*!
  Update all moment features in the database with plane coefficients  
  \param A : first plane coefficient for a plane equation of the following type Ax+By+C=1/Z
  \param B : second plane coefficient for a plane equation of the following type Ax+By+C=1/Z
  \param C : third plane coefficient for a plane equation of the following type Ax+By+C=1/Z  
*/
void vpFeatureMomentDatabase::updateAll(double A, double B, double C){
  std::map<const char*,vpFeatureMoment*,vpFeatureMomentDatabase::cmp_str>::const_iterator itr;
    for(itr = FeatureMoments.begin(); itr != FeatureMoments.end(); itr++){
        (*itr).second->update(A,B,C);
    }
}

/*
std::ostream & operator<<(std::ostream & os, const vpFeatureMomentDatabase& m){
    std::map<const char*,vpMoment*,vpFeatureMomentDatabase::cmp_str>::const_iterator itr;
    os << "{";

    for(itr = m.FeatureMoments.begin(); itr != m.FeatureMoments.end(); itr++){
        os << (*itr).first << ": [" << *((*itr).second) << "],";
    }
    os << "}";

    return os;
}*/
