/****************************************************************************
 *
 * $Id: vpFeatureMomentImpl.h 3317 2011-09-06 14:14:47Z fnovotny $
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
 *
 * Description:
 * Implementation for all supported moment features.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
  \file vpFeatureMomentBasic.h
  \brief Implementation of the interaction matrix computation for vpMomentBasic.
*/

#ifndef __FEATUREMOMENTBASIC_H__
#define __FEATUREMOMENTBASIC_H__
#include <visp/vpFeatureMoment.h>
#include <visp/vpMomentCommon.h>
#include <visp/vpMomentObject.h>
class vpMomentDatabase;
/*!
  \class vpFeatureMomentBasic

  \ingroup VsFeature2

  \brief Functionality computation for basic moment feature. Computes the interaction matrix associated with vpMomentBasic.

    The interaction matrix for the basic moment feature is defined in \cite Tahri05z, equation (13).
    This vpFeatureMoment, as well as it's corresponding moment primitive is double-indexed.
    The interaction matrix \f$ L_{m_{ij}} \f$ is obtained by calling vpFeatureMomentBasic::interaction (i,j) and is associated to \f$ m_{ij} \f$ obtained by vpMomentBasic::get (i,j).
    vpFeatureMomentBasic computes interaction matrices all interaction matrices up to vpMomentObject::getOrder()-1.
    \attention The maximum order reached by vpFeatureMomentBasic is NOT the maximum order of the vpMomentObject, it is one unit smaller.
    For example if you define your vpMomentObject up to order n then vpFeatureMomentBasic will be able to compute interaction matrices up to order n-1 that is
    \f$ L_{m_{ij}} \f$ with \f$ i+j<=n-1 \f$.

    You can see an example of vpFeatureMomentBasic by looking at the documentation of the vpFeatureMoment class.

    This feature depends on:
        - vpMomentBasic

*/
class VISP_EXPORT vpFeatureMomentBasic : public vpFeatureMoment{
protected:
    unsigned int order;
 public:
        vpFeatureMomentBasic(vpMomentDatabase& moments,double A, double B, double C,vpFeatureMomentDatabase* featureMoments=NULL);
        void compute_interaction();

#ifndef DOXYGEN_SHOULD_SKIP_THIS
        /* Add function due to pure virtual definition in vpBasicFeature.h */
        vpMatrix interaction(const unsigned int /* select = FEATURE_ALL */){
          throw vpException(vpException::functionNotImplementedError,"Not implemented!");
        }
#endif

        vpMatrix interaction (unsigned int select_one,unsigned int select_two) const;
        /*!
          Associated moment name.
          */
        const char* momentName() const { return "vpMomentBasic";}
        /*!
          Feature name.
          */
        const char* name() const { return "vpFeatureMomentBasic";}

};
#endif
