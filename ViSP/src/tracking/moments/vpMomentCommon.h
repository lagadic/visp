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
 * Pre-filled moment database with all commonly used moments.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
  \file vpMomentCommon.h
  \brief Pre-filled moment database with all commonly used moments.
*/
#ifndef VPCOMMONMOMENTS_H
#define VPCOMMONMOMENTS_H
#include <vector>

#include <visp/vpConfig.h>
#include <visp/vpMomentDatabase.h>
#include <visp/vpMomentBasic.h>
#include <visp/vpMomentGravityCenter.h>
#include <visp/vpMomentCentered.h>
#include <visp/vpMomentGravityCenterNormalized.h>
#include <visp/vpMomentAreaNormalized.h>
#include <visp/vpMomentCInvariant.h>
#include <visp/vpMomentAlpha.h>

class vpMomentObject;

/*!
  \class vpMomentCommon

  \ingroup TrackingMoments

  \brief This class initializes and allows access to commonly used moments.

    It is a vpMomentDatabase filled with the following moments:
    - vpMomentBasic
    - vpMomentGravityCenter
    - vpMomentCentered
    - vpMomentCenteredNormalized
    - vpMomentAreaNormalized
    - vpMomentCInvariant
    - vpMomentAlpha

    There is no need to do the linkTo operations manually nor is it necessary to care about the order of moment computation.

    This class carries an vpMomentCommon::updateAll method capable of updating AND computing moments from an object (see 4-step process in vpMoment).
    The moments computed by this class are classical moments used in moment-based visual servoing.
    For more information see: 
    - [1] "Point-based and region-based image moments for visual servoing of planar objects" by Omar Tahri and Fran&ccedil;ois Chaumette.

    To initialize this moment set the user needs to compute the following things:
    - the Mu3 value set: set of third-order centered moments computed for a reference object. (\f$\mu_{ij}$ with $i+j = 3\f$ ).
    These values allow the system to save the reference angular position and to perform planar rotations of more than 180 degrees if needed.
    - the destination depth.
    - the surface of the destination object in the end of the visual servoing process.
    - the reference alpha: angular position of the object used to obtain the Mu3 set.

    Shortcuts for each of these prerequisites are provided by this class except depth (methods vpMomentCommon::getMu3, vpMomentCommon::getSurface,vpMomentCommon::getAlpha).

    \attention Make sure your object is at least of order 5 when using this pre-filled database.

*/
class VISP_EXPORT vpMomentCommon : public vpMomentDatabase{
private:
    vpMomentBasic momentBasic;
    vpMomentGravityCenter momentGravity;
    vpMomentCentered momentCentered;
    vpMomentGravityCenterNormalized momentGravityNormalized;
    vpMomentAreaNormalized momentSurfaceNormalized;
    vpMomentCInvariant momentCInvariant;
    vpMomentAlpha momentAlpha;

public:
    vpMomentCommon(double dstSurface,std::vector<double> ref,double refAlpha,double dstZ=1.0);

    static double getAlpha(vpMomentObject& objec);
    static std::vector<double> getMu3(vpMomentObject& object);
    static double getSurface(vpMomentObject& object);

    void updateAll(vpMomentObject& object);
};


#endif // VPCOMMONMOMENTS_H
