/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#include <visp3/core/vpMomentAlpha.h>
#include <visp3/core/vpMomentArea.h>
#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentCInvariant.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentGravityCenterNormalized.h>

#include <vector>

class vpMomentObject;

/*!
  \class vpMomentCommon

  \ingroup group_core_moments

  \brief This class initializes and allows access to commonly used moments.

    It is a vpMomentDatabase filled with the following moments:
    - vpMomentBasic
    - vpMomentGravityCenter
    - vpMomentCentered
    - vpMomentGravityCenterNormalized
    - vpMomentAreaNormalized
    - vpMomentCInvariant
    - vpMomentAlpha
    - vpMomentArea

    There is no need to do the linkTo operations manually nor is it necessary
  to care about the order of moment computation.

    This class carries an vpMomentCommon::updateAll() method capable of
  updating AND computing moments from an object (see 4-step process in
  vpMoment). The moments computed by this class are classical moments used in
  moment-based visual servoing. For more information see \cite Tahri05z.

    To initialize this moment set the user needs to compute the following
  things:
    - the Mu3 value set: set of third-order centered moments computed for a
  reference object. (\f$\mu_{ij}$ with $i+j = 3\f$ ). These values allow the
  system to save the reference angular position and to perform planar
  rotations of more than 180 degrees if needed.
    - the destination depth.
    - the surface of the destination object in the end of the visual servoing
  process.
    - the reference alpha: angular position of the object used to obtain the
  Mu3 set.

    Shortcuts for each of these prerequisites are provided by this class
  except depth (methods vpMomentCommon::getMu3(),
  vpMomentCommon::getSurface(), vpMomentCommon::getAlpha()).

    \attention Make sure your object is at least of order 5 when using this
  pre-filled database.

*/
class VISP_EXPORT vpMomentCommon : public vpMomentDatabase
{
private:
  vpMomentBasic momentBasic;
  vpMomentGravityCenter momentGravity;
  vpMomentCentered momentCentered;
  vpMomentGravityCenterNormalized momentGravityNormalized;
  vpMomentAreaNormalized momentSurfaceNormalized;
  vpMomentCInvariant *momentCInvariant;
  vpMomentAlpha momentAlpha;
  vpMomentArea momentArea;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  vpMomentCommon(const vpMomentCommon &)
  //    : vpMomentDatabase(), momentBasic(), momentGravity(),
  //    momentCentered(), momentGravityNormalized(),
  //      momentSurfaceNormalized(0, 0), momentCInvariant(), momentAlpha(),
  //      momentArea()
  //  {
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!");
  //  }
  //  vpMomentCommon &operator=(const vpMomentCommon &){
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!"); return *this;
  //  }
  //#endif

public:
  vpMomentCommon(double dstSurface, const std::vector<double> &ref, double refAlpha, double dstZ = 1.0,
                 bool flg_sxsyfromnormalized = false);
  virtual ~vpMomentCommon();

  static double getAlpha(vpMomentObject &object);
  static std::vector<double> getMu3(vpMomentObject &object);
  static double getSurface(vpMomentObject &object);

  void updateAll(vpMomentObject &object);
};
#endif // VPCOMMONMOMENTS_H
