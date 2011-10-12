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
 * Descriptor for moments centered and normalized.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
  \file vpMomentCenteredNormalized.h
  \brief Descriptor for moments centered and normalized (also called n).
*/
#ifndef __MOMENTCENTEREDNORMALIZED_H__
#define __MOMENTCENTEREDNORMALIZED_H__
#include <vector>
#include <visp/vpConfig.h>
#include <visp/vpMomentCentered.h>
#include <visp/vpMomentDatabase.h>
class vpMomentObject;

/*!
  \class vpMomentCenteredNormalized

  \ingroup TrackingMoments

  \brief This class defines the double-indexed centered and normalized moment descriptor.

  Centered and normalized moments are defined as follows: \f$\eta_{ij}= \frac{\mu_{ij}}{a}\f$ where \f$\mu_{ij}\f$ is a centered moment and \f$a\f$ is the surface.
  The centered and normalized moments are computed at the highest possible order.
  For example if the vpMomentObject is defined up to order 5, vpMomentCenteredNormalized will be also defined up to order 5.

  vpMomentCenteredNormalized depends on vpMomentCentered.

*/
class VISP_EXPORT vpMomentCenteredNormalized : public vpMomentCentered {
 public:	
        vpMomentCenteredNormalized();

	void compute();
        /*!
          Moment name
          */
	const char* name(){return "vpMomentCenteredNormalized";}
};

#endif
