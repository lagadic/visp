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
 * Base for 2D moment descriptor
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

/*!
  \file vpMoment.h
  \brief Base class for all 2D moments.
*/

#ifndef __MOMENT_H__
#define __MOMENT_H__
#include <vector>
#include <iostream>
#include <visp/vpConfig.h>

class vpMomentDatabase;
class vpMomentObject;


/*!
  \class vpMoment

  \ingroup TrackingMoments

  \brief This class defines shared methods/attributes for 2D moments.

  All moments or combination of moments in the moments module are based on this class.
  A moment uses a vpMomentObject object to access all useful information.
  Moment values are obtained by a 4-step process common for all moment types:
  - Declaration.
  \code
  vpMoment moment;
  \endcode
  - Update with object.
  \code
  moment.update(object);
  \endcode
  - Compute the moment value
  \code
  moment.compute();
  \endcode
  - Access the values:
  \code
  std::vector<double> values = moment.get();
  \endcode

  A moment may also be linked to a vpMomentDatabase. Moments linked to a database are able to access each others values.
  Some moments can be computed only if they are linked to a a database containing their dependencies.
  Linking to a database is done using the vpMoment::linkTo(...) method.

  There are no constraints about format of the array returned by vpMoment::get: any implementation is fine.

  Each moment must have a string name by implementing the char* vpMoment::name() method which allows to identify the moment in the database.
  Each moment must also implement a compute method describing how to obtain its values from the object.

  \attention Order of moment computation DOES matter: when you compute (vpMoment::compute call) a moment, all moment dependencies must be computed.
  Moments pre-implementes dans ViSP:
  - vpMomentAlpha
  - vpMomentBasic
  - vpMomentCentered
  - vpMomentCInvariant
  - vpMomentSInvariant
  - vpMomentCenteredNormalized
  - vpMomentAreaNormalized
*/
class VISP_EXPORT vpMoment{
 private:
        vpMomentObject* object;
        vpMomentDatabase* moments;
        char _name[255];
 protected:
        std::vector<double> values;
        /*!
        Returns the linked moment database.
        \return the moment database
        */
        inline vpMomentDatabase& getMoments(){ return *moments; }


 public:
        inline vpMomentObject& getObject(){ return *object;}
        vpMoment();
        /*!
        Returns all values computed by the moment.
        \return vector of values
        */
        std::vector<double>& get(){ return values;}
        void linkTo(vpMomentDatabase& moments);
        void update(vpMomentObject& object);
        virtual void compute()=0;
        virtual const char* name() = 0;
        friend std::ostream & operator<<(std::ostream & os, const vpMoment& m);

        /*!
        Virtual destructor.
        */
        virtual ~vpMoment() {}
};
#endif
