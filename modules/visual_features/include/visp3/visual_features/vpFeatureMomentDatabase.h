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
 * Pseudo-database used to handle dependencies between moment features.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

/*!
  \file vpFeatureMomentDatabase.h
  \brief Pseudo-database used to handle dependencies between moment features.
*/

#ifndef __FEATUREMOMENTDATABASE_H__
#define __FEATUREMOMENTDATABASE_H__

#include <cstring>
#include <iostream>
#include <map>
#include <visp3/core/vpConfig.h>

class vpFeatureMoment;
class vpMomentObject;
/*!
  \class vpFeatureMomentDatabase

  \ingroup group_visual_features

  \brief This class allows to register all feature moments (implemented in
vpFeatureMoment... classes) so they can access each other according to their
dependencies.

  Like moments (implemented in vpMoment... classes), a vpFeatureMoment needs
to have access to other vpFeatureMoment's values to be computed. In most
cases, a vpFeatureMoment needs both: vpMoments and vpFeatureMoments which
explains the two databases (see vpFeatureMoment::vpFeatureMoment). For example
vpFeatureMomentAlpha needs additionnal information about centered moments
vpMomentCentered AND their interaction matrices obtained by
vpFeatureMomentCentered in order to compute the moment's value from a
vpMomentObject. Like the vpMomentCentered is stored in a vpMomentDatabase, the
vpFeatureMomentCentered should be stored in a vpFeatureMomentDatabase.

  All moment features in a database can access each other freely at any time.
They can also verify if a moment feature is present in the database or not.
  This code illustrates the use of both databases to handle dependencies
between moment primitives and moment features:

\code
#include <visp3/core/vpPoint.h>

#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentCInvariant.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentCInvariant.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentDatabase.h>

#include <visp3/visual_features/vpFeatureMomentCInvariant.h>
#include <visp3/visual_features/vpFeatureMomentBasic.h>
#include <visp3/visual_features/vpFeatureMomentCentered.h>
#include <visp3/visual_features/vpFeatureMomentDatabase.h>
#include <iostream>
#include <vector>

int main()
{
  try{

  vpPoint p;
  std::vector<vpPoint> vec_p; // vector that contains the vertices

  p.set_x(1); p.set_y(1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(2); p.set_y(2); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);

  //////////////////////////////REFERENCE VALUES////////////////////////////////
  vpMomentObject obj(6); // Init object of order 6 because we are
                         // computing C-invariants
  obj.setType(vpMomentObject::DISCRETE); // Discrete mode for object
  obj.fromVector(vec_p);


  vpMomentDatabase mdb; //database for moment primitives. This will
                        //only contain the basic moment.
  vpMomentCentered mc; //Centered moment
  vpMomentBasic bm; //Basic moment
  vpMomentGravityCenter gc; //gravity center
  vpMomentCInvariant ci; //C-type invariant

  bm.linkTo(mdb); //add basic moment to moment database
  mc.linkTo(mdb);  //add centered moment to moment database
  gc.linkTo(mdb); //add gravity center to moment database
  ci.linkTo(mdb); //add C-invariant to moment database

  vpFeatureMomentDatabase fmdb; //feature moment database to store
                                //feature dependencies

  //Declare and link moments to database
  vpFeatureMomentBasic fmb(mdb,0.,0.,1.,&fmdb); fmb.linkTo(fmdb);
  vpFeatureMomentCentered fmc(mdb,0.,0.,1.,&fmdb); fmc.linkTo(fmdb);
  vpFeatureMomentCInvariant fci(mdb,0.,0.,1.,&fmdb); fci.linkTo(fmdb);


  //update the whole moment database
  mdb.updateAll(obj);

  //Compute moments in the correct order with the object
  bm.compute();
  gc.compute();
  mc.compute();
  ci.compute();

  //update the whole feature moment database with a plane
  fmb.update(0.,0.,1.);
  fmc.update(0.,0.,1.);
  fci.update(0.,0.,1.);

  std::cout << fci.interaction(vpFeatureMomentCInvariant::selectC1()) << std::endl;
  }catch(vpException &e){
      std::cout << e.getMessage() << std::endl;
  }

  return 0;
}
\endcode
*/
class VISP_EXPORT vpFeatureMomentDatabase
{
private:
  struct cmp_str {
    bool operator()(const char *a, const char *b) const { return std::strcmp(a, b) < 0; }
    char *operator=(const char *) { return NULL; } // Only to avoid a warning under Visual with /Wall flag
  };
  std::map<const char *, vpFeatureMoment *, cmp_str> featureMomentsDataBase;
  void add(vpFeatureMoment &featureMoment, char *name);

public:
  /*!
    Default constructor.
  */
  vpFeatureMomentDatabase() : featureMomentsDataBase() {}
  /*!
    Virtual destructor that does nothing.
  */
  virtual ~vpFeatureMomentDatabase() {}
  virtual void updateAll(double A = 0.0, double B = 0.0, double C = 1.0);

  vpFeatureMoment &get(const char *type, bool &found);

  // friend VISP_EXPORT std::ostream & operator<<(std::ostream& os, const
  // vpFeatureMomentDatabase& m);
  friend class vpFeatureMoment;
};

#endif
