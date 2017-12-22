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
 * Implementation for all supported moment features.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
  \file vpFeatureMomentGravityCenter.h
  \brief Implementation of the interaction matrix computation for
  vpMomentGravityCenter.
*/
#ifndef __FEATUREMOMENTGRAVITYCENTER_H__
#define __FEATUREMOMENTGRAVITYCENTER_H__
#include <visp3/visual_features/vpFeatureMoment.h>
#ifdef VISP_MOMENTS_COMBINE_MATRICES
class vpMomentDatabase;
/*!
  \class vpFeatureMomentGravityCenter

  \ingroup group_visual_features

  \brief Functionality computation for gravity center moment feature. Computes
the interaction matrix associated with vpMomentGravityCenter.

  The interaction matrix for the is defined in \cite Tahri05z, equation (16).
  It allows to compute the interaction matrices for \f$ (x_g,y_g) \f$.

  These interaction matrices may be selected afterwards by calling
vpFeatureMomentGravityCenter::interaction(). The selection is done by the
following methods: vpFeatureMomentGravityCenter::selectXg for \f$ L_{x_{g}}
\f$ and vpFeatureMomentGravityCenter::selectYg for \f$ L_{y_{g}} \f$. The
following code demonstrates a selection of \f$ L_{y_{g}} \f$:

  \code
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpFeatureMoment.h>
#include <visp3/visual_features/vpFeatureMomentDatabase.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenter.h>
#include <visp3/visual_features/vpFeatureMomentBasic.h>
#include <iostream>
#include <vector>

int main()
{
  vpPoint p;
  std::vector<vpPoint> vec_p; // vector that contains the vertices

  p.set_x(1); p.set_y(1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(2); p.set_y(2); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);

  //////////////////////////////REFERENCE VALUES////////////////////////////////
  vpMomentObject obj(2); // Init object of order 2 because we need
                         // vpFeatureMomentBasic of order 1 (for vpFeatureMomentGravityCenter) which
                         // implies third-order moment primitives
  obj.setType(vpMomentObject::DISCRETE); // Discrete mode for object
  obj.fromVector(vec_p);


  vpMomentDatabase mdb; //database for moment primitives. This will
                        //only contain the basic moment.
  vpMomentBasic bm; //basic moment (this particular moment is nothing
                    //more than a shortcut to the vpMomentObject)
  vpMomentGravityCenter	gc; //gravity center

  bm.linkTo(mdb); //add basic moment to moment database
  gc.linkTo(mdb); //add gravity center to moment database

  vpFeatureMomentDatabase fmdb; //feature moment database to store
                                //feature dependencies

  //Declare and link moments to database
  vpFeatureMomentBasic fmb(mdb,0.,0.,1.,&fmdb); fmb.linkTo(fmdb);
  vpFeatureMomentGravityCenter fgc(mdb,0.,0.,1.,&fmdb); fgc.linkTo(fmdb);

  //update and compute the vpMomentBasic before computing vpMomentGravityCenter
  bm.update(obj);
  bm.compute();
  //update and compute the vpMomentGravityCenter before computing vpFeatureMomentBasic
  gc.update(obj);
  gc.compute();

  fmb.update(0.,0.,1.); //update the vpFeatureMoment with a plane
                     //configuration and compute interaction matrix

  fgc.update(0.,0.,1.); //update the plane configuration for gravity
                     //center feature and compute it's associated matrix.

  std::cout << fgc.interaction(1 << 1) << std::endl;

  return 0;
}
  \endcode

  This code produces the following output:
  \code
0  -1  1.5  3.5  -2.5  -1.5
  \endcode

  You can also use the shortcut selectors
vpFeatureMomentGravityCenter::selectXg or
vpFeatureMomentGravityCenter::selectYg as follows:

  \code
  task.addFeature(db_src.getFeatureGravityNormalized(), db_dst.getFeatureGravityNormalized(),
                 vpFeatureMomentGravityCenter::selectXg() | vpFeatureMomentGravityCenter::selectYg());
  \endcode
  This feature depends on:
      - vpFeatureMomentBasic

  Minimum vpMomentObject order needed to compute this feature: 2.
*/
class VISP_EXPORT vpFeatureMomentGravityCenter : public vpFeatureMoment
{
public:
  /*!
  Initializes the feature with information about the database of moment
  primitives, the object plane and feature database. \param database : Moment
  database. The database of moment primitives (first parameter) is mandatory.
  It is used to access different moment values later used to compute the final
  matrix. \param A_ : Plane coefficient in a \f$ A \times x+B \times y + C =
  \frac{1}{Z} \f$ plane. \param B_ : Plane coefficient in a \f$ A \times x+B
  \times y + C = \frac{1}{Z} \f$ plane. \param C_ : Plane coefficient in a \f$
  A \times x+B \times y + C = \frac{1}{Z} \f$ plane. \param featureMoments :
  Feature database.
  */
  vpFeatureMomentGravityCenter(vpMomentDatabase &database, double A_, double B_, double C_,
                               vpFeatureMomentDatabase *featureMoments = NULL)
    : vpFeatureMoment(database, A_, B_, C_, featureMoments, 2)
  {
  }
  void compute_interaction();
  /*!
    Associated moment name.
  */
  const char *momentName() const { return "vpMomentGravityCenter"; }
  /*!
    Feature name.
    */
  const char *name() const { return "vpFeatureMomentGravityCenter"; }

  /*!
    Shortcut selector for \f$x_g\f$.
    */
  static unsigned int selectXg() { return 1 << 0; }

  /*!
    Shortcut selector for \f$y_g\f$.
    */
  static unsigned int selectYg() { return 1 << 1; }
};

#else
class vpMomentDatabase;
/*!
  \class vpFeatureMomentGravityCenter

  \ingroup group_visual_features

  \brief Functionality computation for gravity center moment feature. Computes
  the interaction matrix associated with vpMomentGravityCenter.

  The interaction matrix for the is defined in \cite Tahri05z, equation (16).
  It allows to compute the interaction matrices for \f$ (x_g,y_g) \f$.

  These interaction matrices may be selected afterwards by calling
  vpFeatureMomentGravityCenter::interaction(). The selection is done by the
  following methods: vpFeatureMomentGravityCenter::selectXg for \f$ L_{x_{g}}
  \f$ and vpFeatureMomentGravityCenter::selectYg for \f$ L_{y_{g}} \f$.

  You can use the selectors vpFeatureMomentGravityCenter::selectXg or
  vpFeatureMomentGravityCenter::selectYg as follows:

  \code
  task.addFeature(db_src.getFeatureGravityNormalized(), db_dst.getFeatureGravityNormalized(),
                  vpFeatureMomentGravityCenter::selectXg() | vpFeatureMomentGravityCenter::selectYg());
  \endcode
  This feature depends on:
  - vpMomentCentered
  - vpMomentGravityCenter

  Minimum vpMomentObject order needed to compute this feature: 2.
*/
class VISP_EXPORT vpFeatureMomentGravityCenter : public vpFeatureMoment
{
public:
  /*!
  Initializes the feature with information about the database of moment
  primitives, the object plane and feature database. \param data_base : Moment
  database. The database of moment primitives (first parameter) is mandatory.
  It is used to access different moment values later used to compute the final
  matrix. \param A_ : Plane coefficient in a \f$ A \times x+B \times y + C =
  \frac{1}{Z} \f$ plane. \param B_ : Plane coefficient in a \f$ A \times x+B
  \times y + C = \frac{1}{Z} \f$ plane. \param C_ : Plane coefficient in a \f$
  A \times x+B \times y + C = \frac{1}{Z} \f$ plane. \param featureMoments :
  Feature database.
  */
  vpFeatureMomentGravityCenter(vpMomentDatabase &data_base, double A_, double B_, double C_,
                               vpFeatureMomentDatabase *featureMoments = NULL)
    : vpFeatureMoment(data_base, A_, B_, C_, featureMoments, 2)
  {
  }
  void compute_interaction();
  /*!
    Associated moment name.
  */
  const char *momentName() const { return "vpMomentGravityCenter"; }
  /*!
    Feature name.
    */
  const char *name() const { return "vpFeatureMomentGravityCenter"; }

  /*!
    Shortcut selector for \f$x_g\f$.
    */
  static unsigned int selectXg() { return 1 << 0; }

  /*!
    Shortcut selector for \f$y_g\f$.
    */
  static unsigned int selectYg() { return 1 << 1; }
};

#endif
#endif
