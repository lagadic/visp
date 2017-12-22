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
 * Pre-filled pseudo-database used to handle dependencies between common
 *moment features.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

/*!
  \file vpFeatureMomentCommon.h
  \brief Pre-filled pseudo-database used to handle dependencies between common
  moment features.
*/

#ifndef __FEATUREMOMENTCOMMON_H__
#define __FEATUREMOMENTCOMMON_H__
#include <visp3/visual_features/vpFeatureMomentAlpha.h>
#include <visp3/visual_features/vpFeatureMomentArea.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentBasic.h>
#include <visp3/visual_features/vpFeatureMomentCInvariant.h>
#include <visp3/visual_features/vpFeatureMomentCentered.h>
#include <visp3/visual_features/vpFeatureMomentCommon.h>
#include <visp3/visual_features/vpFeatureMomentDatabase.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenter.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenterNormalized.h>

class vpMomentDatabase;
class vpServo;
/*!
  \class vpFeatureMomentCommon

  \ingroup group_visual_features

  \brief This class allows to access common vpFeatureMoments in a pre-filled
database.

  It is a vpMomentDatabase filled with the following moments:
  - vpFeatureMomentGravityCenter
  - vpFeatureMomentGravityCenterNormalized
  - vpFeatureMomentAreaNormalized
  - vpFeatureMomentCInvariant
  - vpFeatureMomentAlpha
  - vpFeatureMomentCentered
  - vpFeatureMomentBasic


  There is no need to do the linkTo operations manually nor is it necessary to
care about the order of feature computation.

  This class has an vpMomentCommon::updateAll method capable of updating the
plane parameters AND computing interaction matrices inside the features.

  The moment features computed by this class are classical moments
  features used in moment-based visual servoing.  For more
  information see \cite Tahri05z.

  To initialize this feature set, the user needs to supply a vpMomentDatabase
containing at least the contents of vpMomentCommon.

  The features can be retrieved like from a normal vpFeatureMomentDatabase.
However, some shortcuts to retrieve the features are provided.

  \attention Make sure your object is at least of order 6 when using this
pre-filled database.

  The following code demonstrates the construction of a 6x6 interaction matrix
as described in [1].
\code
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpMomentCommon.h>
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpFeatureMoment.h>
#include <visp3/visual_features/vpFeatureMomentCommon.h>
#include <visp3/vs/vpServo.h>
#include <iostream>
#include <vector>

int main()
{
    // Define source polygon
  vpPoint p;
  std::vector<vpPoint> vec_p; // vector that contains the vertices of the contour polygon

  p.set_x(-0.2); p.set_y(0.1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(+0.3); p.set_y(0.1); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);
  p.set_x(+0.2); p.set_y(-0.1); // coordinates in meters in the image plane (vertex 3)
  vec_p.push_back(p);
  p.set_x(-0.2); p.set_y(-0.15); // coordinates in meters in the image plane (vertex 4)
  vec_p.push_back(p);
  p.set_x(-0.2); p.set_y(0.1); // close the contour (vertex 5 = vertex 1)
  vec_p.push_back(p);


  vpMomentObject src(6); // Create a source moment object with 6 as maximum order
  src.setType(vpMomentObject::DENSE_POLYGON); // The object is defined by a countour polygon
  src.fromVector(vec_p); // Init the dense object with the source polygon
  vec_p.clear();

  //Define destination polygon. This is the source polygon translated
  //of 0.1 on x-axis
  p.set_x(-0.1); p.set_y(0.1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(+0.4); p.set_y(0.1); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);
  p.set_x(+0.3); p.set_y(-0.1); // coordinates in meters in the image plane (vertex 3)
  vec_p.push_back(p);
  p.set_x(-0.1); p.set_y(-0.15); // coordinates in meters in the image plane (vertex 4)
  vec_p.push_back(p);
  p.set_x(-0.1); p.set_y(0.1); // close the contour (vertex 5 = vertex 1)
  vec_p.push_back(p);

  vpMomentObject dst(6); // Create a destination moment object with 6 as maximum order
  dst.setType(vpMomentObject::DENSE_POLYGON); // The object is defined by a countour polygon
  dst.fromVector(vec_p); // Init the dense object with the destination
                         // polygon


  //init classic moment primitives (for source)
  vpMomentCommon mdb_src(vpMomentCommon::getSurface(dst),vpMomentCommon::getMu3(dst),vpMomentCommon::getAlpha(dst),1.);
  //Init classic features
  vpFeatureMomentCommon fmdb_src(mdb_src);

  ////init classic moment primitives (for destination)
  vpMomentCommon mdb_dst(vpMomentCommon::getSurface(dst),vpMomentCommon::getMu3(dst),vpMomentCommon::getAlpha(dst),1.);
  //Init classic features
  vpFeatureMomentCommon fmdb_dst(mdb_dst);

  //update+compute moment primitives from object (for source)
  mdb_src.updateAll(src);
  //update+compute features (+interaction matrices) from plane
  fmdb_src.updateAll(0.,0.,1.);

  //update+compute moment primitives from object (for destination)
  mdb_dst.updateAll(dst);
  //update+compute features (+interaction matrices) from plane
  fmdb_dst.updateAll(0.,0.,1.);

  //define visual servoing task
  vpServo task;
  task.setServo(vpServo::EYEINHAND_CAMERA);
  task.setInteractionMatrixType(vpServo::CURRENT);

  //Add all classic features to the task
  //In this example, source and destination features are translated by 0.1
  //will produce a movement of 0.1 on x-axis.
  task.addFeature(fmdb_src.getFeatureGravityNormalized(),fmdb_dst.getFeatureGravityNormalized());
  task.addFeature(fmdb_src.getFeatureAn(),fmdb_dst.getFeatureAn());
  //the object is NOT symmetric
  //select C4 and C6
  task.addFeature(fmdb_src.getFeatureCInvariant(),fmdb_dst.getFeatureCInvariant(),(1 << 3) | (1 << 5));
  task.addFeature(fmdb_src.getFeatureAlpha(),fmdb_dst.getFeatureAlpha());

  task.setLambda(1) ;
  vpColVector v = task.computeControlLaw() ;

  task.print();

  task.kill();

  return 0;
}
    \endcode
This code produces the following output:
\code
Visual servoing task:
Type of control law
Eye-in-hand configuration
Control in the camera frame
List of visual features : s
0.0166667,-0.00833333,
1,
-0.312148,0.0249916,
-1.43449,
List of desired visual features : s*
0.116667,-0.00833333,
1,
-0.312148,0.0249916,
-1.43449,
Interaction Matrix Ls
-1  0  -6.938893904e-18  0.007291666667  -1.06875  -0.008333333333
0  -1  3.469446952e-18  1.0171875  -0.007291666667  -0.01666666667
0  0  -1  0.0125  0.025  0
0  0  -4.585529113e-15  -0.2983860943  0.5832596643  -4.376751552e-16
0  0  -3.58244462e-15  0.08633028234  -0.2484618767  3.63421192e-16
4.353086256e-17  -1.339411156e-16  -0  -0.03019436997  -0.0168230563  -1
Error vector (s-s*)
-0.1  0  0  1.831867991e-15  -1.072059108e-15  0
Gain : Zero= 1	Inf= 1	Deriv= 0

\endcode
*/
class VISP_EXPORT vpFeatureMomentCommon : public vpFeatureMomentDatabase
{
private:
  vpFeatureMomentGravityCenter featureGravity;
  vpFeatureMomentGravityCenterNormalized featureGravityNormalized;
  vpFeatureMomentAreaNormalized featureAn;
  vpFeatureMomentCInvariant featureCInvariant;
  vpFeatureMomentAlpha featureAlpha;
  vpFeatureMomentCentered featureCentered;
  vpFeatureMomentBasic featureMomentBasic;
  vpFeatureMomentArea feature_moment_area;

public:
  vpFeatureMomentCommon(vpMomentDatabase &moments, double A = 0.0, double B = 0.0, double C = 1.0);
  void updateAll(double A, double B, double C);
  /*!
  Returns alpha.
  */
  vpFeatureMomentAlpha &getFeatureAlpha() { return featureAlpha; }

  /*!
  Returns normalized surface.
  */
  vpFeatureMomentAreaNormalized &getFeatureAn() { return featureAn; }
  /*!
  Returns basic moment.
  */
  vpFeatureMomentBasic &getFeatureMomentBasic() { return featureMomentBasic; }
  /*!
  Returns centered moments.
  */
  vpFeatureMomentCentered &getFeatureCentered() { return featureCentered; }

  /*!
  Returns non-symmetric invariants.
   */
  vpFeatureMomentCInvariant &getFeatureCInvariant() { return featureCInvariant; }
  /*!
      Returns normalized gravity center.
  */
  vpFeatureMomentGravityCenterNormalized &getFeatureGravityNormalized() { return featureGravityNormalized; }
  /*!
      Returns the area
      */
  vpFeatureMomentArea &getFeatureArea() { return feature_moment_area; }
  /*!
          Returns gravity center
  */
  vpFeatureMomentGravityCenter &getFeatureGravityCenter() { return featureGravity; }
};

#endif
