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
 * Base for all moment features
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
  \file vpFeatureMoment.h
  \brief Base class for moment features.

  Handles common system operations like selection, duplication. Functionality is computed in derived classes.
*/
#ifndef __FEATUREMOMENT_H__
#define __FEATUREMOMENT_H__

#include <vector>
#include <visp/vpConfig.h>
#include <visp/vpBasicFeature.h>
class vpMomentObject;
class vpMomentDatabase;
class vpFeatureMomentDatabase;
class vpMoment;

/*!
  \class vpFeatureMoment

  \ingroup VsFeature2

  \brief This class defines shared system methods/attributes for 2D moment features but no functional code. It is used to compute interaction matrixes for moment features..

  While vpMoment-type classes do only compute moment values and can by used for almost anything, vpFeatureMoment-type classes
  are specifically designed for visual servoing. More importantly, a vpFeatureMoment is used to compute the interaction matrix associated to it's moment primitive.

  This class is virtual and cannot be used directly. It defines the following characteristics common to all moment features:
    - Plane orientation parameters (A,B,C):
        Each camera frame corresponds to a physical planar object contained in a plane. This plane's equation has the following form:
        \f$ A \times x+B \times y + C = \frac{1}{Z} \f$.
        These parameters can be updated anytime.
    - Get corresponding moment primitive: for example a vpFeatureMomentCInvariant will provide access to a vpMomentCInvariant instance.
    - Provide access to a feature database (vpFeatureMomentDatabase).
    - All interaction matrices (different from vpBasicFeature::interaction which selects the required interaction matrix).

    Like vpMoment, vpFeatureMoment provides a vpFeatureMoment::update() method. But unlike vpMoment::update() which only acknowledges the new object,
    the vpFeatureMoment::update() acknowledges the new plane parameters AND computes the interaction matrixes associated with the feature.

    A vpFeatureMoment will be often part of a vpFeatureMomentDatabase in the same way a vpMoment is part of a vpMomentDatabase. This database is specified inside the
    vpFeatureMoment::vpFeatureMoment() constructor.
    As a result, a vpFeatureMoment will be able to access other vpFeatureMoments through this database.

    A vpBasicFeature can be duplicated into a vpMomentGenericFeature. In that case, all data in the vpBasicFeature is copied but the feature's name
    is lost. For example if a vpFeatureMomentCInvariant is duplicated, the duplicata will be operational but could not be used in a vpFeatureMomentDatabase.

    Note that you can use vpFeatureMoment to do visual servoing but it is not it's only purpose. You may compute your interaction matrixes with vpFeatureMoment::update()
    and use them for any purpose.

    \attention - A vpFeatureMoment is not responsible for updating the moment primitives it depends on. Make sure your vpMoments are all up to date before computing an interaction matrix using vpFeatureMoment.

    \attention - Be careful with orders. Often, computing a feature of order n requires vpMoment primitives of order n+1.
    Make sure to check the documentation of the specialised vpFeatureMoment classes when deciding to which order you want to initialize the object.
    An object of order 6 should be sufficient for all classic implementations of vpFeatureMoment.

    Here is an example of how to use a vpFeatureMoment (in this case vpFeatureMomentBasic).
    \code
#include <visp/vpPoint.h>
#include <visp/vpMomentObject.h>
#include <visp/vpMomentBasic.h>
#include <visp/vpMomentDatabase.h>
#include <visp/vpFeatureMoment.h>
#include <visp/vpFeatureMomentBasic.h>

int main()
{
  vpPoint p;
  std::vector<vpPoint> vec_p; // vector that contains the vertices

  p.set_x(1); p.set_y(1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(2); p.set_y(2); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);

  //////////////////////////////REFERENCE VALUES////////////////////////////////
  vpMomentObject obj(3); // Init object of order 3 because we need
			 // vpFeatureMomentBasic of order 2 which
			 // implies third-order moment primitives
  obj.setType(vpMomentObject::DISCRETE); // Discrete mode for object
  obj.fromVector(vec_p); 

  vpMomentDatabase mdb; //database for moment primitives. This will
			//only contain the basic moment.
  vpMomentBasic bm; //basic moment (this particular moment is nothing
		    //more than a shortcut to the vpMomentObject)  
  bm.linkTo(mdb); //add basic moment to moment database
   
  vpFeatureMomentBasic fmb(mdb,0,0,1,NULL);

  //update and compute the vpMoment BEFORE doing any operations with
  //vpFeatureMoment
  bm.update(obj);
  bm.compute();
  
  fmb.update(0,0,1); //update the vpFeatureMoment with a plane
		     //configuration
  std::cout << fmb.interaction(1,1) << std::endl;
  
  return 0;
}
    \endcode
*/
class VISP_EXPORT vpFeatureMoment : public vpBasicFeature{
 protected:
        vpMoment* moment;
	vpMoment& getMoment(){return *moment;}
        vpMomentDatabase& moments;
        vpFeatureMomentDatabase* featureMoments;
        std::vector<vpMatrix> interaction_matrices;

        double A;
        double B;
        double C;
        char _name[255];
 public:
	 /*!
         Initializes the feature with information about the database of moment primitives, the object plane, feature database and matrix size.
         \param moments : Moment database. The database of moment primitives (first parameter) is mandatory. It is used to access different moment values later used to compute the final matrix.
         \param A : Plane coefficient in a \f$ A \times x+B \times y + C = \frac{1}{Z} \f$ plane.
         \param B : Plane coefficient in a \f$ A \times x+B \times y + C = \frac{1}{Z} \f$ plane.
         \param C : Plane coefficient in a \f$ A \times x+B \times y + C = \frac{1}{Z} \f$ plane.
	 \param FeatureMoments : Feature database
         \param nbmatrices : If you want to create a new vpFeatureMoment implementation, your feature will often have a matrix size of n lines. You can specify the number of lines by this parameter.

	 */
     vpFeatureMoment(vpMomentDatabase& moments,double A=0.0, double B=0.0, double C=0.0,vpFeatureMomentDatabase* FeatureMoments=NULL,unsigned int nbmatrices=1) :
         moment(NULL),
         moments(moments),
         featureMoments(FeatureMoments),
         interaction_matrices(nbmatrices),
         A(A),B(B),C(C) {}
        virtual ~vpFeatureMoment();

        virtual void 	compute_interaction ();
        vpBasicFeature* duplicate ()  const;
        void 	display (const vpCameraParameters &cam, vpImage< unsigned char > &I, vpColor color=vpColor::green, unsigned int thickness=1) const ;
        void 	display (const vpCameraParameters &cam, vpImage< vpRGBa > &I, vpColor color=vpColor::green, unsigned int thickness=1) const ;

        vpColVector 	error (const vpBasicFeature &s_star, unsigned int select=FEATURE_ALL) const;
	int 	getDimension (unsigned int select=FEATURE_ALL) const;
        void 	init ();
        vpMatrix 	interaction (const unsigned int select=FEATURE_ALL);        
        void linkTo(vpFeatureMomentDatabase& featureMoments);

        /*!
          Name of the moment corresponding to the feature. This allows to locate the moment associated with the feature in the provided database.
          */
        virtual const char* momentName() = 0;
        /*!
          Name of the feature used to locate it in the database of features.
          */
        virtual const char* name() = 0;
        void 	print (const unsigned int select=FEATURE_ALL) const ;

        void update (double A, double B, double C);


};

/*!
  \class vpMomentGenericFeature

  \ingroup VsFeature2

  \brief This class defines a generic feature used for moment feature duplication.

    A vpBasicFeature can be duplicated into a vpMomentGenericFeature. In that case, all data in the vpBasicFeature is copied but the feature's name
    is lost. For example if a vpFeatureMomentCInvariant is duplicated, the duplicata will be operational but could not be used in a vpFeatureMomentDatabase.
    The reason for this is that a vpMomentGenericFeature can refer to anything therefore it has no specific name.

    Duplication is mostly used internally in ViSP.

*/
class VISP_EXPORT vpMomentGenericFeature : public vpFeatureMoment{
public:
    vpMomentGenericFeature(vpMomentDatabase& moments,double A, double B, double C,vpFeatureMomentDatabase* FeatureMoments, vpMoment* moment) : vpFeatureMoment(moments,A,B,C,FeatureMoments){this->moment = moment;}
    /*!
      No specific moment name.
      */
    const char* momentName() { return NULL;}
    /*!
      No specific feature name.
      */
    virtual const char* name() { return NULL;}
};

#endif
