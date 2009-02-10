/****************************************************************************
 *
 * $Id: vpFeatureTranslation.h,v 1.13 2008-11-10 16:54:11 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * 3D translation visual feature.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpFeatureTranslation_H
#define vpFeatureTranslation_H

/*!
  \file vpFeatureTranslation.h
  \brief class that defines the translation visual feature.
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRGBa.h>


/*!
  \class vpFeatureTranslation
  \ingroup VsFeature3

  \brief Class that defines the translation visual feature
  \f$s=(t_x,t_y,t_z)\f$.

  It is convenient to consider two coordinate frames: the current
  camera frame \f$ {\cal{F}}_c \f$ and the desired camera frame \f$
  {\cal{F}}_{c^*} \f$. 

  Let \f$^{c^*}M_c \f$ be the homogeneous matrix that gives the
  orientation and the translation of the current camera frame relative
  to the desired camera frame. 

  \f[
  ^{c^*}M_c = \left(\begin{array}{cc}
  ^{c^*}R_c & ^{c^*}t_c  \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right)
  \f]

  with \f$^{c^*}R_c \f$ the rotation matrix that gives the orientation
  of the current camera frame relative to the desired camera frame and
  \f$^{c^*}t_c \f$ the translation vector that gives the position of
  the current camera frame relative to the desired camera frame. To
  know more about homogeneous matrices see vpHomogeneousMatrix
  documentation.

  This class can be used to manipulate the translation visual feature
  \f$s= ^{c^*}t_c\f$ with components \f$(t_x,t_y,t_z)\f$. The desired
  visual feature \f$ s^* \f$ is equal to zero. The corresponding error
  is than equal to \f$ e=(s-s^*) = ^{c^*}t_c \f$. In this case, the
  interaction matrix related to \f$ s \f$ is given by \f[ L = [
  ^{c^*}R_c \;\; 0_3] \f]

  To initialize the feature \f$(t_x, t_y, t_z)\f$ you may use member
  fonctions like set_Tx(), set_Ty(), set_Tz(), or also buildFrom()
  fonctions.

  The interaction() method allows to compute the interaction matrix
  \f$ L\f$ associated to the translation visual feature, while the
  error() method computes the error vector \f$(s - s^*)\f$ between the
  current visual feature and the desired one.

  The code below shows how to create a eye-in hand visual servoing
  task using a 3D translation feature \f$(t_x,t_y,t_z)\f$ that
  correspond to the 3D translation between the current camera frame
  and the desired camera frame. To control six degrees of freedom, at
  least three other features must be considered like vpFeatureThetaU
  visual features. First we create a current (\f$s\f$) and desired
  (\f$s^*\f$) 3D translation feature, set the task to use the
  interaction matrix associated to the current feature \f$L_s\f$ and
  than compute the camera velocity \f$v=-\lambda \; L_s^+ \;
  (s-s^*)\f$. The current feature \f$s\f$ is updated in the while() loop
  while \f$s^*\f$ is set to zero.

  \code
#include <visp/vpFeatureTranslation.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpServo.h>

int main()
{
  vpServo task; // Visual servoing task

  vpHomogeneousMatrix cdMc;
  // ... cdMc need here to be initialized from for example a pose estimation.

  // Creation of the current visual feature s
  vpFeatureTranslation s;
  s.buildFrom(cdMc); // Initialization of the current feature s=(tx,ty,tz)

  // Set eye-in-hand control law. 
  // The computed velocities will be expressed in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA);
  // Interaction matrix is computed with the current visual features s
  task.setInteractionMatrixType(vpServo::CURRENT); 
  // Set the constant gain
  double lambda = 0.8;
  task.setLambda(lambda);  

  // Add the 3D translation feature to the task
  task.addFeature(s); // s* is here considered as zero

  // Control loop
  while(1) {
    // ... cdMc need here to be initialized from for example a pose estimation.
    
    // Update the current 3D translation visual feature
    s.buildFrom(cdMc);
    
    // compute the control law
    vpColVector v = task.computeControlLaw(); // camera velocity
  }
}
  \endcode

  If you want to deal only with the \f$(t_x,t_y)\f$ subset feature from the 3D 
  translation, you have just to modify the addFeature() call in 
  the previous example by the following line. In that case, the dimension 
  of \f$s\f$ is two.

  \code
  // Add the (tx,ty) subset features from 3D translation to the task
  task.addFeature(s, vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy());
  \endcode
	     
  If you want to build your own control law, this other example shows
  how to create a current (\f$s\f$) and desired (\f$s^*\f$) 3D
  translation visual feature, compute the corresponding error
  vector \f$(s-s^*)\f$ and finally build the interaction matrix \f$L_s\f$.

  \code
#include <visp/vpFeatureTranslation.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>

int main()
{
  vpHomogeneousMatrix cdMc;
  // ... cdMc need here to be initialized from for example a pose estimation.

  // Creation of the current feature s
  vpFeatureTranslation s;
  s.buildFrom(cdMc); // Initialization of the feature

  // Creation of the desired feature s*. By default this feature is 
  // initialized to zero
  vpFeatureTranslation s_star; 

  // Compute the interaction matrix for the translation feature
  vpMatrix L = s.interaction();

  // Compute the error vector (s-s*) for the translation feature
  vpColVector e = s.error(s_star); // e = (s-s*)
}
  \endcode

*/
class VISP_EXPORT vpFeatureTranslation : public vpBasicFeature
{
public:
  // basic construction
  void init() ;
  // basic constructor
  vpFeatureTranslation() ;
  // constructor : build from an homogeneous matrix
  // cdMc is the displacement that the camera has to realize
  vpFeatureTranslation(vpHomogeneousMatrix &cdMc) ;
  //! Destructor. Does nothing.
  virtual ~vpFeatureTranslation() { ; }

  // build from an homogeneous matrix
  // cdMc is the displacement that the camera has to realize
  void buildFrom(const vpHomogeneousMatrix &cdMc) ;

  void set_Tx(const double t_x) ;
  void set_Ty(const double t_y) ;
  void set_Tz(const double t_z) ;

  double get_Tx() const ;
  double get_Ty() const ;
  double get_Tz() const ;


  // feature selection
  /*! 

    Function used to select the \f$ t_x\f$ subset of the translation
    visual feature.

    This function is to use in conjunction with interaction() in order
    to compute the interaction matrix associated to \f$ t_x\f$.

    See the interaction() method for an usage example.

    This function is also useful in the vpServo class to indicate that
    a subset of the visual feature is to use in the control law:

    \code
    vpFeatureTranslation t;
    vpServo task;
    ...
    // Add the (tx,ty) subset features from 3D translation to the task
    task.addFeature(t, vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy());
    \endcode

    \sa selectTy(), selectTz()

  */
  inline static int selectTx()  { return FEATURE_LINE[0] ; }
  /*! 

    Function used to select the \f$ t_y\f$ subset of the translation
    visual feature.

    This function is to use in conjunction with interaction() in order
    to compute the interaction matrix associated to \f$ t_y\f$.

    See the interaction() method for an usage example.

    This function is also useful in the vpServo class to indicate that
    a subset of the visual feature is to use in the control law:

    \code
    vpFeatureTranslation t;
    vpServo task;
    ...
    // Add the (tx,ty) subset features from 3D translation to the task
    task.addFeature(t, vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy());
    \endcode

    \sa selectTx(), selectTz()
  */
  inline static int selectTy()  { return FEATURE_LINE[1] ; }
  /*! 

    Function used to select the \f$ t_z\f$ subset of the translation
    visual feature.

    This function is to use in conjunction with interaction() in order
    to compute the interaction matrix associated to \f$ t_z\f$.

    See the interaction() method for an usage example.

    This function is also useful in the vpServo class to indicate that
    a subset of the visual feature is to use in the control law:

    \code
    vpFeatureTranslation t;
    vpServo task;
    ...
    // Add the (tz) subset feature from 3D translation to the task
    task.addFeature(t, vpFeatureTranslation::selectTz());
    \endcode

    \sa selectTx(), selectTy()
  */
  inline static int selectTz()  { return FEATURE_LINE[2] ; }
  // compute the interaction matrix from a subset a the possible features
  vpMatrix  interaction(const int select = FEATURE_ALL) const;
  // compute the error between two visual features from a subset
  // a the possible features
  vpColVector error(const vpBasicFeature &s_star,
		    const int select = FEATURE_ALL)  ;
  // print the name of the feature
  void print(const int select= FEATURE_ALL) const ;


  //! Feature duplication
  vpFeatureTranslation *duplicate() const ;


  void display(const vpCameraParameters &cam,
               vpImage<unsigned char> &I,
               vpColor::vpColorType color=vpColor::green) const ;
  void display(const vpCameraParameters &cam,
               vpImage<vpRGBa> &I,
               vpColor::vpColorType color=vpColor::green) const ;

private:
  //! displacement that the camera has to realize
  vpHomogeneousMatrix cdMc ;
} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
