/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * ThetaU visual feature.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpFeatureThetaU_H
#define vpFeatureThetaU_H

/*!
  \file vpFeatureThetaU.h
  \brief class that defines the ThetaU visual feature
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpThetaUVector.h>
#include <visp/vpBasicFeature.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRGBa.h>


/*!
  \class vpFeatureThetaU
  \ingroup VsFeature3

  \brief Class that defines a 3D visual feature \f$ s\f$ from a \f$ \theta
  u \f$ axis/angle parametrization that represent the rotation between
  to frames.

  Let us denote \f$ \theta u = (\theta u_x, \theta u_y, \theta u_z)\f$ .

  It is convenient to consider two coordinate frames: the current
  camera frame \f$ {\cal{F}}_c \f$ and the desired camera frame \f$
  {\cal{F}}_{c^*} \f$. 

  Let \f$^{c^*}R_c \f$ be the rotation matrix that gives the
  orientation of the current camera frame relative to the desired camera
  frame. Let \f$ \theta u_{^{c^*}R_c} \f$ to corresponding axis/angle
  representation of this rotation.

  Furthermore, let \f$^{c}R_{c^*} \f$ the rotation matrix that gives the
  orientation of the desired camera frame relative to the current
  camera frame. Let \f$ \theta u_{^{c}R_{c^*}} \f$ to corresponding
  axis/angle representation of this rotation.

  This class can be used to manipulate two kind of visual features:

  - \f$ s = \theta u_{^{c^*}R_c} \f$ if the orientation of current
    camera frame relative to the desired frame has to be
    considered. The desired visual feature \f$ s^* \f$ is equal to
    zero. The corresponding error is than equal to \f$ e=(s-s^*) =
    \theta u_{^{c^*}R_c} \f$. In this case, the interaction matrix
    related to \f$ s \f$ is given by \f[ L = \left[ \begin{array}{cc}
    0_3 & L_{\theta u} \end{array} \right] \f] with \f[
    L_{\theta u} = I_3 + \frac{\theta}{2} \; [u]_\times +
    \left(1 - \frac{sinc \theta}{sinc^2 \frac{\theta}{2}}\right)
    [u]^2_\times \f] where \f$ 0_3 \f$ is a \f$ 3 \times 3 \f$ nul
    matrix, \f$ I_3 \f$ is the \f$3 \times 3\f$ identity matrix, and
    for more readability \f$ \theta \f$ and \f$ u \f$ respectively the
    angle and the axis coordinates of the \f$ \theta u_{^{c^*}R_c} \f$
    representation.

  - \f$ s = \theta u_{^{c}R_{c^*}} \f$ if it is more the orientation
    of the desired camera frame relative to the current frame that has
    to be considered. The desired visual feature \f$ s^* \f$ is equal
    to zero. The corresponding error is than equal to \f$e=(s-s^*) =
    \theta u_{^{c}R_{c^*}} \f$. In this case, the interaction matrix
    related to \f$ s \f$ is given by \f[ L = \left[ \begin{array}{cc}
    0_3 & L_{\theta u} \end{array} \right] \f] with \f[
    L_{\theta u} = -I_3 + \frac{\theta}{2} \; [u]_\times
    - \left(1 - \frac{sinc \theta}{sinc^2 \frac{\theta}{2}}\right)
    [u]^2_\times \f] where \f$ 0_3 \f$ is a \f$ 3 \times 3 \f$ nul
    matrix, \f$ I_3 \f$ is the \f$3 \times 3\f$ identity matrix, and
    for more readability \f$ \theta \f$ and \f$ u \f$ respectively the
    angle and the axis coordinates of the \f$ \theta u_{^{c}R_{c^*}}
    \f$ representation.

  The kind of visual feature is to set during the construction of the
  vpFeatureThetaU() object by using the selector
  vpFeatureThetaU::vpFeatureThetaURotationRepresentationType.

  To initialize the feature \f$(\theta u_x, \theta u_y, \theta u_z)\f$
  you may use vpFeatureThetaU member fonctions like set_TUx(),
  set_TUy(), set_TUz(), or also buildFrom() fonctions.

  Depending on the choice of the visual feature representation, the
  interaction() method allows to compute the interaction matrix \f$
  L \f$ associated to the visual feature, while the error()
  method computes the error vector \f$(s - s^*)\f$ between the current
  visual feature and the desired one.

  To know more on the \f$ \theta u \f$ axis/angle representation for a
  3D rotation see the vpThetaUVector class.

  The code below shows how to create a eye-in hand visual servoing
  task using a 3D \f$\theta u\f$ feature \f$(\theta u_x,\theta u_y,
  \theta u_z)\f$ that correspond to the 3D rotation between the
  current camera frame and the desired camera frame. To control six
  degrees of freedom, at least three other features must be considered
  like vpFeatureTranslation visual features. First we create a current
  (\f$s\f$) 3D \f$\theta u\f$ feature, than set the
  task to use the interaction matrix associated to the current feature
  \f$L_s\f$ and than compute the camera velocity \f$v=-\lambda \;
  L_s^+ \; (s-s^*)\f$. The current feature \f$s\f$ is updated in the
  while() loop while \f$s^*\f$ is considered as zero.

  \code
#include <visp/vpFeatureThetaU.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpServo.h>

int main()
{
  vpServo task; // Visual servoing task

  vpHomogeneousMatrix cMcd;
  // ... cMcd need here to be initialized from for example a pose estimation.

  // Creation of the current feature s that correspond to the rotation
  // in angle/axis parametrization between the current camera frame
  // and the desired camera frame
  vpFeatureThetaU s(vpFeatureThetaU::cRcd);
  s.buildFrom(cMcd); // Initialization of the feature

  // Set eye-in-hand control law. 
  // The computed velocities will be expressed in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA);
  // Interaction matrix is computed with the current visual features s
  task.setInteractionMatrixType(vpServo::CURRENT); 

  // Add the 3D ThetaU feature to the task
  task.addFeature(s); // s* is here considered as zero

  // Control loop
  while(1) {
    // ... cMcd need here to be initialized from for example a pose estimation.
    
    // Update the current ThetaU visual feature
    s.buildFrom(cMcd);
    
    // compute the control law
    vpColVector v = task.computeControlLaw(); // camera velocity
  }
}
  \endcode

  If you want to deal only with the \f$(\theta u_x,\theta u_y)\f$ subset 
  feature from the 3D \f$\theta u\f$ , you have just to modify the 
  addFeature() call in the previous example by the following line. In 
  that case, the dimension of \f$s\f$ is two.

  \code 
  // Add the (ThetaU_x, ThetaU_y) subset features from the 3D ThetaU
  // rotation to the task
  task.addFeature(s, vpFeatureThetaU::selectTUx() | vpFeatureThetaU::selectTUy());
  \endcode

  If you want to build your own control law, this other example shows
  how to create a current (\f$s\f$) and desired (\f$s^*\f$) 3D
  \f$\theta u\f$ visual feature, compute the corresponding error
  vector \f$(s-s^*)\f$ and finally build the interaction matrix \f$L_s\f$.

  \code
#include <visp/vpFeatureThetaU.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>

int main()
{
  vpHomogeneousMatrix cdMc;
  // ... cdMc need here to be initialized from for example a pose estimation.

  // Creation of the current feature s
  vpFeatureThetaU s(vpFeatureThetaU::cdRc);
  s.buildFrom(cdMc); // Initialization of the feature

  // Creation of the desired feature s*. By default this feature is 
  // initialized to zero
  vpFeatureThetaU s_star(vpFeatureThetaU::cdRc); 

  // Compute the interaction matrix L_s for the current ThetaU feature
  vpMatrix L = s.interaction();

  // Compute the error vector (s-s*) for the ThetaU feature
  s.error(s_star);
}
  \endcode
  

*/
class VISP_EXPORT vpFeatureThetaU : public vpBasicFeature
{
public:
  typedef enum
    {
      TUx = 1, /*!< Select the subset \f$ \theta u_x \f$ visual feature
	     from the \f$ \theta u\f$ angle/axis representation. */
      TUy = 2, /*!< Select the subset \f$ \theta u_y \f$ visual feature
	     from the \f$ \theta u\f$ angle/axis representation. */
      TUz = 4  /*!< Select the subset \f$ \theta u_z \f$ visual feature
	     from the \f$ \theta u\f$ angle/axis representation. */
    } vpFeatureThetaUType;
  typedef enum
    {
      cdRc, /*!< Selector used to manipulate the visual feature \f$ s
	      = \theta u_{^{c^*}R_c} \f$. This visual feature
	      represent the orientation of the current camera frame
	      relative to the desired camera frame. */
      cRcd /*!< Selector used to manipulate the visual feature \f$ s = \theta
	     u_{^{c}R_{c^*}} \f$. This visual feature
	      represent the orientation of the desired camera frame
	      relative to the current camera frame. */
    } vpFeatureThetaURotationRepresentationType;
  /*
    attributes and members directly related to the vpBasicFeature needs
    other functionalities are useful but not mandatory
  */

public:
  // Basic construction.
  void init() ;
  // Basic constructor.
  vpFeatureThetaU(vpFeatureThetaURotationRepresentationType r) ;
  vpFeatureThetaU(vpThetaUVector &tu,
		  vpFeatureThetaURotationRepresentationType r) ;
  vpFeatureThetaU(vpRotationMatrix &R,
		  vpFeatureThetaURotationRepresentationType r) ;
  vpFeatureThetaU(vpHomogeneousMatrix &M, 
		  vpFeatureThetaURotationRepresentationType r) ;
  void buildFrom(vpThetaUVector &tu) ;
  // build from a rotation matrix
  void buildFrom(const vpRotationMatrix &R) ;
  // build from an homogeneous  matrix
  void buildFrom(const vpHomogeneousMatrix &M) ;

		  
  //! Destructor. Does nothing.
  virtual ~vpFeatureThetaU() { if (flags != NULL) delete [] flags; /*vpTRACE("0x%x", this)*/ ;}

public:


  void set_TUx(const double tu_x) ;
  void set_TUy(const double tu_y) ;
  void set_TUz(const double tu_z) ;

  double get_TUx()  const ;
  double get_TUy()   const ;
  double get_TUz() const  ;


public:
  /*
    vpBasicFeature method instantiation
  */

  /*! 

    Function used to select the \f$ \theta u_x\f$ subset of the \f$
    \theta u \f$ visual feature.

    This function is to use in conjunction with interaction() in order
    to compute the interaction matrix associated to \f$ \theta u_x\f$.

    See the interaction() method for an usage example.

    This function is also useful in the vpServo class to indicate that
    a subset of the visual feature is to use in the control law:

    \code 
    vpFeatureThetaU tu;
    vpServo task;
    ...
    // Add the (ThetaU_x, ThetaU_y) subset features from the 3D ThetaU
    // rotation to the task
    task.addFeature(tu, vpFeatureThetaU::selectTUx() | vpFeatureThetaU::selectTUy());
    \endcode

    \sa selectTUy(), selectTUz()
  */
  inline static int selectTUx()  { return FEATURE_LINE[0] ; }
  /*! 

    Function used to select the \f$ \theta u_y\f$ subset of the \f$
    \theta u \f$ visual feature.

    This function is to use in conjunction with interaction() in order
    to compute the interaction matrix associated to \f$ \theta u_y\f$.

    See the interaction() method for an usage example.

    This function is also useful in the vpServo class to indicate that
    a subset of the visual feature is to use in the control law:

    \code 
    vpFeatureThetaU tu;
    vpServo task;
    ...
    // Add the (ThetaU_x, ThetaU_y) subset features from the 3D ThetaU
    // rotation to the task
    task.addFeature(tu, vpFeatureThetaU::selectTUx() | vpFeatureThetaU::selectTUy());
    \endcode

    \sa selectTUx(), selectTUz()
  */
  inline static int selectTUy()  { return FEATURE_LINE[1] ; }
  /*! 

    Function used to select the \f$ \theta u_z\f$ subset of the \f$
    \theta u \f$ visual feature.

    This function is to use in conjunction with interaction() in order
    to compute the interaction matrix associated to \f$ \theta u_z\f$.

    See the interaction() method for an usage example.

    This function is also useful in the vpServo class to indicate that
    a subset of the visual feature is to use in the control law:

    \code 
    vpFeatureThetaU tu;
    vpServo task;
    ...
    // Add the (ThetaU_z) subset feature from the 3D ThetaU
    // rotation to the task
    task.addFeature(tu, vpFeatureThetaU::selectTUz());
    \endcode

    \sa selectTUx(), selectTUy()
  */
  inline static int selectTUz()  { return FEATURE_LINE[2] ; }
  // compute the interaction matrix from a subset a the possible features
  vpMatrix  interaction(const int select = FEATURE_ALL);
  // compute the error between two visual features from a subset
  // a the possible features
  vpColVector error(const vpBasicFeature &s_star,
		    const int select = FEATURE_ALL)  ;
  void print(const int select= FEATURE_ALL) const ;

  //! Feature duplication.
  vpFeatureThetaU *duplicate() const ;

public:
  void display(const vpCameraParameters &cam,
               vpImage<unsigned char> &I,
               vpColor color=vpColor::green, 
	       unsigned int thickness=1) const ;
  void display(const vpCameraParameters &cam,
               vpImage<vpRGBa> &I,
               vpColor color=vpColor::green, 
	       unsigned int thickness=1) const ;

 private:
  vpFeatureThetaURotationRepresentationType rotation;
} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
