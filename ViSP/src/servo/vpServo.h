/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Visual servoing control law.
 *
 * Authors:
 * Eric Marchand
 * Nicolas Mansard
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpServo_H
#define vpServo_H

/*!
  \file vpServo.h
  \brief  Class required to compute the visual servoing control law
*/

#include <visp/vpMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpServoException.h>

#include <visp/vpList.h>
#include <visp/vpAdaptiveGain.h>


/*!
  \class vpServo

  \ingroup VsTask
  \brief Class required to compute the visual servoing control law.

  \warning To avoid potential memory leaks, it is mendatory to call
  explicitly the kill() function to destroy the task. Otherwise, the
  destructor ~vpServo() launch an exception
  vpServoException::notKilledProperly.

  The example below shows how to build a position-based visual servo
  from 3D visual features \f$s=({^{c^*}}t_c,\theta u)\f$. In that
  case, we have \f$s^* = 0\f$. Let us denote \f$\theta u\f$ the angle/axis
  parametrization of the rotation \f${^{c^*}}R_c\f$. Moreover,\f$
  {^{c^*}}t_c\f$ and \f${^{c^*}}R_c\f$ represent respectively the
  translation and the rotation between the desired camera frame and
  the current one obtained by pose estimation (see vpPose class).

  \code
#include <visp/vpColVector.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>
#include <visp/vpServo.h>

int main()
{
  // Creation of an homogeneous matrix that represent the displacement
  // the camera has to achieve to move from the desired camera frame
  // and the current one
  vpHomogeneousMatrix cdMc;

  // ... cdMc is here the result of a pose estimation

  // Creation of the current visual feature s = (c*_t_c, ThetaU)
  vpFeatureTranslation s_t(vpFeatureTranslation::cdMc);
  vpFeatureThetaU s_tu(vpFeatureThetaU::cdRc);
  // Set the initial values of the current visual feature s = (c*_t_c, ThetaU)
  s_t.buildFrom(cdMc);
  s_tu.buildFrom(cdMc);

  // Build the desired visual feature s* = (0,0)
  vpFeatureTranslation s_star_t(vpFeatureTranslation::cdMc); // Default initialization to zero 
  vpFeatureThetaU s_star_tu(vpFeatureThetaU::cdRc);// Default initialization to zero 

  vpColVector v; // Camera velocity
  double error;  // Task error

  // Creation of the visual servo task.
  vpServo task;

  // Visual servo task initialization
  // - Camera is monted on the robot end-effector and velocities are
  //   computed in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA); 
  // - Interaction matrix is computed with the current visual features s
  task.setInteractionMatrixType(vpServo::CURRENT); 
  // - Set the contant gain to 1
  task.setLambda(1);
  // - Add current and desired translation feature
  task.addFeature(s_t, s_star_t); 
  // - Add current and desired ThetaU feature for the rotation
  task.addFeature(s_tu, s_star_tu); 

  // Visual servoing loop. The objective is here to update the visual
  // features s = (c*_t_c, ThetaU), compute the control law and apply
  // it to the robot
  do {
    // ... cdMc is here the result of a pose estimation

    // Update the current visual feature s
    s_t.buildFrom(cdMc);  // Update translation visual feature
    s_tu.buildFrom(cdMc); // Update ThetaU visual feature

    v = task.computeControlLaw(); // Compute camera velocity skew
    error =  ( task.getError() ).sumSquare(); // error = s^2 - s_star^2
  } while (error > 0.0001); // Stop the task when current and desired visual features are close

  // A call to kill() is requested here to destroy properly the current
  // and desired feature lists.
  task.kill();
}
  \endcode

*/

class VISP_EXPORT vpServo
{
  /*
    Choice of the visual servoing control law
  */
public:
  typedef enum
    {
      NONE,
      EYEINHAND_CAMERA,
      EYEINHAND_L_cVe_eJe,
      EYETOHAND_L_cVe_eJe,
      EYETOHAND_L_cVf_fVe_eJe,
      EYETOHAND_L_cVf_fJe
    } vpServoType;

  typedef enum
    {
      CURRENT,
      DESIRED,
      MEAN,
      USER_DEFINED
    } vpServoIteractionMatrixType;

  typedef enum
    {
      TRANSPOSE,
      PSEUDO_INVERSE
    } vpServoInversionType;

  typedef enum
    {
      ALL,                /*!< Print all the task information. */
      CONTROLLER,         /*!< Print the type of controller law. */
      ERROR_VECTOR,       /*!< Print the error vector \f$(s-s^*)\f$. */
      FEATURE_CURRENT,    /*!< Print the current features \f$s\f$. */
      FEATURE_DESIRED,    /*!< Print the desired features \f$s^*\f$. */
      GAIN,               /*!< Print the gain \f$\lambda\f$. */
      INTERACTION_MATRIX, /*!< Print the interaction matrix. */
      MINIMUM             /*!< Same as vpServoPrintType::ERROR. */
    } vpServoPrintType;

public:
  // default constructor
  vpServo();
  //! constructor with Choice of the visual servoing control law
  vpServo(vpServoType _servoType) ;
  //! destructor
  virtual ~vpServo() ;
  //! destruction (memory deallocation if required)
  void kill() ;

  //!  Choice of the visual servoing control law
  void setServo(vpServoType _servo_type) ;

  void set_cVe(vpVelocityTwistMatrix &_cVe) { cVe = _cVe ; init_cVe = true ; }
  void set_cVf(vpVelocityTwistMatrix &_cVf) { cVf = _cVf ; init_cVf = true ; }
  void set_fVe(vpVelocityTwistMatrix &_fVe) { fVe = _fVe ; init_fVe = true ; }

  void set_cVe(vpHomogeneousMatrix &cMe) { cVe.buildFrom(cMe); init_cVe=true ;}
  void set_cVf(vpHomogeneousMatrix &cMf) { cVf.buildFrom(cMf); init_cVf=true ;}
  void set_fVe(vpHomogeneousMatrix &fMe) { fVe.buildFrom(fMe); init_fVe=true ;}

  void set_eJe(vpMatrix &_eJe) { eJe = _eJe ; init_eJe = true ; }
  void set_fJe(vpMatrix &_fJe) { fJe = _fJe ; init_fJe = true ; }


  //! Set the type of the interaction matrix (current, mean, desired, user).
  void setInteractionMatrixType(const vpServoIteractionMatrixType &interactionMatrixType,
				const vpServoInversionType &interactionMatrixInversion=PSEUDO_INVERSE) ;

  /*! 
    Set a variable which enable to compute the interaction matrix for each iteration.
    \param forceInteractionMatrixComputation: If true it forces the interaction matrix computation even if it is already done.
  */
  void setForceInteractionMatrixComputation(bool forceInteractionMatrixComputation) {this->forceInteractionMatrixComputation = forceInteractionMatrixComputation;}

  //! set the gain lambda
  void setLambda(double _lambda) { lambda .initFromConstant (_lambda) ; }
  void setLambda(const double at_zero,
		 const double at_infinity,
		 const double deriv_at_zero)
  { lambda .initStandard (at_zero, at_infinity, deriv_at_zero) ; }
  void setLambda(const vpAdaptiveGain& _l){lambda=_l;}

  //! create a new ste of  two visual features
  void addFeature(vpBasicFeature& s, vpBasicFeature& s_star,
	       const unsigned int select=vpBasicFeature::FEATURE_ALL) ;
  //! create a new ste of  two visual features
  void addFeature(vpBasicFeature& s,
	       const unsigned int select=vpBasicFeature::FEATURE_ALL) ;

  //! compute the interaction matrix related to the set of visual features
  vpMatrix computeInteractionMatrix() ;
  // compute the error between the current set of visual features and
  // the desired set of visual features
  vpColVector computeError() ;
  //! compute the desired control law
  vpColVector computeControlLaw() ;
  //! test if all the initialization are correct if true control law can
  //! be computed
  bool testInitialization() ;
  //! test if all the update are correct if true control law can
  //! be computed
  bool testUpdated() ;


  //! Add a secondary task.
  vpColVector secondaryTask(vpColVector &de2dt) ;
  //! Add a secondary task.
  vpColVector secondaryTask(vpColVector &e2, vpColVector &de2dt) ;

  //! Return the task dimension.
  unsigned int getDimension() ;

  /*!
   Return the error \f$(s - s^*)\f$ between the current set of visual features
   \f$s\f$ and the desired set of visual features \f$s^*\f$.
   The error vector is updated after a call of computeError() or computeControlLaw().
\code
  vpServo task;
  ...
  vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
  vpColVector e = task.getError();          // Get the error vector
\endcode
   */
  inline vpColVector getError() const
  {
    return error ;
  }
  /*
    Return the interaction matrix \f$L\f$ used to compute the task jacobian \f$J_1\f$.
    The interaction matrix is updated after a call to computeInteractionMatrix() or computeControlLaw().

\code
  vpServo task;
  ...
  vpColVector v = task.computeControlLaw();    // Compute the velocity corresponding to the visual servoing
  vpMatrix    L = task.getInteractionMatrix(); // Get the interaction matrix used to compute v
\endcode
    \sa getTaskJacobian()
  */
  inline vpMatrix getInteractionMatrix()
  {
    return L;
  }

  /*!
    Return the projection operator \f${\bf I}-{\bf W}^+{\bf W}\f$. This operator is updated
    after a call of computeControlLaw().

\code
  vpServo task;
  ...
  vpColVector  v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
  vpMatrix I_WpW = task.getI_WpW();          // Get the projection operator
\endcode
    \sa getWpW()
  */
  inline vpMatrix getI_WpW() const
  {
    return I_WpW;
  }
  /*!
    Return the visual servo type.
  */
  inline vpServoType getServoType() const
  {
    return servoType;
  }
  /*!
    Return the task jacobian \f$J_1\f$. The task jacobian is updated after a call of computeControlLaw().

    In the general case, the task jacobian is given by \f$J_1 = L {^c}V_a {^a}J_e\f$.
\code
  vpServo task;
  ...
  vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
  vpMatrix   J1 = task.getTaskJacobian();   // Get the task jacobian used to compute v
\endcode
    \sa getTaskJacobianPseudoInverse(), getInteractionMatrix()
  */
  inline vpMatrix getTaskJacobian() const
  {
    return J1;
  }
  /*!
    Return the pseudo inverse of the task jacobian \f$J_1\f$. The task jacobian
    and its pseudo inverse are updated after a call of computeControlLaw().

    \return Pseudo inverse \f${J_1}^{+}\f$ of the task jacobian.
\code
  vpServo task;
  ...
  vpColVector v = task.computeControlLaw();            // Compute the velocity corresponding to the visual servoing
  vpMatrix  J1p = task.getTaskJacobianPseudoInverse(); // Get the pseudo inverse of task jacobian used to compute v
\endcode

  \sa getTaskJacobian()
  */
  inline vpMatrix getTaskJacobianPseudoInverse() const
  {
    return J1p;
  }
  /*!
    Return the rank of the task jacobian. The rank is updated after a call of computeControlLaw().

\code
  vpServo task;
  ...
  vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
  double   rank = task.getTaskRank();       // Get the rank of the task jacobian
\endcode
  */
  inline double getTaskRank() const
  {
    return rankJ1;
  }
  /*!
    Return the projection operator \f${\bf W}^+{\bf W}\f$. This operator is updated
    after a call of computeControlLaw().

\code
  vpServo task;
  ...
  vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
  vpMatrix  WpW = task.getWpW();            // Get the projection operator
\endcode
    \sa getI_WpW()
  */
  inline vpMatrix getWpW() const
  {
    return WpW;
  }


  void print(const vpServo::vpServoPrintType display_level=ALL,
	     std::ostream &os = std::cout) ;
protected:
  //! basic initialization
  void init() ;

public:
  //! Interaction matrix
  vpMatrix L ;
  //! Error \f$(s - s^*)\f$ between the current set of visual features
  //! \f$s\f$ and the desired set of visual features \f$s^*\f$.
  //! This vector is updated after a call of computeError() or computeControlLaw().
  vpColVector error ;
  //! Task Jacobian  \f$J_1 = L {^c}V_a {^a}J_e\f$.
  vpMatrix J1 ;
  //! Pseudo inverse \f${J_1}^{+}\f$ of the task Jacobian.
  vpMatrix J1p ;

  //! Current state of visual features \f$s\f$.
  //! This vector is updated after a call of computeError() or computeControlLaw().
  vpColVector s ;
  //! Desired state of visual features \f$s^*\f$.
  //! This vector is updated after a call of computeError() or computeControlLaw().
  vpColVector sStar ;

  //! Primary task \f$e_1 = {J_1}^{+}(s-s*)\f$
  vpColVector e1 ;
  //! Task \f$e = e_1 + (I-{J_1}^{+} J_1) e_2\f$
  vpColVector e ;


  //! Articular velocity
  vpColVector q_dot ;
  //! Camera velocity
  vpColVector v ;

  //! Chosen visual servoing control law
  vpServoType servoType;

  //! Rank of the task Jacobian
  unsigned int rankJ1 ;

  //! List of visual features (produce \f$s\f$)
  vpList<vpBasicFeature *> featureList ;
  //! List of desired visual features (produce \f$s^*\f$)
  vpList<vpBasicFeature *> desiredFeatureList ;
  //! List of selection among visual features
  //! used for selection of a subset of each visual feature if required
  vpList<unsigned int> featureSelectionList ;

  //! Gain
  vpAdaptiveGain lambda ;

  //! Sign of the interaction +/- 1 (1 for eye-in-hand, -1 for
  //! eye-to-hand configuration)
  int signInteractionMatrix ;
  //! Type of the interaction matrox (current, mean, desired, user)
  vpServoIteractionMatrixType interactionMatrixType ;
  //! Indicates if the transpose or the pseudo inverse of the
  //!interaction matrix should be used to compute the task
  vpServoInversionType inversionType ;

protected:
  /*
    Twist transformation matrix
  */

  //! Twist transformation matrix between Re and Rc
  vpVelocityTwistMatrix cVe ;
  bool init_cVe ;
  //! Twist transformation matrix between Rf and Rc
  vpVelocityTwistMatrix cVf ;
  bool init_cVf ;
  //! Twist transformation matrix between Re and Rf
  vpVelocityTwistMatrix fVe ;
  bool init_fVe ;

  /*
    Jacobians
  */

  //! Jacobian expressed in the end-effector frame.
  vpMatrix eJe ;
  bool init_eJe ;
  //! Jacobian expressed in the robot reference frame.
  vpMatrix fJe ;
  bool init_fJe ;

  /*
    Task building
  */

  //! true if the error has been computed
  bool errorComputed ;
  //! true if the interaction matrix has been computed
  bool interactionMatrixComputed ;
  //! dimension of the task
  unsigned int dim_task ;
  bool taskWasKilled; // flag to indicate if the task was killed
  //! Force the interaction matrix computation even if it is already done.
  bool forceInteractionMatrixComputation;

  //! projection operators WpW
  vpMatrix WpW ;
  //! projection operators I-WpW
  vpMatrix I_WpW ;
} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
