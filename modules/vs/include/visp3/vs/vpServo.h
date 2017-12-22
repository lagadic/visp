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

#include <list>

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/vs/vpServoException.h>

/*!
  \class vpServo

  \ingroup group_task
  Class required to compute the visual servoing control law descbribed
  in \cite Chaumette06a and \cite Chaumette07a.

  \warning To avoid potential memory leaks, it is mendatory to call
  explicitly the kill() function to destroy the task. Otherwise, the
  destructor ~vpServo() launch an exception
  vpServoException::notKilledProperly.

  To learn how to use this class, we suggest first to follow the \ref
tutorial-ibvs. The \ref tutorial-simu-robot-pioneer and \ref tutorial-boost-vs
are also useful for advanced usage of this class.

  The example below shows how to build a position-based visual servo
  from 3D visual features \f$s=({^{c^*}}t_c,\theta u)\f$. In that
  case, we have \f$s^* = 0\f$. Let us denote \f$\theta u\f$ the angle/axis
  parametrization of the rotation \f${^{c^*}}R_c\f$. Moreover,\f$
  {^{c^*}}t_c\f$ and \f${^{c^*}}R_c\f$ represent respectively the
  translation and the rotation between the desired camera frame and
  the current one obtained by pose estimation (see vpPose class).

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>

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
  vpFeatureThetaU s_star_tu(vpFeatureThetaU::cdRc); // Default initialization to zero

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
  typedef enum {
    NONE,
    /*!< No control law is specified. */
    EYEINHAND_CAMERA,
    /*!< Eye in hand visual servoing with the following control law
      \f[{\bf v}_c = -\lambda {\widehat {\bf L}}^{+}_{e} {\bf e}\f]
      where camera velocities are computed. */
    EYEINHAND_L_cVe_eJe,
    /*!< Eye in hand visual servoing with the following control law
      \f[{\dot {\bf q}} = -\lambda \left( {{\widehat {\bf L}}_{e} {^c}{\bf
      V}_e {^e}{\bf J}_e} \right)^{+} {\bf e}\f] where joint velocities are
      computed. */
    EYETOHAND_L_cVe_eJe,
    /*!< Eye to hand visual servoing with the following control law
      \f[{\dot {\bf q}} = \lambda \left( {{\widehat {\bf L}}_{e} {^c}{\bf V}_e
      {^e}{\bf J}_e} \right)^{+} {\bf e}\f] where joint velocities are
      computed. */
    EYETOHAND_L_cVf_fVe_eJe,
    /*!< Eye to hand visual servoing with the following control law
      \f[{\dot {\bf q}} = \lambda \left( {{\widehat {\bf L}}_{e} {^c}{\bf V}_f
      {^f}{\bf V}_e {^e}{\bf J}_e} \right)^{+} {\bf e}\f] where joint
      velocities are computed. */
    EYETOHAND_L_cVf_fJe
    /*!< Eye to hand visual servoing with the following control law
      \f[{\dot {\bf q}} = \lambda \left( {{\widehat {\bf L}}_{e} {^c}{\bf V}_f
      {^f}{\bf J}_e} \right)^{+} {\bf e}\f] where joint velocities are
      computed. */
  } vpServoType;

  typedef enum {
    CURRENT,
    /*!< In the control law (see vpServo::vpServoType), uses the interaction
       matrix \f${\widehat {\bf L}}_s \f$computed using the current features
       \f$\bf s\f$. */
    DESIRED,
    /*!< In the control law (see vpServo::vpServoType), uses the interaction
       matrix \f${\widehat {\bf L}}_{s^*} \f$computed using the desired
       features \f${\bf s}^*\f$. */
    MEAN,
    /*!< In the control law (see vpServo::vpServoType), uses the interaction
       matrix \f${\widehat {\bf L}} = \left({\widehat {\bf L}}_s + {\widehat
       {\bf L}}_{s^*}\right)/2 \f$. */
    USER_DEFINED
    /*!< In the control law (see vpServo::vpServoType), uses an interaction
       matrix set by the user. */
  } vpServoIteractionMatrixType;

  typedef enum {
    TRANSPOSE,     /*!< In the control law (see vpServo::vpServoType), uses the
                      transpose instead of the pseudo inverse. */
    PSEUDO_INVERSE /*!< In the control law (see vpServo::vpServoType), uses
                      the pseudo inverse. */
  } vpServoInversionType;

  typedef enum {
    ALL,                /*!< Print all the task information. */
    CONTROLLER,         /*!< Print the type of controller law. */
    ERROR_VECTOR,       /*!< Print the error vector \f$\bf e = (s-s^*)\f$. */
    FEATURE_CURRENT,    /*!< Print the current features \f$\bf s\f$. */
    FEATURE_DESIRED,    /*!< Print the desired features \f${\bf s}^*\f$. */
    GAIN,               /*!< Print the gain \f$\lambda\f$. */
    INTERACTION_MATRIX, /*!< Print the interaction matrix. */
    MINIMUM             /*!< Same as vpServo::vpServoPrintType::ERROR_VECTOR. */
  } vpServoPrintType;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  vpServo(const vpServo &)
  //    : L(), error(), J1(), J1p(), s(), sStar(), e1(), e(), q_dot(), v(),
  //    servoType(vpServo::NONE),
  //      rankJ1(0), featureList(), desiredFeatureList(),
  //      featureSelectionList(), lambda(), signInteractionMatrix(1),
  //      interactionMatrixType(DESIRED), inversionType(PSEUDO_INVERSE),
  //      cVe(), init_cVe(false), cVf(), init_cVf(false), fVe(),
  //      init_fVe(false), eJe(), init_eJe(false), fJe(), init_fJe(false),
  //      errorComputed(false), interactionMatrixComputed(false), dim_task(0),
  //      taskWasKilled(false), forceInteractionMatrixComputation(false),
  //      WpW(), I_WpW(), P(), sv(), mu(4.), e1_initial()
  //  {
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!");
  //  }
  //  vpServo &operator=(const vpServo &){
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!"); return *this;
  //  }
  //#endif

public:
  // default constructor
  vpServo();
  // constructor with Choice of the visual servoing control law
  explicit vpServo(vpServoType servoType);
  // destructor
  virtual ~vpServo();

  // create a new ste of  two visual features
  void addFeature(vpBasicFeature &s, vpBasicFeature &s_star, const unsigned int select = vpBasicFeature::FEATURE_ALL);
  // create a new ste of  two visual features
  void addFeature(vpBasicFeature &s, const unsigned int select = vpBasicFeature::FEATURE_ALL);

  // compute the desired control law
  vpColVector computeControlLaw();
  // compute the desired control law
  vpColVector computeControlLaw(double t);
  vpColVector computeControlLaw(double t, const vpColVector &e_dot_init);

  // compute the error between the current set of visual features and
  // the desired set of visual features
  vpColVector computeError();
  // compute the interaction matrix related to the set of visual features
  vpMatrix computeInteractionMatrix();

  // Return the task dimension.
  unsigned int getDimension() const;
  /*!
   Return the error \f$\bf e = (s - s^*)\f$ between the current set of visual
features \f$\bf s\f$ and the desired set of visual features \f$\bf s^*\f$. The
error vector is updated after a call of computeError() or computeControlLaw().
\code
  vpServo task;
  ...
  vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
  vpColVector e = task.getError(); // Get the error vector
\endcode
   */
  inline vpColVector getError() const { return error; }

  /*
     Return the interaction matrix \f$L\f$ used to compute the task jacobian
 \f$J_1\f$. The interaction matrix is updated after a call to
 computeInteractionMatrix() or computeControlLaw().

 \code
   vpServo task;
   ...
   vpColVector v = task.computeControlLaw();    // Compute the velocity
 corresponding to the visual servoing vpMatrix    L =
 task.getInteractionMatrix(); // Get the interaction matrix used to compute v
 \endcode
     \sa getTaskJacobian()
   */
  inline vpMatrix getInteractionMatrix() const { return L; }

  vpMatrix getI_WpW() const;
  /*!
     Return the visual servo type.
   */
  inline vpServoType getServoType() const { return servoType; }

  vpMatrix getLargeP() const;

  vpMatrix getTaskJacobian() const;
  vpMatrix getTaskJacobianPseudoInverse() const;
  unsigned int getTaskRank() const;

  /*!
     Get task singular values.

     \return Singular values that relies on the task jacobian pseudo inverse.
     */
  inline vpColVector getTaskSingularValues() const { return sv; }

  vpMatrix getWpW() const;

  /*!
    Return the velocity twist matrix used to transform a velocity skew vector
    from end-effector frame into the camera frame.
  */
  vpVelocityTwistMatrix get_cVe() const { return cVe; }
  /*!
    Return the velocity twist matrix used to transform a velocity skew vector
    from robot fixed frame (also called world or base frame) into the camera
    frame.
  */
  vpVelocityTwistMatrix get_cVf() const { return cVf; }
  /*!
    Return the velocity twist matrix used to transform a velocity skew vector
    from robot end-effector frame into the fixed frame (also called world or
    base frame).
  */
  vpVelocityTwistMatrix get_fVe() const { return fVe; }
  /*!
    Return the robot jacobian expressed in the end-effector frame.
  */
  vpMatrix get_eJe() const { return eJe; }
  /*!
    Return the robot jacobian expressed in the robot fixed frame (also called
    world or base frame).
  */
  vpMatrix get_fJe() const { return fJe; }

  // destruction (memory deallocation if required)
  void kill();

  void print(const vpServo::vpServoPrintType display_level = ALL, std::ostream &os = std::cout);

  // Add a secondary task.
  vpColVector secondaryTask(const vpColVector &de2dt, const bool &useLargeProjectionOperator = false);
  // Add a secondary task.
  vpColVector secondaryTask(const vpColVector &e2, const vpColVector &de2dt,
                            const bool &useLargeProjectionOperator = false);
  // Add a secondary task to avoid the joint limit.
  vpColVector secondaryTaskJointLimitAvoidance(const vpColVector &q, const vpColVector &dq, const vpColVector &jointMin,
                                               const vpColVector &jointMax, const double &rho = 0.1,
                                               const double &rho1 = 0.3, const double &lambda_tune = 0.7) const;

  void setCameraDoF(const vpColVector &dof);

  /*!
    Set a variable which enables to compute the interaction matrix at each
    iteration.

    When the interaction matrix is computed from the desired features \f${\bf
    s}^*\f$ which are in general constant, the interaction matrix \f${\widehat
    {\bf L}}_{s^*}\f$ is computed just at the first iteration of the servo
    loop. Sometimes, when the desired features are time dependent \f${{\bf
    s}(t)}^*\f$ or varying, the interaction matrix need to be computed at each
    iteration of the servo loop. This method allows to force the computation
    of \f${\widehat {\bf L}}\f$ in this particular case.

    \param force_computation : If true it forces the interaction matrix
    computation even if it is already done.

  */
  void setForceInteractionMatrixComputation(bool force_computation)
  {
    this->forceInteractionMatrixComputation = force_computation;
  }

  /*!
    Set the interaction matrix type (current, desired, mean or user defined)
    and how its inverse is computed. \param interactionMatrixType : The
    interaction matrix type. See vpServo::vpServoIteractionMatrixType for more
    details. \param interactionMatrixInversion : How is the inverse computed.
    See vpServo::vpServoInversionType for more details.
    */
  void setInteractionMatrixType(const vpServoIteractionMatrixType &interactionMatrixType,
                                const vpServoInversionType &interactionMatrixInversion = PSEUDO_INVERSE);

  /*!
    Set the gain \f$\lambda\f$ used in the control law (see
    vpServo::vpServoType) as constant.

    The usage of an adaptive gain allows to reduce the convergence time, see
    setLambda(const vpAdaptiveGain&).

    \param c : Constant gain. Values are in general between 0.1 and 1. Higher
    is the gain, higher are the velocities that may be applied to the robot.
   */
  void setLambda(double c) { lambda.initFromConstant(c); }

  /*!
    Set the gain \f$\lambda\f$ used in the control law (see
    vpServo::vpServoType) as adaptive. Value of \f$\lambda\f$ that is used in
    computeControlLaw() depend on the infinity norm of the task Jacobian.

    The usage of an adaptive gain rather than a constant gain allows to reduce
    the convergence time.

    \param gain_at_zero : the expected gain when \f$x=0\f$: \f$\lambda(0)\f$.
    \param gain_at_infinity : the expected gain when \f$x=\infty\f$:
    \f$\lambda(\infty)\f$. \param slope_at_zero : the expected slope of
    \f$\lambda(x)\f$ when \f$x=0\f$: \f${\dot \lambda}(0)\f$.

    For more details on these parameters see vpAdaptiveGain class.
   */
  void setLambda(const double gain_at_zero, const double gain_at_infinity, const double slope_at_zero)
  {
    lambda.initStandard(gain_at_zero, gain_at_infinity, slope_at_zero);
  }
  /*!
    Set the gain \f$\lambda\f$ used in the control law (see
    vpServo::vpServoType) as adaptive. Value of \f$\lambda\f$ that is used in
    computeControlLaw() depend on the infinity norm of the task Jacobian.

    The usage of an adaptive gain rather than a constant gain allows to reduce
    the convergence time. \sa vpAdaptiveGain
   */
  void setLambda(const vpAdaptiveGain &l) { lambda = l; }
  /*!
    Set the value of the parameter \f$\mu\f$ used to ensure the continuity of
    the velocities computed using computeControlLaw(double).

    A recommended value is 4.
  */
  void setMu(double mu_) { this->mu = mu_; }
  //  Choice of the visual servoing control law
  void setServo(const vpServoType &servo_type);

  /*!
    Set the velocity twist matrix used to transform a velocity skew vector
    from end-effector frame into the camera frame.
   */
  void set_cVe(const vpVelocityTwistMatrix &cVe_)
  {
    this->cVe = cVe_;
    init_cVe = true;
  }
  /*!
    Set the velocity twist matrix used to transform a velocity skew vector
    from end-effector frame into the camera frame.
   */
  void set_cVe(const vpHomogeneousMatrix &cMe)
  {
    cVe.buildFrom(cMe);
    init_cVe = true;
  }
  /*!
    Set the velocity twist matrix used to transform a velocity skew vector
    from robot fixed frame (also called world or base frame) into the camera
    frame.
   */
  void set_cVf(const vpVelocityTwistMatrix &cVf_)
  {
    this->cVf = cVf_;
    init_cVf = true;
  }
  /*!
    Set the velocity twist matrix used to transform a velocity skew vector
    from robot fixed frame (also called world or base frame) into the camera
    frame.
   */
  void set_cVf(const vpHomogeneousMatrix &cMf)
  {
    cVf.buildFrom(cMf);
    init_cVf = true;
  }
  /*!
    Set the velocity twist matrix used to transform a velocity skew vector
    from robot end-effector frame into the fixed frame (also called world or
    base frame).
   */
  void set_fVe(const vpVelocityTwistMatrix &fVe_)
  {
    this->fVe = fVe_;
    init_fVe = true;
  }
  /*!
    Set the velocity twist matrix used to transform a velocity skew vector
    from robot end-effector frame into the fixed frame (also called world or
    base frame).
   */
  void set_fVe(const vpHomogeneousMatrix &fMe)
  {
    fVe.buildFrom(fMe);
    init_fVe = true;
  }

  /*!
    Set the robot jacobian expressed in the end-effector frame.
   */
  void set_eJe(const vpMatrix &eJe_)
  {
    this->eJe = eJe_;
    init_eJe = true;
  }
  /*!
    Set the robot jacobian expressed in the robot fixed frame (also called
    world or base frame).
   */
  void set_fJe(const vpMatrix &fJe_)
  {
    this->fJe = fJe_;
    init_fJe = true;
  }

  /*!
    Test if all the initialization are correct. If true, the control law can
    be computed.
    */
  bool testInitialization();
  /*!
    Test if all the update are correct. If true control law can be computed.
    */
  bool testUpdated();

protected:
  //! Basic initialization.
  void init();

  /*!
    Compute the classic projetion operator and the large projection operator.
   */
  void computeProjectionOperators();

public:
  //! Interaction matrix
  vpMatrix L;
  //! Error \f$(s - s^*)\f$ between the current set of visual features
  //! \f$s\f$ and the desired set of visual features \f$s^*\f$.
  //! This vector is updated after a call of computeError() or
  //! computeControlLaw().
  vpColVector error;
  //! Task Jacobian  \f$J_1 = L {^c}V_a {^a}J_e\f$.
  vpMatrix J1;
  //! Pseudo inverse \f${J_1}^{+}\f$ of the task Jacobian.
  vpMatrix J1p;

  //! Current state of visual features \f$s\f$.
  //! This vector is updated after a call of computeError() or
  //! computeControlLaw().
  vpColVector s;
  //! Desired state of visual features \f$s^*\f$.
  //! This vector is updated after a call of computeError() or
  //! computeControlLaw().
  vpColVector sStar;

  //! Primary task \f$e_1 = {J_1}^{+}(s-s*)\f$
  vpColVector e1;
  //! Task \f$e = e_1 + (I-{J_1}^{+} J_1) e_2\f$
  vpColVector e;

  //! Articular velocity
  vpColVector q_dot;
  //! Camera velocity
  vpColVector v;

  //! Chosen visual servoing control law
  vpServoType servoType;

  //! Rank of the task Jacobian
  unsigned int rankJ1;

  //! List of current visual features \f$\bf s\f$.
  std::list<vpBasicFeature *> featureList;
  //! List of desired visual features \f$\bf s^*\f$.
  std::list<vpBasicFeature *> desiredFeatureList;
  //! List of selection among visual features
  //! used for selection of a subset of each visual feature if required.
  std::list<unsigned int> featureSelectionList;

  //! Gain used in the control law.
  vpAdaptiveGain lambda;

  //! Sign of the interaction +/- 1 (1 for eye-in-hand, -1 for
  //! eye-to-hand configuration)
  int signInteractionMatrix;
  //! Type of the interaction matrox (current, mean, desired, user)
  vpServoIteractionMatrixType interactionMatrixType;
  //! Indicates if the transpose or the pseudo inverse of the
  //! interaction matrix should be used to compute the task.
  vpServoInversionType inversionType;

protected:
  /*
    Twist transformation matrix
  */

  //! Twist transformation matrix between Re and Rc.
  vpVelocityTwistMatrix cVe;
  bool init_cVe;
  //! Twist transformation matrix between Rf and Rc.
  vpVelocityTwistMatrix cVf;
  bool init_cVf;
  //! Twist transformation matrix between Re and Rf.
  vpVelocityTwistMatrix fVe;
  bool init_fVe;

  /*
    Jacobians
  */

  //! Jacobian expressed in the end-effector frame.
  vpMatrix eJe;
  bool init_eJe;
  //! Jacobian expressed in the robot reference frame.
  vpMatrix fJe;
  bool init_fJe;

  /*
    Task building
  */

  //! true if the error has been computed.
  bool errorComputed;
  //! true if the interaction matrix has been computed.
  bool interactionMatrixComputed;
  //! Dimension of the task updated during computeControlLaw().
  unsigned int dim_task;
  //! Flag to indicate if the task was killed
  bool taskWasKilled;
  //! Force the interaction matrix computation even if it is already done.
  bool forceInteractionMatrixComputation;

  //! Projection operators \f$\bf WpW\f$.
  vpMatrix WpW;
  //! Projection operators \f$\bf I-WpW\f$.
  vpMatrix I_WpW;
  /*!
    New Large projection operator (see equation(24) in the paper
  \cite Marey:2010). This projection operator allows performing secondary task
  even when the main task is full rank. \f[
   {\bf P} =\bar{\lambda }\left ( \left \| {\bf e} \right \| \right ){\bf P}_{
  \left \| {\bf e } \right \| } \left ( 1 - \bar{\lambda }\left ( \left \|
  {\bf e } \right \| \right ) \right ) \left (  {\bf I-W^+W}\right ) \f]

  with

  \f[
  {\bf P}_{\left \| {\bf e } \right \| } = I_{n} - \frac{1}{{\bf e }^\top {\bf
  J_{{\bf e }} } {\bf J_{{\bf e }}^\top }
  {\bf e }}{\bf J_{{\bf e }}^\top }{\bf e }{\bf e }^\top{\bf J_{{\bf e }} }
  \f]

   */
  vpMatrix P;

  //! Singular values from the pseudo inverse.
  vpColVector sv;

  double mu;

  vpColVector e1_initial;

  //! Boolean to know if cJc is identity (for fast computation)
  bool iscJcIdentity;

  //! A diag matrix used to determine which are the degrees of freedom that
  //! are controlled in the camera frame
  vpMatrix cJc;
};

#endif
