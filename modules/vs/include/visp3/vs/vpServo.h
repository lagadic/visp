/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
 * \file vpServo.h
 * \brief  Class required to compute the visual servoing control law.
 */

#ifndef _vpServo_h_
#define _vpServo_h_

#include <list>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/vs/vpServoException.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpServo
 *
 * \ingroup group_task
 * Class required to compute the visual servoing control law described
 * in \cite Chaumette06a and \cite Chaumette07a.
 *
 * \warning To avoid potential memory leaks, it is mandatory to call
 * explicitly the kill() function to destroy the task. Otherwise, the
 * destructor ~vpServo() launch an exception
 * vpServoException::notKilledProperly.
 *
 * To learn how to use this class, we suggest first to follow the \ref
 * tutorial-ibvs. The \ref tutorial-simu-robot-pioneer and \ref tutorial-boost-vs
 * are also useful for advanced usage of this class.
 *
 * The example below shows how to build a position-based visual servo
 * from 3D visual features \f$s=({^{c^*}}t_c,\theta u)\f$. In that
 * case, we have \f$s^* = 0\f$. Let us denote \f$\theta u\f$ the angle/axis
 * parametrization of the rotation \f${^{c^*}}R_c\f$. Moreover,\f$
 * {^{c^*}}t_c\f$ and \f${^{c^*}}R_c\f$ represent respectively the
 * translation and the rotation between the desired camera frame and
 * the current one obtained by pose estimation (see vpPose class).
 *
 * \code
 * #include <visp3/core/vpColVector.h>
 * #include <visp3/core/vpHomogeneousMatrix.h>
 * #include <visp3/core/vpMatrix.h>
 * #include <visp3/visual_features/vpFeatureThetaU.h>
 * #include <visp3/visual_features/vpFeatureTranslation.h>
 * #include <visp3/vs/vpServo.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   // Creation of an homogeneous matrix that represent the displacement
 *   // the camera has to achieve to move from the desired camera frame
 *   // and the current one
 *   vpHomogeneousMatrix cdMc;
 *
 *   // ... cdMc is here the result of a pose estimation
 *
 *   // Creation of the current visual feature s = (c*_t_c, ThetaU)
 *   vpFeatureTranslation s_t(vpFeatureTranslation::cdMc);
 *   vpFeatureThetaU s_tu(vpFeatureThetaU::cdRc);
 *   // Set the initial values of the current visual feature s = (c*_t_c, ThetaU)
 *   s_t.build(cdMc);
 *   s_tu.build(cdMc);
 *
 *   // Build the desired visual feature s* = (0,0)
 *   vpFeatureTranslation s_star_t(vpFeatureTranslation::cdMc); // Default initialization to zero
 *   vpFeatureThetaU s_star_tu(vpFeatureThetaU::cdRc); // Default initialization to zero
 *
 *   vpColVector v; // Camera velocity
 *   double error;  // Task error
 *
 *   // Creation of the visual servo task.
 *   vpServo task;
 *
 *   // Visual servo task initialization
 *   // - Camera is mounted on the robot end-effector and velocities are
 *   //   computed in the camera frame
 *   task.setServo(vpServo::EYEINHAND_CAMERA);
 *   // - Interaction matrix is computed with the current visual features s
 *   task.setInteractionMatrixType(vpServo::CURRENT);
 *   // - Set the constant gain to 1
 *   task.setLambda(1);
 *   // - Add current and desired translation feature
 *   task.addFeature(s_t, s_star_t);
 *   // - Add current and desired ThetaU feature for the rotation
 *   task.addFeature(s_tu, s_star_tu);
 *
 *   // Visual servoing loop. The objective is here to update the visual
 *   // features s = (c*_t_c, ThetaU), compute the control law and apply
 *   // it to the robot
 *   do {
 *     // ... cdMc is here the result of a pose estimation
 *
 *     // Update the current visual feature s
 *     s_t.build(cdMc);  // Update translation visual feature
 *     s_tu.build(cdMc); // Update ThetaU visual feature
 *
 *     v = task.computeControlLaw(); // Compute camera velocity skew
 *     error =  ( task.getError() ).sumSquare(); // error = s^2 - s_star^2
 *   } while (error > 0.0001); // Stop the task when current and desired visual features are close
 * }
 * \endcode
*/
class VISP_EXPORT vpServo
{

public:
  /*!
   * Choice of the visual servoing control law.
   */
  typedef enum
  {
    /*!
     * No control law is specified.
     */
    NONE,
    /*!
     * Eye in hand visual servoing with the following control law
     * \f[{\bf v}_c = -\lambda {\widehat {\bf L}}^{+}_{e} {\bf e}\f]
     * where camera velocities are computed.
     */
    EYEINHAND_CAMERA,
    /*!
     * Eye in hand visual servoing with the following control law
     * \f[{\dot {\bf q}} = -\lambda \left( {{\widehat {\bf L}}_{e} {^c}{\bf
     * V}_e {^e}{\bf J}_e} \right)^{+} {\bf e}\f] where joint velocities are
     * computed.
     */
    EYEINHAND_L_cVe_eJe,
    /*!
     * Eye to hand visual servoing with the following control law
     * \f[{\dot {\bf q}} = \lambda \left( {{\widehat {\bf L}}_{e} {^c}{\bf V}_e
     * {^e}{\bf J}_e} \right)^{+} {\bf e}\f] where joint velocities are
     * computed.
     */
    EYETOHAND_L_cVe_eJe,
    /*!
     * Eye to hand visual servoing with the following control law
     * \f[{\dot {\bf q}} = \lambda \left( {{\widehat {\bf L}}_{e} {^c}{\bf V}_f
     * {^f}{\bf V}_e {^e}{\bf J}_e} \right)^{+} {\bf e}\f] where joint
     * velocities are computed.
     */
    EYETOHAND_L_cVf_fVe_eJe,
    /*!
     * Eye to hand visual servoing with the following control law
     * \f[{\dot {\bf q}} = \lambda \left( {{\widehat {\bf L}}_{e} {^c}{\bf V}_f
     * {^f}{\bf J}_e} \right)^{+} {\bf e}\f] where joint velocities are
     * computed.
     */
    EYETOHAND_L_cVf_fJe
  } vpServoType;

  /*!
   * Choice of the interaction matrix type used in the visual servoing control law.
   */
  typedef enum
  {
    /*!
     * In the control law (see vpServo::vpServoType), uses the interaction
     * matrix \f${\widehat {\bf L}}_s \f$computed using the current features
     * \f$\bf s\f$.
     */
    CURRENT,
    /*!
     * In the control law (see vpServo::vpServoType), uses the interaction
     * matrix \f${\widehat {\bf L}}_{s^*} \f$computed using the desired
     * features \f${\bf s}^*\f$.
     */
    DESIRED,
    /*!
     * In the control law (see vpServo::vpServoType), uses the interaction
     *  matrix \f${\widehat {\bf L}} = \left({\widehat {\bf L}}_s + {\widehat
     *  {\bf L}}_{s^*}\right)/2 \f$.
     */
    MEAN,
    /*!
     * In the control law (see vpServo::vpServoType), uses an interaction
     * matrix set by the user.
     */
    USER_DEFINED
  } vpServoIteractionMatrixType;

  /*!
   * Choice of the interaction matrix inversion method.
   */
  typedef enum
  {
    /*!
     * In the control law (see vpServo::vpServoType), uses the
     * transpose instead of the pseudo inverse.
     */
    TRANSPOSE,
    /*!
     * In the control law (see vpServo::vpServoType), uses the pseudo inverse.
     */
    PSEUDO_INVERSE
  } vpServoInversionType;

  /*!
   * Choice of the information to print.
   */
  typedef enum
  {
    ALL,                //!< Print all the task information.
    CONTROLLER,         //!< Print the type of controller law.
    ERROR_VECTOR,       //!< Print the error vector \f$\bf e = (s-s^*)\f$.
    FEATURE_CURRENT,    //!< Print the current features \f$\bf s\f$.
    FEATURE_DESIRED,    //!< Print the desired features \f${\bf s}^*\f$.
    GAIN,               //!< Print the gain \f$\lambda\f$.
    INTERACTION_MATRIX, //!< Print the interaction matrix.
    MINIMUM             //!< Same as vpServo::vpServoPrintType::ERROR_VECTOR.
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
  /*!
   * Default constructor that initializes the following settings:
   * - No control law is specified. The user has to call setServo() to specify
   *   the control law.
   * - In the control law, the interaction matrix \f${\widehat {\bf L}}_e \f$ is
   *   computed with the desired features \f${\bf s}^*\f$. Using
   *   setInteractionMatrixType() you can also compute the interaction matrix with
   *   the current visual features, or from the mean \f$\left({\widehat {\bf L}}_s
   *   + {\widehat {\bf L}}_{s^*}\right)/2\f$.
   * - In the control law the pseudo inverse will be used. The method
   *   setInteractionMatrixType() allows to use the transpose instead.
   *
   * \warning By default the threshold used to compute the pseudo-inverse is set to 1e-6.
   * Advanced user can modify this value using setPseudoInverseThreshold().
   */
  vpServo();

  /*!
   * Constructor that allows to choose the visual servoing control law.
   *
   * \param servo_type : Visual servoing control law.
   *
   * The other settings are the following:
   * - In the control law, the interaction matrix \f${\widehat {\bf L}}_e \f$ is
   *   computed with the desired features \f${\bf s}^*\f$. Using
   *   setInteractionMatrixType() you can also compute the interaction matrix with
   *   the current visual features, or from the mean \f$\left({\widehat {\bf L}}_s
   *   + {\widehat {\bf L}}_{s^*}\right)/2\f$.
   * - In the control law the pseudo inverse will be used. The method
   *   setInteractionMatrixType() allows to use the transpose instead.
   */
  VP_EXPLICIT vpServo(vpServoType servo_type);

  /*!
   * Destructor.
   *
   * Since ViSP > 3.3.0 calls kill() to destroy the current and desired feature lists.
   *
   * \sa kill()
   */
  virtual ~vpServo();

  /*!
   * Add a new set of 2 features \f$\bf s\f$ and \f${\bf s}^*\f$ in the task.
   *
   * \param s_cur : Current visual feature denoted \f$\bf s\f$.
   * \param s_star : Desired visual feature denoted \f${\bf s}^*\f$.
   * \param select : Feature selector. By default all the features in \e s and \e
   * s_star are used, but is is possible to specify which one is used in case of
   * multiple features.
   *
   * The following sample code explain how to use this method to add a visual
   * feature point \f$(x,y)\f$:
   * \code
   * vpFeaturePoint s, s_star;
   * ...
   * vpServo task;
   * task.addFeature(s, s_star);
   * \endcode
   *
   * For example to use only the \f$x\f$ visual feature, the previous code
   * becomes:
   * \code
   * vpFeaturePoint s, s_star;
   * ...
   * vpServo task;
   * task.addFeature(s, s_star, vpFeaturePoint::selectX());
   * \endcode
   */
  void addFeature(vpBasicFeature &s_cur, vpBasicFeature &s_star, unsigned int select = vpBasicFeature::FEATURE_ALL);

  /*!
   * Add a new features \f$\bf s\f$ in the task. The desired visual feature
   * denoted \f${\bf s}^*\f$ is equal to zero.
   *
   * \param s_cur : Current visual feature denoted \f$\bf s\f$.
   * \param select : Feature selector. By default all the features in \e s are
   * used, but is is possible to specify which one is used in case of multiple
   * features.
   *
   * The following sample code explain how to use this method to add a \f$\theta
   * {\bf u} =(\theta u_x, \theta u_y, \theta u_z)\f$ feature:
   * \code
   * vpFeatureThetaU s(vpFeatureThetaU::cRcd);
   * ...
   * vpServo task;
   * task.addFeature(s);
   * \endcode
   *
   * For example to use only the \f$\theta u_x\f$ feature, the previous code
   * becomes:
   * \code
   * vpFeatureThetaU s(vpFeatureThetaU::cRcd);
   * ...
   * vpServo task;
   * task.addFeature(s, vpFeatureThetaU::selectTUx);
   * \endcode
   */
  void addFeature(vpBasicFeature &s_cur, unsigned int select = vpBasicFeature::FEATURE_ALL);

  /*!
   * Compute the control law specified using setServo(). See vpServo::vpServoType
   * for more details concerning the control laws that are available. The \ref
   * tutorial-ibvs and \ref tutorial-boost-vs are also useful to illustrate the
   * usage of this function.
   *
   * The general form of the control law is the following:
   *
   * \f[
   * {\bf \dot q}  = \pm \lambda {{\bf \widehat J}_e}^+ {\bf e}
   * \f]
   *
   * where :
   * - \f${\bf \dot q}\f$ is the resulting velocity command to apply to the
   *   robot.
   * - the sign of the control law depends on the eye in hand or eye to hand
   *   configuration.
   * - \f$\bf J\f$ is the Jacobian of the task. It is function of the interaction
   *   matrix and of the robot Jacobian.
   * - \f$\bf e = (s-s^*)\f$ is the error to regulate.
   *
   * To ensure continuous sequencing the computeControlLaw(double) function can
   * be used. It will ensure that the velocities that are computed are
   * continuous.
   */
  vpColVector computeControlLaw();

  /*!
   * Compute the control law specified using setServo(). See vpServo::vpServoType
   * for more details concerning the control laws that are available. The \ref
   * tutorial-boost-vs is also useful to illustrate the usage of this function.
   *
   * To the general form of the control law given in computeControlLaw(), we add
   * here an additional term that comes from the task sequencing approach
   * described in \cite Mansard07e equation (17). This additional term allows to
   * compute continuous velocities by avoiding abrupt changes in the command.
   *
   * The form of the control law considered here is the following:
   *
   * \f[
   * {\bf \dot q} = \pm \lambda {{\bf \widehat J}_e}^+ {\bf e} \mp \lambda {{\bf
   * \widehat J}_{e(0)}}^+ {{\bf e}(0)} \exp(-\mu t) \f]
   *
   * where :
   * - \f${\bf \dot q}\f$ is the resulting continuous velocity command to apply
   *   to the robot.
   * - the sign of the control law depends on the eye in hand or eye to hand
   *   configuration.
   * - \f$\bf J\f$ is the Jacobian of the task. It is function of the interaction
   *   matrix and of the robot Jacobian.
   * - \f$\bf e = (s-s^*)\f$ is the error to regulate.
   * - \f$t\f$ is the time given as parameter of this method.
   * - \f$\mu\f$ is a gain that is set by default to 4 and that could be modified
   *   using setMu().
   * - \f${\bf \widehat J}_{e(0)}^+ {\bf e}(0)\f$ is the value of \f${\bf
   *   \widehat J}_e^+ {\bf e}\f$ when \f$t=0\f$. This value is internally stored
   *   either at the first call of this method, or when \e t parameter is set to 0.
   *
   * \param t : Time in second. When set to zero, \f${{\bf \widehat J}_{e(0)}}^+
   * {{\bf e}(0)}\f$ is refreshed internally.
   */
  vpColVector computeControlLaw(double t);

  /*!
   * Compute the control law specified using setServo(). See vpServo::vpServoType
   * for more details concerning the control laws that are available.
   *
   * To the general form of the control law given in computeControlLaw(), we add
   * here an additional term that comes from the task sequencing approach
   * described in \cite Mansard07e equation (17). This additional term allows to
   * compute continuous velocities by avoiding abrupt changes in the command.
   *
   * The form of the control law considered here is the following:
   *
   * \f[
   * {\bf \dot q} = \pm \lambda {{\bf \widehat J}_e}^+ {\bf e} + \left({\bf \dot
   * e}(0) \mp \lambda {{\bf \widehat J}_{e(0)}}^+ {{\bf e}(0)}\right) \exp(-\mu
   * t) \f]
   *
   * where :
   * - \f${\bf \dot q}\f$ is the resulting continuous velocity command to apply
   *   to the robot.
   * - the sign of the control law depends on the eye in hand or eye to hand
   *   configuration.
   * - \f$\bf J\f$ is the Jacobian of the task. It is function of the interaction
   *   matrix and of the robot Jacobian.
   * - \f$\bf e = (s-s^*)\f$ is the error to regulate.
   * - \f$t\f$ is the time given as parameter of this method.
   * - \f$\mu\f$ is a gain that is set by default to 4 and that could be modified
   *   using setMu().
   * - \f${\bf \widehat J}_{e(0)}^+ {\bf e}(0)\f$ is the value of \f${\bf
   *   \widehat J}_e^+ {\bf e}\f$ when \f$t=0\f$. This value is internally stored
   * either at the first call of this method, or when \e t parameter is set to 0.
   *
   * \param t : Time in second. When set to zero, \f${{\bf \widehat J}_{e(0)}}^+
   * {{\bf e}(0)}\f$ is refreshed internally. \param e_dot_init : Initial value
   * of \f${\bf \dot e}(0)\f$.
   */
  vpColVector computeControlLaw(double t, const vpColVector &e_dot_init);

  /*!
   * Compute the error \f$\bf e =(s - s^*)\f$ between the current set of visual
   * features \f$\bf s\f$ and the desired set of visual features \f$\bf s^*\f$.
   *
   * \return The error vector \f$\bf e\f$.
   */
  vpColVector computeError();

  /*!
   * Compute and return the interaction matrix related to the set of visual
   * features.
   *
   * \return The interaction matrix \f${\widehat {\bf L}}_e\f$ used in the
   * control law specified using setServo().
   */
  vpMatrix computeInteractionMatrix();

  /*!
   * Return the task dimension.
   */
  unsigned int getDimension() const;

  /*!
   * Return the error \f$\bf e = (s - s^*)\f$ between the current set of visual
   * features \f$\bf s\f$ and the desired set of visual features \f$\bf s^*\f$. The
   * error vector is updated after a call of computeError() or computeControlLaw().
   * \code
   * vpServo task;
   * ...
   * vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
   * vpColVector e = task.getError(); // Get the error vector
   * \endcode
   */
  inline vpColVector getError() const { return error; }

  /*!
   * Return the interaction matrix \f$L\f$ used to compute the task jacobian
   * \f$J_1\f$. The interaction matrix is updated after a call to
   * computeInteractionMatrix() or computeControlLaw().
   *
   * \code
   * vpServo task;
   * ...
   * vpColVector v = task.computeControlLaw();   // Compute the velocity corresponding to the visual servoing vpMatrix
   * L = task.getInteractionMatrix();            // Get the interaction matrix used to compute v
   * \endcode
   * \sa getTaskJacobian()
   */
  inline vpMatrix getInteractionMatrix() const { return L; }

  /*!
   * Return the projection operator \f${\bf I}-{\bf W}^+{\bf W}\f$. This
   * operator is updated after a call of computeControlLaw().
   *
   * \code
   * vpServo task;
   * ...
   * vpColVector  v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
   * vpMatrix I_WpW = task.getI_WpW(); // Get the projection operator
   * \endcode
   * \sa getWpW()
   */
  vpMatrix getI_WpW() const { return I_WpW; }

  /*!
   *  Return the visual servo type.
   */
  inline vpServoType getServoType() const { return servoType; }

  /*!
   * Return the large projection operator. This operator is updated
   * after a call of computeControlLaw().
   *
   * \code
   * vpServo task;
   * ...
   * vpColVector  v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
   * vpMatrix P = task.getP();          // Get the large projection operator
   * \endcode
   * \sa getP()
   */
  vpMatrix getLargeP() const { return P; }

  /*!
   * Return the task jacobian \f$J\f$. The task jacobian is updated after a call
   * of computeControlLaw().
   *
   * In the general case, the task jacobian is given by \f${\bf J} = {\widehat
   * {\bf L}} {^c}{\bf V}_a {^a}{\bf J}_e\f$.
   * \code
   * vpServo task;
   * ...
   * vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing vpMatrix
   * J = task.getTaskJacobian(); // Get the task jacobian used to compute v
   * \endcode
   * \sa getTaskJacobianPseudoInverse(), getInteractionMatrix()
   */
  vpMatrix getTaskJacobian() const { return J1; }

  /*!
   * Return the pseudo inverse of the task jacobian \f$J\f$.
   *
   * In the general case, the task jacobian is given by \f${\bf J} = {\widehat
   * {\bf L}} {^c}{\bf V}_a {^a}{\bf J}_e\f$.
   *
   * The task jacobian and its pseudo inverse are updated after a call of computeControlLaw().
   *
   * \return Pseudo inverse \f${J}^{+}\f$ of the task jacobian.
   * \code
   * vpServo task;
   * ...
   * vpColVector v = task.computeControlLaw();          // Compute the velocity corresponding to the visual servoing
   * vpMatrix Jp = task.getTaskJacobianPseudoInverse(); // Get the pseudo inverse of task jacobian used to compute v
   * \endcode
   *
   * \sa getTaskJacobian()
   */
  vpMatrix getTaskJacobianPseudoInverse() const { return J1p; }

  /*!
   * Return the rank of the task jacobian. The rank is updated after a call of computeControlLaw().
   *
   * \code
   * vpServo task;
   * ...
   * vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
   * unsigned int rank = task.getTaskRank();   // Get the rank of the task jacobian
   * \endcode
   */
  unsigned int getTaskRank() const { return rankJ1; }

  /*!
   * Get task singular values.
   *
   * \return Singular values that relies on the task jacobian pseudo inverse.
   */
  inline vpColVector getTaskSingularValues() const { return sv; }

  /*!
   * Return the projection operator \f${\bf W}^+{\bf W}\f$. This operator is
   * updated after a call of computeControlLaw().
   *
   * When the dimension of the task is equal to the number of degrees of freedom
   * available \f${\bf W^+W = I}\f$.
   *
   * \code
   * vpServo task;
   * ...
   * vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
   * vpMatrix  WpW = task.getWpW(); // Get the projection operator
   * \endcode
   * \sa getI_WpW()
   */
  vpMatrix getWpW() const { return WpW; }

  /*!
   * Return the velocity twist matrix used to transform a velocity skew vector
   * from end-effector frame into the camera frame.
   */
  vpVelocityTwistMatrix get_cVe() const { return cVe; }
  /*!
   * Return the velocity twist matrix used to transform a velocity skew vector
   * from robot fixed frame (also called world or base frame) into the camera
   * frame.
   */
  vpVelocityTwistMatrix get_cVf() const { return cVf; }

  /*!
   * Return the velocity twist matrix used to transform a velocity skew vector
   * from robot end-effector frame into the fixed frame (also called world or
   * base frame).
   */
  vpVelocityTwistMatrix get_fVe() const { return fVe; }

  /*!
   * Return the robot jacobian expressed in the end-effector frame.
   */
  vpMatrix get_eJe() const { return eJe; }

  /*!
   * Return the robot jacobian expressed in the robot fixed frame (also called
   * world or base frame).
   */
  vpMatrix get_fJe() const { return fJe; }

  /*!
   * Return pseudo-inverse threshold used to test the singular values. If
   * a singular value is lower than this threshold we consider that the
   * matrix is not full rank.
   *
   * \sa setPseudoInverseThreshold()
   */
  double getPseudoInverseThreshold() const { return m_pseudo_inverse_threshold; }

  /*!
   * Task destruction. Kill the current and desired visual feature lists.
   *
   * This function is called in the destructor. Since ViSP > 3.3.0 it is no more
   * mandatory to call explicitly kill().
   *
   * \code
   * vpServo task ;
   * vpFeatureThetaU s;
   * ...
   * task.addFeature(s); // Add current ThetaU feature
   *
   * task.kill(); // This call is no more mandatory since achieved in the destructor
   * \endcode
   */
  void kill();

  /*!
   * Prints on \e os stream information about the task:
   *
   * \param display_level : Indicates which are the task information to print. See
   * vpServo::vpServoPrintType for more details.
   *
   * \param os : Output stream.
   */
  void print(const vpServo::vpServoPrintType display_level = ALL, std::ostream &os = std::cout);

  /*!
   * Compute and return the secondary task vector according to the classic
   * projection operator \f${\bf I-W^+W}\f$ (see equation(7) in the paper
   * \cite Marchand05b) or the new large projection operator (see equation(24)
   * in the paper \cite Marey:2010).
   *
   * \param de2dt : Value of \f$\frac{\partial {\bf e_2}}{\partial t}\f$ the
   * derivative of the secondary task \f${\bf e}_2\f$.
   * \param useLargeProjectionOperator : if true will be use the large projection
   * operator, if false the classic one (default).
   *
   * \return The secondary task vector.
   *
   * If the classic projection operator is used ( useLargeProjectionOperator =
   * false (default value)) this function return:
   *
   * \f[
   * ({\bf I-W^+W})\frac{\partial {\bf e_2}}{\partial t}
   * \f]
   *
   * Note that the secondary task vector need than to be added to the primary
   * task which can be in the general case written as: \f[
   * -\lambda {\bf W^+W {\widehat {\bf J}}_e^+({\bf s-s^*})}
   * \f]
   *
   * Otherwise if the new large projection operator is used (
   * useLargeProjectionOperator = true ) this function return:
   *
   * \f[
   * {\bf P}\frac{\partial {\bf e_2}}{\partial t}
   * \f]
   *
   * where
   *
   * \f[
   * {\bf P} =\bar{\lambda }\left ( \left \| {\bf e} \right \| \right ){\bf P}_{
   * \left \| {\bf e } \right \| } \left ( 1 - \bar{\lambda }\left ( \left \|
   * {\bf e } \right \| \right ) \right ) \left (  {\bf I-W^+W}\right ) \f]
   *
   * with
   *
   * \f[
   * {\bf P}_{\left \| {\bf e } \right \| } = I_{n} - \frac{1}{{\bf e }^\top {\bf
   * J_{{\bf e }} } {\bf J_{{\bf e }}^\top }{\bf e }}{\bf J_{{\bf e }}^\top }{\bf
   * e }{\bf e }^\top{\bf J_{{\bf e }} } \f]
   *
   * \warning computeControlLaw() must be call prior to this function since it
   * updates the projection operators.
   *
   * The following sample code shows how to use this method to compute a
   * secondary task using the classic projection operator:
   * \code
   * vpColVector v;
   * // Velocity applied to the robot vpColVector de2dt; vpServo task;
   * ...
   * v  = task.computeControlLaw(); // Compute the primary task
   * v += task.secondaryTask(de2dt) // Compute and add the secondary task using the classical projection operator
   * \endcode
   *
   * The following sample code shows how to use this method to compute a
   * secondary task using the large projection operator:
   * \code
   * vpColVector v;
   * // Velocity applied to the robot vpColVector de2dt; vpServo task;
   * ...
   * v  = task.computeControlLaw(); // Compute the primary task
   * v += task.secondaryTask(de2dt, true) // Compute and add the secondary task using the large projection operator
   * \endcode
   *
   * \sa computeControlLaw()
   */
  vpColVector secondaryTask(const vpColVector &de2dt, const bool &useLargeProjectionOperator = false);

  /*!
   * Compute and return the secondary task vector according to the classic
   * projection operator \f${\bf I-W^+W}\f$ (see equation(7) in the paper
   * \cite Marchand05b) or the new large projection operator (see equation(24)
   * in the paper \cite Marey:2010).
   *
   * \param e2 : Value of the secondary task \f${\bf e}_2\f$.
   * \param de2dt : Value of \f$\frac{\partial {\bf e_2}}{\partial t}\f$ the
   * derivative of the secondary task \f${\bf e}_2\f$.
   * \param useLargeProjectionOperator: if true will be use the large projection
   * operator, if false the classic one (default).
   *
   * \return The secondary task vector.
   *
   * If the classic projection operator is used ( useLargeProjectionOperator =
   * false (default value)) this function return:
   *
   * \f[
   * -\lambda ({\bf I-W^+W}) {\bf e_2} +  ({\bf I-W^+W})\frac{\partial {\bf
   * e_2}}{\partial t} \f]
   *
   * Note that the secondary task vector need than to be added to the primary
   * task which can be in the general case written as: \f[
   * -\lambda {\bf W^+W {\widehat {\bf J}}_e^+({\bf s-s^*})}
   * \f]
   *
   * Otherwise if the new large projection operator is used (
   * useLargeProjectionOperator = true ) this function return:
   *
   * \f[
   * -\lambda {\bf P} {\bf e_2} + {\bf P}\frac{\partial {\bf e_2}}{\partial t}
   * \f]
   *
   * where
   *
   * \f[
   * {\bf P} =\bar{\lambda }\left ( \left \| {\bf e} \right \| \right ){\bf P}_{
   * \left \| {\bf e } \right \| } \left ( 1 - \bar{\lambda }\left ( \left \|
   * {\bf e } \right \| \right ) \right ) \left (  {\bf I-W^+W}\right ) \f]
   *
   * with
   *
   * \f[
   * {\bf P}_{\left \| {\bf e } \right \| } = I_{n} - \frac{1}{{\bf e }^\top {\bf
   * J_{{\bf e }} } {\bf J_{{\bf e }}^\top }{\bf e }}{\bf J_{{\bf e }}^\top }{\bf
   * e }{\bf e }^\top{\bf J_{{\bf e }} } \f]
   *
   * \warning computeControlLaw() must be call prior to this function since it
   * updates the projection operators.
   *
   * The following sample code shows how to use this method to compute a
   * secondary task using the classical projection operator:
   * \code
   * vpColVector v;
   * // Velocity applied to the robot vpColVector e2; vpColVector de2dt; vpServo
   * task;
   * ...
   * v  = task.computeControlLaw();     // Compute the primary task
   * v += task.secondaryTask(e2, de2dt) // Compute and add the secondary task using the classical projection operator
   * \endcode
   *
   * The following sample code shows how to use this method to compute a
   * secondary task  using the large projection operator:
   * \code
   * vpColVector v;
   * // Velocity applied to the robot vpColVector e2; vpColVector de2dt; vpServo
   * task;
   * ...
   * v  = task.computeControlLaw();     // Compute the primary task
   * v += task.secondaryTask(e2, de2dt, true) // Compute and add the secondary task using the large projection operator
   * \endcode
   *
   * \sa computeControlLaw()
   */
  vpColVector secondaryTask(const vpColVector &e2, const vpColVector &de2dt,
                            const bool &useLargeProjectionOperator = false);

  /*!
   * Compute and return the secondary task vector for joint limit avoidance
   * \cite Marey:2010b using the new large projection operator (see equation(24)
   * in the paper \cite Marey:2010). The robot avoids the joint limits very
   * smoothly even when the main task constrains all the robot degrees of freedom.
   *
   * \param q : Actual joint positions vector
   *
   * \param dq : Actual joint velocities vector
   *
   * \param qmin : Vector containing the low limit value of each joint in the chain.
   * \param qmax : Vector containing the high limit value of each joint in the chain.
   *
   * \param rho : tuning parameter  \f${\left [ 0,\frac{1}{2} \right]}\f$
   * used to define the safe configuration for the joint. When the joint
   * angle value cross the max or min boundaries (\f${ q_{l_{0}}^{max} }\f$ and
   * \f${q_{l_{0}}^{min}}\f$) the secondary task is activated gradually.
   *
   * \param rho1 : tuning parameter \f${\left ] 0,1 \right ]}\f$ to compute the external
   * boundaries (\f${q_{l_{1}}^{max}}\f$ and \f${q_{l_{1}}^{min}}\f$) for the joint
   * limits. Here the secondary task it completely activated with the highest gain.
   *
   * \param lambda_tune : value \f${\left [ 0,1 \right ]}\f$ used to tune the
   * difference in magnitude between the absolute value of the elements of the
   * primary task and the elements of the secondary task. (See equation (17)
   * \cite Marey:2010b )
   *
   * \code
   * vpServo task;
   * vpColVector qmin;
   * vpColVector qmax;
   * vpColVector q;
   * vpColVector dq;
   * // Fill vector qmin and qmax with min and max limits of the joints (same joint order than vector q).
   * // Update vector of joint position q and velocities dq;
   * ...
   * // Compute the velocity corresponding to the visual servoing
   * vpColVector  v = task.computeControlLaw();
   * // Compute and add the secondary task for the joint limit avoidance
   * // using the large projection operator
   * v += task.secondaryTaskJointLimitAvoidance(q, dq, qmin, qmax)
   * \endcode
   */
  vpColVector secondaryTaskJointLimitAvoidance(const vpColVector &q, const vpColVector &dq, const vpColVector &qmin,
                                               const vpColVector &qmax, const double &rho = 0.1,
                                               const double &rho1 = 0.3, const double &lambda_tune = 0.7);

  /*!
   * Set a 6-dim column vector representing the degrees of freedom that are
   * controlled in the camera frame. When set to 1, all the 6 dof are controlled.
   *
   * \param dof : Degrees of freedom to control in the camera frame.
   * Below we give the correspondence between the index of the vector and the
   * considered dof:
   * - dof[0] = 1 if translation along X is controled, 0 otherwise;
   * - dof[1] = 1 if translation along Y is controled, 0 otherwise;
   * - dof[2] = 1 if translation along Z is controled, 0 otherwise;
   * - dof[3] = 1 if rotation along X is controled, 0 otherwise;
   * - dof[4] = 1 if rotation along Y is controled, 0 otherwise;
   * - dof[5] = 1 if rotation along Z is controled, 0 otherwise;
   *
   * The following example shows how to use this function to control only wx, wy
   * like a pan/tilt:
   * \code
   * #include <visp3/visual_features/vpFeaturePoint.h>
   * #include <visp3/vs/vpServo.h>
   *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
   * int main()
   * {
   *   vpServo servo;
   *   servo.setServo(vpServo::EYEINHAND_CAMERA);
   *   vpFeaturePoint s, sd;
   *   servo.addFeature(s, sd);
   *
   *   vpColVector dof(6, 1);
   *   dof[0] = 0; // turn off vx
   *   dof[1] = 0; // turn off vy
   *   dof[2] = 0; // turn off vz
   *   dof[5] = 0; // turn off wz
   *   servo.setCameraDoF(dof);
   *
   *   while(1) {
   *     // vpFeatureBuilder::create(s, ...);       // update current feature
   *
   *     vpColVector v = servo.computeControlLaw(); // compute control law
   *     // only v[3] and v[4] corresponding to wx and wy are different from 0
   *   }
   * }
   * \endcode
   */
  void setCameraDoF(const vpColVector &dof);

  /*!
   * Set a variable which enables to compute the interaction matrix at each
   * iteration.
   *
   * When the interaction matrix is computed from the desired features \f${\bf
   * s}^*\f$ which are in general constant, the interaction matrix \f${\widehat
   * {\bf L}}_{s^*}\f$ is computed just at the first iteration of the servo
   * loop. Sometimes, when the desired features are time dependent \f${{\bf
   * s}(t)}^*\f$ or varying, the interaction matrix need to be computed at each
   * iteration of the servo loop. This method allows to force the computation
   * of \f${\widehat {\bf L}}\f$ in this particular case.
   *
   * \param force_computation : If true it forces the interaction matrix
   * computation even if it is already done.
  */
  void setForceInteractionMatrixComputation(bool force_computation)
  {
    this->forceInteractionMatrixComputation = force_computation;
  }

  /*!
   * Set the interaction matrix type (current, desired, mean or user defined)
   * and how its inverse is computed.
   *
   * \param interactionMatrixType : The interaction matrix type. See vpServo::vpServoIteractionMatrixType for more
   * details.
   *
   * \param interactionMatrixInversion : How is the inverse computed. See vpServo::vpServoInversionType for more details.
   */
  void setInteractionMatrixType(const vpServoIteractionMatrixType &interactionMatrixType,
                                const vpServoInversionType &interactionMatrixInversion = PSEUDO_INVERSE);

  /*!
   * Set the gain \f$\lambda\f$ used in the control law (see
   * vpServo::vpServoType) as constant.
   *
   * The usage of an adaptive gain allows to reduce the convergence time, see
   * setLambda(const vpAdaptiveGain&).
   *
   * \param c : Constant gain. Values are in general between 0.1 and 1. Higher
   * is the gain, higher are the velocities that may be applied to the robot.
   */
  void setLambda(double c) { lambda.initFromConstant(c); }

  /*!
   * Set the gain \f$\lambda\f$ used in the control law (see
   * vpServo::vpServoType) as adaptive. Value of \f$\lambda\f$ that is used in
   * computeControlLaw() depend on the infinity norm of the task Jacobian.
   *
   * The usage of an adaptive gain rather than a constant gain allows to reduce
   * the convergence time.
   *
   * \param gain_at_zero : the expected gain when \f$x=0\f$: \f$\lambda(0)\f$.
   * \param gain_at_infinity : the expected gain when \f$x=\infty\f$: \f$\lambda(\infty)\f$.
   * \param slope_at_zero : the expected slope of \f$\lambda(x)\f$ when \f$x=0\f$: \f${\dot \lambda}(0)\f$.
   *
   * For more details on these parameters see vpAdaptiveGain class.
   */
  void setLambda(double gain_at_zero, double gain_at_infinity, double slope_at_zero)
  {
    lambda.initStandard(gain_at_zero, gain_at_infinity, slope_at_zero);
  }

  /*!
   * Set the gain \f$\lambda\f$ used in the control law (see
   * vpServo::vpServoType) as adaptive. Value of \f$\lambda\f$ that is used in
   * computeControlLaw() depend on the infinity norm of the task Jacobian.
   *
   * The usage of an adaptive gain rather than a constant gain allows to reduce
   * the convergence time.
   *
   * \sa vpAdaptiveGain
   */
  void setLambda(const vpAdaptiveGain &l) { lambda = l; }

  /*!
   * Set the value of the parameter \f$\mu\f$ used to ensure the continuity of
   * the velocities computed using computeControlLaw(double).
   *
   * A recommended value is 4.
   */
  void setMu(double mu_) { this->mu = mu_; }

  /*!
   * Set the visual servoing control law.
   * \param servo_type : Control law that will be considered.
   * See vpServo::vpServoType to see the possible values.
   */
  void setServo(const vpServoType &servo_type);

  /*!
   * Set the velocity twist matrix used to transform a velocity skew vector
   * from end-effector frame into the camera frame.
   */
  void set_cVe(const vpVelocityTwistMatrix &cVe_)
  {
    this->cVe = cVe_;
    init_cVe = true;
  }

  /*!
   * Set the velocity twist matrix used to transform a velocity skew vector
   * from end-effector frame into the camera frame.
   */
  void set_cVe(const vpHomogeneousMatrix &cMe)
  {
    cVe.build(cMe);
    init_cVe = true;
  }

  /*!
   * Set the velocity twist matrix used to transform a velocity skew vector
   * from robot fixed frame (also called world or base frame) into the camera
   * frame.
   */
  void set_cVf(const vpVelocityTwistMatrix &cVf_)
  {
    this->cVf = cVf_;
    init_cVf = true;
  }

  /*!
   * Set the velocity twist matrix used to transform a velocity skew vector
   * from robot fixed frame (also called world or base frame) into the camera
   * frame.
   */
  void set_cVf(const vpHomogeneousMatrix &cMf)
  {
    cVf.build(cMf);
    init_cVf = true;
  }

  /*!
   * Set the velocity twist matrix used to transform a velocity skew vector
   * from robot end-effector frame into the fixed frame (also called world or
   * base frame).
   */
  void set_fVe(const vpVelocityTwistMatrix &fVe_)
  {
    this->fVe = fVe_;
    init_fVe = true;
  }

  /*!
   * Set the velocity twist matrix used to transform a velocity skew vector
   * from robot end-effector frame into the fixed frame (also called world or
   * base frame).
   */
  void set_fVe(const vpHomogeneousMatrix &fMe)
  {
    fVe.build(fMe);
    init_fVe = true;
  }

  /*!
   * Set the robot jacobian expressed in the end-effector frame.
   */
  void set_eJe(const vpMatrix &eJe_)
  {
    this->eJe = eJe_;
    init_eJe = true;
  }

  /*!
   * Set the robot jacobian expressed in the robot fixed frame (also called
   * world or base frame).
   */
  void set_fJe(const vpMatrix &fJe_)
  {
    this->fJe = fJe_;
    init_fJe = true;
  }

  /*!
   * Set the pseudo-inverse threshold used to test the singular values. If
   * a singular value is lower than this threshold we consider that the
   * matrix is not full rank.
   * \param pseudo_inverse_threshold : Value to use. Default value is set to 1e-6.
   * \sa getPseudoInverseThreshold()
   */
  void setPseudoInverseThreshold(double pseudo_inverse_threshold)
  {
    m_pseudo_inverse_threshold = pseudo_inverse_threshold;
  }

  /*!
   * Test if all the initialization are correct. If true, the control law can
   * be computed.
   */
  bool testInitialization();

  /*!
   * Test if all the update are correct. If true control law can be computed.
   */
  bool testUpdated();

protected:
  /*!
   * Initialize the servo with the following settings:
   *
   * - No control law is specified. The user has to call setServo() to specify
   *   the control law.
   * - In the control law, the interaction matrix \f${\widehat {\bf L}}_e \f$ is
   *   computed with the desired features \f${\bf s}^*\f$. Using
   *   setInteractionMatrixType() you can also compute the interaction matrix with
   *   the current visual features, or from the mean \f$\left({\widehat {\bf L}}_s
   *   + {\widehat {\bf L}}_{s^*}\right)/2\f$.
   * - In the control law the pseudo inverse will be used. The method
   *   setInteractionMatrixType() allows to use the transpose instead.
   */
  void init();

  /*!
   * Compute the classic projection operator and the large projection operator.
   */
  void computeProjectionOperators(const vpMatrix &J1_, const vpMatrix &I_, const vpMatrix &I_WpW_,
                                  const vpColVector &error_, vpMatrix &P_) const;

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
  //! Type of the interaction matrix (current, mean, desired, user)
  vpServoIteractionMatrixType interactionMatrixType;
  //! Indicates if the transpose or the pseudo inverse of the
  //! interaction matrix should be used to compute the task.
  vpServoInversionType inversionType;

protected:
  /*
    Twist transformation matrix
  */

  /*!
   * Twist transformation matrix between camera frame (c) and robot end-effector
   * frame (e).
   */
  vpVelocityTwistMatrix cVe;

  /*!
   * Boolean indicating if twist transformation matrix between camera frame (c)
   * and robot end-effector frame (e) is set by the user and thus differs from eye
   * matrix.
   */
  bool init_cVe;
  //! Twist transformation matrix between camera frame (c) and robot base frame (f).
  vpVelocityTwistMatrix cVf;
  /*!
   * Boolean indicating if twist transformation matrix between camera frame (c)
   * and robot base frame (f) is set by the user and thus differs from eye
   * matrix.
   */
  bool init_cVf;
  /*!
   * Twist transformation matrix between robot base frame (f) and robot
   * end-effector frame (e).
   */
  vpVelocityTwistMatrix fVe;
  /*!
   * Boolean indicating if twist transformation matrix between robot base frame (f)
   * and robot end-effector frame(e) is set by the user and thus differs from eye
   * matrix.
   */
  bool init_fVe;

  /*
   * Jacobians
   */

  //! Jacobian expressed in the end-effector frame (e).
  vpMatrix eJe;
  /*!
   * Boolean indicating if Jacobian expressed in the end-effector frame (e)
   * is set by the user and thus differs from eye matrix.
   */
  bool init_eJe;

  //! Jacobian expressed in the robot base frame (f).
  vpMatrix fJe;
  /*!
   * Boolean indicating if Jacobian expressed in the robot base frame (f)
   * is set by the user and thus differs from eye matrix.
   */
  bool init_fJe;

  /*
   * Task building
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

  //! Identity matrix.
  vpMatrix I;
  //! Projection operators \f$\bf WpW\f$.
  vpMatrix WpW;
  //! Projection operators \f$\bf I-WpW\f$.
  vpMatrix I_WpW;
  /*!
   * New Large projection operator (see equation(24) in the paper
   * \cite Marey:2010). This projection operator allows performing secondary task
   * even when the main task is full rank. \f[
   * {\bf P} =\bar{\lambda }\left ( \left \| {\bf e} \right \| \right ){\bf P}_{
   * \left \| {\bf e } \right \| } \left ( 1 - \bar{\lambda }\left ( \left \|
   * {\bf e } \right \| \right ) \right ) \left (  {\bf I-W^+W}\right ) \f]
   *
   * with
   *
   * \f[
   * {\bf P}_{\left \| {\bf e } \right \| } = I_{n} - \frac{1}{{\bf e }^\top {\bf
   * J_{{\bf e }} } {\bf J_{{\bf e }}^\top }
   * {\bf e }}{\bf J_{{\bf e }}^\top }{\bf e }{\bf e }^\top{\bf J_{{\bf e }} }
   * \f]
   */
  vpMatrix P;

  //! Singular values from the pseudo inverse.
  vpColVector sv;

  /*!
   * Gain \f$ mu \f$ used to compute the control law with the task
   * sequencing approach.
   * \see computeControlLaw(double) and computeControlLaw(double, const vpColVector &)
   */
  double mu;

  /*!
   * First primary task value \f${\bf \widehat J}_e^+ {\bf e}\f$ used in the control
   * law with the task sequencing approach when time is initial \f$t=0\f$.
   * \see computeControlLaw(double) and computeControlLaw(double, const vpColVector &)
   */
  vpColVector e1_initial;

  //! Boolean to know if cJc is identity (for fast computation)
  bool iscJcIdentity;

  /*!
   * A diag matrix used to determine which are the degrees of freedom that
   * are controlled in the camera frame (c).
   */
  vpMatrix cJc;

  bool m_first_iteration; //!< True until first call of computeControlLaw() is achieved

  double m_pseudo_inverse_threshold; //!< Threshold used in the pseudo inverse
};
END_VISP_NAMESPACE
#endif
