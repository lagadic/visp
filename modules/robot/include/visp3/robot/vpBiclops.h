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
 * Interface for the Biclops robot.
 */

#ifndef _vpBiclops_h_
#define _vpBiclops_h_

#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpBiclops
 *
 * \ingroup group_robot_real_ptu
 *
 * \brief Jacobian, geometric model functionalities... for Biclops, pan, tilt
 * head.
 *
 * Two different Denavit-Hartenberg representations of the robot are
 * implemented. As mentioned in vpBiclops::DenavitHartenbergModel they differ
 * in the orientation of the tilt axis. Use setDenavitHartenbergModel() to
 * select the representation.
 *
 * See http://www.traclabs.com/tracbiclops.htm for more details concerning the
 * hardware.
 *
*/
class VISP_EXPORT vpBiclops
{
public:
  /*!
   * Two different Denavit-Hartenberg representations of the robot are
   * implemented. As you can see in the next image, they differ in the orientation of the tilt axis.
   *
   * \image html img-biclops-frames.jpg Biclops PT models
   *
   * The first representation, vpBiclops::DH1 is given by:
   *
   * | Joint | \f$a_i\f$ | \f$d_i\f$ | \f$\alpha_i\f$ | \f$\theta_i\f$    |
   * | :---: | :-------: | :-------: | -------------: | ----------------: |
   * |     1 |         0 |         0 |   \f$-\pi/2\f$ |         \f$q_1\f$ |
   * |     2 |         0 |         0 |   \f$ \pi/2\f$ | \f$q_2 + \pi/2\f$ |
   *
   * The second one, vpBiclops::DH2 is given by:
   *
   * | Joint | \f$a_i\f$ | \f$d_i\f$ | \f$\alpha_i\f$ | \f$\theta_i\f$    |
   * | :---: | :-------: | :-------: | -------------: | ----------------: |
   * |     1 |         0 |         0 |   \f$ \pi/2\f$ |         \f$q_1\f$ |
   * |     2 |         0 |         0 |   \f$-\pi/2\f$ | \f$q_2 - \pi/2\f$ |
   *
   * where \f$q_1, q_2\f$ are respectively the pan and tilt joint
   * positions.
   *
   * In those representations, the pan is oriented from left to right, while
   * the tilt is oriented
   * - in vpBiclops::DH1 from down to top,
   * - in vpBiclops::DH2 from top to down.
   */
  typedef enum
  {
    DH1, //!< First Denavit-Hartenberg representation.
    DH2  //!< Second Denavit-Hartenberg representation.
  } DenavitHartenbergModel;

public:
  static const unsigned int ndof; //!< Number of dof

  // Geometric model
  static const float h;              //<! Vertical offset from last joint to camera frame used in set_cMe()
  static const float panJointLimit;  //!< Pan axis +/- joint limit in rad
  static const float tiltJointLimit; //!< Tilt axis +/- joint limit in rad
  static const float speedLimit;     //!< Pan and tilt axis max velocity in rad/s to perform a displacement

protected:
  DenavitHartenbergModel m_dh_model; //!< Denavit-Hartenberg model
  vpHomogeneousMatrix m_cMe; //!< Camera frame to PT end-effector frame transformation

public:
  /*!
   * Default constructor. Call init() that sets vpBiclops::DH1 Denavit-Hartenberg model.
   */
  vpBiclops(void);

  /*!
   * Destructor that does nothing.
   */
  virtual ~vpBiclops() { };

  /** @name Inherited functionalities from vpBiclops */
  //@{

  /*!
   * Initialization.
   * - By default vpBiclops::DH1 Denavit-Hartenberg model is selected.
   * - Initialize also the default \f${^c}{\bf M}_e\f$ transformation calling set_cMe().
   * \f[
   *   {^c}{\bf M}_e = \left(
   *     \begin{matrix}
   *        0 & 1 & 0 & 0 \\
   *       -1 & 0 & 0 & h \\
   *        0 & 0 & 1 & 0 \\
   *        0 & 0 & 0 & 1
   *     \end{matrix}
   *   \right)
   * \f]
   */
  void init(void);

  /*!
   * Compute the direct geometric model of the camera: fMc
   *
   * \warning Provided for compatibility with previous versions. Use rather
   * get_fMc(const vpColVector &, vpHomogeneousMatrix &).
   *
   * \param q : Joint position for pan and tilt axis.
   *
   * \param fMc : Homogeneous matrix corresponding to the direct geometric model
   * of the camera. Describes the transformation between the robot reference
   * frame (called fixed) and the camera frame.
   *
   * \sa get_fMc(const vpColVector &, vpHomogeneousMatrix &)
   */
  void computeMGD(const vpColVector &q, vpHomogeneousMatrix &fMc) const;

  /*!
   * Return the direct geometric model of the camera: fMc
   *
   * \warning Provided for compatibility with previous versions. Use rather
   * get_fMc(const vpColVector &).
   *
   * \param q : Joint position for pan and tilt axis.
   *
   * \return fMc, the homogeneous matrix corresponding to the direct geometric
   * model of the camera. Describes the transformation between the robot
   * reference frame (called fixed) and the camera frame.
   *
   * \sa get_fMc(const vpColVector &)
   */
  vpHomogeneousMatrix computeMGD(const vpColVector &q) const;

  /*!
   * Compute the direct geometric model of the camera in terms of pose vector.
   *
   * \warning Provided for compatibility with previous versions. Use rather
   * get_fMc(const vpColVector &, vpPoseVector &).
   *
   * \param q : Joint position for pan and tilt axis.
   *
   * \param fPc : Pose vector corresponding to the transformation between the
   * robot reference frame (called fixed) and the camera frame.
   *
   * \sa get_fMc(const vpColVector &, vpPoseVector &)
   */
  void computeMGD(const vpColVector &q, vpPoseVector &fPc) const;

  /*!
   * Return the transformation \f${^c}{\bf M}_e\f$ between the camera frame and
   * the end effector frame.
   */
  vpHomogeneousMatrix get_cMe() const { return m_cMe; }

  /*!
   * Get the twist matrix corresponding to the transformation between the
   * camera frame and the end effector frame. The end effector frame is located
   * on the tilt axis.
   *
   * \param[out] cVe : Twist transformation between camera and end effector frame to
   * express a velocity skew from end effector frame in camera frame.
   */
  void get_cVe(vpVelocityTwistMatrix &cVe) const;

  /*!
   * Compute the direct geometric model of the camera: fMc
   *
   * \param[in] q : Joint position for pan and tilt axis.
   *
   * \param[out] fMc : Homogeneous matrix corresponding to the direct geometric model
   * of the camera. Describes the transformation between the robot reference
   * frame (called fixed) and the camera frame.
   */
  void get_fMc(const vpColVector &q, vpHomogeneousMatrix &fMc) const;

  /*!
   * Compute the direct geometric model of the camera in terms of pose vector.
   *
   * \param[in] q : Joint position for pan and tilt axis.
   *
   * \param[out] fPc : Pose vector corresponding to the direct geometric model
   * of the camera. Describes the transformation between the robot reference
   * frame (called fixed) and the camera frame.
   */
  void get_fMc(const vpColVector &q, vpPoseVector &fPc) const;

  /*!
   * Return the direct geometric model of the camera: fMc
   *
   * \param[in] q : Joint position for pan and tilt axis.
   *
   * \return fMc, the homogeneous matrix corresponding to the direct geometric
   * model of the camera. Describes the transformation between the robot
   * reference frame (called fixed) and the camera frame.
   */
  vpHomogeneousMatrix get_fMc(const vpColVector &q) const;

  /*!
   * Return the direct geometric model of the end effector: fMe
   *
   * \param[in] q : Joint position for pan and tilt axis.
   *
   * \return fMe, the homogeneous matrix corresponding to the direct geometric
   * model of the end effector. Describes the transformation between the robot
   * reference frame (called fixed) and the end effector frame.
   */
  vpHomogeneousMatrix get_fMe(const vpColVector &q) const;

  /*!
   * Get the robot jacobian expressed in the end-effector frame.
   *
   * \warning Re is not the embedded camera frame. It corresponds to the frame
   * associated to the tilt axis (see also get_cMe).
   *
   * \param[in] q : Joint position for pan and tilt axis.
   *
   * \param[out] eJe : Jacobian between end effector frame and end effector frame (on
   * tilt axis).
   */
  void get_eJe(const vpColVector &q, vpMatrix &eJe) const;

  /*!
   * Get the robot jacobian expressed in the robot reference frame
   *
   * \param[in] q : Joint position for pan and tilt axis.
   *
   * \param[out] fJe : Jacobian between reference frame (or fix frame) and end
   * effector frame (on tilt axis).
   */
  void get_fJe(const vpColVector &q, vpMatrix &fJe) const;

  /*!
   * Return the Denavit-Hartenberg representation used to model the head.
   * \sa vpBiclops::DenavitHartenbergModel
   */
  inline vpBiclops::DenavitHartenbergModel getDenavitHartenbergModel() const { return m_dh_model; }

  /*!
   * Set the default homogeneous matrix corresponding to the transformation
   * between the camera frame and the end effector frame. The end effector frame
   * is located on the tilt axis.
   *
   * \f[
   *   {^c}{\bf M}_e = \left(
   *     \begin{matrix}
   *        0 & 1 & 0 & 0 \\
   *       -1 & 0 & 0 & h \\
   *        0 & 0 & 1 & 0 \\
   *        0 & 0 & 0 & 1
   *     \end{matrix}
   *   \right)
   * \f]
   */
  void set_cMe();

  /*!
   * Set the transformation between the camera frame and the end effector
   * frame.
   */
  void set_cMe(const vpHomogeneousMatrix &cMe) { m_cMe = cMe; }

  /*!
   * Set the Denavit-Hartenberg representation used to model the head.
   *
   * \param[in] dh_model : Denavit-Hartenberg model. \sa vpBiclops::DenavitHartenbergModel
   */
  inline void setDenavitHartenbergModel(vpBiclops::DenavitHartenbergModel dh_model = vpBiclops::DH1)
  {
    m_dh_model = dh_model;
  }

//@}

/*!
 * Set output stream with Biclops parameters.
 * @param os : Output stream.
 * @param dummy : Not used.
 * @return Output stream with the Biclops parameters.
 */
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpBiclops &dummy);
};
END_VISP_NAMESPACE
#endif
