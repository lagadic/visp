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

#ifndef _vpRobotBiclopsController_h_
#define _vpRobotBiclopsController_h_

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#ifdef VISP_HAVE_BICLOPS

#include <iostream>
#include <stdio.h>

#include <Biclops.h>  // Contrib for Biclops sdk
#include <PMDUtils.h> // Contrib for Biclops sdk

BEGIN_VISP_NAMESPACE
#if defined(_WIN32)
class VISP_EXPORT Biclops; // needed for dll creation
#endif

/* ------------------------------------------------------------------------ */
/* --- CLASS ------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!
 * \class vpRobotBiclopsController
 *
 * \ingroup group_robot_real_ptu
 *
 * \brief Interface to Biclops, pan, tilt, verge head for computer vision
 * applications.
 *
 * See http://www.traclabs.com/tracbiclops.htm for more details.
 *
 * This class uses libraries libBiclops.so, libUtils.so and libPMD.so and
 * includes Biclops.h and PMDUtils.h provided by Traclabs.
*/
class VISP_EXPORT vpRobotBiclops::vpRobotBiclopsController
{
public:
  /*!
   * Biclops controller status
   */
  typedef enum
  {
    STOP, //!< Have to stop the robot.
    SPEED //!< Can send the desired speed.
  } vpControllerStatusType;

public:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  /*!
    * Biclops head shared memory structure.
    */
  typedef struct
  {
    vpControllerStatusType status[2];
    double q_dot[2];        //!< Desired speed.
    double actual_q[2];     //!< Current measured position of each axes.
    double actual_q_dot[2]; //!< Current measured velocity of each axes.
    bool jointLimit[2];     //!< Indicates if an axe is in joint limit.
  } shmType;
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  vpRobotBiclopsController(const vpRobotBiclopsController &)
      //    : m_biclops(), m_axisMask(0), m_panAxis(nullptr), m_tiltAxis(nullptr),
      //      m_vergeAxis(nullptr),
      //      m_panProfile(), m_tiltProfile(), m_vergeProfile(), m_shm(),
      //      m_stopControllerThread(false)
      //  {
      //    throw vpException(vpException::functionNotImplementedError, "Not
      //    implemented!");
      //  }
      //  vpRobotBiclopsController &operator=(const vpRobotBiclopsController &){
      //    throw vpException(vpException::functionNotImplementedError, "Not
      //    implemented!"); return *this;
      //  }
      //#endif

public:
  /*!
   * Default constructor.
   */
  vpRobotBiclopsController();

  /*!
   * Destructor.
   */
  virtual ~vpRobotBiclopsController();

  /*!
   * Initialize the Biclops by homing all axis.
   *
   * \param configfile : Biclops configuration file.
   *
   * \exception vpRobotException::notInitializedError If the Biclops head cannot
   * be initialized. The initialization can failed,
   * - if the head is not powered on,
   * - if the head is not connected to your computer throw a serial cable,
   * - if you try to open a bad serial port. Check you config file to verify
   *   which is the used serial port.
   */
  void init(const std::string &configfile);

  /*!
   * Set the Biclops axis position. The motion of the axis is synchronized to end
   * on the same time.
   *
   * \warning Wait the end of the positioning.
   *
   * \param q : The position to set for each axis.
   *
   * \param percentVelocity : The velocity displacement to reach the new position
   * in the range [0: 100.0]. 100 % corresponds to the maximal admissible
   * speed. The maximal admissible speed is given by vpBiclops::speedLimit.
   */
  void setPosition(const vpColVector &q, double percentVelocity);

  /*!
   * Apply a velocity to each axis of the Biclops robot.
   *
   * \warning This method is non blocking.
   *
   * \param q_dot : Velocity to apply.
   */
  void setVelocity(const vpColVector &q_dot);

  /*!
   * Get the Biclops joint positions.
   *
   * \return The axis joint positions in radians.
   */
  vpColVector getPosition();

  /*!
   * Get the Biclops actual joint positions.
   *
   * \return The axis actual joint positions in radians.
   */
  vpColVector getActualPosition();

  /*!
   * Get the Biclops joint velocities.
   *
   * \return The axis joint velocities in rad/s.
   */
  vpColVector getVelocity();

  /*!
   * Get the Biclops actual joint velocities.
   *
   * \return The axis actual joint velocities in rad/s.
   */
  vpColVector getActualVelocity();

  /*!
   * Return a pointer to the PMD pan axis.
   */
  PMDAxisControl *getPanAxis() { return m_panAxis; };

  /*!
   * Return a pointer to the PMD tilt axis.
   */
  PMDAxisControl *getTiltAxis() { return m_tiltAxis; };

  /*!
   * Return a pointer to the PMD verge axis.
   */
  PMDAxisControl *getVergeAxis() { return m_vergeAxis; };

  /*!
   * Update the shared memory.
   *
   * \param shm : Content to write in the shared memory.
   */
  void writeShm(shmType &shm);

  /*!
   * Get a copy of the shared memory.
   *
   * \return A copy of the shared memory.
   */
  shmType readShm();

  /*!
   * Return true when control thread is requested to stop.
   */
  bool isStopRequested() { return m_stopControllerThread; }

  /*!
   * Request to stop the control thread.
   * @param stop : When true, request to stop the control thread.
   */
  void stopRequest(bool stop) { m_stopControllerThread = stop; }

private:
  Biclops m_biclops; // THE interface to Biclops.
  int m_axisMask;

  // Pointers to each axis (populated once controller is initialized).
  PMDAxisControl *m_panAxis;
  PMDAxisControl *m_tiltAxis;
  PMDAxisControl *m_vergeAxis;

  PMDAxisControl::Profile m_panProfile;
  PMDAxisControl::Profile m_tiltProfile;
  PMDAxisControl::Profile m_vergeProfile;

  shmType m_shm;
  bool m_stopControllerThread;
};
END_VISP_NAMESPACE
#endif // #ifdef VISP_HAVE_BICLOPS

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

#endif /* #ifndef _vpRobotBiclopsController_h_ */
