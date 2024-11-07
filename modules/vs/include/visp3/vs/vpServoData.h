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
 * Save data during the task execution.
 */

/*!
  \file vpServoData.h
  \brief  save data during the task execution
*/

#ifndef _vpServoData_h_
#define _vpServoData_h_

#include <visp3/core/vpConfig.h>
// Servo
#include <visp3/vs/vpServo.h>

#include <iostream>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpServoData
 * \ingroup group_task
 * \brief Save data during the task execution when using vpServo.
*/
class VISP_EXPORT vpServoData
{
private:
  std::ofstream velocityFile;
  std::ofstream errorFile;
  std::ofstream errorNormFile;
  std::ofstream sFile;
  std::ofstream sStarFile;
  std::ofstream vNormFile;

  //! flag to known if velocity should be output in cm and degrees (true)
  //! or in m/rad
  bool cmDeg;

public:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  vpServoData(const vpServoData &sd)
    : velocityFile(), errorFile(), errorNormFile(), sFile(), sStarFile(), vNormFile(), cmDeg(false)
  {
    *this = sd;
  }
  vpServoData &operator=(const vpServoData &)
  {
    throw vpException(vpException::functionNotImplementedError, "Not implemented!");
  }
#endif

  /*!
   * Default constructor.
   */
  vpServoData() : velocityFile(), errorFile(), errorNormFile(), sFile(), sStarFile(), vNormFile(), cmDeg(false) { ; }

  /*!
   * Destructor that closes all data files if needed.
   */
  virtual ~vpServoData() { close(); }

  //! Velocity output are set in cm and deg.
  void setCmDeg();
  //! Velocity output are set in meter and deg (default).
  void setMeterRad();
  //! Save visual-servoing control law data.
  void save(const vpServo &task);

  /*!
   * Set the directory in which data are saved.
   * In this directory, creates the following files:
   * - `vel.dat` that contains velocities computed
   * - `error.dat` that contains visual-servo error \f$ {\bf e} = ({\bf s} - {\bf s}^*)\f$
   * - `errornorm.dat` that contains the sum square of the visual-servo error \f$ {\bf e} \f$
   * - `s.dat` that contains the current feature vector \f$ \bf s \f$
   * - `sStar.dat` that contains the desired feature vector \f$ {\bf s}^* \f$
   *
   * @param directory : Path to the folder that contains data files to save.
   *
   * \sa close()
   */
  void open(const std::string &directory);

  /*!
   * Close all data files open with open() function.
   *
   * \sa open()
   */
  void close();
};
END_VISP_NAMESPACE
#endif
