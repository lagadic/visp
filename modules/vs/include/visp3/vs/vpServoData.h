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
 * Save data during the task execution.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpServoData_H
#define vpServoData_H

/*!
  \file vpServoData.h
  \brief  save data during the task execution
*/

// Servo
#include <visp3/vs/vpServo.h>

#include <iostream>

/*!
  \class vpServoData
  \ingroup group_task
  \brief Save data during the task execution.
*/
class VISP_EXPORT vpServoData
{

private:
  char baseDirectory[FILENAME_MAX];

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

  vpServoData() : velocityFile(), errorFile(), errorNormFile(), sFile(), sStarFile(), vNormFile(), cmDeg(false) { ; }
  virtual ~vpServoData() { ; }

  //! velocity output in cm and deg
  void setCmDeg();
  //! velocity output in meter and deg (default)
  void setMeterRad();

  void save(const vpServo &task);
  void open(const char *baseDirectory);
  void close();

  void empty();
  void push();
  void display(vpImage<unsigned char> &I);
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
