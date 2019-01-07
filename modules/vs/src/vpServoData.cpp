/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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

/*!
  \file vpServoData.cpp
  \brief save data during the task execution
*/

// Servo
#include <visp3/vs/vpServo.h>

#include <visp3/core/vpIoException.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/vs/vpServoData.h>

void vpServoData::open(const char *directory)
{
  try {
    if (vpIoTools::checkDirectory(directory) == false)
      vpIoTools::makeDirectory(directory);

    char s[FILENAME_MAX];

    sprintf(s, "%s/vel.dat", directory);
    velocityFile.open(s);
    sprintf(s, "%s/error.dat", directory);
    errorFile.open(s);
    sprintf(s, "%s/errornorm.dat", directory);
    errorNormFile.open(s);
    sprintf(s, "%s/s.dat", directory);
    sFile.open(s);
    sprintf(s, "%s/sStar.dat", directory);
    sStarFile.open(s);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

void vpServoData::setCmDeg() { cmDeg = true; }
void vpServoData::setMeterRad() { cmDeg = false; }
void vpServoData::save(const vpServo &task)
{
  if (cmDeg == false)
    velocityFile << task.q_dot.t();
  else {
    for (unsigned int i = 0; i < 3; i++)
      velocityFile << task.q_dot[i] * 100 << " ";
    for (unsigned int i = 4; i < 6; i++)
      velocityFile << vpMath::deg(task.q_dot[i]) << " ";
    velocityFile << std::endl;
  }
  errorFile << (task.getError()).t();
  errorNormFile << (task.getError()).sumSquare() << std::endl;
  vNormFile << task.q_dot.sumSquare() << std::endl;

  sFile << task.s.t();
  sStarFile << task.sStar.t();
}

void vpServoData::close()
{
  velocityFile.close();
  errorFile.close();
  errorNormFile.close();
  sFile.close();
  sStarFile.close();
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
