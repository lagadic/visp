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
 * Laser scan data structure.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpLaserScan_h
#define vpLaserScan_h

/*!
  \file vpLaserScan.h

  \brief Implements a laser scan data structure.

*/

#include "visp3/sensor/vpScanPoint.h"

#include <vector>

/*!

  \class vpLaserScan
  \ingroup group_sensor_laserscanner

  \brief Implements a laser scan data structure that contains
  especially the list of scanned points that have been recorded for
  this laser scan.

  Other data as the start/stop angle, the start/end timestamp are
  also considered.
 */
class VISP_EXPORT vpLaserScan
{
public:
  /*! Default constructor that initialize all the internal variable to zero.
   */
  vpLaserScan()
    : listScanPoints(), startTimestamp(0), endTimestamp(0), measurementId(0), numSteps(0), startAngle(0), stopAngle(0),
      numPoints(0)
  {
  }
  /*! Copy constructor. */
  vpLaserScan(const vpLaserScan &scan)
    : listScanPoints(scan.listScanPoints), startTimestamp(0), endTimestamp(0), measurementId(0), numSteps(0),
      startAngle(0), stopAngle(0), numPoints(0)
  {
    startTimestamp = scan.startTimestamp;
    endTimestamp = scan.endTimestamp;
    measurementId = scan.measurementId;
    numSteps = scan.numSteps;
    startAngle = scan.startAngle;
    stopAngle = scan.stopAngle;
    numPoints = scan.numPoints;
  }
  /*! Default destructor that does nothing. */
  virtual ~vpLaserScan(){};
  /*! Add the scan point at the end of the list. */
  inline void addPoint(const vpScanPoint &p) { listScanPoints.push_back(p); }
  /*! Drop the list of points. */
  inline void clear() { listScanPoints.clear(); }
  /*! Get the list of points. */
  inline std::vector<vpScanPoint> getScanPoints() { return listScanPoints; }
  /*! Specifies the id of former measurements and increases with
      every measurement. */
  inline void setMeasurementId(const unsigned short &id) { this->measurementId = id; }
  /*! Start time of measurement. */
  inline void setStartTimestamp(const double &start_timestamp) { this->startTimestamp = start_timestamp; }
  /*! End time of measurement. */
  inline void setEndTimestamp(const double &end_timestamp) { this->endTimestamp = end_timestamp; }
  /*! Angular steps per scanner rotation. */
  inline void setNumSteps(const unsigned short &num_steps) { this->numSteps = num_steps; }
  /*! Start angle of the measurement in angular steps. */
  inline void setStartAngle(const short &start_angle) { this->startAngle = start_angle; }
  /*! Stop angle of the measurement in angular steps. */
  inline void setStopAngle(const short &stop_angle) { this->stopAngle = stop_angle; }
  /*! Number of measured points of the measurement. */
  inline void setNumPoints(const unsigned short &num_points) { this->numPoints = num_points; }
  /*! Return the measurement start time. */
  inline double getStartTimestamp() { return startTimestamp; }
  /*! Return the measurement end time. */
  inline double getEndTimestamp() { return endTimestamp; }

private:
  std::vector<vpScanPoint> listScanPoints;
  double startTimestamp;
  double endTimestamp;
  unsigned short measurementId;
  unsigned short numSteps;
  short startAngle;
  short stopAngle;
  unsigned short numPoints;
};

#endif
