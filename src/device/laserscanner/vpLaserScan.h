/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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

#include "visp/vpScanPoint.h"

#include <vector>

/*!

  \class vpLaserScan

  \brief Implements a laser scan data structure that contains
  especially the list of scanned points that have been recorded for
  this laser scan.

  Other data as the start/stop angle, the start/end timestamp are
  also considered.
 */
class VISP_EXPORT vpLaserScan
{
 public:
  /*! Default constructor that initialize all the internal variable to zero. */
  vpLaserScan() {
    startTimestamp = 0;
    endTimestamp = 0;
    measurementId = 0;
    numSteps = 0;
    startAngle = 0;
    stopAngle = 0;
    numPoints = 0;
  }
  /*! Copy constructor. */
  vpLaserScan(const vpLaserScan &scan) {
    startTimestamp = scan.startTimestamp;
    endTimestamp = scan.endTimestamp;
    measurementId = scan.measurementId;
    numSteps = scan.numSteps;
    startAngle = scan.startAngle;
    stopAngle = scan.stopAngle;
    numPoints = scan.numPoints;
    listScanPoints = scan.listScanPoints;
  }
  /*! Default destructor that does nothing. */
  virtual ~vpLaserScan() {};
  /*! Add the scan point at the end of the list. */
  inline void addPoint(const vpScanPoint &p) {
    listScanPoints.push_back( p );
  }
  /*! Drop the list of points. */
  inline void clear() {
    listScanPoints.clear(  );
  }
  /*! Get the list of points. */
  inline std::vector<vpScanPoint> getScanPoints() {
    return listScanPoints;
  }
  /*! Specifies the id of former measurements and increases with
      every measurement. */
  inline void setMeasurementId(const unsigned short &measurementId) {
    this->measurementId = measurementId;
  }
  /*! Start time of measurement. */
  inline void setStartTimestamp(const double &startTimestamp) {
    this->startTimestamp = startTimestamp;
  }
  /*! End time of measurement. */
  inline void setEndTimestamp(const double &endTimestamp) {
    this->endTimestamp = endTimestamp;
  }
  /*! Angular steps per scanner rotation. */
  inline void setNumSteps(const unsigned short &numSteps) {
    this->numSteps = numSteps;
  }
  /*! Start angle of the measurement in angular steps. */
  inline void setStartAngle(const short &startAngle) {
    this->startAngle = startAngle;
  }
  /*! Stop angle of the measurement in angular steps. */
  inline void setStopAngle(const short &stopAngle) {
    this->stopAngle = stopAngle;
  }
  /*! Number of measured points of the measurement. */
  inline void setNumPoints(const unsigned short &numPoints) {
    this->numPoints = numPoints;
  }
  /*! Return the measurement start time. */
  inline double getStartTimestamp() {
    return startTimestamp;
  }
  /*! Return the measurement end time. */
  inline double getEndTimestamp() {
    return endTimestamp;
  }

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
