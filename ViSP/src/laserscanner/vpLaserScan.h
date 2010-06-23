/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

#include <vector>

#include "visp/vpConfig.h"
#include "visp/vpScanPoint.h"


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
  }
  /*! Default destructor that does nothing. */
  virtual ~vpLaserScan() {};
  /*! Add the scan point at the end of the list. */
  inline void addPoint(const vpScanPoint &p) {
    listScanPoints.push_back( p );
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
