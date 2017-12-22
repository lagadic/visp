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
 * Base class for object detection.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpDetectorBase_h__
#define __vpDetectorBase_h__

#include <assert.h>
#include <string>
#include <utility>
#include <vector>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRect.h>

/*!
  \class vpDetectorBase
  \ingroup group_detection_barcode group_detection_face
  Base class for object detection.

  This class is a generic class that can be used to detect:
  - bar codes like QRcodes of Data matrices. The example given in
  tutorial-barcode-detector.cpp shows how to detect one or more bar codes in
  an image. In tutorial-barcode-detector-live.cpp you will find an other
  example that shows how to use this class to detect bar codes in images
  acquired by a camera.
  - faces. An example is provided in tutorial-face-detector-live.cpp.
 */
class VISP_EXPORT vpDetectorBase
{
protected:
  std::vector<std::vector<vpImagePoint> > m_polygon; //!< For each object,
                                                     //!< defines the polygon
                                                     //!< that contains the
                                                     //!< object.
  std::vector<std::string> m_message;                //!< Message attached to each object.
  size_t m_nb_objects;                               //!< Number of detected objects.

public:
  /*!
     Default constructor.
   */
  vpDetectorBase();
  /*!
     Default destructor.
     */
  virtual ~vpDetectorBase() {}

  /*!
    Detect objects in an image.
    \param I : Image where to detect objects.
    \return true if one or multiple objects are detected, false otherwise.
   */
  virtual bool detect(const vpImage<unsigned char> &I) = 0;

  /** @name Inherited functionalities from vpDetectorBase */
  //@{

  /*!
    Return the bounding box of the ith object.
   */
  vpRect getBBox(size_t i) const;

  /*!
    Return the center of gravity location of the ith object.
   */
  vpImagePoint getCog(size_t i) const;

  /*!
    Returns the contained message of the ith object if there is one.
   */
  std::vector<std::string> &getMessage() { return m_message; }

  /*!
    Returns the contained message of the ith object if there is one.
   */
  std::string &getMessage(size_t i);

  /*!
    Return the number of objects that are detected.
    */
  size_t getNbObjects() const { return m_nb_objects; }

  /*!
    Returns object container box as a vector of points.
   */
  std::vector<std::vector<vpImagePoint> > &getPolygon() { return m_polygon; }

  /*!
    Returns ith object container box as a vector of points.
   */
  std::vector<vpImagePoint> &getPolygon(size_t i);

  //@}
};

#endif
