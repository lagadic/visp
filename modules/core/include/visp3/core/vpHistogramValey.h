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
 * Gray level histogram manipulation.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpHistogramValey.h
  \brief Declaration of the vpHistogramValey class.
  Class vpHistogramValey defines a gray level histogram valey.

*/

#ifndef vpHistogramValey_h
#define vpHistogramValey_h

#include <visp3/core/vpHistogramPeak.h>

/*!
  \class vpHistogramValey

  \ingroup group_core_histogram
  \brief Declaration of the valey (minimum value) in a gray level image
  histogram.

  A valey is internally represented as a gray \e level and a \e value.
  The \e value represents the number of pixels having the gray \e level.

*/

class VISP_EXPORT vpHistogramValey : vpHistogramPeak
{
public:
  vpHistogramValey() : vpHistogramPeak(){};

  vpHistogramValey(unsigned char lvl, unsigned val) : vpHistogramPeak(lvl, val){};

  vpHistogramValey(const vpHistogramValey &v) : vpHistogramPeak(v){};

  /*! Destructor that does nothing. */
  virtual ~vpHistogramValey() {}

  vpHistogramValey &operator=(const vpHistogramValey &v);
  bool operator==(const vpHistogramValey &v) const;

  /*!

    Set the valey gray \e level. To set the number of pixels having this
    gray level use setValue().

    \param lvl : Location of the valey or gray level.

    \sa setValue(), set()

  */
  inline void setLevel(unsigned char lvl) { this->level = lvl; };
  /*!

    Set the valey number of pixels having a same gray level. To set the
    gray level of this valey use setLevel().

    \param val : Number of pixels having the same location or gray level.

    \sa setPosition(), set()

  */
  inline void setValue(unsigned val) { this->value = val; };
  /*!

    Set the valey gray \e level and number of pixels at this location.

    \param lvl : Location of the valey or gray level.
    \param val : Number of pixels having the same location or gray level.

    \sa setLevel(), setValue()

  */
  inline void set(unsigned char lvl, unsigned val)
  {
    this->level = lvl;
    this->value = val;
  };

  /*!

    Get the valey gray \e level. The number of pixels having this
    gray level is available through getValue().

    \return Location of the valey or gray level.

    \sa getValue()

  */
  inline unsigned char getLevel() const { return level; };
  /*!

    Get the valey number of pixels having a same gray level. The
    corresponding gray level is available through getLevel().

    \return  Number of pixels having the same location or gray level.

    \sa getLevel()

  */
  inline unsigned getValue() const { return value; };

  //---------------------------------
  // Printing
  //---------------------------------
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const vpHistogramValey &v);
};

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
