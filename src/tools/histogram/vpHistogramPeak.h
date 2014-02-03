/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Gray level histogram manipulation.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/



/*!
  \file vpHistogramPeak.h
  \brief Declaration of the vpHistogramPeak class.
  Class vpHistogramPeak defines a gray level histogram peak.

*/

#ifndef vpHistogramPeak_h
#define vpHistogramPeak_h

#include <visp/vpConfig.h>

#include <ostream>

/*!
  \class vpHistogramPeak

  \ingroup Histogram
  \brief Declaration of the peak (maximum value) in a gray level image
  histogram.

  A peak is internally represented as a gray \e level and a \e value.
  The \e value represents the number of pixels having the gray \e level.

*/

class VISP_EXPORT vpHistogramPeak
{
public :
  vpHistogramPeak();
  vpHistogramPeak(unsigned char level, unsigned value);
  vpHistogramPeak(const vpHistogramPeak & p);

  /*! Destructor that does nothing. */
  virtual ~vpHistogramPeak() {}

  vpHistogramPeak & operator=(const vpHistogramPeak &p);
  bool operator==(const vpHistogramPeak &p) const;

  /*!

    Set the peak gray \e level. To set the number of pixels having this
    gray level use setValue().

    \param level : Location of the peak or gray \e level.

    \sa setValue(), set()

  */
  inline void setLevel(unsigned char level)
    {
      this->level = level;
    };
  /*!

    Set the peak number of pixels having a same gray level. To set the
    gray level of this peak use setLevel().

    \param value    : Number of pixels having the same location or gray level.

    \sa setLevel(), set()

  */
  inline void setValue(unsigned value) 
    {
      this->value = value; 
    };
  /*!

    Set the peak gray \e level and number of pixels at this gray level.

    \param level : Location of the peak or gray level.
    \param value : Number of pixels having the same location or gray level.

    \sa setLevel(), setValue()

  */
  inline void set(unsigned char level, unsigned value)
    {
      this->level = level;
      this->value = value; 
    };
      
  /*!

    Get the peak gray \e level. The number of pixels having this
    gray level is available throw getValue().

    \return Location of the peak or gray level.

    \sa getValue()

  */
  inline unsigned char getLevel() const
    { 
      return level;
    };
  /*!

    Get the peak number of pixels having a same gray level. The
    corresponding gray level is available throw getLevel().

    \return    : Number of pixels having the same location or gray level.

    \sa getLevel()

  */
  inline unsigned getValue() const
    {
      return value;
    };

  //---------------------------------
  // Printing
  //---------------------------------
  friend VISP_EXPORT std::ostream &operator << (std::ostream &s,	const vpHistogramPeak &p);


protected :
  unsigned char level; //! Gray level ot the peak.
  unsigned value;         //! Number of pixels on the gray level.
};


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
