/****************************************************************************
 *
 * $Id: vpHistogramPeak.h,v 1.3 2008-09-26 15:20:58 fspindle Exp $
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
 * This file is part of the ViSP toolkit.
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

#include <ostream>

#include <visp/vpConfig.h>

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
  friend VISP_EXPORT std::ostream &operator << (std::ostream &s,
						const vpHistogramPeak &p);


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
