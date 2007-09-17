/****************************************************************************
 *
 * $Id: vpHistogramValey.h,v 1.2 2007-09-17 09:16:13 fspindle Exp $
 *
 * Copyright (C) 1998-2007 Inria. All rights reserved.
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
  \file vpHistogramValey.h
  \brief Declaration of the vpHistogramValey class.
  Class vpHistogramValey defines a gray level histogram valey.

*/

#ifndef vpHistogramValey_h
#define vpHistogramValey_h

#include <visp/vpConfig.h>
#include <visp/vpHistogramPeak.h>

/*!
  \class vpHistogramValey

  \brief Declaration of the valey (minimum value) in a gray level image
  histogram.

  A valey is internally represented as a gray \e level and a \e value.
  The \e value represents the number of pixels having the gray \e level.

*/

class VISP_EXPORT vpHistogramValey : vpHistogramPeak
{
 public:
  vpHistogramValey() :
    vpHistogramPeak() {};
    
  vpHistogramValey(unsigned char level, unsigned value) : 
    vpHistogramPeak(level, value) {};

  vpHistogramValey(const vpHistogramValey & v) : 
    vpHistogramPeak(v) {};

  vpHistogramValey & operator=(const vpHistogramValey &v);
  bool operator==(const vpHistogramValey &v) const;

  /*!

    Set the valey gray \e level. To set the number of pixels having this
    gray level use setValue().

    \param level : Location of the valey or gray level.

    \sa setValue(), set()

  */
  inline void setLevel(unsigned char level)
    {
      this->level = level;
    };
  /*!

    Set the valey number of pixels having a same gray level. To set the
    gray level of this valey use setLevel().

    \param value    : Number of pixels having the same location or gray level.

    \sa setPosition(), set()

  */
  inline void setValue(unsigned value) 
    {
      this->value = value; 
    };
  /*!

    Set the valey gray \e level and number of pixels at this location.

    \param level : Location of the valey or gray level.
    \param value : Number of pixels having the same location or gray level.

    \sa setLevel(), setValue()

  */
  inline void set(unsigned char level, unsigned value)
    {
      this->level = level;
      this->value = value; 
    };
      
  /*!

    Get the valey gray \e level. The number of pixels having this
    gray level is available throw getValue().

    \return Location of the valey or gray level.

    \sa getValue()

  */
  inline unsigned char getLevel() const
    { 
      return level;
    };
  /*!

    Get the valey number of pixels having a same gray level. The
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
						const vpHistogramValey &v);
};

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
