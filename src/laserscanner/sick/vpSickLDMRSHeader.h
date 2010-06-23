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
 * Sick LD-MRS laser driver.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpSickLDMRSHeader_h
#define vpSickLDMRSHeader_h

//#include <arpa/inet.h>
#include <iostream>
#include <vector>

#include "visp/vpConfig.h"

/*!
  \file vpSickLDMRSHeader.h

  \brief Implements the structure of the Sick LD-MRS data header
  messages that are transmitted by the laser scanner.

*/

/*!

  \class vpSickLDMRSHeader

  \brief Class that defines the structure of the Sick LD-MRS data header
  messages that are transmitted by the laser scanner.

  \warning The header size should be always 24 bytes.
*/
class VISP_EXPORT vpSickLDMRSHeader
{
 private:
  // Header data
  
  unsigned int magicWord;      ///< The magic word
  unsigned int sizeOfPrevMsg;  ///< Size of the previous message in bytes
  unsigned int sizeOfThisMsg;  ///< Size of this message in bytes
  unsigned char  reserved;     ///< Reserved for future use
  unsigned char  deviceID;     ///< ID of the device that has sent this message
  unsigned short dataType;     ///< Type of the data in the message body
  unsigned int timeSec;	       ///< The number of seconds elapsed since 1900-01-01 00:00:00
  unsigned int timeSecFractionOffset; ///< Offset in 1/(2^32) second resolution to #timeSec
 public:
  enum MagicWord {
    MagicWordC2 = 0xAFFEC0C2   ///< The magic word that allows to identify the messages that are sent by the Sick LD-MRS.
  };
  enum DataType {
    MeasuredData = 0x2202      ///< Flag to indicate that the body of a message contains measured data.
  };

 public:
  /*! Returns the magic word 0xAFFEC0C2. */
  inline unsigned int getMagicWord() {
    return ntohl(magicWord);
  }
  /*! Returns the size in bytes of the message body following the header. */
  inline unsigned int getMsgLenght() {
    return ntohl(sizeOfThisMsg);
  }
  /*! Returns the timestamp. */
  inline double getTimestamp() {
    unsigned int sec = ntohl(timeSec);
    unsigned int frac = ntohl(timeSecFractionOffset);
    return (sec + frac/4294967296.); // 4294967296 = 2^32
  }
  /*! Returns the type of data in the message body. For the moment
      only vpSickLDMRSHeader::DataType messages are considered. */
  inline unsigned short getDataType() {
    return ntohs(dataType);
  }
  
};

#endif
