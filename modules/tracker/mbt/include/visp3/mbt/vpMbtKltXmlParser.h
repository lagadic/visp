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
 * Load XML parameters of the Model based tracker (using point features).
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
 * \file vpMbtKltXmlParser.h
 * \brief Parse an Xml file to extract configuration parameters of a Mbt Klt
 * object.
 */
#ifndef vpMbtKltXmlParser_HH
#define vpMbtKltXmlParser_HH

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <libxml/xmlmemory.h> /* Fonctions de la lib XML. */

#include <visp3/mbt/vpMbXmlParser.h>

/*!
  \class vpMbtKltXmlParser
  \brief Parse an Xml file to extract configuration parameters of a Mbt Klt
  object. \ingroup group_mbt_xml_parser

  Data parser for the KLT model based tracker.

*/
class VISP_EXPORT vpMbtKltXmlParser : virtual public vpMbXmlParser
{
protected:
  //! Border of the mask used on Klt points
  unsigned int maskBorder;
  //! Maximum of Klt features
  unsigned int maxFeatures;
  //! Windows size
  unsigned int winSize;
  //! Quality of the Klt points
  double qualityValue;
  //! Minimum distance between klt points
  double minDist;
  //! Harris free parameters
  double harrisParam;
  //! Block size
  unsigned int blockSize;
  //! Number of pyramid levels
  unsigned int pyramidLevels;

  typedef enum {
    klt = vpMbXmlParser::last,
    mask_border,
    max_features,
    window_size,
    quality,
    min_distance,
    harris,
    size_block,
    pyramid_lvl,
    last
  } dataToParseMbKlt;

public:
  /** @name Public Member Functions Inherited from vpMbtKltXmlParser */
  //@{
  vpMbtKltXmlParser();
  virtual ~vpMbtKltXmlParser();

  /*!
    Get the size of a block.

    \return blockSize
  */
  inline unsigned int getBlockSize() const { return blockSize; }

  /*!
    Get the Harris free parameter.

    \return harrisParam
  */
  inline double getHarrisParam() const { return harrisParam; }

  /*!
Get the Border of the mask.

\return faceBorder
*/
  inline unsigned int getMaskBorder() const { return maskBorder; }

  /*!
    Get the maximum number of features for the KLT.

    \return maxFeatures
  */
  inline unsigned int getMaxFeatures() const { return maxFeatures; }

  /*!
    Get the minimum distance between KLT points.

    \return minDist
  */
  inline double getMinDistance() const { return minDist; }

  /*!
    Get the number of pyramid levels

    \return pyramidLevels
  */
  inline unsigned int getPyramidLevels() const { return pyramidLevels; }

  /*!
    Get the quality of the KLT.

    \return quality
  */
  inline double getQuality() const { return qualityValue; }

  /*!
    Get the size of the window used in the KLT tracker.

    \return winSize
  */
  inline unsigned int getWindowSize() const { return winSize; }

  void parse(const char *filename);

  virtual void readMainClass(xmlDocPtr doc, xmlNodePtr node);
  void read_klt(xmlDocPtr doc, xmlNodePtr node);

  /*!
    Set the size of a block.

    \param bs : New blockSize
  */
  inline void setBlockSize(const unsigned int &bs) { blockSize = bs; }

  /*!
    Set the Harris free parameter.

    \param hp : New harrisParam
  */
  inline void setHarrisParam(const double &hp) { harrisParam = hp; }

  /*!
    Set the Border of the mask.

    \param mb = new maskBorder
  */
  inline void setMaskBorder(const unsigned int &mb) { maskBorder = mb; }

  /*!
    Set the maximum number of features for the KLT.

    \param mF : New maxFeatures
  */
  inline void setMaxFeatures(const unsigned int &mF) { maxFeatures = mF; }

  /*!
    Set the minimum distance between KLT points.

    \param mD : New minDist
  */
  inline void setMinDistance(const double &mD) { minDist = mD; }

  /*!
    Set the number of pyramid levels

    \param pL : New pyramidLevels
  */
  inline void setPyramidLevels(const unsigned int &pL) { pyramidLevels = pL; }

  /*!
    Set the quality of the KLT.

    \param q : New quality
  */
  inline void setQuality(const double &q) { qualityValue = q; }

  /*!
    Set the size of the window used in the KLT tracker.

    \param w : New winSize
  */
  inline void setWindowSize(const unsigned int &w) { winSize = w; }

  void writeMainClass(xmlNodePtr node);
  //@}

protected:
  /** @name Protected Member Functions Inherited from vpMbtKltXmlParser */
  //@{
  void init();
  //@}
};

#endif

#endif
