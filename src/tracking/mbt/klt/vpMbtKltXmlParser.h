/****************************************************************************
 *
 * $Id: vpMbtKltXmlParser.h 3530 2012-01-03 10:52:12Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Read MBT KLT Tracker information in an XML file
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
 * \file vpMbtKltXmlParser.h
 * \brief Parse an Xml file to extract configuration parameters of a Mbt Klt object.
*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifndef vpMbtKltXmlParser_HH
#define vpMbtKltXmlParser_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <libxml/xmlmemory.h>      /* Fonctions de la lib XML. */

#include <visp/vpXmlParser.h>
#include <visp/vpCameraParameters.h>

/*!
  \class vpMbtKltXmlParser
  \ingroup ModelBasedTracking

  Data parser for the KLT model based tracker.

 */
class VISP_EXPORT vpMbtKltXmlParser: public vpXmlParser
{
protected:
  //! Threshold used to remove outliers
  double threshold;
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
  //! Angle to determine if a face appeared
  double angleAppear;
  //! Angle to determine if a face disappeared
  double angleDisappear;
  //! Camera parameters.
  vpCameraParameters cam;
    
  typedef enum{
    conf,
    klt,
    mask_border,
    threshold_outlier,
    max_features,
    window_size,
    quality,
    min_distance,
    harris,
    size_block,
    pyramid_lvl,
    face,
    angle_appear,
    angle_disappear,
    camera,
    height,
    width,
    u0,
    v0,
    px,
    py
  } dataToParse;


public:

	vpMbtKltXmlParser();
	virtual ~vpMbtKltXmlParser();

  /*!
    Get the angle to determine if a face appeared.

    \return angleAppear
  */
  inline double getAngleAppear() const {return angleAppear;}
  
  /*!
    Get the angle to determine if a face disappeared.

    \return angleDisappear
  */
  inline double getAngleDisappear() const {return angleDisappear;}
  
  /*!
    Get the size of a block.

    \return blockSize
  */
  inline unsigned int getBlockSize() const {return blockSize;}
  
  /*!
    Get the camera parameters.

    \return cam
  */
  void getCameraParameters(vpCameraParameters& _cam) const { _cam = cam;}
  
  /*!
    Get the Harris free parameter.

    \return harrisParam
  */
  inline double getHarrisParam() const {return harrisParam;}
  
	/*!
    Get the Border of the mask.

    \return faceBorder
  */
  inline unsigned int getMaskBorder() const {return maskBorder;}
  
  /*!
    Get the maximum number of features for the KLT.

    \return maxFeatures
  */
  inline unsigned int getMaxFeatures() const {return maxFeatures;}
  
  /*!
    Get the minimum distance between KLT points.

    \return minDist
  */
  inline double getMinDistance() const {return minDist;}
  
  /*!
    Get the number of pyramid levels

    \return pyramidLevels
  */
  inline unsigned int getPyramidLevels() const {return pyramidLevels;} 
  
  /*!
    Get the quality of the KLT.

    \return quality
  */
  inline double getQuality() const {return qualityValue;}
  
  /*!
    Get the threshold used to remove outliers.

    \return threshold
  */
  inline double getThresholdOutliers() const {return threshold;}
  
  /*!
    Get the size of the window used in the KLT tracker.

    \return winSize
  */
  inline unsigned int getWindowSize() const {return winSize;}
  
  void parse(const char * filename);
  
  void readMainClass(xmlDocPtr doc, xmlNodePtr node);
  void read_camera (xmlDocPtr doc, xmlNodePtr node);
  void read_klt(xmlDocPtr doc, xmlNodePtr node);
  void read_face(xmlDocPtr doc, xmlNodePtr node);
  
  /*!
    Set the angle to determine if a face appeared.

    \param aappear : New angleAppear
  */
  inline void setAngleAppear(const double &aappear) {angleAppear = aappear;}
  
  /*!
    Set the angle to determine if a face disappeared.

    \param adisappear : New angleDisappear
  */
  inline void setAngleDisappear(const double &adisappear) {angleDisappear = adisappear;}
  
  /*!
    Set the size of a block.

    \param bs : New blockSize
  */
  inline void setBlockSize(const unsigned int &bs) {blockSize = bs;}
  
  /*!
    Set the camera parameters.

    \param _cam : New cam
  */
  void setCameraParameters(const vpCameraParameters& _cam) {cam = _cam;}
  
  /*!
    Set the Harris free parameter.

    \param hp : New harrisParam
  */
  inline void setHarrisParam(const double &hp) {harrisParam = hp;}
  
  /*!
    Set the Border of the mask.

    \param mn = new maskBorder
  */
  inline void setMaskBorder(const unsigned int &mb) {maskBorder = mb;}
  
  /*!
    Set the maximum number of features for the KLT.

    \param mF : New maxFeatures
  */
  inline void setMaxFeatures(const unsigned int &mF) {maxFeatures = mF;}
  
  /*!
    Set the minimum distance between KLT points.

    \param mD : New minDist
  */
  inline void setMinDistance(const double &mD) {minDist = mD;}
  
  /*!
    Set the number of pyramid levels

    \param pL : New pyramidLevels
  */
  inline void setPyramidLevels(const unsigned int &pL) {pyramidLevels = pL;} 
  
  /*!
    Set the quality of the KLT.

    \param q : New quality
  */
  inline void setQuality(const double &q) {qualityValue = q;}
  
  /*!
    Set the threshold used to remove outliers.

    \param t : New threshold
  */
  inline void setThresholdOutliers(const double &t) {threshold = t;}
  
  /*!
    Set the size of the window used in the KLT tracker.

    \param w : New winSize
  */
  inline void setWindowSize(const unsigned int &w) {winSize = w;}
  
  void writeMainClass(xmlNodePtr node);
	
protected:
  void init();

};

#endif

#endif

#endif


