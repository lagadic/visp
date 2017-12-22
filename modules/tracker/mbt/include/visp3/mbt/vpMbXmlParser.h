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
 * Load XML Parameter for Model Based Tracker.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
 * \file vpMbXmlParser.h
 * \brief Parse an Xml file to extract configuration parameters of a mbtConfig
 * object.
 */

#ifndef vpMbXmlParser_HH
#define vpMbXmlParser_HH

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <libxml/xmlmemory.h> /* Fonctions de la lib XML.                */

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParser.h>

/*!
  \class vpMbXmlParser
  \brief Parse an Xml file to extract configuration parameters of a mbtConfig
  object. \ingroup group_mbt_xml_parser

  Data parser for the model based tracker.

 */
class VISP_EXPORT vpMbXmlParser : public vpXmlParser
{
protected:
  //! Camera parameters.
  vpCameraParameters cam;
  //! Angle to determine if a face appeared
  double angleAppear;
  //! Angle to determine if a face disappeared
  double angleDisappear;
  //! Is near clipping distance specified?
  bool hasNearClipping;
  //! Near clipping distance
  double nearClipping;
  //! Is far clipping distance specified?
  bool hasFarClipping;
  //! Near clipping distance
  double farClipping;
  //! Fov Clipping
  bool fovClipping;
  //! If true, the LOD is enabled, otherwise it is not
  bool useLod;
  //! Minimum line length to track a segment when LOD is enabled
  double minLineLengthThreshold;
  //! Minimum polygon area to track a face when LOD is enabled
  double minPolygonAreaThreshold;

  typedef enum {
    conf,
    face,
    angle_appear,
    angle_disappear,
    near_clipping,
    far_clipping,
    fov_clipping,
    camera,
    height,
    width,
    u0,
    v0,
    px,
    py,
    lod,
    use_lod,
    min_line_length_threshold,
    min_polygon_area_threshold,
    last
  } dataToParseMb;

public:
  /** @name Public Member Functions Inherited from vpMbXmlParser */
  //@{
  vpMbXmlParser();
  virtual ~vpMbXmlParser();

  /*!
    Get the angle to determine if a face appeared.

    \return angleAppear
  */
  inline double getAngleAppear() const { return angleAppear; }

  /*!
    Get the angle to determine if a face disappeared.

    \return angleDisappear
  */
  inline double getAngleDisappear() const { return angleDisappear; }

  void getCameraParameters(vpCameraParameters &_cam) const { _cam = this->cam; }

  /*!
    Get the far clipping distance.

    \return farClipping
  */
  inline double getFarClippingDistance() const { return farClipping; }

  /*!
    Use FOV clipping

    \return True if yes, False otherwise.
  */
  inline bool getFovClipping() const { return fovClipping; }

  /*!
     Get the state of LOD setting.

     \return True if LOD is enabled, false otherwise.
   */
  inline bool getLodState() const { return useLod; }

  /*!
     Get the minimum line length to track a segment when LOD is enabled.

     \return The minimum line length.
   */
  inline double getMinLineLengthThreshold() const { return minLineLengthThreshold; }

  /*!
     Get the minimum polygon area to track a face when LOD is enabled.

     \return The minimum polygon area.
   */
  inline double getMinPolygonAreaThreshold() const { return minPolygonAreaThreshold; }

  /*!
    Get the near clipping distance.

    \return nearClipping
  */
  inline double getNearClippingDistance() const { return nearClipping; }

  /*!
    Has Far clipping been specified?

    \return True if yes, False otherwise.
  */
  inline bool hasFarClippingDistance() const { return hasFarClipping; }

  /*!
    Has Near clipping been specified?

    \return True if yes, False otherwise.
  */
  inline bool hasNearClippingDistance() const { return hasNearClipping; }

  void parse(const char *filename);

  virtual void readMainClass(xmlDocPtr doc, xmlNodePtr node);
  void read_camera(xmlDocPtr doc, xmlNodePtr node);
  void read_face(xmlDocPtr doc, xmlNodePtr node);
  void read_lod(xmlDocPtr doc, xmlNodePtr node);

  /*!
    Set the angle to determine if a face appeared.

    \param aappear : New angleAppear
  */
  inline void setAngleAppear(const double &aappear) { angleAppear = aappear; }

  /*!
    Set the angle to determine if a face disappeared.

    \param adisappear : New angleDisappear
  */
  inline void setAngleDisappear(const double &adisappear) { angleDisappear = adisappear; }

  void setCameraParameters(const vpCameraParameters &_cam) { cam = _cam; }

  /*!
    Set the far clipping distance.

    \param fclip : New farClipping
  */
  inline void setFarClippingDistance(const double &fclip) { farClipping = fclip; }

  /*!
    Set the near clipping distance.

    \param nclip : New nearClipping
  */
  inline void setNearClippingDistance(const double &nclip) { nearClipping = nclip; }

  void writeMainClass(xmlNodePtr node);
  //@}

protected:
  /** @name Protected Member Functions Inherited from vpMbXmlParser */
  //@{
  void init();
  //@}
};

#endif

#endif /* NMBXMLPARSER_H_ */
