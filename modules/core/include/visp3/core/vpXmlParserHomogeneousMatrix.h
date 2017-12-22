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
 * XML parser to load and save Homogeneous Matrix in a XML file
 *
 * Authors:
 * Giovanni Claudio
 *
 *****************************************************************************/

/*!
  \file vpXmlParserHomogeneousMatrix.h
  \brief Declaration of the vpXmlParserHomogeneousMatrix class.
  Class vpXmlParserHomogeneousMatrix allowed to load and save Homogeneous
  Matrixes in a file XML

*/

#ifndef vpXMLPARSERHOMOGENEOUSMATRIX_H
#define vpXMLPARSERHOMOGENEOUSMATRIX_H

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <libxml/xmlmemory.h> /* Functions of libxml.                */
#include <string>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpXmlParser.h>

/*!
  \class vpXmlParserHomogeneousMatrix

  \ingroup group_core_transformations

  \brief XML parser to load and save an homogeneous matrix in a file.

  \warning This class is only available if libxml2 is installed and detected
by ViSP. Installation instructions are provided here
https://visp.inria.fr/3rd_xml2.

  To have a complete description of the homogeneous matrix implemented in
ViSP, see vpHomogeneousMatrix.

  Example of an XML file "homogeneous_matrixes.xml" containing a Pose vector
  that will be converted in an homogeneous matrix:

  \code
<?xml version="1.0"?>
<root>
  <homogeneous_transformation>
    <!--Name of the homogeneous matrix-->
    <name>eMc</name>
    <values>
      <!--Translation vector with values in meters -->
      <tx>1.00</tx>
      <ty>1.30</ty>
      <tz>3.50</tz>
      <!--Rotational vector expressed in angle axis representation with values
in radians --> <theta_ux>0.20</theta_ux> <theta_uy>0.30</theta_uy>
      <theta_uz>0.50</theta_uz>
    </values>
  </homogeneous_transformation>
</root>
  \endcode

  Example of loading an existing homogeneous matrix from an XML file.
  \code

#include <iostream>
#include <string>

#include <visp3/core/vpXmlParserHomogeneousMatrix.h>

int main(int argc, char* argv[])
{
#ifdef VISP_HAVE_XML2
  vpHomogeneousMatrix eMc;

  // Create a XML parser
  vpXmlParserHomogeneousMatrix p;

  // Define the name of the matrix to load
  std::string name = "eMc";

  if (p.parse(eMc,"homogeneous_matrixes.xml", name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot found the Homogeneous matrix named " << name << "." << std::endl;
  }
  else
    std::cout << "Homogeneous matrix " << name <<": " << std::endl << eMc << std::endl;
#endif

  return 0;
}
  \endcode

  Example of writing an homogenoeus matrix in a XML file.
  \note Before writing an homogeneous matrix check if there
  is already in the xml file a matrix with the same name.
  If you are sure to overwrite it please delete it manually
  from the file before.

  \code
#include <iostream>
#include <string>

#include <visp3/core/vpXmlParserHomogeneousMatrix.h>

int main(int argc, char* argv[])
{
#ifdef VISP_HAVE_XML2
  // Create Pose Vector and convert to homogeneous matrix
  vpPoseVector r(1.0,1.3,3.5,0.2,0.3,0.5);
  vpHomogeneousMatrix M(r);

  // Create a XML parser
  vpXmlParserHomogeneousMatrix p;

  // Define the name of the matrix
  std::string name_M =  "eMe";

  // Define name of the file xml to fill
  char filename[FILENAME_MAX];
  sprintf(filename, "%s", "homogeneous_matrixes.xml");

  if (p.save(M, filename, name_M) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot save the Homogeneous matrix" << std::endl;
  }

  vpXmlParser::cleanup();
#endif
  return 0;
}
  \endcode
*/

class VISP_EXPORT vpXmlParserHomogeneousMatrix : public vpXmlParser
{

public:
  /* --- XML Code------------------------------------------------------------
   */
  typedef enum {
    CODE_XML_BAD = -1,
    CODE_XML_OTHER,
    CODE_XML_M,
    CODE_XML_M_NAME,
    CODE_XML_VALUE,
    CODE_XML_TX,
    CODE_XML_TY,
    CODE_XML_TZ,
    CODE_XML_TUX,
    CODE_XML_TUY,
    CODE_XML_TUZ
  } vpXmlCodeType;

  typedef enum { SEQUENCE_OK, SEQUENCE_ERROR } vpXmlCodeSequenceType;

private:
  vpHomogeneousMatrix m_M;
  std::string m_name;

public:
  vpXmlParserHomogeneousMatrix();
  vpXmlParserHomogeneousMatrix(vpXmlParserHomogeneousMatrix &twinParser);
  //! Default destructor.
  virtual ~vpXmlParserHomogeneousMatrix() {}

  // get/set functions
  vpHomogeneousMatrix getHomogeneousMatrix() const { return this->m_M; }
  std::string getHomogeneousMatrixName() const { return this->m_name; }

  vpXmlParserHomogeneousMatrix &operator=(const vpXmlParserHomogeneousMatrix &twinparser);
  int parse(vpHomogeneousMatrix &M, const std::string &filename, const std::string &name);

  int save(const vpHomogeneousMatrix &M, const std::string &filename, const std::string &name);

  void setHomogeneousMatrixName(const std::string &name) { this->m_name = name; }

private:
  int read(xmlDocPtr doc, xmlNodePtr node, const std::string &name);

  int count(xmlDocPtr doc, xmlNodePtr node, const std::string &name);

  int read_matrix(xmlDocPtr doc, xmlNodePtr node, const std::string &name);

  vpXmlCodeSequenceType read_values(xmlDocPtr doc, xmlNodePtr node, vpHomogeneousMatrix &M);

  static vpXmlCodeSequenceType str2xmlcode(char *str, vpXmlCodeType &res);
  void myXmlReadIntChild(xmlDocPtr doc, xmlNodePtr node, int &res, vpXmlCodeSequenceType &code_error);

  void myXmlReadDoubleChild(xmlDocPtr doc, xmlNodePtr node, double &res, vpXmlCodeSequenceType &code_error);

  void myXmlReadCharChild(xmlDocPtr doc, xmlNodePtr node, char **res);
  int write(xmlNodePtr node, const std::string &name);

private:
  /*!

    \param 2doc : a pointer representing the document
    \param node : the root node of the document
  */
  virtual void readMainClass(xmlDocPtr, xmlNodePtr){};

  /*!


    \param node2 : the root node of the document
  */
  virtual void writeMainClass(xmlNodePtr){};
};
#endif // VISP_HAVE_XML2
#endif
