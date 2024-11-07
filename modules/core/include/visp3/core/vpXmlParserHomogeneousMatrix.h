/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * XML parser to load and save Homogeneous Matrix in a XML file.
 */

/*!
  \file vpXmlParserHomogeneousMatrix.h
  \brief Declaration of the vpXmlParserHomogeneousMatrix class.
  Class vpXmlParserHomogeneousMatrix allowed to load and save Homogeneous
  Matrixes in a file XML

*/

#ifndef VP_XML_PARSER_HOMOGENEOUS_MATRIX_H
#define VP_XML_PARSER_HOMOGENEOUS_MATRIX_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PUGIXML)
#include <visp3/core/vpHomogeneousMatrix.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpXmlParserHomogeneousMatrix

  \ingroup group_core_transformations

  \brief XML parser to load and save an homogeneous matrix in a file.

  \warning This class is only available if pugixml third-party is successfully
built.

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
  // Create Pose Vector and convert to homogeneous matrix
  vpPoseVector r(1.0,1.3,3.5,0.2,0.3,0.5);
  vpHomogeneousMatrix M(r);

  // Create a XML parser
  vpXmlParserHomogeneousMatrix p;

  // Define the name of the matrix
  std::string name_M =  "eMe";

  // Define name of the file xml to fill
  std::string filename = "homogeneous_matrixes.xml";

  if (p.save(M, filename, name_M) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot save the Homogeneous matrix" << std::endl;
  }
  return 0;
}
  \endcode
*/

class VISP_EXPORT vpXmlParserHomogeneousMatrix
{
public:
  typedef enum { SEQUENCE_OK, SEQUENCE_ERROR } vpXmlCodeSequenceType;

  vpXmlParserHomogeneousMatrix();
  ~vpXmlParserHomogeneousMatrix();

  // get/set functions
  vpHomogeneousMatrix getHomogeneousMatrix() const;
  std::string getHomogeneousMatrixName() const;

  int parse(vpHomogeneousMatrix &M, const std::string &filename, const std::string &name);

  int save(const vpHomogeneousMatrix &M, const std::string &filename, const std::string &name);

  void setHomogeneousMatrixName(const std::string &name);

private:
  vpXmlParserHomogeneousMatrix(const vpXmlParserHomogeneousMatrix &hm);            // noncopyable
  vpXmlParserHomogeneousMatrix &operator=(const vpXmlParserHomogeneousMatrix &); //

  // PIMPL idiom
  class Impl;
  Impl *m_impl;
};
END_VISP_NAMESPACE
#endif
#endif
