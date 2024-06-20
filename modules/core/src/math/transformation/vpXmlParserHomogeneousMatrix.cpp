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
 * XML parser to load and save Homogeneous Matrix in a XML file
 */

/*!
  \file vpXmlParserHomogeneousMatrix.cpp
  \brief Definition of the vpXmlParserHomogeneousMatrix class member
  functions. Class vpXmlParserHomogeneousMatrix allowed to load and save an
  homogeneous matrix in a XML file.
*/
#include <visp3/core/vpXmlParserHomogeneousMatrix.h>

#if defined(VISP_HAVE_PUGIXML)
#include <pugixml.hpp>

/* ----------------------------- LABEL XML ----------------------------- */
/* --------------------------------------------------------------------- */
#define LABEL_XML_ROOT "root"
#define LABEL_XML_M "homogeneous_transformation"
#define LABEL_XML_M_NAME "name"
#define LABEL_XML_VALUE "values"
#define LABEL_XML_TRANSLATION "translation"
#define LABEL_XML_TX "tx"
#define LABEL_XML_TY "ty"
#define LABEL_XML_TZ "tz"
#define LABEL_XML_ROTATION "rotation"
#define LABEL_XML_TUX "theta_ux"
#define LABEL_XML_TUY "theta_uy"
#define LABEL_XML_TUZ "theta_uz"

BEGIN_VISP_NAMESPACE
#ifndef DOXYGEN_SHOULD_SKIP_THIS
class vpXmlParserHomogeneousMatrix::Impl
{
private:
  /* --- XML Code------------------------------------------------------------
   */
  enum vpXmlCodeType
  {
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
  };

public:
  Impl() : m_M(), m_name() { }

  int parse(vpHomogeneousMatrix &M, const std::string &filename, const std::string &name)
  {
    pugi::xml_document doc;
    if (!doc.load_file(filename.c_str())) {
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << " I cannot open the file " << filename << std::endl;

      return SEQUENCE_ERROR;
    }

    pugi::xml_node node = doc.document_element();
    if (!node) {
      return SEQUENCE_ERROR;
    }

    int ret = read(node, name);

    M = m_M;

    return ret;
  }

  /*!
    Read Homogeneous matrix values from a XML file.

    \param node_ : XML tree, pointing on a marker equipment.
    \param name : name of the Homogeneous Matrix
    \return error code.
   */
  int read(const pugi::xml_node &node_, const std::string &name)
  {
    vpXmlCodeType prop;

    vpXmlCodeSequenceType back = SEQUENCE_OK;
    unsigned int nbM = 0;

    for (pugi::xml_node node = node_.first_child(); node; node = node.next_sibling()) {
      if (node.type() == pugi::node_element) {
        if (SEQUENCE_OK != str2xmlcode(node.name(), prop)) {
          prop = CODE_XML_OTHER;
          back = SEQUENCE_ERROR;
        }

        if (prop == CODE_XML_M) {
          if (SEQUENCE_OK == read_matrix(node, name)) {
            nbM++;
          }
        }
        else {
          back = SEQUENCE_ERROR;
        }
      }
    }

    if (nbM == 0) {
      back = SEQUENCE_ERROR;
      std::cerr << "No Homogeneous matrix is available" << std::endl << "with name: " << name << std::endl;
    }
    else if (nbM > 1) {
      back = SEQUENCE_ERROR;
      std::cerr << nbM << " There are more Homogeneous matrix" << std::endl
        << "with the same name : " << std::endl
        << "precise your choice..." << std::endl;
    }

    return back;
  }

  /*!
    Read Homogeneous Matrix fields from a XML file.

    \param node_ : XML tree, pointing on a marker equipment.
    \param name : name of the Homogeneous matrix

    \return error code.
   */
  int read_matrix(const pugi::xml_node &node_, const std::string &name)
  {
    vpXmlCodeType prop;
    /* read value in the XML file. */
    std::string M_name_tmp = "";
    vpHomogeneousMatrix M_tmp;

    vpXmlCodeSequenceType back = SEQUENCE_OK;

    for (pugi::xml_node node = node_.first_child(); node; node = node.next_sibling()) {
      if (node.type() == pugi::node_element) {
        if (SEQUENCE_OK != str2xmlcode(node.name(), prop)) {
          prop = CODE_XML_OTHER;
          back = SEQUENCE_ERROR;
        }

        switch (prop) {
        case CODE_XML_M_NAME: {
          M_name_tmp = node.text().as_string();
          break;
        }

        case CODE_XML_VALUE: // VALUE
          if (name == M_name_tmp) {
            std::cout << "Found Homogeneous Matrix with name: \"" << M_name_tmp << "\"" << std::endl;
            back = read_values(node, M_tmp);
          }
          break;

        case CODE_XML_BAD:
        case CODE_XML_OTHER:
        case CODE_XML_M:
        case CODE_XML_TX:
        case CODE_XML_TY:
        case CODE_XML_TZ:
        case CODE_XML_TUX:
        case CODE_XML_TUY:
        case CODE_XML_TUZ:

        default:
          back = SEQUENCE_ERROR;
          break;
        }
      }
    }

    if (!(name == M_name_tmp)) {
      back = SEQUENCE_ERROR;
    }
    else {
      this->m_M = M_tmp;
      // --comment: std::cout << "Convert in Homogeneous Matrix:"<< std::endl;
      // --comment: std::cout << this-> M << std::endl;
      this->m_name = M_name_tmp;
    }
    return back;
  }

  /*!
    Read homogeneous matrix fields from a XML file.

    \param node_ : XML tree, pointing on a marker equipment.
    \param M : Homogeneous matrix to fill with read data (output).

    \return error code.
   */
  vpXmlCodeSequenceType read_values(const pugi::xml_node &node_, vpHomogeneousMatrix &M)
  {
    // counter of the number of read parameters
    int nb = 0;
    vpXmlCodeType prop;
    /* read value in the XML file. */

    double tx_ = 0.;
    double ty_ = 0.;
    double tz_ = 0.;
    double tux_ = 0.;
    double tuy_ = 0.;
    double tuz_ = 0.;

    vpXmlCodeSequenceType back = SEQUENCE_OK;

    for (pugi::xml_node node = node_.first_child(); node; node = node.next_sibling()) {
      if (node.type() == pugi::node_element) {
        if (SEQUENCE_OK != str2xmlcode(node.name(), prop)) {
          prop = CODE_XML_OTHER;
          back = SEQUENCE_ERROR;
        }

        switch (prop) {
        case CODE_XML_TX:
          tx_ = node.text().as_double();
          nb++;
          break;
        case CODE_XML_TY:
          ty_ = node.text().as_double();
          nb++;
          break;
        case CODE_XML_TZ:
          tz_ = node.text().as_double();
          nb++;
          break;
        case CODE_XML_TUX:
          tux_ = node.text().as_double();
          nb++;
          break;
        case CODE_XML_TUY:
          tuy_ = node.text().as_double();
          nb++;
          break;
        case CODE_XML_TUZ:
          tuz_ = node.text().as_double();
          nb++;
          break;

        case CODE_XML_BAD:
        case CODE_XML_OTHER:
        case CODE_XML_M:
        case CODE_XML_M_NAME:
        case CODE_XML_VALUE:

        default:
          back = SEQUENCE_ERROR;
          break;
        }
      }
    }

    if (nb != 6) {
      std::cerr << "ERROR in 'model' field:\n";
      std::cerr << "it must contain 6 parameters\n";

      return SEQUENCE_ERROR;
    }

    // Create the Homogeneous matrix
    M.build(tx_, ty_, tz_, tux_, tuy_, tuz_);

    //  --comment: std::cout << "Read values from file:" << std::endl;
    //  --comment: std::cout << "tx:" << tx_<< std::endl;
    //  --comment: std::cout << "ty:" << ty_<< std::endl;
    //  --comment: std::cout << "tz:" << tz_<< std::endl;
    //  --comment: std::cout << "tux:" << tux_<< std::endl;
    //  --comment: std::cout << "tuy:" << tuy_<< std::endl;
    //  --comment: std::cout << "tuz:" << tuz_<< std::endl;

    return back;
  }

  int save(const vpHomogeneousMatrix &M, const std::string &filename, const std::string &name)
  {
    pugi::xml_document doc;
    pugi::xml_node node;

    if (!doc.load_file(filename.c_str(), pugi::parse_default | pugi::parse_comments)) {
      node = doc.append_child(pugi::node_declaration);
      node.append_attribute("version") = "1.0";
      node = doc.append_child(LABEL_XML_ROOT);
      pugi::xml_node nodeComment = node.append_child(pugi::node_comment);
      nodeComment.set_value("This file stores homogeneous matrix used\n"
                            "   in the vpHomogeneousMatrix Class of ViSP available\n"
                            "   at https://visp.inria.fr/download/ .\n"
                            "   It can be read with the parse method of\n"
                            "   the vpXmlParserHomogeneousMatrix class.");
    }

    node = doc.document_element();
    if (!node) {
      return SEQUENCE_ERROR;
    }

    m_M = M;

    int M_isFound = count(node, name);

    if (M_isFound) {
      std::cout << "There is already an homogeneous matrix " << std::endl
        << "available in the file with the input name: " << name << "." << std::endl
        << "Please delete it manually from the xml file." << std::endl;
      return SEQUENCE_ERROR;
    }

    write(node, name);

    doc.save_file(filename.c_str(), PUGIXML_TEXT("  "));

    return SEQUENCE_OK;
  }

  /*!
    Read homogeneous matrix names from a XML file and read if there is already a
    homogeneous matrix with the same name.

    \param node_ : XML tree, pointing on a marker equipment.
    \param name : name of the homogeneous matrix.

    \return 1 if there is an homogeneous matrix corresponding with the input
    name, 0 otherwise.
   */
  int count(const pugi::xml_node &node_, const std::string &name)
  {
    vpXmlCodeType prop;
    int nbM = 0;

    for (pugi::xml_node node = node_.first_child(); node; node = node.next_sibling()) {
      if (node.type() == pugi::node_element) {
        if (SEQUENCE_OK != str2xmlcode(node.name(), prop)) {
          prop = CODE_XML_OTHER;
        }
        if (prop == CODE_XML_M) {
          if (SEQUENCE_OK == read_matrix(node, name)) {
            nbM++;
          }
        }
      }
    }

    return nbM;
  }

  /*!
    Write Homogeneous Matrix in an XML Tree.

    \param node : XML tree, pointing on a marker equipment.
    \param name : name of the Homogeneous Matrix.

    \return error code.
   */
  int write(pugi::xml_node &node, const std::string &name)
  {
    int back = SEQUENCE_OK;

    pugi::xml_node node_tmp;
    pugi::xml_node node_matrix;
    pugi::xml_node node_values;

    // Convert from Rotational matrix to Theta-U vector
    vpRotationMatrix R;
    m_M.extract(R);

    vpThetaUVector tu(R);

    // <homogeneous_transformation>
    node_tmp = node.append_child(pugi::node_comment);
    node_tmp.set_value("Homogeneous Matrix");
    node_matrix = node.append_child(LABEL_XML_M);
    {
      //<name>
      if (!name.empty()) {
        node_tmp = node_matrix.append_child(pugi::node_comment);
        node_tmp.set_value("Name of the homogeneous matrix");
        node_matrix.append_child(LABEL_XML_M_NAME).append_child(pugi::node_pcdata).set_value(name.c_str());
      }

      //<values>
      node_values = node_matrix.append_child(LABEL_XML_VALUE);
      {
        node_tmp = node_values.append_child(pugi::node_comment);
        node_tmp.set_value("Translation vector with values in meters");

        //<tx>
        node_values.append_child(LABEL_XML_TX).append_child(pugi::node_pcdata).text() = m_M[0][3];

        //<ty>
        node_values.append_child(LABEL_XML_TY).append_child(pugi::node_pcdata).text() = m_M[1][3];

        //<tz>
        node_values.append_child(LABEL_XML_TZ).append_child(pugi::node_pcdata).text() = m_M[2][3];

        node_tmp = node_values.append_child(pugi::node_comment);
        node_tmp.set_value("Rotational vector expressed in angle axis "
                           "representation with values in radians");

        //<tux>
        node_values.append_child(LABEL_XML_TUX).append_child(pugi::node_pcdata).text() = tu[0];

        //<tuy>
        node_values.append_child(LABEL_XML_TUY).append_child(pugi::node_pcdata).text() = tu[1];

        //<tuz>
        node_values.append_child(LABEL_XML_TUZ).append_child(pugi::node_pcdata).text() = tu[2];
      }
    }

    return back;
  }

  /*!
    Translate a string (label) to a xml code.
    \param str : string to translate.
    \param res : resulting code.

    \return error code.
  */
  vpXmlCodeSequenceType str2xmlcode(const char *str, vpXmlCodeType &res)
  {
    vpXmlCodeType val_int = CODE_XML_BAD;
    vpXmlCodeSequenceType back = vpXmlParserHomogeneousMatrix::SEQUENCE_OK;

    if (!strcmp(str, LABEL_XML_M)) {
      val_int = CODE_XML_M;
    }
    else if (!strcmp(str, LABEL_XML_M_NAME)) {
      val_int = CODE_XML_M_NAME;
    }
    else if (!strcmp(str, LABEL_XML_VALUE)) {
      val_int = CODE_XML_VALUE;
    }
    else if (!strcmp(str, LABEL_XML_TX)) {
      val_int = CODE_XML_TX;
    }
    else if (!strcmp(str, LABEL_XML_TY)) {
      val_int = CODE_XML_TY;
    }
    else if (!strcmp(str, LABEL_XML_TZ)) {
      val_int = CODE_XML_TZ;
    }
    else if (!strcmp(str, LABEL_XML_TUX)) {
      val_int = CODE_XML_TUX;
    }
    else if (!strcmp(str, LABEL_XML_TUY)) {
      val_int = CODE_XML_TUY;
    }
    else if (!strcmp(str, LABEL_XML_TUZ)) {
      val_int = CODE_XML_TUZ;
    }
    else {
      val_int = CODE_XML_OTHER;
    }
    res = val_int;

    return back;
  }

  vpHomogeneousMatrix getHomogeneousMatrix() const { return m_M; }
  std::string getHomogeneousMatrixName() const { return m_name; }

  void setHomogeneousMatrixName(const std::string &name) { m_name = name; }

private:
  vpHomogeneousMatrix m_M;
  std::string m_name;
};
#endif // DOXYGEN_SHOULD_SKIP_THIS

vpXmlParserHomogeneousMatrix::vpXmlParserHomogeneousMatrix() : m_impl(new Impl()) { }

vpXmlParserHomogeneousMatrix::~vpXmlParserHomogeneousMatrix() { delete m_impl; }

/*!
  Parse an xml file to load an homogeneous matrix
  \param M : homogeneous matrix to fill.
  \param filename : name of the xml file to parse.
  \param name : name of the homogeneous matrix to find in the xml file.

  \return error code.
*/
int vpXmlParserHomogeneousMatrix::parse(vpHomogeneousMatrix &M, const std::string &filename, const std::string &name)
{
  return m_impl->parse(M, filename, name);
}

/*!
  Save an homogeneous matrix in an xml file.
  \param M : homogeneous matrix to save.
  \param filename : name of the xml file to fill.
  \param name : name of the homogeneous matrix.

  \return error code.
*/
int vpXmlParserHomogeneousMatrix::save(const vpHomogeneousMatrix &M, const std::string &filename,
                                       const std::string &name)
{
  return m_impl->save(M, filename, name);
}

vpHomogeneousMatrix vpXmlParserHomogeneousMatrix::getHomogeneousMatrix() const
{
  return m_impl->getHomogeneousMatrix();
}

std::string vpXmlParserHomogeneousMatrix::getHomogeneousMatrixName() const
{
  return m_impl->getHomogeneousMatrixName();
}

void vpXmlParserHomogeneousMatrix::setHomogeneousMatrixName(const std::string &name)
{
  m_impl->setHomogeneousMatrixName(name);
}
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_core.a(vpXmlParserHomogeneousMatrix.cpp.o) has no symbols
void dummy_vpXmlParserHomogeneousMatrix() { };

#endif
