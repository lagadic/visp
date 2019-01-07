/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
  \file vpXmlParserHomogeneousMatrix.cpp
  \brief Definition of the vpXmlParserHomogeneousMatrix class member
  functions. Class vpXmlParserHomogeneousMatrix allowed to load and save an
  homogeneous matrix in a XML file.

*/
#include <visp3/core/vpXmlParserHomogeneousMatrix.h>
#ifdef VISP_HAVE_XML2

#include <stdlib.h>
#include <string.h>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpThetaUVector.h>
/* --------------------------------------------------------------------------
 */
/* --- LABEL XML ------------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */

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

/*!
  Default constructor
*/
vpXmlParserHomogeneousMatrix::vpXmlParserHomogeneousMatrix() : vpXmlParser(), m_M(), m_name() {}
/*!
  Copy constructor
  \param twinParser : parser object to copy
*/
vpXmlParserHomogeneousMatrix::vpXmlParserHomogeneousMatrix(vpXmlParserHomogeneousMatrix &twinParser)
  : vpXmlParser(twinParser), m_M(), m_name()
{
  *this = twinParser;
}

/*!
  Copy operator
  \param twinParser : parser object to copy
  \return a copy of the input.
*/
vpXmlParserHomogeneousMatrix &vpXmlParserHomogeneousMatrix::operator=(const vpXmlParserHomogeneousMatrix &twinParser)
{
  this->m_M = twinParser.m_M;
  this->m_name = twinParser.m_name;

  return *this;
}

/*!
  Parse an xml file to load an homogeneous matrix
  \param M : homogeneous matrix to fill.
  \param filename : name of the xml file to parse.
  \param name : name of the homogeneous matrix to find in the xml file.

  \return error code.
*/
int vpXmlParserHomogeneousMatrix::parse(vpHomogeneousMatrix &M, const std::string &filename, const std::string &name)
{
  xmlDocPtr doc;
  xmlNodePtr node;

  doc = xmlParseFile(filename.c_str());
  if (doc == NULL) {
    std::cerr << std::endl << "ERROR:" << std::endl;
    std::cerr << " I cannot open the file " << filename << std::endl;

    return SEQUENCE_ERROR;
  }

  node = xmlDocGetRootElement(doc);
  if (node == NULL) {
    xmlFreeDoc(doc);
    return SEQUENCE_ERROR;
  }

  int ret = this->read(doc, node, name);

  M = m_M;

  xmlFreeDoc(doc);

  return ret;
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
  xmlDocPtr doc;
  xmlNodePtr node;

  doc = xmlReadFile(filename.c_str(), NULL, XML_PARSE_NOWARNING + XML_PARSE_NOERROR + XML_PARSE_NOBLANKS);
  if (doc == NULL) {
    doc = xmlNewDoc((xmlChar *)"1.0");
    node = xmlNewNode(NULL, (xmlChar *)LABEL_XML_ROOT);
    xmlDocSetRootElement(doc, node);
    xmlNodePtr node_tmp = xmlNewComment((xmlChar *)"This file stores homogeneous matrix used\n"
                                                   "   in the vpHomogeneousMatrix Class of ViSP available\n"
                                                   "   at https://visp.inria.fr/download/ .\n"
                                                   "   It can be read with the parse method of\n"
                                                   "   the vpXmlParserHomogeneousMatrix class.");
    xmlAddChild(node, node_tmp);
  }

  node = xmlDocGetRootElement(doc);
  if (node == NULL) {
    xmlFreeDoc(doc);
    return SEQUENCE_ERROR;
  }

  this->m_M = M;

  int M_isFound = count(doc, node, name);

  if (M_isFound > 0) {
    // vpCERROR
    std::cout << "There is already an homogeneous matrix " << std::endl
              << "available in the file with the input name: " << name << "." << std::endl
              << "Please delete it manually from the xml file." << std::endl;
    xmlFreeDoc(doc);
    return SEQUENCE_ERROR;
  }

  write(node, name);

  xmlSaveFormatFile(filename.c_str(), doc, 1);
  xmlFreeDoc(doc);
  //  std::cout << "Homogeneous matrix '"<< name << "' saved in the file named
  //  "<< filename << " correctly." << std::endl;

  return SEQUENCE_OK;
}

/*!
  Read Homogeneous matrix values from a XML file.

  \param doc : XML file.
  \param node : XML tree, pointing on a marker equipement.
  \param name : name of the Homogeneous Matrix
  \return error code.
 */
int vpXmlParserHomogeneousMatrix::read(xmlDocPtr doc, xmlNodePtr node, const std::string &name)
{
  //    char * val_char;
  vpXmlCodeType prop;

  vpXmlCodeSequenceType back = SEQUENCE_OK;
  unsigned int nbM = 0;

  for (node = node->xmlChildrenNode; node != NULL; node = node->next) {
    if (node->type != XML_ELEMENT_NODE)
      continue;
    if (SEQUENCE_OK != str2xmlcode((char *)(node->name), prop)) {
      prop = CODE_XML_OTHER;
      back = SEQUENCE_ERROR;
    }

    if (prop == CODE_XML_M) {
      if (SEQUENCE_OK == this->read_matrix(doc, node, name))
        nbM++;
    } else
      back = SEQUENCE_ERROR;
  }

  if (nbM == 0) {
    back = SEQUENCE_ERROR;
    vpCERROR << "No Homogeneous matrix is available" << std::endl << "with name: " << name << std::endl;
  } else if (nbM > 1) {
    back = SEQUENCE_ERROR;
    vpCERROR << nbM << " There are more Homogeneous matrix" << std::endl
             << "with the same name : " << std::endl
             << "precise your choice..." << std::endl;
  }

  return back;
}
/*!
  Read homogeneous matrix names from a XML file and read if there is already a
  homogeneous matrix with the same name.

  \param doc : XML file.
  \param node : XML tree, pointing on a marker equipement.
  \param name : name of the homogeneous matrix.

  \return 1 if there is an homogeneous matrix corresponding with the input
  name, 0 otherwise.
 */
int vpXmlParserHomogeneousMatrix::count(xmlDocPtr doc, xmlNodePtr node, const std::string &name)
{
  //    char * val_char;
  vpXmlCodeType prop;
  int nbM = 0;

  for (node = node->xmlChildrenNode; node != NULL; node = node->next) {
    if (node->type != XML_ELEMENT_NODE)
      continue;
    if (SEQUENCE_OK != str2xmlcode((char *)(node->name), prop)) {
      prop = CODE_XML_OTHER;
    }
    if (prop == CODE_XML_M) {
      if (SEQUENCE_OK == this->read_matrix(doc, node, name))
        nbM++;
    }
  }

  return nbM;
}

/*!
  Read Homogeneous Matrix fields from a XML file.

  \param doc : XML file.
  \param node : XML tree, pointing on a marker equipement.
  \param name : name of the Homogeneous matrix

  \return error code.

 */
int vpXmlParserHomogeneousMatrix::read_matrix(xmlDocPtr doc, xmlNodePtr node, const std::string &name)
{
  vpXmlCodeType prop;
  /* read value in the XML file. */
  std::string M_name_tmp = "";
  vpHomogeneousMatrix M_tmp;

  vpXmlCodeSequenceType back = SEQUENCE_OK;

  for (node = node->xmlChildrenNode; node != NULL; node = node->next) {
    // vpDEBUG_TRACE (15, "Carac : %s.", node ->name);
    if (node->type != XML_ELEMENT_NODE)
      continue;
    if (SEQUENCE_OK != str2xmlcode((char *)(node->name), prop)) {
      prop = CODE_XML_OTHER;
      back = SEQUENCE_ERROR;
    }

    switch (prop) {
    case CODE_XML_M_NAME: {
      char *val_char = xmlReadCharChild(doc, node);
      M_name_tmp = val_char;
      xmlFree(val_char);
      break;
    }

    case CODE_XML_VALUE: // VALUE
      if (name == M_name_tmp) {
        std::cout << "Found Homogeneous Matrix with name: \"" << M_name_tmp << "\"" << std::endl;
        back = read_values(doc, node, M_tmp);
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

  if (!(name == M_name_tmp)) {
    back = SEQUENCE_ERROR;
  } else {
    this->m_M = M_tmp;
    // std::cout << "Convert in Homogeneous Matrix:"<< std::endl;
    // std::cout << this-> M << std::endl;
    this->m_name = M_name_tmp;
  }
  return back;
}

/*!
  Read homogeneous matrix fields from a XML file.

  \param doc : XML file.
  \param node : XML tree, pointing on a marker equipement.
  \param M_tmp : homogeneous matrix to fill with read data (output).

  \return error code.

 */
vpXmlParserHomogeneousMatrix::vpXmlCodeSequenceType
vpXmlParserHomogeneousMatrix::read_values(xmlDocPtr doc, xmlNodePtr node, vpHomogeneousMatrix &M)
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
  // int validation = 0;

  for (node = node->xmlChildrenNode; node != NULL; node = node->next) {
    // vpDEBUG_TRACE (15, "Carac : %s.", node ->name);
    if (node->type != XML_ELEMENT_NODE)
      continue;
    if (SEQUENCE_OK != str2xmlcode((char *)(node->name), prop)) {
      prop = CODE_XML_OTHER;
      back = SEQUENCE_ERROR;
    }

    switch (prop) {

    case CODE_XML_TX:
      tx_ = xmlReadDoubleChild(doc, node);
      nb++;
      break;
    case CODE_XML_TY:
      ty_ = xmlReadDoubleChild(doc, node);
      nb++;
      break;
    case CODE_XML_TZ:
      tz_ = xmlReadDoubleChild(doc, node);
      nb++;
      break;
    case CODE_XML_TUX:
      tux_ = xmlReadDoubleChild(doc, node);
      nb++;
      break;
    case CODE_XML_TUY:
      tuy_ = xmlReadDoubleChild(doc, node);
      nb++;
      break;
    case CODE_XML_TUZ:
      tuz_ = xmlReadDoubleChild(doc, node);
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

  if (nb != 6) {
    vpCERROR << "ERROR in 'model' field:\n";
    vpCERROR << "it must contain 6 parameters\n";

    return SEQUENCE_ERROR;
  }

  // Create the Homogeneous matrix
  M.buildFrom(tx_, ty_, tz_, tux_, tuy_, tuz_);

  //  std::cout << "Read values from file:" << std::endl;
  //  std::cout << "tx:" << tx_<< std::endl;
  //  std::cout << "ty:" << ty_<< std::endl;
  //  std::cout << "tz:" << tz_<< std::endl;
  //  std::cout << "tux:" << tux_<< std::endl;
  //  std::cout << "tuy:" << tuy_<< std::endl;
  //  std::cout << "tuz:" << tuz_<< std::endl;

  return back;
}

/*!
  Write Homogeneous Matrix in an XML Tree.

  \param node : XML tree, pointing on a marker equipement.
  \param name : name of the Homogeneous Matrix.


  \return error code.
 */
int vpXmlParserHomogeneousMatrix::write(xmlNodePtr node, const std::string &name)
{
  int back = SEQUENCE_OK;

  xmlNodePtr node_tmp;
  xmlNodePtr node_matrix;
  xmlNodePtr node_values;

  // Convert from Rotational matrix to Theta-U vector
  vpRotationMatrix R;
  m_M.extract(R);

  vpThetaUVector tu(R);

  // <homogeneous_transformation>
  node_tmp = xmlNewComment((xmlChar *)"Homogeneous Matrix");
  xmlAddChild(node, node_tmp);
  node_matrix = xmlNewNode(NULL, (xmlChar *)LABEL_XML_M);
  xmlAddChild(node, node_matrix);
  {
    //<name>

    if (!name.empty()) {
      node_tmp = xmlNewComment((xmlChar *)"Name of the homogeneous matrix");
      xmlAddChild(node_matrix, node_tmp);
      xmlNewTextChild(node_matrix, NULL, (xmlChar *)LABEL_XML_M_NAME, (xmlChar *)name.c_str());
    }

    //<values>

    node_values = xmlNewNode(NULL, (xmlChar *)LABEL_XML_VALUE);
    xmlAddChild(node_matrix, node_values);
    {
      char str[11];

      node_tmp = xmlNewComment((xmlChar *)"Translation vector with values in meters");
      xmlAddChild(node_values, node_tmp);

      //<tx>
      sprintf(str, "%f", m_M[0][3]);
      xmlNewTextChild(node_values, NULL, (xmlChar *)LABEL_XML_TX, (xmlChar *)str);

      //<ty>
      sprintf(str, "%f", m_M[1][3]);
      xmlNewTextChild(node_values, NULL, (xmlChar *)LABEL_XML_TY, (xmlChar *)str);

      //<tz>
      sprintf(str, "%f", m_M[2][3]);
      xmlNewTextChild(node_values, NULL, (xmlChar *)LABEL_XML_TZ, (xmlChar *)str);

      node_tmp = xmlNewComment((xmlChar *)"Rotational vector expressed in angle axis "
                                          "representation with values in radians");
      xmlAddChild(node_values, node_tmp);

      //<tux>
      sprintf(str, "%f", tu[0]);
      xmlNewTextChild(node_values, NULL, (xmlChar *)LABEL_XML_TUX, (xmlChar *)str);

      //<tuy>
      sprintf(str, "%f", tu[1]);
      xmlNewTextChild(node_values, NULL, (xmlChar *)LABEL_XML_TUY, (xmlChar *)str);

      //<tuz>
      sprintf(str, "%f", tu[2]);
      xmlNewTextChild(node_values, NULL, (xmlChar *)LABEL_XML_TUZ, (xmlChar *)str);
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

vpXmlParserHomogeneousMatrix::vpXmlCodeSequenceType vpXmlParserHomogeneousMatrix::str2xmlcode(char *str,
                                                                                              vpXmlCodeType &res)
{
  vpXmlCodeType val_int = CODE_XML_BAD;
  vpXmlCodeSequenceType back = vpXmlParserHomogeneousMatrix::SEQUENCE_OK;

  // DEBUG_TRACE (9, "# Entree :str=%s.", str);

  if (!strcmp(str, LABEL_XML_M)) {
    val_int = CODE_XML_M;
  } else if (!strcmp(str, LABEL_XML_M_NAME)) {
    val_int = CODE_XML_M_NAME;
  } else if (!strcmp(str, LABEL_XML_VALUE)) {
    val_int = CODE_XML_VALUE;
  } else if (!strcmp(str, LABEL_XML_TX)) {
    val_int = CODE_XML_TX;
  } else if (!strcmp(str, LABEL_XML_TY)) {
    val_int = CODE_XML_TY;
  } else if (!strcmp(str, LABEL_XML_TZ)) {
    val_int = CODE_XML_TZ;
  } else if (!strcmp(str, LABEL_XML_TUX)) {
    val_int = CODE_XML_TUX;
  } else if (!strcmp(str, LABEL_XML_TUY)) {
    val_int = CODE_XML_TUY;
  } else if (!strcmp(str, LABEL_XML_TUZ)) {
    val_int = CODE_XML_TUZ;
  } else {
    val_int = CODE_XML_OTHER;
  }
  res = val_int;

  return back;
}
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning:
// libvisp_core.a(vpXmlParserHomogeneousMatrix.cpp.o) has no symbols
void dummy_vpXmlParserHomogeneousMatrix(){};
#endif // VISP_HAVE_XML2
