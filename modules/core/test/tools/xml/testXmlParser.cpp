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
 * Example which describes how to use the xml parser class.
 *
 * Author:
 * Romain Tallonneau
 *
 *****************************************************************************/

/*!
  \example testXmlParser.cpp

  XML parser example.

  This example contains the declaration of a class used to read and write data
  in a xml file like:
  \code
  <config>
      <range>5.5</range>
      <step>7</step>
      <size_filter>3</size_filter>
      <name>Object</name>
  </config>
  \endcode

*/

#include <visp3/core/vpConfig.h>

#include <iostream>
#if defined(VISP_HAVE_XML2)

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParser.h>
#include <visp3/io/vpParseArgv.h>

#include <string>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/* --------------------------------------------------------------------------
 */
/*                               CLASS EXAMPLE */
/* --------------------------------------------------------------------------
 */

/*!
  \class vpExampleDataParser
  \brief Class example used to show how to implement a xml parser based on the
  vpXmlParser
*/
class vpExampleDataParser : public vpXmlParser
{
protected:
  double m_range;
  int m_step;
  int m_size_filter;
  std::string m_name;

  typedef enum { config, range, step, size_filter, name } dataToParse;

public:
  vpExampleDataParser();
  virtual ~vpExampleDataParser();

  // Data accessors.
  double getRange() const { return m_range; }
  int getStep() const { return m_step; }
  int getSizeFilter() const { return m_size_filter; }
  std::string getName() const { return m_name; }

  void setRange(const double _range) { m_range = _range; }
  void setStep(const int _step) { m_step = _step; }
  void setSizeFilter(const int _size_filter) { m_size_filter = _size_filter; }
  void setName(const std::string &_name) { m_name = _name; }

protected:
  virtual void readMainClass(xmlDocPtr doc, xmlNodePtr node);
  virtual void writeMainClass(xmlNodePtr node);
};

/*!
  Constructor.
  Initialise the map according to the data to parse, and initialise data to
  default values.

*/
vpExampleDataParser::vpExampleDataParser() : m_range(0.), m_step(0), m_size_filter(0), m_name("")
{
  nodeMap["config"] = config;
  nodeMap["range"] = range;
  nodeMap["step"] = step;
  nodeMap["size_filter"] = size_filter;
  nodeMap["name"] = name;
}

/*!
  Destructor.

*/
vpExampleDataParser::~vpExampleDataParser() {}

/*!
  Read the main class. This method corresponds to the parsing of the main
  document (which contains the whole data in the class). At this point, the
  document exists and is open.

  \param doc : Pointer to the document to parse.
  \param node : Pointer to the root node of the document.
*/
void vpExampleDataParser::readMainClass(xmlDocPtr doc, xmlNodePtr node)
{
  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case range:
          this->m_range = xmlReadDoubleChild(doc, dataNode);
          break;
        case step:
          this->m_step = xmlReadIntChild(doc, dataNode);
          break;
        case size_filter:
          this->m_size_filter = xmlReadIntChild(doc, dataNode);
          break;
        case name: {
          this->m_name = xmlReadStringChild(doc, dataNode);
        } break;
        default:
          vpTRACE("unknown tag in readConfigNode : %d, %s", iter_data->second, (iter_data->first).c_str());
          break;
        }
      }
    }
  }
}

/*!
  Write the data in the file.
  The file has already been opened or created in the save() method. And the
  root node (corresponding to the main tag) has already been writen.

  \param node : Pointer to the root node.
*/
void vpExampleDataParser::writeMainClass(xmlNodePtr node)
{
  xmlWriteDoubleChild(node, (const char *)"range", m_range);
  xmlWriteIntChild(node, (const char *)"step", m_step);
  xmlWriteIntChild(node, (const char *)"size_filter", m_size_filter);
  xmlWriteCharChild(node, (const char *)"name", m_name.c_str());
}

#endif // doxygen

/* --------------------------------------------------------------------------
 */
/*                         COMMAND LINE OPTIONS */
/* --------------------------------------------------------------------------
 */

// List of allowed command line options
#define GETOPTARGS "cdo:h"

void usage(const char *name, const char *badparam, const std::string &opath, const std::string &user);
bool getOptions(int argc, const char **argv, std::string &opath, const std::string &user);

/*!

Print the program options.

\param name : Program name.
\param badparam : Bad parameter name.
\param opath : Output image path.
\param user : Username.

 */
void usage(const char *name, const char *badparam, const std::string &opath, const std::string &user)
{
  fprintf(stdout, "\n\
Write and read data in a xml file.\n\
          \n\
SYNOPSIS\n\
  %s [-o <output image path>] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -o <output data path>                               %s\n\
     Set data output path.\n\
     From this directory, creates the \"%s\"\n\
     subdirectory depending on the username, where \n\
     dataTestXml.xml file is written.\n\
                  \n\
  -h\n\
     Print the help.\n\n", opath.c_str(), user.c_str());

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param opath : Output data path.
  \param user : Username.
  \return false if the program has to be stopped, true otherwise.
*/
bool getOptions(int argc, const char **argv, std::string &opath, const std::string &user)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'o':
      opath = optarg_;
      break;
    case 'h':
      usage(argv[0], NULL, opath, user);
      return false;
      break;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, opath, user);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, opath, user);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

/* --------------------------------------------------------------------------
 */
/*                               MAIN FUNCTION */
/* --------------------------------------------------------------------------
 */

int main(int argc, const char **argv)
{
  try {
    std::string opt_opath;
    std::string opath;
    std::string filename;
    std::string username;

    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "  testXmlParser.cpp" << std::endl << std::endl;
    std::cout << "  writing and readind data using a xml parser" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

// Set the default output path
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    opt_opath = "/tmp";
#elif defined(_WIN32)
    opt_opath = "C:\\temp";
#endif

    // Get the user login name
    vpIoTools::getUserName(username);

    // Read the command line options
    if (getOptions(argc, argv, opt_opath, username) == false) {
      exit(-1);
    }

    // Get the option values
    if (!opt_opath.empty())
      opath = opt_opath;

    // Append to the output path string, the login name of the user
    std::string dirname = vpIoTools::createFilePath(opath, username);

    // Test if the output path exist. If no try to create it
    if (vpIoTools::checkDirectory(dirname) == false) {
      try {
        // Create the dirname
        vpIoTools::makeDirectory(dirname);
      } catch (...) {
        usage(argv[0], NULL, opath, username);
        std::cerr << std::endl << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << dirname << std::endl;
        std::cerr << "  Check your -o " << opath << " option " << std::endl;
        exit(-1);
      }
    }

    filename = dirname + vpIoTools::path("/") + "dataTestXml.xml";

    // Write data using a parser.
    {
      vpExampleDataParser parser1;

      // Acquire data from measurments or tests.
      parser1.setRange(3.5);
      parser1.setStep(2);
      parser1.setSizeFilter(5);
      parser1.setName("cube");

      std::cout << "Write data to " << filename << std::endl;
      parser1.save(filename);
    }

    // Read data using another parser.
    {
      vpExampleDataParser parser2;

      parser2.parse(filename);

      std::cout << "Read from " << filename << std::endl;
      std::cout << "Range : " << parser2.getRange() << std::endl;
      std::cout << "Step : " << parser2.getStep() << std::endl;
      std::cout << "Filter size : " << parser2.getSizeFilter() << std::endl;
      std::cout << "name : " << parser2.getName() << std::endl;
    }

    // Clean up memory allocated by the xml library
    vpXmlParser::cleanup();
    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}

#else

int main()
{
  std::cout << "Xml parser requires libxml2." << std::endl;
  return 0;
}
#endif
