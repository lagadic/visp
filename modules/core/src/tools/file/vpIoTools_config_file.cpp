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
 * Io tools dedicated to npy.
 */

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpEndian.h>
#include <visp3/core/vpIoTools.h>

BEGIN_VISP_NAMESPACE

std::string vpIoTools::baseName = "";
std::string vpIoTools::baseDir = "";
std::string vpIoTools::configFile = "";
std::vector<std::string> vpIoTools::configVars = std::vector<std::string>();
std::vector<std::string> vpIoTools::configValues = std::vector<std::string>();

const char vpIoTools::separator =
#if defined(_WIN32)
'\\';
#else
'/';
#endif

namespace
{
std::string &ltrim(std::string &s)
{
#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int c) { return !std::isspace(c); }));
#else
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
#endif
  return s;
}

std::string &rtrim(std::string &s)
{
#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
  s.erase(std::find_if(s.rbegin(), s.rend(), [](int c) { return !std::isspace(c); }).base(), s.end());
#else
  s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
#endif
  return s;
}
} // namespace
/*!
  Sets the base name (prefix) of the experiment files.

  \param s : Prefix of the experiment files.
*/
void vpIoTools::setBaseName(const std::string &s) { baseName = s; }

/*!
  Sets the base directory of the experiment files.

  \param dir : Directory where the data will be saved.
*/
void vpIoTools::setBaseDir(const std::string &dir) { baseDir = dir + "/"; }

/*!
  Gets the base name (prefix) of the experiment files.

  \return the base name of the experiment files.
*/
std::string vpIoTools::getBaseName() { return baseName; }

/*!
  Gets the full path of the experiment files : baseDir/baseName

  \return the full path of the experiment files.
*/
std::string vpIoTools::getFullName() { return baseDir + baseName; }

/*!
  Reads the configuration file and parses it.

  \param confFile : path to the file containing the configuration parameters to
  parse.

  \return true if succeed, false otherwise.
 */
bool vpIoTools::loadConfigFile(const std::string &confFile)
{
  configFile = path(confFile);
  configVars.clear();
  configValues.clear();
  std::ifstream confContent(configFile.c_str(), std::ios::in);

  if (confContent.is_open()) {
    std::string line, var, val;
    unsigned long int k;
    int c;
    std::string stop[3] = { " ", "\t", "#" };
    while (std::getline(confContent, line)) {
      if ((line.compare(0, 1, "#") != 0) && (line.size() > 2)) {
        // name of the variable
        k = static_cast<unsigned long>(line.find(" "));
        var = line.substr(0, k);
        // look for the end of the actual value
        c = 200;
        for (unsigned i = 0; i < 3; ++i) {
          c = vpMath::minimum(c,
            static_cast<int>(line.find(stop[i], static_cast<size_t>(k) + static_cast<size_t>(1))));
        }
        if (c == -1) {
          c = static_cast<int>(line.size());
        }
        unsigned long int c_ = static_cast<unsigned long int>(c);
        val = line.substr(static_cast<size_t>(k) + static_cast<size_t>(1),
          static_cast<size_t>(c_) - static_cast<size_t>(k) - static_cast<size_t>(1));
        configVars.push_back(var);
        configValues.push_back(val);
      }
    }
    confContent.close();
  }
  else {
    return false;
  }
  return true;
}

/*!
  Tries to read the parameter named \e var as a \e float.

  \param var : Name of the parameter in the configuration file.
  \param value : Value to be read.

  \return true if the parameter could be read.
*/
bool vpIoTools::readConfigVar(const std::string &var, float &value)
{
  bool found = false;
  size_t configvars_size = configVars.size();
  size_t k = 0;
  while ((k < configvars_size) && (!found)) {
    if (configVars[k] == var) {
      if (configValues[k].compare("PI") == 0) {
        value = static_cast<float>(M_PI);
      }
      else if (configValues[k].compare("PI/2") == 0) {
        value = static_cast<float>(M_PI / 2.0);
      }
      else if (configValues[k].compare("-PI/2") == 0) {
        value = static_cast<float>(-M_PI / 2.0);
      }
      else {
        value = static_cast<float>(atof(configValues[k].c_str()));
      }
      found = true;
    }
    ++k;
  }
  if (!found) {
    std::cout << var << " not found in config file" << std::endl;
  }
  return found;
}

/*!
  Tries to read the parameter named \e var as a \e double.

  \param var : Name of the parameter in the configuration file.
  \param value : Value to be read.

  \return true if the parameter could be read.
*/
bool vpIoTools::readConfigVar(const std::string &var, double &value)
{
  bool found = false;
  size_t configvars_size = configVars.size();
  size_t k = 0;
  while ((k < configvars_size) && (!found)) {
    if (configVars[k] == var) {
      if (configValues[k].compare("PI") == 0) {
        value = M_PI;
      }
      else if (configValues[k].compare("PI/2") == 0) {
        value = M_PI / 2;
      }
      else if (configValues[k].compare("-PI/2") == 0) {
        value = -M_PI / 2;
      }
      else {
        value = atof(configValues[k].c_str());
      }
      found = true;
    }
    ++k;
  }
  if (!found) {
    std::cout << var << " not found in config file" << std::endl;
  }
  return found;
}

/*!
  Tries to read the parameter named \e var as a \e int.

  \param var : Name of the parameter in the configuration file.
  \param value : Value to be read.

  \return true if the parameter could be read.
*/
bool vpIoTools::readConfigVar(const std::string &var, int &value)
{
  bool found = false;
  size_t configvars_size = configVars.size();
  size_t k = 0;
  while ((k < configvars_size) && (!found)) {
    if (configVars[k] == var) {
      value = atoi(configValues[k].c_str());
      found = true;
    }
    ++k;
  }
  if (!found) {
    std::cout << var << " not found in config file" << std::endl;
  }
  return found;
}

/*!
  Tries to read the parameter named \e var as a \e unsigned int.

  \param var : Name of the parameter in the configuration file.
  \param value : Value to be read.

  \return true if the parameter could be read.
*/
bool vpIoTools::readConfigVar(const std::string &var, unsigned int &value)
{
  int v = 0;
  bool found = readConfigVar(var, v);
  value = static_cast<unsigned int>(v);
  return found;
}

/*!
  Tries to read the parameter named \e var as a \e bool.

  \param var : Name of the parameter in the configuration file.
  \param value : Value to be read.

  \return true if the parameter could be read.
*/
bool vpIoTools::readConfigVar(const std::string &var, bool &value)
{
  int v = 0;
  bool found = readConfigVar(var, v);
  value = (v != 0);
  return found;
}

/*!
  Tries to read the parameter named \e var as a \e vpColor.

  \param var : Name of the parameter in the configuration file.
  \param value : Value to be read. See vpColor.cpp for the color number.

  \return true if the parameter could be read.
*/
bool vpIoTools::readConfigVar(const std::string &var, vpColor &value)
{
  unsigned int v = 0;
  bool found = readConfigVar(var, v);
  value = vpColor::getColor(v);
  return found;
}

/*!
  Tries to read the parameter named \e var as a \e std::string.

  \param var : Name of the parameter in the configuration file.
  \param value : Value to be read.

  \return true if the parameter could be read.
*/
bool vpIoTools::readConfigVar(const std::string &var, std::string &value)
{
  bool found = false;
  size_t configvars_size = configVars.size();
  size_t k = 0;
  while ((k < configvars_size) && (!found)) {
    if (configVars[k] == var) {
      value = configValues[k];
      found = true;
    }
    ++k;
  }
  if (!found) {
    std::cout << var << " not found in config file" << std::endl;
  }
  return found;
}

/*!
  Tries to read the parameter named \e var as a \e vpMatrix.
  If \e nCols and \e nRows are indicated, will resize the matrix.
  Otherwise, will try to read as many values as indicated by the dimension of
  \e value.

  \param var : Name of the parameter in the configuration file.
  \param value : Value to be read.
  \param nCols : Column dimension if resized.
  \param nRows : Row dimension if resized

  \return true if the parameter could be read.
*/
bool vpIoTools::readConfigVar(const std::string &var, vpArray2D<double> &value, const unsigned int &nCols,
                              const unsigned int &nRows)
{
  bool found = false;
  std::string nb;
  size_t configvars_size = configVars.size();
  size_t k = 0;
  while ((k < configvars_size) && (!found)) {
    if (configVars[k] == var) {
      found = true;
      // resize or not
      if ((nCols != 0) && (nRows != 0)) {
        value.resize(nRows, nCols);
      }
      size_t ind = 0, ind2;
      unsigned int value_rows = value.getRows();
      unsigned int value_cols = value.getCols();
      for (unsigned int i = 0; i < value_rows; ++i) {
        for (unsigned int j = 0; j < value_cols; ++j) {
          ind2 = configValues[k].find(",", ind);
          nb = configValues[k].substr(ind, ind2 - ind);
          if (nb.compare("PI") == 0) {
            value[i][j] = M_PI;
          }
          else if (nb.compare("PI/2") == 0) {
            value[i][j] = M_PI / 2;
          }
          else if (nb.compare("-PI/2") == 0) {
            value[i][j] = -M_PI / 2;
          }
          else {
            value[i][j] = atof(nb.c_str());
          }
          ind = ind2 + 1;
        }
      }
    }
    ++k;
  }
  if (found == false) {
    std::cout << var << " not found in config file" << std::endl;
  }
  return found;
}

// construct experiment filename & path

/*!
  Augments the prefix of the experiment files by \e strTrue if \e cond is
  verified, and by \e strFalse otherwise.

  \param strTrue : String to add if \e cond is true
  \param cond : Condition managing the file name
  \param strFalse : String to add if \e cond is false (default "")
*/
void vpIoTools::addNameElement(const std::string &strTrue, const bool &cond, const std::string &strFalse)
{
  if (cond) {
    baseName += "_" + strTrue;
  }
  else if (strFalse != "") {
    baseName += "_" + strFalse;
  }
}

/*!
  Augments the prefix of the experiment files by \e strTrue followed by \e
  val.

  \param strTrue : String to add
  \param val : Value to add

*/
void vpIoTools::addNameElement(const std::string &strTrue, const double &val)
{
  // if(val != 0.)
  if (std::fabs(val) < std::numeric_limits<double>::epsilon()) {
    std::stringstream valS;
    valS.precision(4);
    valS << val;
    baseName += "_" + strTrue + valS.str();
  }
}

/*!
  Creates the directory \e baseDir/baseName. If already exists, empties
  it if \e empty is true.
  Useful to save the images corresponding to a particular experiment.

  \param empty : Indicates if the new directory has to be emptied

*/
void vpIoTools::createBaseNamePath(const bool &empty)
{
  std::string path = baseDir + baseName;
  if (vpIoTools::checkDirectory(path) == false) {
    try {
      vpIoTools::makeDirectory(path);
      std::cout << "Creating directory " << path << std::endl;
    }
    catch (...) {
      throw(vpException(vpException::fatalError, "Cannot create base name directory %s", path.c_str()));
    }
  }
  else {
    if (empty) {
      std::cout << "Emptying directory " << path << std::endl;
      vpIoTools::remove(path + "/*");
    }
  }
}

/*!
  Copy the initial configuration file to the experiment directory.

  \param actuallySave : If false, do not copy the file.

*/
void vpIoTools::saveConfigFile(const bool &actuallySave)
{
  if (actuallySave) {
    std::string dest = baseDir + "/" + baseName + "_config.txt";
    // file copy
    vpIoTools::copy(configFile, dest);
  }
}

/*!
   Read a 16-bits integer value stored in little endian.
 */
void vpIoTools::readBinaryValueLE(std::ifstream &file, int16_t &short_value)
{
  file.read((char *)(&short_value), sizeof(short_value));

#ifdef VISP_BIG_ENDIAN
  // Swap bytes order from little endian to big endian
  short_value = vpEndian::swap16bits((uint16_t)short_value);
#endif
}

/*!
   Read a 16-bits unsigned integer value stored in little endian.
 */
void vpIoTools::readBinaryValueLE(std::ifstream &file, uint16_t &ushort_value)
{
  file.read((char *)(&ushort_value), sizeof(ushort_value));

#ifdef VISP_BIG_ENDIAN
  // Swap bytes order from little endian to big endian
  ushort_value = vpEndian::swap16bits(ushort_value);
#endif
}

/*!
   Read a 32-bits integer value stored in little endian.
 */
void vpIoTools::readBinaryValueLE(std::ifstream &file, int32_t &int_value)
{
  file.read((char *)(&int_value), sizeof(int_value));

#ifdef VISP_BIG_ENDIAN
  // Swap bytes order from little endian to big endian
  int_value = vpEndian::swap32bits((uint32_t)int_value);
#endif
}

/*!
   Read a 32-bits unsigned integer value stored in little endian.
 */
void vpIoTools::readBinaryValueLE(std::ifstream &file, uint32_t &uint_value)
{
  file.read((char *)(&uint_value), sizeof(uint_value));

#ifdef VISP_BIG_ENDIAN
  // Swap bytes order from little endian to big endian
  uint_value = vpEndian::swap32bits(uint_value);
#endif
}

/*!
   Read a float value stored in little endian.
 */
void vpIoTools::readBinaryValueLE(std::ifstream &file, float &float_value)
{
  file.read((char *)(&float_value), sizeof(float_value));

#ifdef VISP_BIG_ENDIAN
  // Swap bytes order from little endian to big endian
  float_value = vpEndian::swapFloat(float_value);
#endif
}

/*!
   Read a double value stored in little endian.
 */
void vpIoTools::readBinaryValueLE(std::ifstream &file, double &double_value)
{
  file.read((char *)(&double_value), sizeof(double_value));

#ifdef VISP_BIG_ENDIAN
  // Swap bytes order from little endian to big endian
  double_value = vpEndian::swapDouble(double_value);
#endif
}

/*!
   Write a 16-bits integer value in little endian.
 */
void vpIoTools::writeBinaryValueLE(std::ofstream &file, const int16_t short_value)
{
#ifdef VISP_BIG_ENDIAN
  // Swap bytes order to little endian
  uint16_t swap_short = vpEndian::swap16bits((uint16_t)short_value);
  file.write((char *)(&swap_short), sizeof(swap_short));
#else
  file.write((char *)(&short_value), sizeof(short_value));
#endif
}

/*!
   Write a 16-bits unsigned integer value in little endian.
 */
void vpIoTools::writeBinaryValueLE(std::ofstream &file, const uint16_t ushort_value)
{
#ifdef VISP_BIG_ENDIAN
  // Swap bytes order to little endian
  uint16_t swap_ushort = vpEndian::swap16bits(ushort_value);
  file.write((char *)(&swap_ushort), sizeof(swap_ushort));
#else
  file.write((char *)(&ushort_value), sizeof(ushort_value));
#endif
}

/*!
   Write a 32-bits integer value in little endian.
 */
void vpIoTools::writeBinaryValueLE(std::ofstream &file, const int32_t int_value)
{
#ifdef VISP_BIG_ENDIAN
  // Swap bytes order to little endian
  uint32_t swap_int = vpEndian::swap32bits((uint32_t)int_value);
  file.write((char *)(&swap_int), sizeof(swap_int));
#else
  file.write((char *)(&int_value), sizeof(int_value));
#endif
}

/*!
   Write a 32-bits unsigned integer value in little endian.
 */
void vpIoTools::writeBinaryValueLE(std::ofstream &file, const uint32_t uint_value)
{
#ifdef VISP_BIG_ENDIAN
  // Swap bytes order to little endian
  uint32_t swap_int = vpEndian::swap32bits(uint_value);
  file.write((char *)(&swap_int), sizeof(swap_int));
#else
  file.write((char *)(&uint_value), sizeof(uint_value));
#endif
}

/*!
   Write a float value in little endian.
 */
void vpIoTools::writeBinaryValueLE(std::ofstream &file, float float_value)
{
#ifdef VISP_BIG_ENDIAN
  // Swap bytes order to little endian
  float swap_float = vpEndian::swapFloat(float_value);
  file.write((char *)(&swap_float), sizeof(swap_float));
#else
  file.write((char *)(&float_value), sizeof(float_value));
#endif
}

/*!
   Write a double value in little endian.
 */
void vpIoTools::writeBinaryValueLE(std::ofstream &file, double double_value)
{
#ifdef VISP_BIG_ENDIAN
  // Swap bytes order to little endian
  double swap_double = vpEndian::swapDouble(double_value);
  file.write((char *)(&swap_double), sizeof(swap_double));
#else
  file.write((char *)(&double_value), sizeof(double_value));
#endif
}

bool vpIoTools::parseBoolean(std::string input)
{
  std::transform(input.begin(), input.end(), input.begin(), ::tolower);
  std::istringstream is(input);
  bool b;
  // Parse string to boolean either in the textual representation
  // (True/False)  or in numeric representation (1/0)
  is >> (input.size() > 1 ? std::boolalpha : std::noboolalpha) >> b;
  return b;
}

/*!
   Remove leading and trailing whitespaces from a string.
 */
std::string vpIoTools::trim(std::string s) { return ltrim(rtrim(s)); }

END_VISP_NAMESPACE
