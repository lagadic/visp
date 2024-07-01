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
 * Directory management.
 */

/*!
 * \file vpIoTools.h
 * \brief File and directories basic tools.
 */

#ifndef VP_IO_TOOLS_H
#define VP_IO_TOOLS_H

#include <visp3/core/vpConfig.h>

#include <iostream>
#include <sstream>
#include <stdint.h> //for uint32_t related types ; works also with >= VS2010 / _MSC_VER >= 1600
#include <stdlib.h>
#include <string>
#include <vector>
#include <numeric>
#include <visp3/core/vpColor.h>

#include <memory>
#include <map>
#include <cassert>
#include <complex>

#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98

namespace visp
{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
// https://github.com/BinomialLLC/basis_universal/blob/ad9386a4a1cf2a248f7bbd45f543a7448db15267/encoder/basisu_miniz.h#L665
static inline unsigned long vp_mz_crc32(unsigned long crc, const unsigned char *ptr, size_t buf_len)
{
  static const unsigned int s_crc32[16] = { 0, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c };
  unsigned int crcu32 = static_cast<unsigned int>(crc);
  if (!ptr) return 0;
  crcu32 = ~crcu32;
  while (buf_len--) {
    unsigned char b = *ptr++;
    crcu32 = (crcu32 >> 4) ^ s_crc32[(crcu32 & 0xF) ^ (b & 0xF)];
    crcu32 = (crcu32 >> 4) ^ s_crc32[(crcu32 & 0xF) ^ (b >> 4)];
  }
  return ~crcu32;
}
#endif // DOXYGEN_SHOULD_SKIP_THIS

#ifdef VISP_HAVE_MINIZ
namespace cnpy
{
// Copyright (C) 2011  Carl Rogers
// Released under MIT License
// license available in LICENSE file, or at http://www.opensource.org/licenses/mit-license.php
struct NpyArray
{
  NpyArray(const std::vector<size_t> &_shape, size_t _word_size, bool _fortran_order) :
    shape(_shape), word_size(_word_size), fortran_order(_fortran_order)
  {
    num_vals = 1;
    for (size_t i = 0; i < shape.size(); ++i) num_vals *= shape[i];
    data_holder = std::shared_ptr<std::vector<char> >(
        new std::vector<char>(num_vals * word_size));
  }

  NpyArray() : shape(0), word_size(0), fortran_order(0), num_vals(0) { }

  template<typename T>
  T *data()
  {
    return reinterpret_cast<T *>(&(*data_holder)[0]);
  }

  template<typename T>
  const T *data() const
  {
    return reinterpret_cast<T *>(&(*data_holder)[0]);
  }

  template<typename T>
  std::vector<T> as_vec() const
  {
    const T *p = data<T>();
    return std::vector<T>(p, p+num_vals);
  }

  size_t num_bytes() const
  {
    return data_holder->size();
  }

  std::shared_ptr<std::vector<char> > data_holder;
  std::vector<size_t> shape;
  size_t word_size;
  bool fortran_order;
  size_t num_vals;
};

using npz_t = std::map<std::string, NpyArray>;
VISP_EXPORT npz_t npz_load(std::string fname);
VISP_EXPORT char BigEndianTest();
VISP_EXPORT char map_type(const std::type_info &t);
template<typename T> std::vector<char> create_npy_header(const std::vector<size_t> &shape);
VISP_EXPORT void parse_npy_header(FILE *fp, size_t &word_size, std::vector<size_t> &shape, bool &fortran_order);
VISP_EXPORT void parse_npy_header(unsigned char *buffer, size_t &word_size, std::vector<size_t> &shape, bool &fortran_order);
VISP_EXPORT void parse_zip_footer(FILE *fp, uint16_t &nrecs, size_t &global_header_size, size_t &global_header_offset);
VISP_EXPORT NpyArray npz_load(std::string fname, std::string varname);
VISP_EXPORT NpyArray npy_load(std::string fname);

template<typename T> std::vector<char> &operator+=(std::vector<char> &lhs, const T rhs)
{
  //write in little endian
  for (size_t byte = 0; byte < sizeof(T); ++byte) {
    char val = *((char *)&rhs+byte);
    lhs.push_back(val);
  }
  return lhs;
}

template<> inline std::vector<char> &operator+=(std::vector<char> &lhs, const std::string rhs)
{
  lhs.insert(lhs.end(), rhs.begin(), rhs.end());
  return lhs;
}

template<> inline std::vector<char> &operator+=(std::vector<char> &lhs, const char *rhs)
{
//write in little endian
  size_t len = strlen(rhs);
  lhs.reserve(len);
  for (size_t byte = 0; byte < len; ++byte) {
    lhs.push_back(rhs[byte]);
  }
  return lhs;
}

/*!
  Save an array of data (\p data) into the \p fname npy file. This function is similar to the
  <a href="https://numpy.org/doc/stable/reference/generated/numpy.save.html">numpy.save</a> function.
  \param[in] fname : Path to the npy file.
  \param[in] data : Pointer to an array of basic datatype (int, float, double, std::complex<double>, ...).
  \param[in] shape : Shape of the array, e.g. Nz x Ny x Nx.
  \param[in] mode : Writing mode, i.e. overwrite (w) or append (a) to the file.
  \warning This function has only been tested on little endian platform.
  \note Original library: <a href="https://github.com/rogersce/cnpy">cnpy</a> with MIT license.
 */
template<typename T> void npy_save(std::string fname, const T *data, const std::vector<size_t> shape, std::string mode = "w")
{
  FILE *fp = NULL;
  std::vector<size_t> true_data_shape; //if appending, the shape of existing + new data

  if (mode == "a") fp = fopen(fname.c_str(), "r+b");

  if (fp) {
    //file exists. we need to append to it. read the header, modify the array size
    size_t word_size;
    bool fortran_order;
    parse_npy_header(fp, word_size, true_data_shape, fortran_order);
    assert(!fortran_order);

    if (word_size != sizeof(T)) {
      std::cout<<"libnpy error: "<<fname<<" has word size "<<word_size<<" but npy_save appending data sized "<<sizeof(T)<<"\n";
      assert(word_size == sizeof(T));
    }
    if (true_data_shape.size() != shape.size()) {
      std::cout<<"libnpy error: npy_save attempting to append misdimensioned data to "<<fname<<"\n";
      assert(true_data_shape.size() != shape.size());
    }

    for (size_t i = 1; i < shape.size(); ++i) {
      if (shape[i] != true_data_shape[i]) {
        std::cout<<"libnpy error: npy_save attempting to append misshaped data to "<<fname<<"\n";
        assert(shape[i] == true_data_shape[i]);
      }
    }
    true_data_shape[0] += shape[0];
  }
  else {
    fp = fopen(fname.c_str(), "wb");
    true_data_shape = shape;
  }

  std::vector<char> header = create_npy_header<T>(true_data_shape);
  size_t nels = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<size_t>());

  fseek(fp, 0, SEEK_SET);
  fwrite(&header[0], sizeof(char), header.size(), fp);
  fseek(fp, 0, SEEK_END);
  fwrite(data, sizeof(T), nels, fp);
  fclose(fp);
}

/*!
  Save the specified \p fname array of data (\p data) into the \p zipname npz file. This function is similar to the
  <a href="https://numpy.org/doc/stable/reference/generated/numpy.savez.html">numpy.savez</a> function.
  \param[in] zipname : Path to the npz file.
  \param[in] fname : Identifier for the corresponding array of data.
  \param[in] data : Pointer to an array of basic datatype (int, float, double, std::complex<double>, ...).
  \param[in] shape : Shape of the array, e.g. Nz x Ny x Nx.
  \param[in] mode : Writing mode, i.e. overwrite (w) or append (a) to the file.
  \warning This function has only been tested on little endian platform.
  \note Original library: <a href="https://github.com/rogersce/cnpy">cnpy</a> with MIT license.
 */
template<typename T> void npz_save(std::string zipname, std::string fname, const T *data, const std::vector<size_t> &shape, std::string mode = "w")
{
  //first, append a .npy to the fname
  fname += ".npy";

  //now, on with the show
  FILE *fp = NULL;
  uint16_t nrecs = 0;
  size_t global_header_offset = 0;
  std::vector<char> global_header;

  if (mode == "a") fp = fopen(zipname.c_str(), "r+b");

  if (fp) {
    //zip file exists. we need to add a new npy file to it.
    //first read the footer. this gives us the offset and size of the global header
    //then read and store the global header.
    //below, we will write the the new data at the start of the global header then append the global header and footer below it
    size_t global_header_size;
    parse_zip_footer(fp, nrecs, global_header_size, global_header_offset);
    fseek(fp, static_cast<long>(global_header_offset), SEEK_SET);
    global_header.resize(global_header_size);
    size_t res = fread(&global_header[0], sizeof(char), global_header_size, fp);
    if (res != global_header_size) {
      throw std::runtime_error("npz_save: header read error while adding to existing zip");
    }
    fseek(fp, static_cast<long>(global_header_offset), SEEK_SET);
  }
  else {
    fp = fopen(zipname.c_str(), "wb");
  }

  std::vector<char> npy_header = create_npy_header<T>(shape);

  size_t nels = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<size_t>());
  size_t nbytes = nels*sizeof(T) + npy_header.size();

  //get the CRC of the data to be added
  uint32_t crc = vp_mz_crc32(0L, (uint8_t *)&npy_header[0], npy_header.size());
  crc = vp_mz_crc32(crc, (uint8_t *)data, nels*sizeof(T));

  //build the local header
  std::vector<char> local_header;
  local_header += "PK"; //first part of sig
  local_header += static_cast<uint16_t>(0x0403); //second part of sig
  local_header += static_cast<uint16_t>(20); //min version to extract
  local_header += static_cast<uint16_t>(0); //general purpose bit flag
  local_header += static_cast<uint16_t>(0); //compression method
  local_header += static_cast<uint16_t>(0); //file last mod time
  local_header += static_cast<uint16_t>(0);     //file last mod date
  local_header += static_cast<uint32_t>(crc); //crc
  local_header += static_cast<uint32_t>(nbytes); //compressed size
  local_header += static_cast<uint32_t>(nbytes); //uncompressed size
  local_header += static_cast<uint16_t>(fname.size()); //fname length
  local_header += static_cast<uint16_t>(0); //extra field length
  local_header += fname;

  //build global header
  global_header += "PK"; //first part of sig
  global_header += static_cast<uint16_t>(0x0201); //second part of sig
  global_header += static_cast<uint16_t>(20); //version made by
  global_header.insert(global_header.end(), local_header.begin()+4, local_header.begin()+30);
  global_header += static_cast<uint16_t>(0); //file comment length
  global_header += static_cast<uint16_t>(0); //disk number where file starts
  global_header += static_cast<uint16_t>(0); //internal file attributes
  global_header += static_cast<uint32_t>(0); //external file attributes
  global_header += static_cast<uint32_t>(global_header_offset); //relative offset of local file header, since it begins where the global header used to begin
  global_header += fname;

  //build footer
  std::vector<char> footer;
  footer += "PK"; //first part of sig
  footer += static_cast<uint16_t>(0x0605); //second part of sig
  footer += static_cast<uint16_t>(0); //number of this disk
  footer += static_cast<uint16_t>(0); //disk where footer starts
  footer += static_cast<uint16_t>(nrecs+1); //number of records on this disk
  footer += static_cast<uint16_t>(nrecs+1); //total number of records
  footer += static_cast<uint32_t>(global_header.size()); //nbytes of global headers
  footer += static_cast<uint32_t>(global_header_offset + nbytes + local_header.size()); //offset of start of global headers, since global header now starts after newly written array
  footer += static_cast<uint16_t>(0); //zip file comment length

  //write everything
  fwrite(&local_header[0], sizeof(char), local_header.size(), fp);
  fwrite(&npy_header[0], sizeof(char), npy_header.size(), fp);
  fwrite(data, sizeof(T), nels, fp);
  fwrite(&global_header[0], sizeof(char), global_header.size(), fp);
  fwrite(&footer[0], sizeof(char), footer.size(), fp);
  fclose(fp);
}

/*!
  Save the specified 1-D array of data (\p data) into the \p fname npz file. This function is similar to the
  <a href="https://numpy.org/doc/stable/reference/generated/numpy.save.html">numpy.save</a> function.
  \param[in] fname : Path to the npy file.
  \param[in] data : Pointer to a 1-D array of basic datatype (int, float, double, std::complex<double>, ...).
  \param[in] mode : Writing mode, i.e. overwrite (w) or append (a) to the file.
  \warning This function has only been tested on little endian platform.
  \note Original library: <a href="https://github.com/rogersce/cnpy">cnpy</a> with MIT license.
 */
template<typename T> void npy_save(std::string fname, const std::vector<T> data, std::string mode = "w")
{
  std::vector<size_t> shape;
  shape.push_back(data.size());
  npy_save(fname, &data[0], shape, mode);
}

/*!
  Save the specified \p fname 1-D array of data (\p data) into the \p zipname npz file. This function is similar to the
  <a href="https://numpy.org/doc/stable/reference/generated/numpy.savez.html">numpy.savez</a> function.
  \param[in] zipname : Path to the npz file.
  \param[in] fname : Identifier for the corresponding array of data.
  \param[in] data : Pointer to a 1-D array of basic datatype (int, float, double, std::complex<double>, ...).
  \param[in] mode : Writing mode, i.e. overwrite (w) or append (a) to the file.
  \warning This function has only been tested on little endian platform.
  \note Original library: <a href="https://github.com/rogersce/cnpy">cnpy</a> with MIT license.
 */
template<typename T> void npz_save(std::string zipname, std::string fname, const std::vector<T> data, std::string mode = "w")
{
  std::vector<size_t> shape;
  shape.push_back(data.size());
  npz_save(zipname, fname, &data[0], shape, mode);
}

template<typename T> std::vector<char> create_npy_header(const std::vector<size_t> &shape)
{
  std::vector<char> dict;
  dict += "{'descr': '";
  dict += BigEndianTest();
  dict += map_type(typeid(T));
  dict += std::to_string(sizeof(T));
  dict += "', 'fortran_order': False, 'shape': (";
  dict += std::to_string(shape[0]);
  for (size_t i = 1; i < shape.size(); ++i) {
    dict += ", ";
    dict += std::to_string(shape[i]);
  }
  if (shape.size() == 1) dict += ",";
  dict += "), }";
  //pad with spaces so that preamble+dict is modulo 16 bytes. preamble is 10 bytes. dict needs to end with \n
  int remainder = 16 - (10 + dict.size()) % 16;
  dict.insert(dict.end(), remainder, ' ');
  dict.back() = '\n';

  std::vector<char> header;
  header += static_cast<char>(0x93);
  header += "NUMPY";
  header += static_cast<char>(0x01); //major version of numpy format
  header += static_cast<char>(0x00); //minor version of numpy format
  header += static_cast<uint16_t>(dict.size());
  header.insert(header.end(), dict.begin(), dict.end());

  return header;
}

} // namespace cnpy
#endif
} // namespace VISP_NAMESPACE_NAME
#endif

/*!
 * \class vpIoTools
 * \ingroup group_core_files_io
 * \brief File and directories basic tools.
 *
 * The example below shows how to manipulate the functions of this
 * class to create first a directory which name corresponds to the user
 * name and then create a file in this directory.
 *
 * \code
 * #include <fstream>
 * #include <iostream>
 * #include <string>
 * #include <visp3/core/vpIoTools.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   std::string username;
 *   vpIoTools::getUserName(username);
 *
 *   // Test if a username directory exist. If no try to create it
 *   if (vpIoTools::checkDirectory(username) == false) {
 *     try {
 *       // Create a directory with name "username"
 *       vpIoTools::makeDirectory(username);
 *     }
 *     catch (...) {
 *       std::cout << "Cannot create " << username << " directory" << std::endl;
 *       return EXIT_FAILURE;
 *     }
 *   }
 *   // Create a empty filename with name "username/file.txt"
 *   std::ofstream f;
 *   std::string filename = username + "/file.txt";
 *   // Under Windows converts the filename string into "username\\file.txt"
 *   filename = vpIoTools::path(filename);
 *   std::cout << "Create: " << filename << std::endl;
 *   f.open(filename.c_str());
 *   f.close();
 *
 *   // Rename the file
 *   std::string newfilename = username + "/newfile.txt";
 *   std::cout << "Rename: " << filename << " in: " << newfilename << std::endl;
 *   if (vpIoTools::rename(filename, newfilename) == false)
 *     std::cout << "Unable to rename: " << filename << std::endl;
 *
 *   // Remove the file
 *   std::cout << "Remove: " << newfilename << std::endl;
 *   if (vpIoTools::remove(newfilename) == false)
 *     std::cout << "Unable to remove: " << newfilename << std::endl;
 *
 *   return EXIT_SUCCESS;
 * }
 * \endcode
 *
 * The example below shows how to read a configuration file and how to create a name
 * for experiment files. We assume the following file "/home/user/demo/config.txt" :
 * \code
 * expNumber 2
 * save 0
 * lambda 0.4
 * use2D 0
 * use3D 1
 * \endcode
 *
 * \code
 * #include <iostream>
 * #include <string>
 * #include <visp3/core/vpIoTools.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   // reading configuration file
 *   vpIoTools::loadConfigFile("/home/user/demo/config.txt");
 *   std::string nExp;vpIoTools::readConfigVar("expNumber", nExp); // nExp <- "2"
 *   double lambda;vpIoTools::readConfigVar("lambda", lambda);     // lambda <- 0.4
 *   bool use2D;vpIoTools::readConfigVar("use2D", use2D);          // use2D <- false
 *   bool use3D;vpIoTools::readConfigVar("use3D", use3D);          // use3D <- true
 *   bool doSave;vpIoTools::readConfigVar("save", doSave);         //  doSave <- false
 *
 *   // creating name for experiment files
 *   vpIoTools::setBaseDir("/home/user/data");
 *   // full name <- "/home/user/data/exp2"
 *   vpIoTools::setBaseName("exp" + nExp);
 *   // full name <- "/home/user/data/exp2" since use2D==false
 *   vpIoTools::addNameElement("2D", use2D);
 *   // full name <- "/home/user/data/exp2_3D"
 *   vpIoTools::addNameElement("3D", use3D);
 *   // full name <- "/home/user/data/exp2_3D_lambda0.4"
 *   vpIoTools::addNameElement("lambda", lambda);
 *
 *   // Saving file.Would copy "/home/user/demo/config.txt" to
 *   // "/home/user/data/exp2_3D_lambda0.4_config.txt" if doSave was true
 *   vpIoTools::saveConfigFile(doSave);
 *   // create sub directory
 *   vpIoTools::createBaseNamePath();  // creates "/home/user/data/exp2_3D_lambda0.4/"
 * }
 * \endcode
 */
BEGIN_VISP_NAMESPACE
class VISP_EXPORT vpIoTools
{

public:
  static const std::string &getBuildInformation();
  static std::string getTempPath();
  static void getUserName(std::string &username);
  static std::string getUserName();
  static std::string getenv(const std::string &env);
  static std::string getViSPImagesDataPath();
  static void getVersion(const std::string &version, unsigned int &major, unsigned int &minor, unsigned int &patch);
  static bool checkDirectory(const std::string &dirname);
  static bool checkFifo(const std::string &filename);
  static bool checkFilename(const std::string &filename);
  static bool copy(const std::string &src, const std::string &dst);

  static void makeDirectory(const std::string &dirname);
  static void makeFifo(const std::string &dirname);
  static std::string makeTempDirectory(const std::string &dirname);
  static std::string path(const std::string &pathname);

  static bool remove(const std::string &filename);
  static bool rename(const std::string &oldfilename, const std::string &newfilename);

  /*!
   * Define the directory separator character, backslash ('\') for windows
   * platform or slash ('/') otherwise.
   */
  static const char separator;

  static std::string toUpperCase(const std::string &input);
  static std::string toLowerCase(const std::string &input);
  static std::string getAbsolutePathname(const std::string &pathname);
  static std::string getFileExtension(const std::string &pathname, bool checkFile = false);
  static long getIndex(const std::string &filename, const std::string &format);
  static std::string getName(const std::string &pathname);
  static std::string getNameWE(const std::string &pathname);
  static std::string getParent(const std::string &pathname);
  static std::string createFilePath(const std::string &parent, const std::string &child);
  static bool isAbsolutePathname(const std::string &pathname);
  static bool isSamePathname(const std::string &pathname1, const std::string &pathname2);
  static std::pair<std::string, std::string> splitDrive(const std::string &pathname);
  static std::vector<std::string> splitChain(const std::string &chain, const std::string &sep);
  static std::vector<std::string> getDirFiles(const std::string &dirname);

  /*!
   * @name Configuration file parsing
   */
  //@{
  // read configuration file
  static bool loadConfigFile(const std::string &confFile);
  static bool readConfigVar(const std::string &var, float &value);
  static bool readConfigVar(const std::string &var, double &value);
  static bool readConfigVar(const std::string &var, int &value);
  static bool readConfigVar(const std::string &var, unsigned int &value);
  static bool readConfigVar(const std::string &var, bool &value);
  static bool readConfigVar(const std::string &var, std::string &value);
  static bool readConfigVar(const std::string &var, vpColor &value);
  static bool readConfigVar(const std::string &var, vpArray2D<double> &value, const unsigned int &nCols = 0,
                            const unsigned int &nRows = 0);

  // construct experiment filename & path
  static void setBaseName(const std::string &s);
  static void setBaseDir(const std::string &dir);
  static void addNameElement(const std::string &strTrue, const bool &cond = true, const std::string &strFalse = "");
  static void addNameElement(const std::string &strTrue, const double &val);
  static std::string getBaseName();
  static std::string getFullName();

  // write files
  static void saveConfigFile(const bool &actuallySave = true);
  static void createBaseNamePath(const bool &empty = false);
  //@}

  static void readBinaryValueLE(std::ifstream &file, int16_t &short_value);
  static void readBinaryValueLE(std::ifstream &file, uint16_t &ushort_value);
  static void readBinaryValueLE(std::ifstream &file, int32_t &int_value);
  static void readBinaryValueLE(std::ifstream &file, uint32_t &int_value);
  static void readBinaryValueLE(std::ifstream &file, float &float_value);
  static void readBinaryValueLE(std::ifstream &file, double &double_value);

  static void writeBinaryValueLE(std::ofstream &file, const int16_t short_value);
  static void writeBinaryValueLE(std::ofstream &file, const uint16_t ushort_value);
  static void writeBinaryValueLE(std::ofstream &file, const int32_t int_value);
  static void writeBinaryValueLE(std::ofstream &file, const uint32_t int_value);
  static void writeBinaryValueLE(std::ofstream &file, float float_value);
  static void writeBinaryValueLE(std::ofstream &file, double double_value);

  static bool parseBoolean(std::string input);
  static std::string trim(std::string s);

protected:
  static std::string baseName;
  static std::string baseDir;
  static std::string configFile;
  static std::vector<std::string> configVars;
  static std::vector<std::string> configValues;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  static int mkdir_p(const std::string &path, int mode);
#endif
};
END_VISP_NAMESPACE
#endif
