/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * An argument parser that can both use JSON files and command line arguments as inputs.
 *
 * Authors:
 * Samuel Felton
 *
 *****************************************************************************/

#ifndef vpJsonArgumentParser_h
#define vpJsonArgumentParser_h
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json.hpp>
#include <visp3/core/vpException.h>


 /**
 * @brief Convert a command line argument to a json representation. By default, will call the parsing function of the JSON library
 *
 * @param arg the argument, represented as a string
 * @return json the json representation of the argument
 */

template<typename T>
nlohmann::json convertCommandLineArgument(const std::string &arg)
{
  nlohmann::json j = nlohmann::json::parse(arg);
  return j;
}
/**
 * @brief Specialization of command line parsing for strings: a shell may eat the quotes, which would be necessary for JSON parsing to work.
 * This function thus directly converts the string to a JSON representation: no parsing is performed.
 * @param arg the string argument
 * @return nlohmann::json The JSON representation of the string
 */
template<>
nlohmann::json convertCommandLineArgument<std::string>(const std::string &arg)
{
  nlohmann::json j = arg;
  return j;
}
/*!
  \class vpJsonArgumentParser
  \ingroup module_io_cmd_parser
  \brief Command line argument parsing with support for JSON files. If a JSON file is supplied, it is parsed and command line arguments take precedence over values given in the file.

  To be used, this class requires the 3rd party JSON library to be installed and enabled when installing ViSP.

  This argument parser can take any number and type of arguments, as long they can be serialized to and from JSON.


*/
class VISP_EXPORT vpJsonArgumentParser
{
public:
  /**
   * @brief Create a new argument parser, that can take into account both a JSON configuration file and command line arguments.
   *
   * @param description Description of the program tied to this parser
   * @param jsonFileArgumentName Name of the argument that points to the JSON file to load
   * @param nestSeparator Delimiter that is used map a nested json object to a command line argument. For example, with a delimiter set to "/", the command line argument "a/b" will map to the json key "b" in the following json document:
   * \code
   * {
   *  "a": {
   *    "b": 10.0
   *  },
   *  "otherArgument": false
   * }
   * \endcode
   *
   */
  vpJsonArgumentParser(const std::string &description, const std::string &jsonFileArgumentName, const std::string &nestSeparator);

  std::string help() const;



  template<typename T>
  vpJsonArgumentParser &addArgument(const std::string &name, T &parameter, const bool required = true, const std::string &help = "No description")
  {
    const auto getter = [name, this](nlohmann::json &j, bool create) -> nlohmann::json * {
      size_t pos = 0;
      nlohmann::json *f = &j;
      std::string token;
      std::string name_copy = name;

      while ((pos = name_copy.find(nestSeparator)) != std::string::npos) {
        token = name_copy.substr(0, pos);
        name_copy.erase(0, pos + nestSeparator.length());
        if (create && !f->contains(token)) {
          (*f)[token] = {};
        }
        else if (!f->contains(token)) {
          return nullptr;
        }
        f = &(f->at(token));
      }
      if (create && !f->contains(name_copy)) {
        (*f)[name_copy] = {};
      }
      else if (!f->contains(name_copy)) {
        return nullptr;
      }
      f = &(f->at(name_copy));
      return f;
    };
    parsers[name] = [&parameter, required, getter, name](nlohmann::json &j) {
      const nlohmann::json *field = getter(j, false);
      if (field != nullptr) {
        if (field->empty()) {
          std::stringstream ss;
          ss << "Argument " << name << "is required, but no value was provided" << std::endl;
          throw vpException(vpException::badValue, ss.str());
        }
        field->get_to(parameter);
      }
      else {
        std::stringstream ss;
        ss << "Argument " << name << "is required, but no value was provided" << std::endl;
        throw vpException(vpException::badValue, ss.str());
      }
    };
    updaters[name] = [getter, &parameter, this](nlohmann::json &j, const std::string &s) {
      nlohmann::json *field = getter(j, true);
      *field = convertCommandLineArgument<T>(s);
    };
    helpers[name] = [help, parameter, required]() -> std::string {
      std::stringstream ss;
      nlohmann::json repr = parameter;
      ss << help << std::endl << "Default: " << repr;
      if (required) {
        ss << std::endl << "Required";
      }
      else {
        ss << std::endl << "Optional";
      }
      return ss.str();
    };

    nlohmann::json *exampleField = getter(exampleJson, true);
    *exampleField = parameter;

    return *this;
  }

  void parse(int argc, const char *argv []);


private:
  std::string description; // Program description
  std::string jsonFileArgumentName; // Name of the argument  that points to the json file: ./program --config settings.json. Here jsonFileArgumentName == "--config"
  std::string nestSeparator; // JSON nesting delimiter character. Used to access JSON nested objects from a single string
  std::map<std::string, std::function<void(nlohmann::json &)>> parsers; // Functions that update the variables with the values contained in the JSON document (should be used after calling updaters)
  std::map<std::string, std::function<void(nlohmann::json &, const std::string &)>> updaters; // Update the base json document with command line arguments
  std::map<std::string, std::function<std::string()>> helpers; // Functions that output the usage and description of command line arguments: used when the help flag is given as argument
  nlohmann::json exampleJson; // Example JSON argument file: displayed when user calls for help
};

#endif // VISP_HAVE_NLOHMANN_JSON

#endif