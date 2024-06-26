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
 * An argument parser that can both use JSON files and command line arguments as inputs.
 */

#ifndef VP_JSON_ARGUMENT_PARSER_H
#define VP_JSON_ARGUMENT_PARSER_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json.hpp>
#include <visp3/core/vpException.h>
#include <sstream>


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

BEGIN_VISP_NAMESPACE

/*!
  \class vpJsonArgumentParser
  \ingroup module_io_cmd_parser
  \brief Command line argument parsing with support for JSON files.
  If a JSON file is supplied, it is parsed and command line arguments take precedence over values given in the file.

  \warning To be used, this class requires the 3rd party JSON library to be installed and enabled when installing ViSP.

  This argument parser can take any number and type of arguments, as long they can be serialized to and from JSON.

  A very basic program that uses both a JSON file and command line arguments can be found below:
  \code{.cpp}
  #include <visp3/io/vpJsonArgumentParser.h>
  #include <iostream>
  int main(int argc, char* argv[])
  {
    double d = 1.0;
    std::string s = "Default";

    vpJsonArgumentParser parser("Example program for arguments with vpJsonArgumentParser", "config", "/");

    parser.add_argument("scalar", d, true, "An important value: must be defined by the user")
          .add_argument("string", s, false, "An optional value: if left unspecified, will default to its initialized value (\"Default\")")
          .parse(argc, argv);

    std::cout << "Scalar = " << d << std::endl;
    std::cout << "String = " << s << std::endl;
  }
  \endcode
  Compiling this sample and calling the program with the arguments from the command line would yield:
  \code{.sh}
  $ ./program scalar 2.0 string "A new value"
  Scalar = 2.0
  String = a new value
  $ ./program scalar 2.0
  Scalar = 2.0
  String = default
  \endcode
  Here the arguments are specified from the command line. Since the "string" argument is optional, it does not have to be specified.

  For programs with more arguments it is helpful to use a JSON file that contains a base configuration. For the program above, a JSON file could look like:
  \code{.json}
  {
    "scalar": 3.0,
    "string": "Some base value"
  }
  \endcode
  we could then call the program with:
  \code{.sh}
  $ ./program config my_settings.json
  Scalar = 3.0
  String = Some base value
  \endcode

  The values contained in the JSON file can be overridden with command line arguments
  \code{.sh}
  $ ./program config my_settings.json scalar 5
  Scalar = 5.0
  String = Some base value
  \endcode

  The program can also be called with the "-h" or "--help" argument to display the help associated to the arguments, as well as an example json configuration file
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

  /**
   * @brief Generate a help message, containing the description of the arguments, their default value and whether they are required or not.
   * This message also contains an example json file, generated from the default values of the arguments.
   * This method is called when running the program with the "-h" or "--help" arguments.
   *
   * @return The help message
   */
  std::string help() const;


  /**
   * @brief Add an argument that can be provided by the user, either via command line or through the json file.
   *
   * @tparam T Type of the argument to pass.
   * The methods from_json(const nlohmann::json&, T&) and to_json(nlohmann::json&, const T&) must be defined.
   * This is the case for most basic types or stl containers. For your own types, you should define the method.
   * @param name Name of the parameter that will be used to look up the argument values when parsing command line arguments or the json file.
   * This name may contain the nestSeparator, in which case the look up in the JSON file will seek a nested object to parse.
   * @param parameter Reference where the parsed value will be stored. It is modified when calling parse.
   * @param required Whether this argument is required. If it is, it should be specified either through command line or through the json file.
   * If not, then you should take special care to initialize \p parameter with a sensible value.
   * @param help The description of the argument.
   * @return vpJsonArgumentParser& returns self, allowing argument definition chaining
   */
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
      const bool fieldHasNoValue = field == nullptr || (field != nullptr && field->is_null());
      if (required && fieldHasNoValue) {
        std::stringstream ss;
        ss << "Argument " << name << " is required, but no value was provided" << std::endl;
        throw vpException(vpException::badValue, ss.str());
      }
      else if (!fieldHasNoValue) {
        field->get_to(parameter);
      }
      };

    updaters[name] = [getter](nlohmann::json &j, const std::string &s) {
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

  /**
   * @brief Parse the arguments.
   *
   * @param argc Number of arguments (including program name)
   * @param argv Arguments
   */
  void parse(int argc, const char *argv[]);


private:
  std::string description; // Program description
  std::string jsonFileArgumentName; // Name of the argument  that points to the json file: ./program --config settings.json. Here jsonFileArgumentName == "--config"
  std::string nestSeparator; // JSON nesting delimiter character. Used to access JSON nested objects from a single string
  std::map<std::string, std::function<void(nlohmann::json &)>> parsers; // Functions that update the variables with the values contained in the JSON document (should be used after calling updaters)
  std::map<std::string, std::function<void(nlohmann::json &, const std::string &)>> updaters; // Update the base json document with command line arguments
  std::map<std::string, std::function<std::string()>> helpers; // Functions that output the usage and description of command line arguments: used when the help flag is given as argument
  nlohmann::json exampleJson; // Example JSON argument file: displayed when user calls for help
};

END_VISP_NAMESPACE

#endif // VISP_HAVE_NLOHMANN_JSON

#endif
