/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 */

#include <visp3/core/vpConfig.h>
#include <visp3/io/vpJsonArgumentParser.h>
#include <visp3/core/vpException.h>
#include <fstream>

#if defined(VISP_HAVE_NLOHMANN_JSON)
BEGIN_VISP_NAMESPACE

using json = nlohmann::json; //! json namespace shortcut

vpJsonArgumentParser::vpJsonArgumentParser(const std::string &description, const std::string &jsonFileArgumentName,
                                           const std::string &nestSeparator) :
  m_description(description),
  m_jsonFileArgumentName(jsonFileArgumentName),
  m_nestSeparator(nestSeparator)
{
  if (m_jsonFileArgumentName.empty()) {
    throw vpException(vpException::badValue, "The JSON file argument must not be empty");
  }

  if (m_nestSeparator.empty()) {
    throw vpException(vpException::badValue, "You must provide a JSON nesting delimiter to be able to parse JSON");
  }

  m_helpers[m_jsonFileArgumentName] = []() -> std::string {
    return "Path to the JSON configuration file. Values in this files are loaded, and can be overridden by command line arguments.\nOptional";
    };
}

std::string vpJsonArgumentParser::help() const
{
  std::stringstream ss;

  ss << "Program description: " << m_description << std::endl;
  ss << "Arguments: " << std::endl;
  unsigned spacesBetweenArgAndDescription = 0;
  for (const auto &helper : m_helpers) {
    if (helper.first.size() > spacesBetweenArgAndDescription) {
      spacesBetweenArgAndDescription = static_cast<unsigned int>(helper.first.size());
    }
  }
  spacesBetweenArgAndDescription += 4;

  for (const auto &helper : m_helpers) {
    std::stringstream argss(helper.second());
    std::string line;
    bool first = true;
    while (getline(argss, line, '\n')) {
      const unsigned lineSpace = first ? spacesBetweenArgAndDescription - static_cast<unsigned>(helper.first.size()) : spacesBetweenArgAndDescription;
      const std::string spaceBetweenArgAndDescription(lineSpace, ' ');
      if (first) {
        ss << "\t" << helper.first << spaceBetweenArgAndDescription << line << std::endl;
      }
      else {
        ss << "\t" << spaceBetweenArgAndDescription << line << std::endl;
      }
      first = false;

    }
    ss << std::endl;
  }
  ss << "Example JSON configuration file: " << std::endl << std::endl;
  ss << m_exampleJson.dump(2) << std::endl;
  return ss.str();
}

vpJsonArgumentParser &vpJsonArgumentParser::addFlag(const std::string &name, bool &parameter, const std::string &help)
{
  m_argumentType[name] = FLAG;
  const auto getter = [name, this](nlohmann::json &j, bool create) -> nlohmann::json *{
    size_t pos = 0;
    nlohmann::json *f = &j;
    std::string token;
    std::string name_copy = name;

    while ((pos = name_copy.find(m_nestSeparator)) != std::string::npos) {
      token = name_copy.substr(0, pos);

      name_copy.erase(0, pos + m_nestSeparator.length());
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

  m_parsers[name] = [&parameter, getter, name](nlohmann::json &j) {
    const nlohmann::json *field = getter(j, false);
    const bool fieldHasNoValue = ((field == nullptr) || (field != nullptr && field->is_null()));
    if (!fieldHasNoValue && (field->type() == json::value_t::boolean && (*field) == true)) {
      parameter = !parameter;
    }
    };

  m_updaters[name] = [getter](nlohmann::json &j, const std::string &) {
    nlohmann::json *field = getter(j, true);
    *field = true;
    };

  m_helpers[name] = [help, parameter]() -> std::string {
    std::stringstream ss;
    nlohmann::json repr = parameter;
    ss << help << std::endl << "Default: " << repr;
    return ss.str();
    };

  nlohmann::json *exampleField = getter(m_exampleJson, true);
  *exampleField = parameter;

  return *this;
}

void vpJsonArgumentParser::parse(int argc, const char *argv[])
{
  json j;
  const std::vector<std::string> arguments(argv + 1, argv + argc);
  std::vector<unsigned> ignoredArguments;
  const auto jsonFileArgumentPos = std::find(arguments.begin(), arguments.end(), m_jsonFileArgumentName);
  // Load JSON file if present
  if (jsonFileArgumentPos != arguments.end()) {
    ignoredArguments.push_back(static_cast<unsigned>(jsonFileArgumentPos - arguments.begin() + 1));
    ignoredArguments.push_back(static_cast<unsigned>(jsonFileArgumentPos - arguments.begin() + 2));

    if (jsonFileArgumentPos == arguments.end() - 1) {
      throw vpException(vpException::ioError, "No JSON file was provided");
    }
    const std::string jsonFileName = *(jsonFileArgumentPos + 1);
    std::ifstream jsonFile(jsonFileName);
    if (!jsonFile.good()) {
      std::stringstream ss;
      ss << "Could not open JSON file " << jsonFileName << "! Make sure it exists and is readable" << std::endl;
      throw vpException(vpException::ioError, ss.str());
    }
    j = json::parse(jsonFile);
    jsonFile.close();
  }
  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    bool stop_for_loop = false;
    if (std::find(ignoredArguments.begin(), ignoredArguments.end(), i) != ignoredArguments.end()) {
      stop_for_loop = true;
    }
    if (!stop_for_loop) {
      if (arg == "-h" || arg == "--help") {
        std::cout << help() << std::endl;
        exit(1);
      }

      if (m_parsers.find(arg) != m_parsers.end()) {
        if (m_argumentType[arg] == WITH_FIELD) {
          if (i < argc - 1) {
            m_updaters[arg](j, std::string(argv[i + 1]));
            ++i;
          }
          else {
            std::stringstream ss;
            ss << "Argument " << arg << " was passed but no value was provided" << std::endl;
            throw vpException(vpException::ioError, ss.str());
          }
        }
        else if (m_argumentType[arg] == FLAG) {
          m_updaters[arg](j, std::string());
        }
      }
      else {
        std::cerr << "Unknown parameter when parsing: " << arg << std::endl;
      }
    }
  }

  // Get the values from json document and store them in the arguments passed by ref in addArgument
  for (const auto &parser : m_parsers) {
    parser.second(j);
  }
}

END_VISP_NAMESPACE

#endif
