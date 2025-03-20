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
 */

/*!
  \file vpDynamicFactory.h
  \brief Factory-type class that allows for the dynamic registration of subclasses
*/
#ifndef VP_DYNAMIC_FACTORY_H
#define VP_DYNAMIC_FACTORY_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

#include <map>
#include <functional>

BEGIN_VISP_NAMESPACE
template<typename T>
class VISP_EXPORT vpDynamicFactory
{
public:
#if defined(VISP_HAVE_NLOHMANN_JSON)
  void registerType(const std::string &key, const std::function<std::shared_ptr<T>()> &function)
  {
    if (m_jsonBuildables.find(key) != m_jsonBuildables.end() || m_jsonRawBuilders.find(key) != m_jsonRawBuilders.end()) {
      throw vpException(vpException::badValue, "Type %s was already registered in the factory", key.c_str());
    }
    m_jsonBuildables[key] = function;
  }

  void registerTypeRaw(const std::string &key, const std::function<std::shared_ptr<T>(const std::string &)> function)
  {
    if (m_jsonBuildables.find(key) != m_jsonBuildables.end() || m_jsonRawBuilders.find(key) != m_jsonRawBuilders.end()) {
      throw vpException(vpException::badValue, "Type %s was already registered in the factory", key.c_str());
    }

    m_jsonRawBuilders.insert({ key, function });
    // m_jsonRawBuilders[key] = [](const std::string &s) -> std::shared_ptr<T> {

  }

  std::shared_ptr<T> buildFromJson(const nlohmann::json &j)
  {
    const std::string key = m_keyFinder(j);

    if (m_jsonBuildables.find(key) != m_jsonBuildables.end()) {
      std::shared_ptr<T> res = m_jsonBuildables[key]();
      res->loadJsonConfiguration(j);
      return res;
    }

    else if (m_jsonRawBuilders.find(key) != m_jsonRawBuilders.end()) {
      return m_jsonRawBuilders[key](j.dump());
    }
    else {
      return nullptr;
    }
  }


  void setJsonKeyFinder(const std::function<std::string(const nlohmann::json &)> &finderFn)
  {
    m_keyFinder = finderFn;
  }
#endif

  virtual ~vpDynamicFactory() { }

protected:

  vpDynamicFactory() = default;
#if defined(VISP_HAVE_NLOHMANN_JSON)
  std::map<std::string, std::function<std::shared_ptr<T>()>> m_jsonBuildables;
  std::map<std::string, std::function<std::shared_ptr<T>(const std::string &)>> m_jsonRawBuilders;

  std::function<std::string(const nlohmann::json &)> m_keyFinder; //! Function to retrieve the key from a json object
#endif
};

END_VISP_NAMESPACE

#endif
