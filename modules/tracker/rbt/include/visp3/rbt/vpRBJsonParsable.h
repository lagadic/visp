#ifndef VP_RB_JSON_PARSABLE_H
#define VP_RB_JSON_PARSABLE_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>

#include <string>
#include <iostream>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json.hpp>
#endif

BEGIN_VISP_NAMESPACE


/**
 * \brief A set of utilities to perform initialization.
 *
 * \ingroup group_rbt_core
*/
class VISP_EXPORT vpRBJsonParsable
{
public:
#if defined(VISP_HAVE_NLOHMANN_JSON)
  virtual void loadJsonConfiguration(const nlohmann::json &j) = 0;
  virtual bool verify(const nlohmann::json &j) const
  {
    const nlohmann::json explanation = explain();
    std::vector<std::string> expectedKeys;
    for (const auto &it: explanation.items()) {
      expectedKeys.push_back(it.key());
    }

    // Check for extranuous keys that should not be present
    for (const auto &it: j.items()) {
      if (std::find(expectedKeys.begin(), expectedKeys.end(), it.key()) == expectedKeys.end()) {
        throw vpException(vpException::badValue, "Got an unexpected key when parsing JSON: %s\nJSON is %s", it.key(), j.dump(4));
      }
    }

    // Check that required keys are present


    return true;

  }

  virtual nlohmann::json explain() const = 0;

protected:
  template<typename T>
  static nlohmann::json parameterBase(const std::string &name, const std::string &explanation, bool required, const T &value)
  {
    return {
      {"name", name},
      {"description", explanation},
      {"required", required},
      {"exampleValue", value}
    };
  }
  static nlohmann::json parameter(const std::string &name, const std::string &explanation, bool required, double value)
  {
    nlohmann::json j = parameterBase(name, explanation, required, value);
    j["type"] = "double";
    return j;
  }

  static nlohmann::json parameter(const std::string &name, const std::string &explanation, bool required, int value)
  {
    nlohmann::json j = parameterBase(name, explanation, required, value);
    j["type"] = "int";
    return j;
  }


  static nlohmann::json parameter(const std::string &name, const std::string &explanation, bool required, bool value)
  {
    nlohmann::json j = parameterBase(name, explanation, required, value);
    j["type"] = "bool";
    return j;
  }


  static nlohmann::json flipToDict(const std::vector<nlohmann::json> &values)
  {

    nlohmann::json j;
    for (const nlohmann::json &v: values) {

      nlohmann::json vv = v;

      vv.erase("name");
      j[v.at("name").get<std::string>()] = vv;
    }
    return j;
  }



#endif
};

END_VISP_NAMESPACE

#endif
