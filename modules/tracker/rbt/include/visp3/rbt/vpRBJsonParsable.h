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


class vpRBConfigurableBase
{
public:
  vpRBConfigurableBase(const std::string &name, const std::string &description, bool required)
    : m_name(name), m_description(description), m_required(required)
  { }
  virtual ~vpRBConfigurableBase() { }

  const std::string &name() const { return m_name; }
  const std::string &description() const { return m_description; }

  bool required() const { return m_required; }


  virtual nlohmann::ordered_json explanation() const
  {
    return {
      {"name", m_name},
      {"description", m_description},
      {"required", m_required}
    };
  }

protected:
  std::string m_name;
  std::string m_description;
  bool m_required;
};

template<typename T>
class VISP_EXPORT vpRBParameter : public vpRBConfigurableBase
{
public:

  vpRBParameter(const std::string &name, const std::string &description, bool required, std::function<T()> &getter, std::function<void(const T)> &setter)
    : vpRBConfigurableBase(name, description, required)
  {

  }

  virtual nlohmann::ordered_json explanation() const
  {
    nlohmann::ordered_json j = vpRBConfigurableBase::explanation();
    j["exampleValue"] = m_getter();
    return j;
  }

  T getValue() const
  {
    return m_getter();
  }

  void update(const T value) const
  {
    m_setter(value);
  }
protected:
  std::function<T()> m_getter;
  std::function<void(const T)> m_setter;
};

class VISP_EXPORT vpRBConfigurable : public vpRBConfigurableBase
{
public:
  vpRBConfigurable(const std::string &name, const std::string &description, bool required)
    : vpRBConfigurableBase(name, description, required)
  {

  }

  vpRBConfigurable &parameter(const vpRBParameter<bool> &param)
  {
    m_boolParams.push_back(param);
    return *this;
  }

  vpRBConfigurable &parameter(const vpRBParameter<int> &param)
  {
    m_intParams.push_back(param);
    return *this;
  }

  vpRBConfigurable &parameter(const vpRBParameter<double> &param)
  {
    m_doubleParams.push_back(param);
    return *this;
  }
  vpRBConfigurable &parameter(std::shared_ptr<vpRBConfigurable> param)
  {
    m_subObjectParams.push_back(param);
    return *this;
  }
#ifdef VISP_HAVE_NLOHMANN_JSON
  void checkParameterExists(const vpRBConfigurableBase &p, const std::string &parameterType, const nlohmann::json &j, const nlohmann::json::const_iterator &it)
  {
    if (p.required() && it == j.end()) {
      std::stringstream ss;
      ss << "Parameter " << p.name() << " is required when parsing object " << m_name << " But it was not found." << std::endl;
      ss << "Parameter description: " << p.description() << std::endl;
      ss << "Parameter type: " << parameterType;
      throw vpException(vpException::badValue, ss.str());
    }
  }

  template<typename T>
  void parseParametersOfType(const nlohmann::json &j, std::vector<vpRBParameter<T>> &params, const std::string &paramType, const std::function<bool(const nlohmann::json &)> &isValidType)
  {
    for (vpRBParameter<T> &p: params) {
      const auto it = j.find(p.name());
      checkParameterExists(p, paramType, j, it);
      if (it != j.end()) {

        if (it->is_null()) {
          if (p.required()) {
            std::stringstream ss;
            ss << "Parameter " << p.name() << "Expected an actual value but got a null value";
            throw vpException(vpException::badValue, ss.str());
          }
          else {
            continue;
          }
        }
        if (!isValidType(*it)) {
          std::stringstream ss;
          ss << "Parameter " << p.name() << "Expected a value of type " << paramType << " But got another type";
          throw vpException(vpException::badValue, ss.str());
        }
        p.update(it->template get<T>());
      }
    }
  }

  virtual void parse(const nlohmann::json &j)
  {
    parseParametersOfType(j, m_boolParams, "boolean",
      [](const nlohmann::json &v) -> bool { return v.is_boolean(); });
    parseParametersOfType(j, m_intParams, "integer",
      [](const nlohmann::json &v) -> bool { return v.is_number_integer(); });
    parseParametersOfType(j, m_doubleParams, "float",
      [](const nlohmann::json &v) -> bool { return v.is_number_float(); });
    parseParametersOfType(j, m_stringParams, "string",
      [](const nlohmann::json &v) -> bool { return v.is_string(); });

    for (std::shared_ptr<vpRBConfigurable> p: m_subObjectParams) {

      const auto it = j.find(p->name());
      if (p->required() && it == j.end()) {
        std::stringstream ss;
        ss << "Parameter " << p->name() << " is required but was not found";
        ss << "Parameter description: " << p->description() << std::endl;
        throw vpException(vpException::badValue, ss.str());
      }
      if (it != j.end()) {

        if (it->is_null()) {
          if (p->required()) {
            std::stringstream ss;
            ss << "Parameter " << p->name() << " expected an actual value but got null";
            throw vpException(vpException::badValue, ss.str());
          }
          else {
            continue;
          }
        }
        p->parse(*it);
      }
    }
  }
#endif

private:

  std::vector<vpRBParameter<bool>> m_boolParams;
  std::vector<vpRBParameter<int>> m_intParams;
  std::vector<vpRBParameter<double>> m_doubleParams;
  std::vector<vpRBParameter<std::string>> m_stringParams;
  std::vector<std::shared_ptr<vpRBConfigurable>> m_subObjectParams;
};


/**
 * \ingroup group_rbt_core
*/
class VISP_EXPORT vpRBJsonParsable
{
public:
  vpRBJsonParsable()
  {
    addUncheckedKey("description");
  }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  virtual void loadJsonConfiguration(const nlohmann::json &j) = 0;
  virtual bool verify(const nlohmann::json &j) const
  {
    const nlohmann::ordered_json explanation = explain();
    std::vector<std::string> expectedKeys;
    for (const auto &it: explanation.items()) {
      expectedKeys.push_back(it.key());
    }

    // Check for extranuous keys that should not be present
    for (const auto &it: j.items()) {
      if (isUncheckedKey(it.key())) {
        continue;
      }
      if (std::find(expectedKeys.begin(), expectedKeys.end(), it.key()) == expectedKeys.end()) {
        throw vpException(vpException::badValue, "Got an unexpected key when parsing JSON: %s\nJSON is %s", it.key().c_str(), j.dump(4).c_str());
      }
    }
    return true;

  }

  virtual nlohmann::ordered_json explain() const = 0;

protected:

  void addUncheckedKey(const std::string &key)
  {
    m_uncheckedJsonKeys.push_back(key);
  }

  bool isUncheckedKey(const std::string &key) const
  {
    return std::find(m_uncheckedJsonKeys.begin(), m_uncheckedJsonKeys.end(), key) != m_uncheckedJsonKeys.end();
  }


  template<typename T>
  static nlohmann::ordered_json parameterBase(const std::string &name, const std::string &explanation, bool required, const T &value)
  {
    return {
      {"name", name},
      {"description", explanation},
      {"required", required},
      {"exampleValue", value}
    };
  }
  static nlohmann::ordered_json parameter(const std::string &name, const std::string &explanation, bool required, double value)
  {
    nlohmann::ordered_json j = parameterBase(name, explanation, required, value);
    j["type"] = "double";
    return j;
  }

  static nlohmann::ordered_json parameter(const std::string &name, const std::string &explanation, bool required, int value)
  {
    nlohmann::ordered_json j = parameterBase(name, explanation, required, value);
    j["type"] = "int";
    return j;
  }


  static nlohmann::ordered_json parameter(const std::string &name, const std::string &explanation, bool required, bool value)
  {
    nlohmann::ordered_json j = parameterBase(name, explanation, required, value);
    j["type"] = "bool";
    return j;
  }


  static nlohmann::ordered_json flipToDict(const std::vector<nlohmann::ordered_json> &values)
  {

    nlohmann::ordered_json j;
    for (const nlohmann::ordered_json &v: values) {

      nlohmann::json vv = v;

      vv.erase("name");
      j[v.at("name").get<std::string>()] = vv;
    }
    return j;
  }

private:
  std::vector<std::string> m_uncheckedJsonKeys;


#endif
};

END_VISP_NAMESPACE

#endif
