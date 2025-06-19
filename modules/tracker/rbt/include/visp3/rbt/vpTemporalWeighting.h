#ifndef VP_TEMPORAL_WEIGHTING_H
#define VP_TEMPORAL_WEIGHTING_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json_fwd.hpp)
#endif

class VISP_EXPORT vpTemporalWeighting
{
public:
  virtual double weight(const double progress) const;
  virtual ~vpTemporalWeighting() { }
#if defined(VISP_HAVE_NLOHMANN_JSON)
  static std::shared_ptr<vpTemporalWeighting> parseTemporalWeighting(const nlohmann::json &j);
#endif
};

class VISP_EXPORT vpFixedTemporalWeighting : public vpTemporalWeighting
{
public:
  vpFixedTemporalWeighting(double weight) : m_weight(weight) { }

  double weight(const double /*progress*/) const VP_OVERRIDE;

  double getWeight() const { return m_weight; }
  void setWeight(double weight) { m_weight = weight; }
private:
  double m_weight;
};

class VISP_EXPORT vpSigmoidTemporalWeighting : public vpTemporalWeighting
{
public:
  vpSigmoidTemporalWeighting(double minWeight, double maxWeight, double location, double power)
  {
    setLocation(location);
    setSlopePower(power);
    setMinimumWeight(minWeight);
    setMaximumWeight(maxWeight);

    if (maxWeight < minWeight) {
      throw vpException(vpException::badValue, "Maximum weight should be equal or greater than the minimum weight");
    }
  }

  double getLocation() const { return m_location; }
  void setLocation(double location)
  {
    if (location < 0 || location > 1) {
      throw vpException(vpException::badValue, "Location parameter needs to be between 0 and 1");
    }
    m_location = location;
  }

  double weight(const double progress) const VP_OVERRIDE;

  double getSlopePower() const { return m_power; }
  void setSlopePower(double power) { m_power = power; }

  double getMinimumWeight() const { return m_minWeight; }
  void setMinimumWeight(double w) { m_minWeight = w; }

  double getMaximumWeight() const { return m_maxWeight; }
  void setMaximumWeight(double w) { m_maxWeight = w; }


private:
  double m_location;
  double m_power;
  double m_minWeight;
  double m_maxWeight;

};

#endif
