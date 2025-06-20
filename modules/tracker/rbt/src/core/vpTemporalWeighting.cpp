#include <visp3/rbt/vpTemporalWeighting.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif


#if defined(VISP_HAVE_NLOHMANN_JSON)
std::shared_ptr<vpTemporalWeighting> vpTemporalWeighting::parseTemporalWeighting(const nlohmann::json &j)
{
  if (j.is_number()) {
    return std::make_shared<vpFixedTemporalWeighting>(j.get<double>());
  }
  else if (j.is_object()) {
    bool slopeIncreasing = j.value("increasing", true);
    double slopePower = j.value("slopePower", 1.0); // Linear increase

    if ((!slopeIncreasing && slopePower > 0.0) || (slopeIncreasing && slopePower < 0.0)) {
      slopePower = -slopePower;
    }
    return std::make_shared<vpSigmoidTemporalWeighting>(
      j.value("minWeight", 0.0),
      j.value("maxWeight", 1.0),
      j.value("midpointLocation", 0.5),
      slopePower
    );

  }
  else {
    throw vpException(vpException::badValue, "Wrong weighting type");
  }

}
std::shared_ptr<vpTemporalWeighting> vpTemporalWeighting::parseTemporalWeightingRawJson(const std::string &s)
{
  nlohmann::json j = nlohmann::json::parse(s);
  return parseTemporalWeighting(j);
}


#endif


double vpFixedTemporalWeighting::weight(const double /*progress*/) const
{
  return m_weight;
}

double vpSigmoidTemporalWeighting::weight(const double progress) const
{

  double w = 1.0;
  if (progress == 0.0 || m_location == 1.0) {
    if (m_power > 0.0) {
      w = 0.0;
    }
    else {
      w = 1.0;
    }
  }
  else {
    double f1 = m_location / progress;
    double f2 = (1.0 - progress) / (1.0 - m_location);

    w = (1.0 / (1.0 + (std::pow(f1 * f2, m_power))));
  }
  return m_minWeight + w * (m_maxWeight - m_minWeight);
}
