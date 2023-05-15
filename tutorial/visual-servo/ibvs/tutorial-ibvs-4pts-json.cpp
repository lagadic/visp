/*! \example tutorial-ibvs-4pts-json.cpp */
#include <iostream>
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>


#include <nlohmann/json.hpp>
using json = nlohmann::json;

//! [Enum]
enum InteractionMatrixTypeSubset {
  UNKNOWN = -1,
  CURRENT,
  DESIRED,
  MEAN
};
NLOHMANN_JSON_SERIALIZE_ENUM(InteractionMatrixTypeSubset, {
  {UNKNOWN, nullptr}, // Default value if the json string is not in "current", "desired" or "mean"
  {CURRENT, "current"},
  {DESIRED, "desired"},
  {MEAN, "mean"}}
);
//! [Enum]


//! [Arguments]
struct Arguments {
  // Default values
  Arguments() :
    lambda(0.5), cdMo(0, 0, 0.75, 0, 0, 0),
    cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50)),
    samplingTime(0.04), errorThreshold(0.0001), interactionMatrixType(CURRENT)
  {
  }
  vpServo::vpServoIteractionMatrixType getInteractionMatrixType() const
  {
    switch (interactionMatrixType) {
    case CURRENT:
      return vpServo::CURRENT;
    case DESIRED:
      return vpServo::DESIRED;
    case MEAN:
      return vpServo::MEAN;
    default:
      throw vpException(vpException::badValue, "Unexpected value");
    }
    return vpServo::CURRENT;
  }
  double lambda; // Control law gain
  vpHomogeneousMatrix cdMo; // Target (desired) camera pose
  vpHomogeneousMatrix cMo; // Initial camera pose
  double samplingTime; // Robot sampling time
  double errorThreshold; // Error threshold. Once error is below, consider servoing as successful
  InteractionMatrixTypeSubset interactionMatrixType;

};
//! [Arguments]

//!  [Arguments conversion]
// Read script arguments from JSON. All values are optional and if an argument is not present,
// the default value defined in the constructor is kept
void from_json(const json& j, Arguments& a)
{
  a.lambda = j.value("lambda", a.lambda);
  if (a.lambda <= 0) {
    throw vpException(vpException::badValue, "Lambda should be > 0");
  }

  a.cMo = j.value("cMo", a.cMo);
  a.cdMo = j.value("cdMo", a.cdMo);

  a.samplingTime = j.value("samplingTime", a.samplingTime);
  if (a.samplingTime <= 0) {
    throw vpException(vpException::badValue, "Sampling time should be > 0");
  }

  a.errorThreshold = j.value("errorThreshold", a.errorThreshold);
  if (a.errorThreshold <= 0) {
    throw vpException(vpException::badValue, "Error threshold should be > 0");
  }

  a.interactionMatrixType = j.value("interactionMatrix", a.interactionMatrixType);
  if (a.interactionMatrixType == UNKNOWN) {
    throw vpException(vpException::badValue, "Unknown interaction matrix type defined in JSON");
  }
}

void to_json(json& j, const Arguments& a)
{
  j = json {
    {"lambda", a.lambda},
    {"cMo", a.cMo},
    {"cdMo", a.cdMo},
    {"errorThreshold", a.errorThreshold},
    {"samplingTime", a.samplingTime},
    {"interactionMatrix", a.interactionMatrixType}
  };
}
//!  [Arguments conversion]

//! [JSON input conversion]
Arguments readArguments(const std::string& path)
{
  Arguments a;

  if (!path.empty()) {
    std::ifstream file(path);
    if (!file.good()) {
      std::stringstream ss;
      ss << "Problem opening file " << path << ". Make sure it exists and is readable" << std::endl;
      throw vpException(vpException::badValue, ss.str());
    }
    json j;
    try {
      j = json::parse(file);
    }
    catch (json::parse_error& e) {
      std::stringstream msg;
      msg << "Could not parse JSON file : \n";

      msg << e.what() << std::endl;
      msg << "Byte position of error: " << e.byte;
      throw vpException(vpException::ioError, msg.str());
    }
    a = j; // Call from_json(const json& j, Argument& a) to read json into arguments a
    file.close();
  }
  else {
    std::cout << "Using default arguments. Try using a JSON file to set the arguments of the visual servoing." << std::endl;
  }
  return a;
}
//! [JSON input conversion]

//! [Custom ViSP object conversion]
void to_json(json& j, const vpFeaturePoint& p) {
  j = json {
    {"x", p.get_x()},
    {"y", p.get_y()},
    {"z", p.get_Z()}
  };
}
//! [Custom ViSP object conversion]


//! [Result structure]
struct ServoingExperimentData {
  ServoingExperimentData(const Arguments& arguments, const std::vector<vpFeaturePoint>& desiredFeatures) : arguments(arguments), desiredFeatures(desiredFeatures) {}

  void onIter(const vpHomogeneousMatrix& cMo, const double errorNorm, const std::vector<vpFeaturePoint>& points,
              const vpColVector& velocity, const vpMatrix& interactionMatrix)
  {
    vpPoseVector r(cMo);
    trajectory.push_back(r);
    errorNorms.push_back(errorNorm);
    points3D.push_back(points);
    velocities.push_back(velocity);
    interactionMatrices.push_back(interactionMatrix);
  }

  Arguments arguments;
  std::vector<vpFeaturePoint> desiredFeatures;
  std::vector<vpPoseVector> trajectory;
  std::vector<double> errorNorms;
  std::vector<std::vector<vpFeaturePoint>> points3D;
  std::vector<vpColVector> velocities;
  std::vector<vpMatrix> interactionMatrices;
};

void to_json(json& j, const ServoingExperimentData& res)
{
  j = json {
    {"parameters", res.arguments},
    {"trajectory", res.trajectory},
    {"errorNorm", res.errorNorms},
    {"features", res.points3D},
    {"desiredFeatures", res.desiredFeatures},
    {"velocities", res.velocities},
    {"interactionMatrices", res.interactionMatrices}
  };
}
//! [Result structure]

//! [write json to file]
void saveResults(const ServoingExperimentData& results, const std::string& path)
{
  std::ofstream file(path);
  const json j = results;
  file << j.dump(4);
  file.close();
}
//! [write json to file]

int main(int argc, char* argv[])
{
  std::string arguments_path = "";
  std::string output_path = "";
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--settings" && i + 1 < argc) {
      arguments_path = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--output" && i + 1 < argc) {
      output_path = std::string(argv[i + 1]);
    }
  }

  if (output_path.empty()) {
    std::cerr << "JSON output path must be specified" << std::endl;
    return EXIT_FAILURE;
  }
  const Arguments args = readArguments(arguments_path);

  try {
    vpHomogeneousMatrix cdMo = args.cdMo;
    vpHomogeneousMatrix cMo = args.cMo;
    std::cout << cdMo << std::endl;

    vpPoint point[4];
    point[0].setWorldCoordinates(-0.1, -0.1, 0);
    point[1].setWorldCoordinates(0.1, -0.1, 0);
    point[2].setWorldCoordinates(0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);

    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(args.getInteractionMatrixType());
    task.setLambda(args.lambda);

    vpFeaturePoint p[4], pd[4];
    std::vector<vpFeaturePoint> features;
    features.resize(4);
    for (unsigned int i = 0; i < 4; i++) {
      point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
      task.addFeature(p[i], pd[i]);
      features[i] = pd[i];
    }
    ServoingExperimentData results(args, features);

    vpHomogeneousMatrix wMc, wMo;
    vpSimulatorCamera robot;
    robot.setSamplingTime(args.samplingTime);
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    unsigned int iter = 0;
    while (1) {
      robot.getPosition(wMc);
      cMo = wMc.inverse() * wMo;
      for (unsigned int i = 0; i < 4; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
        features[i] = p[i];
      }
      vpColVector v = task.computeControlLaw();
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      const double errorNorm = task.getError().sumSquare();
      results.onIter(cMo, errorNorm, features, v, task.getInteractionMatrix());
      if (errorNorm < args.errorThreshold)
        break;

      vpTime::wait(100);

      iter++;
    }
    std::cout << "Convergence in " << iter << " iterations" << std::endl;
    saveResults(results, output_path);
  }
  catch (const vpException& e) {
    std::cout << "Caught an exception: " << e << std::endl;
  }
}
#else
int main()
{
  std::cerr << "Cannot run tutorial: ViSP is not built with JSON integration. Install the JSON library and recompile ViSP" << std::endl;
}
#endif