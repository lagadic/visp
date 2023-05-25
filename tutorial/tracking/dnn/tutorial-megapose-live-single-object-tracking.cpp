//! \example tutorial-dnn-object-detection-live.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/detection/vpDetectorDNNOpenCV.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/dnn_tracker/vpMegaPose.h>
#include <visp3/dnn_tracker/vpMegaPoseTracker.h>


#if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(VISP_HAVE_OPENCV_DNN) && \
    (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) && defined(VISP_HAVE_NLOHMANN_JSON)

#include <nlohmann/json.hpp>
using json = nlohmann::json;

class vpJsonArgumentParser
{
public:
  vpJsonArgumentParser(const std::string &description, const std::string &jsonFileArgumentName, const std::string &nestSeparator) : description(description), jsonFileArgumentName(jsonFileArgumentName),
    nestSeparator(nestSeparator)
  {
    if (jsonFileArgumentName.empty()) {
      throw vpException(vpException::badValue, "The JSON file argument must not be empty!");
    }

    helpers[jsonFileArgumentName] = []() -> std::string {
      return "Path to the JSON configuration file. Values in this files are loaded, and can be overriden by command line arguments.\nOptional";
    };
  }


  std::string help() const
  {
    std::stringstream ss;

    ss << "Program description: " << description << std::endl;
    ss << "Arguments: " << std::endl;
    unsigned spacesBetweenArgAndDescription = 0;
    for(const auto& helper: helpers) {
      if(helper.first.size() > spacesBetweenArgAndDescription) {
        spacesBetweenArgAndDescription = helper.first.size();
      }
    }
    spacesBetweenArgAndDescription += 4;
    
    for(const auto& helper: helpers) {
      std::stringstream argss(helper.second());
      std::string line;
      bool first = true;
      while (getline(argss, line, '\n')) {
        const unsigned lineSpace = first ? spacesBetweenArgAndDescription - helper.first.size() : spacesBetweenArgAndDescription;
        const std::string spaceBetweenArgAndDescription(lineSpace, ' ');
        if(first) {
          ss << "\t" << helper.first << spaceBetweenArgAndDescription << line << std::endl;
        } else {
          ss << "\t" << spaceBetweenArgAndDescription << line << std::endl;
        }
        first = false;

      }
      ss << std::endl;
    }
    ss << "Example JSON configuration file: " << std::endl << std::endl;
    ss << exampleJson.dump(2) << std::endl;
    return ss.str();
  }

  template<typename T>
  vpJsonArgumentParser &addArgument(const std::string &name, T &parameter, const bool required = true, const std::string &help = "No description")
  {
    const auto getter = [name, this](json &j, bool create) -> json * {
      size_t pos = 0;
      json *f = &j;
      std::string token;
      std::string name_copy = name;
      std::cout << name_copy << std::endl;
      std::cout << "j = " << j.dump(4) << std::endl;
      std::cout << "create = " << create << std::endl;
      while ((pos = name_copy.find(nestSeparator)) != std::string::npos) {
        std::cout << "start iter" << std::endl;
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
    parsers[name] = [&parameter, required, getter, name](json &j) {
      const json *field = getter(j, false);
      if (field != nullptr) {
        if (field->empty()) {
          std::stringstream ss;
          ss << "Argument " << name << "is required, but no value was provided" << std::endl;
          throw vpException(vpException::ioError, ss.str());
        }
        field->get_to(parameter);
      }
    };
    updaters[name] = [getter, &parameter, this](json &j, const std::string &s) {
      json *field = getter(j, true);
      *field = convertCommandLineArgument(parameter, s);
    };
    helpers[name] = [help, parameter, required]() -> std::string {
      std::stringstream ss;
      json repr = parameter;
      ss << help << std::endl << "Default: " << repr;
      if(required) {
        ss << std::endl << "Required";
      } else {
        ss << std::endl << "Optional";
      }
      return ss.str();
    };
    
    json* exampleField = getter(exampleJson, true);
    *exampleField = parameter;

    return *this;
  }

  template<typename T>
  json convertCommandLineArgument(const T &, const std::string &arg)
  {
    json j = json::parse(arg);
    return j;
  }

  json convertCommandLineArgument(const std::string &, const std::string &arg)
  {
    json j = arg;
    return j;
  }

  void parse(int argc, const char *argv [])
  {
    json j;
    const std::vector<std::string> arguments(argv + 1, argv + argc);
    std::vector<unsigned> ignoredArguments;
    const auto jsonFileArgumentPos = std::find(arguments.begin(), arguments.end(), jsonFileArgumentName);
    // Load JSON file if present
    if (jsonFileArgumentPos != arguments.end()) {
      ignoredArguments.push_back(jsonFileArgumentPos - arguments.begin() + 1);
      ignoredArguments.push_back(jsonFileArgumentPos - arguments.begin() + 2);
      
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
      if(std::find(ignoredArguments.begin(), ignoredArguments.end(), i) != ignoredArguments.end()) {
        continue;
      }
      if(arg == "-h" || "--help") {
        std::cout << help() << std::endl;
        exit(1);
      }

      if (parsers.find(arg) != parsers.end()) {
        if (i < argc - 1) {
          std::cout << argv[i + 1] << std::endl;
          updaters[arg](j, std::string(argv[i + 1]));
          ++i;
        }
        else {
          std::stringstream ss;
          ss << "Argument " << arg << " was passed but no value was provided" << std::endl;
          throw vpException(vpException::ioError, ss.str());
        }
      }
      else {
        std::cerr << "Unknown parameter when parsing: " << arg << std::endl;
      }
    }
    for (const auto &parser : parsers) {
      parser.second(j);
    }
  }


private:
  std::string description;
  std::string jsonFileArgumentName;
  std::string nestSeparator;
  std::map<std::string, std::function<void(json &)>> parsers;
  std::map<std::string, std::function<void(json &, const std::string &)>> updaters;
  std::map<std::string, std::function<std::string()>> helpers;
  json exampleJson;
  
};


/**
 * Add the megapose rendering on top of the actual image I.
 * Require I and overlay to be of the same size
*/
void overlayRender(vpImage<vpRGBa>& I, const vpImage<vpRGBa>& overlay) {
  const vpRGBa black = vpRGBa(0,0,0);
  for(unsigned int i = 0; i < I.getHeight(); ++i) {
    for(unsigned int j = 0; j < I.getWidth(); ++j) {
      if(overlay[i][j] != black) {
        I[i][j] = overlay[i][j];
      }
    }
  }
}

int main(int argc, const char *argv [])
{
  unsigned width = 640, height = 480;
  vpCameraParameters cam;
  std::string detectorModelPath = "path/to/model.onnx";
  std::string detectorFramework = "onnx";
  std::string detectorConfig = "none";
  std::vector<std::string> labels = {"cube"};
  double detectorMeanR = 104.f, detectorMeanG = 177.f, detectorMeanB = 123.f;
  double detectorConfidenceThreshold = 0.5, detectorNmsThreshold = 0.4, detectorFilterThreshold = 0.25;
  std::string detectorTypeString = "yolov7";
  std::string videoDevice = "0";
  std::string megaposeAddress = "127.0.0.1";
  unsigned megaposePort = 5555;
  int refinerIterations = 1;
  int coarseNumSamples = 576;
  try {
    vpJsonArgumentParser parser("Single object tracking with Megapose", "--config", "/");
    parser.addArgument("width", width, true, "The image width")
      .addArgument("height", height, true, "The image height")
      .addArgument("camera", cam, true, "The camera intrinsic parameters. Should correspond to a perspective projection model without distortion.")
      .addArgument("video-device", videoDevice, true, "Video device")
      
      .addArgument("detector/model-path", detectorModelPath, true, "Path to the model")
      .addArgument("detector/config", detectorConfig, true, "Path to the model configuration. Set to none if config is not required.")
      .addArgument("detector/framework", detectorFramework, true, "Detector framework")
      .addArgument("detector/type", detectorTypeString, true, "Detector type")
      .addArgument("detector/labels", labels, true, "Detection class labels")
      .addArgument("detector/mean/red", detectorMeanR, true, "Detector mean red component. Used to normalize image")
      .addArgument("detector/mean/green", detectorMeanG, true, "Detector mean green component. Used to normalize image")
      .addArgument("detector/mean/blue", detectorMeanB, true, "Detector mean red component. Used to normalize image")
      .addArgument("detector/confidenceThreshold", detectorConfidenceThreshold, false, "Detector confidence threshold. When a detection with a confidence below this threshold, it is ignored")
      .addArgument("detector/nmsThreshold", detectorNmsThreshold, false, "Detector confidence threshold. When a detection with a confidence below this threshold, it is ignored")
      .addArgument("detector/filterThreshold", detectorFilterThreshold, false)

      .addArgument("megapose/address", megaposeAddress, true, "IP address of the megapose server.")
      .addArgument("megapose/port", megaposePort, true, "Port on which the megapose server listens for connections.")
      .addArgument("megapose/refinerIterations", refinerIterations, false, "Number of megapose refiner model iterations. A higher count may lead to better accuracy, at the cost of more processing time")
      .addArgument("megapose/initialisationNumSamples", coarseNumSamples, false, "Number of megapose renderings used for the initial pose estimation.");

    parser.parse(argc, argv);
    std::cout << "Finished parsing" << std::endl;
    std::cout << width << " " << height << std::endl;


    if(cam.get_projModel() != vpCameraParameters::perspectiveProjWithoutDistortion) {
      throw vpException(vpException::badValue, "The camera projection model should be without distortion, as other models are ignored by Megapose");
    }

    vpDetectorDNNOpenCV::DNNResultsParsingType detectorType = vpDetectorDNNOpenCV::dnnResultsParsingTypeFromString(detectorTypeString);
    double opt_dnn_scale_factor = 1.0;
    bool opt_dnn_swapRB = false;
    bool opt_step_by_step = false;
    bool opt_verbose = false;

    cv::VideoCapture capture;
    bool hasCaptureOpeningSucceeded;
    if (vpMath::isNumber(videoDevice)) {
      hasCaptureOpeningSucceeded = capture.open(std::atoi(videoDevice.c_str()));
    }
    else {
      hasCaptureOpeningSucceeded = capture.open(videoDevice);
    }
    if (!hasCaptureOpeningSucceeded) {
      std::cout << "Capture from camera: " << videoDevice << " didn't work" << std::endl;
      return EXIT_FAILURE;
    }

    vpImage<vpRGBa> I;
#if defined(VISP_HAVE_X11)
    vpDisplayX d;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d;
#endif
    d.setDownScalingFactor(vpDisplay::SCALE_AUTO);

    //! [DNN params]
    vpDetectorDNNOpenCV::NetConfig netConfig(detectorConfidenceThreshold, detectorNmsThreshold, labels,
                                             cv::Size(width, height), detectorFilterThreshold);
    vpDetectorDNNOpenCV dnn(netConfig, detectorType);
    dnn.readNet(detectorModelPath, detectorConfig, detectorFramework);
    dnn.setMean(detectorMeanR, detectorMeanG, detectorMeanB);
    dnn.setScaleFactor(opt_dnn_scale_factor);
    dnn.setSwapRB(opt_dnn_swapRB);
    //! [DNN params]

    //vpMegaPose megapose(megaposeAddress, megaposePort, cam, height, width);

    cv::Mat frame;
    while (true) {
      capture >> frame;
      if (frame.empty())
        break;

      if (I.getSize() == 0) {
        vpImageConvert::convert(frame, I);
        d.init(I);
        vpDisplay::setTitle(I, "Megapose object pose estimation");
        if (opt_verbose) {
          std::cout << "Process image: " << I.getWidth() << " x " << I.getHeight() << std::endl;
        }
      }
      else {
        vpImageConvert::convert(frame, I);
      }
      if (opt_verbose) {
        std::cout << "Process new image" << std::endl;
      }

      vpDisplay::display(I);
      double t_vector = vpTime::measureTimeMs();
      //! [DNN object detection vector mode]
      std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D> detections_vec;
      dnn.detect(frame, detections_vec);
      //! [DNN object detection vector mode]
      t_vector = vpTime::measureTimeMs() - t_vector;

      


      //! [DNN class ids and confidences vector mode]
      for (auto detection : detections_vec) {
        if (opt_verbose) {
          std::cout << "  Bounding box    : " << detection.getBoundingBox() << std::endl;
          std::cout << "  Class Id        : " << detection.getClassId() << std::endl;
          std::optional<std::string> classname_opt = detection.getClassName();
          std::cout << "  Class name      : " << (classname_opt ? *classname_opt : "Not known") << std::endl;
          std::cout << "  Confidence score: " << detection.getConfidenceScore() << std::endl;
        }
        detection.display(I);
      }
        //! [DNN class ids and confidences vector mode]

        std::ostringstream oss_vec;
        oss_vec << "Detection time (vector): " << t_vector << " ms";
        if (opt_verbose) {
          // Displaying timing result in console
          std::cout << "  " << oss_vec.str() << std::endl;
        }
        // Displaying timing result on the image
        vpDisplay::displayText(I, 80, 20, oss_vec.str(), vpColor::red);


      // // UI display
      if (opt_step_by_step) {
        vpDisplay::displayText(I, 20, 20, "Left click to display next image", vpColor::red);
      }
      vpDisplay::displayText(I, 40, 20, "Right click to quit", vpColor::red);

      vpDisplay::flush(I);
      vpMouseButton::vpMouseButtonType button;

      if (vpDisplay::getClick(I, button, opt_step_by_step)) {
        if (button == vpMouseButton::button1) {
          // Left click => next image
          continue;
        }
        else if (button == vpMouseButton::button3) {
          // Right click => stop the program
          break;
        }
      }
    }

  }
  catch (const vpException &e) {
    std::cout << e.what() << std::endl;
  }
}
#else
int main()
{
  std::cout << "Compile ViSP with the DNN tracker module, the JSON 3rd party library and the OpenCV detection module" << std::endl;
  return EXIT_SUCCESS;
}

#endif