//! \example tutorial-dnn-object-detection-live.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/detection/vpDetectorDNNOpenCV.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(VISP_HAVE_OPENCV_DNN) && \
    (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) && defined(VISP_HAVE_NLOHMANN_JSON)

#include <nlohmann/json.hpp>
using json = nlohmann::json;

class vpJsonArgumentParser {
public:
  vpJsonArgumentParser(const std::string& description, const std::string& jsonFileArgumentName, const std::string& nestSeparator): description(description),  jsonFileArgumentName(jsonFileArgumentName),
  nestSeparator(nestSeparator) 
  {
    if(jsonFileArgumentName.empty()) {
      throw vpException(vpException::badValue, "The JSON file argument must not be empty!");
    }
  }

  std::string help() const
  {
    std::stringstream ss;
    ss << description << std::endl;
    return ss.str();
  }

  template<typename T>
  vpJsonArgumentParser& addArgument(const std::string& name, T& parameter, const bool required = true, const std::string& help = "No description") 
  {
    const auto getter = [name, this](json& j, bool create) -> json* {
      size_t pos = 0;
      json* f = &j;
      std::string token;
      std::string name_copy = name;
      std::cout << name_copy << std::endl;
      std::cout << "j = " << j.dump(4) << std::endl;
      while ((pos = name_copy.find(nestSeparator)) != std::string::npos) {
        token = name_copy.substr(0, pos);
        name_copy.erase(0, pos + nestSeparator.length());
        if(create && !f->contains(token)) {
          (*f)[token] = {};
        } else if(!f->contains(token)) {
          return nullptr;
        }
        f = &(f->at(token));
      }
      if(create && !f->contains(name_copy)) {
        (*f)[name_copy] = {};
      } else if(f->contains(name_copy)) {
        return nullptr;
      }
      f = &(f->at(name_copy));
      return f;
    };
    parsers[name] = [&parameter, required, getter, name](json& j) {
      const json* field = getter(j, false);
      if(field != nullptr) {
        if(field->empty()) {
          std::stringstream ss;
          ss << "Argument " << name << "is required, but no value was provided" << std::endl;
          throw vpException(vpException::ioError, ss.str());
        }
        field->get_to(parameter);
      }
    };
    updaters[name] = [getter, &parameter, this](json& j, const std::string& s) {
      json* field = getter(j, true);
      *field = convertCommandLineArgument(parameter, s);
    };
    return *this;
  }

  template<typename T>
  json convertCommandLineArgument(const T&, const std::string& arg) {
    json j = json::parse(arg);
    return j;
  }
  
  json convertCommandLineArgument(const std::string&, const std::string& arg) {
    json j = arg;
    return j;
  }

  void parse(int argc, const char* argv[]) 
  {
    json j;
    const std::vector<std::string> arguments(argv + 1, argv + argc);
    const auto jsonFileArgumentPos = std::find(arguments.begin(), arguments.end(), jsonFileArgumentName);
    // Load JSON file if present
    if(jsonFileArgumentPos != arguments.end()) {
      if(jsonFileArgumentPos == arguments.end() - 1) {
        throw vpException(vpException::ioError, "No JSON file was provided");
      }
      const std::string jsonFileName = *(jsonFileArgumentPos + 1);
      std::cout << jsonFileName << std::endl; 
    }
    // Parse command line arguments
    for(int i = 1; i < argc; ++i) 
    {
      const std::string arg = argv[i];
      if(parsers.find(arg) != parsers.end()) {
        if(i < argc - 1) {
          std::cout << argv[i + 1] << std::endl;
          updaters[arg](j, std::string(argv[i + 1]));
          ++i;
        } else {
          std::stringstream ss;
          ss << "Argument " << arg << " was passed but no value was provided" << std::endl;
          throw vpException(vpException::ioError, ss.str());
        }
      } else {
        std::cerr << "Unknown parameter when parsing: " << arg << std::endl;
      }
    }
    for(const auto& parser: parsers) {
      std::cout << "Key = " << parser.first << std::endl;
      parser.second(j);
    }
  }


private:
  std::string description;
  std::string jsonFileArgumentName;
  std::string nestSeparator;
  std::map<std::string, std::function<void(json&)>> parsers;
  std::map<std::string, std::function<void(json&, const std::string&)>> updaters;
};


typedef enum {
  DETECTION_CONTAINER_MAP = 0,
  DETECTION_CONTAINER_VECTOR = 1,
  DETECTION_CONTAINER_BOTH = 2,
  DETECTION_CONTAINER_COUNT = 3
} ChosenDetectionContainer;

std::string chosenDetectionContainerToString(const ChosenDetectionContainer &choice)
{
  switch (choice) {
  case DETECTION_CONTAINER_MAP:
    return "map";
  case DETECTION_CONTAINER_VECTOR:
    return "vector";
  case DETECTION_CONTAINER_BOTH:
    return "both";
  default:
    break;
  }
  return "unknown";
}

ChosenDetectionContainer chosenDetectionContainerFromString(const std::string &choiceStr)
{
  ChosenDetectionContainer choice(DETECTION_CONTAINER_COUNT);
  bool hasFoundMatch = false;
  for (unsigned int i = 0; i < DETECTION_CONTAINER_COUNT && !hasFoundMatch; i++) {
    ChosenDetectionContainer candidate = (ChosenDetectionContainer)i;
    hasFoundMatch = (chosenDetectionContainerToString(candidate) == vpIoTools::toLowerCase(choiceStr));
    if (hasFoundMatch) {
      choice = candidate;
    }
  }
  return choice;
}

std::string getAvailableDetectionContainer()
{
  std::string availableContainers("< ");
  for (unsigned int i = 0; i < DETECTION_CONTAINER_COUNT - 1; i++) {
    std::string name = chosenDetectionContainerToString((ChosenDetectionContainer)i);
    availableContainers += name + " , ";
  }
  availableContainers +=
      chosenDetectionContainerToString((ChosenDetectionContainer)(DETECTION_CONTAINER_COUNT - 1)) + " >";
  return availableContainers;
}

int main(int argc, const char *argv[])
{
  unsigned width = 640, height = 480;
  vpCameraParameters cam;
  std::string detectorModelPath = "";
  std::string detectorFramework = "onnx";
  std::string detectorConfig = "none";
  std::vector<std::string> labels;
  double detectorMeanR = 104.f, detectorMeanG = 177.f, detectorMeanB = 123.f;
  double detectorConfidenceThreshold = 0.5, detectorNmsThreshold = 0.4, detectorFilterThreshold = 0.25;
  std::string detectorTypeString = "yolov7";
  std::string videoDevice = "0";
  
  try {
    vpJsonArgumentParser parser("Single object tracking with Megapose", "--config", "/");
    parser.addArgument("width", width, true, "The image width")
          .addArgument("height", height, true, "The image height")
          .addArgument("camera", cam, true, "The camera intrinsic parameters")
          .addArgument("video-device", videoDevice, true, "Video device")
          .addArgument("detector/model-path", detectorModelPath, true, "Path to the model")
          .addArgument("detector/config", detectorConfig, true, "Path to the model configuration. Set to none if config is not required.")
          .addArgument("detector/framework", detectorFramework, true, "Detector framework")
          .addArgument("detector/type", detectorTypeString, true, "Detector type")
          .addArgument("detector/labels", labels, true, "Detection class labels")
          .addArgument("detector/mean/red", detectorMeanR, true, "Detector mean red component. Used to normalize image")
          .addArgument("detector/mean/green", detectorMeanG, true, "Detector mean green component. Used to normalize image")
          .addArgument("detector/mean/blue", detectorMeanB, true, "Detector mean red component. Used to normalize image")
          .addArgument("detector/confidenceThreshold", detectorConfidenceThreshold, true, "Detector confidence threshold. When a detection with a confidence below this threshold, it is ignored")
          .addArgument("detector/nmsThreshold", detectorNmsThreshold, true, "Detector confidence threshold. When a detection with a confidence below this threshold, it is ignored")
          .addArgument("detector/filterThreshold", detectorFilterThreshold, true)
          ;
    
    parser.parse(argc, argv);

    std::cout << width << " " << height << std::endl;


    vpDetectorDNNOpenCV::DNNResultsParsingType opt_dnn_type = vpDetectorDNNOpenCV::RESNET_10;
    double opt_dnn_scale_factor = 1.0;
    bool opt_dnn_swapRB = false;
    bool opt_step_by_step = false;
    ChosenDetectionContainer opt_dnn_containerType = DETECTION_CONTAINER_MAP;
    bool opt_verbose = false;

    cv::VideoCapture capture;
    bool hasCaptureOpeningSucceeded;
    if (vpMath::isNumber(videoDevice)) {
      hasCaptureOpeningSucceeded = capture.open(std::atoi(videoDevice.c_str()));
    } else {
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
    vpDetectorDNNOpenCV dnn(netConfig, opt_dnn_type);
    dnn.readNet(detectorModelPath, detectorConfig, detectorFramework);
    dnn.setMean(detectorMeanR, detectorMeanG, detectorMeanB);
    dnn.setScaleFactor(opt_dnn_scale_factor);
    dnn.setSwapRB(opt_dnn_swapRB);
    //! [DNN params]

    cv::Mat frame;
    while (true) {
      capture >> frame;
      if (frame.empty())
        break;

      if (I.getSize() == 0) {
        vpImageConvert::convert(frame, I);
        d.init(I);
        vpDisplay::setTitle(I, "DNN object detection");
        if (opt_verbose) {
          std::cout << "Process image: " << I.getWidth() << " x " << I.getHeight() << std::endl;
        }
      } else {
        vpImageConvert::convert(frame, I);
      }
      if (opt_verbose) {
        std::cout << "Process new image" << std::endl;
      }

      vpDisplay::display(I);

      if (opt_dnn_containerType == DETECTION_CONTAINER_MAP || opt_dnn_containerType == DETECTION_CONTAINER_BOTH) {
        double t = vpTime::measureTimeMs();
        //! [DNN object detection map mode]
        std::map<std::string, std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D> > detections;
        dnn.detect(frame, detections);
        //! [DNN object detection map mode]
        t = vpTime::measureTimeMs() - t;

        //! [DNN class ids and confidences map mode]
        for (auto key_val : detections) {
          if (opt_verbose) {
            std::cout << "  Class name      : " << key_val.first << std::endl;
          }
          for (vpDetectorDNNOpenCV::DetectedFeatures2D detection : key_val.second) {
            if (opt_verbose) {
              std::cout << "  Bounding box    : " << detection.getBoundingBox() << std::endl;
              std::cout << "  Class Id        : " << detection.getClassId() << std::endl;
              if (detection.getClassName())
                std::cout << "  Class name      : " << detection.getClassName().value() << std::endl;
              std::cout << "  Confidence score: " << detection.getConfidenceScore() << std::endl;
            }
            detection.display(I);
          }
        }
        //! [DNN class ids and confidences map mode]

        std::ostringstream oss_map;
        oss_map << "Detection time (map): " << t << " ms";
        if (opt_verbose) {
          // Displaying timing result in console
          std::cout << "  " << oss_map.str() << std::endl;
        }
        // Displaying timing result on the image
        vpDisplay::displayText(I, 60, 20, oss_map.str(), vpColor::red);
      }

      if (opt_dnn_containerType == DETECTION_CONTAINER_VECTOR || opt_dnn_containerType == DETECTION_CONTAINER_BOTH) {
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
      }

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
        } else if (button == vpMouseButton::button3) {
          // Right click => stop the program
          break;
        }
      }
    }

  } catch (const vpException &e) {
    std::cout << e.what() << std::endl;
  }
}
#else
int main() {
  std::cout << "Compile ViSP with the DNN tracker module, the JSON 3rd party library and the OpenCV detection module" << std::endl;
  return EXIT_SUCCESS;
}

#endif