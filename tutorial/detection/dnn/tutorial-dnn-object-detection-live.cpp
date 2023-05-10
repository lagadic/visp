//! \example tutorial-dnn-object-detection-live.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorDNNOpenCV.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

typedef enum ChosenDetectionContainer
{
  MAP    = 0,
  VECTOR = 1,
  BOTH   = 2,
  COUNT  = 3
} ChosenDetectionContainer;

std::string chosenDetectionContainerToString(const ChosenDetectionContainer& choice)
{
  switch(choice)
  {
    case MAP:
      return "map";
    case VECTOR:
      return "vector";
    case BOTH:
      return "both";
    default:
      break;
  }
  return "unknown";
}

ChosenDetectionContainer chosenDetectionContainerFromString(const std::string& choiceStr)
{
  ChosenDetectionContainer choice(COUNT);
  bool hasFoundMatch = false;
  for(unsigned int i = 0; i < ChosenDetectionContainer::COUNT && !hasFoundMatch; i++)
  {
    ChosenDetectionContainer candidate = (ChosenDetectionContainer)i;
    hasFoundMatch = (chosenDetectionContainerToString(candidate) == vpIoTools::toLowerCase(choiceStr));
    if(hasFoundMatch)
    {
      choice = candidate;
    }
  }
  return choice;
}

std::string getAvailableDetectionContainer()
{
  std::string availableContainers("< ");
  for(unsigned int i = 0; i < ChosenDetectionContainer::COUNT - 1; i++)
  {
    std::string name = chosenDetectionContainerToString((ChosenDetectionContainer) i);
    availableContainers += name + " , ";
  }
  availableContainers += chosenDetectionContainerToString((ChosenDetectionContainer) (ChosenDetectionContainer::COUNT - 1)) + " >";
  return availableContainers;
}

int main(int argc, const char *argv[])
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(VISP_HAVE_OPENCV_DNN) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
  try {
    std::string opt_device("0");
    //! [OpenCV DNN face detector]
    std::string opt_dnn_model = "opencv_face_detector_uint8.pb";
    std::string opt_dnn_config = "opencv_face_detector.pbtxt";
    std::string opt_dnn_framework = "none";
    std::string opt_dnn_label_file = "";
    vpDetectorDNNOpenCV::DNNResultsParsingType opt_dnn_type = vpDetectorDNNOpenCV::RESNET_10;
    //! [OpenCV DNN face detector]
    int opt_dnn_width = 300, opt_dnn_height = 300;
    double opt_dnn_meanR = 104.0, opt_dnn_meanG = 177.0, opt_dnn_meanB = 123.0;
    double opt_dnn_scale_factor = 1.0;
    bool opt_dnn_swapRB = false;
    bool opt_step_by_step = false;
    float opt_dnn_confThresh = 0.5f;
    float opt_dnn_nmsThresh = 0.4f;
    double opt_dnn_filterThresh = 0.25;
    ChosenDetectionContainer opt_dnn_containerType = ChosenDetectionContainer::MAP;
    bool opt_verbose = false;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--device" && i + 1 < argc) {
        opt_device = std::string(argv[++i]);
      } else if (std::string(argv[i]) == "--step-by-step") {
        opt_step_by_step = true;
      } else if (std::string(argv[i]) == "--model" && i + 1 < argc) {
        opt_dnn_model = std::string(argv[++i]);
      } else if (std::string(argv[i]) == "--type" && i + 1 < argc) {
        opt_dnn_type =  vpDetectorDNNOpenCV::dnnResultsParsingTypeFromString(std::string(argv[++i]));
      } else if (std::string(argv[i]) == "--config" && i + 1 < argc) {
        opt_dnn_config = std::string(argv[++i]);
        if(opt_dnn_config.find("none") != std::string::npos)
        {
          opt_dnn_config = std::string();
        }
      } else if (std::string(argv[i]) == "--framework" && i + 1 < argc) {
        opt_dnn_framework = std::string(argv[++i]);
        if(opt_dnn_framework.find("none") != std::string::npos)
        {
          opt_dnn_framework = std::string();
        }
      } else if (std::string(argv[i]) == "--width" && i + 1 < argc) {
        opt_dnn_width = atoi(argv[++i]);
      } else if (std::string(argv[i]) == "--height" && i + 1 < argc) {
        opt_dnn_height = atoi(argv[++i]);
      } else if (std::string(argv[i]) == "--mean" && i + 3 < argc) {
        opt_dnn_meanR = atof(argv[++i]);
        opt_dnn_meanG = atof(argv[++i]);
        opt_dnn_meanB = atof(argv[++i]);
      } else if (std::string(argv[i]) == "--scale" && i + 1 < argc) {
        opt_dnn_scale_factor = atof(argv[++i]);
      } else if (std::string(argv[i]) == "--swapRB") {
        opt_dnn_swapRB = true;
      } else if (std::string(argv[i]) == "--confThresh" && i + 1 < argc) {
        opt_dnn_confThresh = (float)atof(argv[++i]);
      } else if (std::string(argv[i]) == "--nmsThresh" && i + 1 < argc) {
        opt_dnn_nmsThresh = (float)atof(argv[++i]);
      } else if (std::string(argv[i]) == "--filterThresh" && i + 1 < argc) {
        opt_dnn_filterThresh = atof(argv[++i]);
      } else if (std::string(argv[i]) == "--labels" && i + 1 < argc) {
        opt_dnn_label_file = std::string(argv[++i]);
      } else if (std::string(argv[i]) == "--container" && i + 1 < argc) {
        opt_dnn_containerType = chosenDetectionContainerFromString(std::string(argv[++i]));
      } else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
        opt_verbose = true;
      } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nSYNOPSIS " << std::endl
                  << argv[0]
                  << " [--device <video>]"
                  << " [--model <dnn weights file>]"
                  << " [--type <dnn type>]"
                  << " [--config <dnn config file]"
                  << " [--framework <name>]"
                  << " [--width <blob width>] [--height <blob height>]"
                  << " [--mean <meanR meanG meanB>]"
                  << " [--scale <scale factor>]"
                  << " [--swapRB]"
                  << " [--confThresh <threshold>]"
                  << " [--nmsThresh <threshold>]"
                  << " [--filterThresh <threshold>]"
                  << " [--labels <file>]"
                  << " [--container <type>]"
                  << " [--step-by-step]"
                  << " [--verbose, -v]"
                  << " [--help, -h]"
                  << std::endl;
        std::cout << "\nOPTIONS " << std::endl
                  << "  --device <video>" << std::endl
                  << "      Camera device number or video name used to stream images." << std::endl
                  << "      To use the first camera found on the bus set 0. On Ubuntu setting 0" << std::endl
                  << "      will use /dev/video0 device. To use a video simply put the name of" << std::endl
                  << "      the video, like \"path/my-video.mp4\" or \"path/image-\%04d.png\"" << std::endl
                  << "      if your video is a sequence of images." << std::endl
                  << "      Default: " << opt_device << std::endl
                  << std::endl
                  << "  --model <dnn weights file>" << std::endl
                  << "      Path to dnn network trained weights." << std::endl
                  << "      Default: " << opt_dnn_model << std::endl
                  << std::endl
                  << "  --type <dnn type>" << std::endl
                  << "      Type of dnn network. Admissible values are in " << std::endl
                  << "      " << vpDetectorDNNOpenCV::getAvailableDnnResultsParsingTypes() << std::endl
                  << "      Default: " << opt_dnn_type << std::endl
                  << std::endl
                  << "  --config <dnn config file>" << std::endl
                  << "      Path to dnn network config file or \"none\" not to use one. " << std::endl
                  << "      Default: " << opt_dnn_config << std::endl
                  << std::endl
                  << "  --framework <name>" << std::endl
                  << "      Framework name or \"none\" not to specify one. " << std::endl
                  << "      Default: " << opt_dnn_framework << std::endl
                  << std::endl
                  << "  --width <blob width>" << std::endl
                  << "      Input images will be resized to this width. " << std::endl
                  << "      Default: " << opt_dnn_width << std::endl
                  << std::endl
                  << "  --height <blob height>" << std::endl
                  << "      Input images will be resized to this height. " << std::endl
                  << "      Default: " << opt_dnn_height << std::endl
                  << std::endl
                  << "  --mean <meanR meanG meanB>" << std::endl
                  << "      Mean RGB subtraction values. " << std::endl
                  << "      Default: " << opt_dnn_meanR << " " << opt_dnn_meanG << " " << opt_dnn_meanB << std::endl
                  << std::endl
                  << "  --scale <scale factor>" << std::endl
                  << "      Scale factor used to normalize the range of pixel values. " << std::endl
                  << "      Default: " << opt_dnn_scale_factor << std::endl
                  << std::endl
                  << "  --swapRB" << std::endl
                  << "      When used this option allows to swap Red and Blue channels. " << std::endl
                  << std::endl
                  << "  --confThresh <threshold>" << std::endl
                  << "      Confidence threshold. " << std::endl
                  << "      Default: " << opt_dnn_confThresh << std::endl
                  << std::endl
                  << "  --nmsThresh <threshold>" << std::endl
                  << "      Non maximum suppression threshold. " << std::endl
                  << "      Default: " << opt_dnn_nmsThresh << std::endl
                  << std::endl
                  << "  --filterThresh <threshold >" << std::endl
                  << "      Filter threshold. Set 0. to disable." << std::endl
                  << "      Default: " << opt_dnn_filterThresh << std::endl
                  << std::endl
                  << "  --labels <file>" << std::endl
                  << "      Path to label file either in txt or yaml format. Keep empty if unknown." << std::endl
                  << "      Default: \"" << opt_dnn_label_file << "\"" << std::endl
                  << std::endl
                  << "  --container <type>" << std::endl
                  << "      Container type in " << getAvailableDetectionContainer() << std::endl
                  << "      Default: " << chosenDetectionContainerToString(opt_dnn_containerType) << std::endl
                  << std::endl
                  << "  --step-by-step" << std::endl
                  << "      Enable step by step mode, waiting for a user click to process next image." << std::endl
                  << std::endl
                  << "  --verbose, -v" << std::endl
                  << "      Enable verbose mode." << std::endl
                  << std::endl
                  << "  --help, -h" << std::endl
                  << "      Display this helper message." << std::endl
                  << std::endl;
        return EXIT_SUCCESS;
      }
    }

    std::cout << "Video device         : " << opt_device << std::endl;
    std::cout << "Model                : " << opt_dnn_model << std::endl;
    std::cout << "Type                 : " << vpDetectorDNNOpenCV::dnnResultsParsingTypeToString(opt_dnn_type) << std::endl;
    std::cout << "Config               : " << (opt_dnn_config.empty() ? "\"None\"" : opt_dnn_config) << std::endl;
    std::cout << "Framework            : " << (opt_dnn_framework.empty() ? "\"None\"" : opt_dnn_framework) << std::endl;
    std::cout << "Label file (optional): " << (opt_dnn_label_file.empty() ? "None" : opt_dnn_label_file)  << std::endl;
    std::cout << "Width x Height       : " << opt_dnn_width << " x " << opt_dnn_height << std::endl;
    std::cout << "Mean RGB             : " << opt_dnn_meanR << " " << opt_dnn_meanG << " " << opt_dnn_meanB << std::endl;
    std::cout << "Scale                : " << opt_dnn_scale_factor << std::endl;
    std::cout << "Swap RB?             : " << opt_dnn_swapRB << std::endl;
    std::cout << "Confidence threshold : " << opt_dnn_confThresh << std::endl;
    std::cout << "NMS threshold        : " << opt_dnn_nmsThresh << std::endl;
    std::cout << "Filter threshold     : " << (opt_dnn_filterThresh > std::numeric_limits<double>::epsilon() ? std::to_string(opt_dnn_filterThresh) : "disabled") << std::endl;

    cv::VideoCapture capture;
    bool hasCaptureOpeningSucceeded;
    if (vpMath::isNumber(opt_device)) {
      hasCaptureOpeningSucceeded = capture.open(std::atoi(opt_device.c_str()));
    } else {
      hasCaptureOpeningSucceeded = capture.open(opt_device);
    }
    if(!hasCaptureOpeningSucceeded)
    {
      std::cout << "Capture from camera: " <<  opt_device << " didn't work" << std::endl;
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

    if (! opt_dnn_label_file.empty() && !vpIoTools::checkFilename(opt_dnn_label_file)) {
      throw(vpException(vpException::fatalError, "The file containing the classes labels \"" + opt_dnn_label_file + "\" does not exist !"));
    }

    //! [DNN params]
    vpDetectorDNNOpenCV::NetConfig netConfig(opt_dnn_confThresh, opt_dnn_nmsThresh, opt_dnn_label_file, cv::Size(opt_dnn_width, opt_dnn_height), opt_dnn_filterThresh);
    vpDetectorDNNOpenCV dnn(netConfig, opt_dnn_type);
    dnn.readNet(opt_dnn_model, opt_dnn_config, opt_dnn_framework);
    dnn.setMean(opt_dnn_meanR, opt_dnn_meanG, opt_dnn_meanB);
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

      if (opt_dnn_containerType == ChosenDetectionContainer::MAP || opt_dnn_containerType == ChosenDetectionContainer::BOTH)
      {
        double t = vpTime::measureTimeMs();
        //! [DNN object detection map mode]
        std::map<std::string, std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D>> detections;
        dnn.detect(frame, detections);
        //! [DNN object detection map mode]
        t = vpTime::measureTimeMs() - t;

        //! [DNN class ids and confidences map mode]
        for( auto key_val : detections)
        {
          if (opt_verbose) {
            std::cout << "  Class name      : " << key_val.first << std::endl;
          }
          for(vpDetectorDNNOpenCV::DetectedFeatures2D detection : key_val.second )
          {
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

      if (opt_dnn_containerType == ChosenDetectionContainer::VECTOR || opt_dnn_containerType == ChosenDetectionContainer::BOTH)
      {
        double t_vector = vpTime::measureTimeMs();
        //! [DNN object detection vector mode]
        std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D> detections_vec;
        dnn.detect(frame, detections_vec);
        //! [DNN object detection vector mode]
        t_vector = vpTime::measureTimeMs() - t_vector;

        //! [DNN class ids and confidences vector mode]
        for( auto detection : detections_vec) {
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
        }
        else if(button == vpMouseButton::button3) {
          // Right click => stop the program
          break;
        }
      }
    }

  } catch (const vpException &e) {
    std::cout << e.what() << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
