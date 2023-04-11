//! \example tutorial-dnn-object-detection-live.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorDNNOpenCV.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>


int main(int argc, const char *argv[])
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(VISP_HAVE_OPENCV_DNN) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
  try {
    int opt_device = 0;
    std::string input = "";
    //! [OpenCV DNN face detector]
    std::string model = "opencv_face_detector_uint8.pb";
    std::string config = "opencv_face_detector.pbtxt";
    std::string framework = "tensorflow";
    vpDetectorDNNOpenCV::DNNResultsParsingType type = vpDetectorDNNOpenCV::RESNET_10;
    //! [OpenCV DNN face detector]
    int inputWidth = 300, inputHeight = 300;
    double meanR = 104.0, meanG = 177.0, meanB = 123.0;
    double scaleFactor = 1.0;
    bool swapRB = false;
    bool hasToWaitClick = false;
    float confThresh = 0.5f;
    float nmsThresh = 0.4f;
    double detectionFilter = 0.25;
    std::string labelFile = "";
    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--device" && i + 1 < argc) {
        opt_device = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "--waitForClick") {
        hasToWaitClick = true;
      } else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
        input = std::string(argv[i + 1]);
      } else if (std::string(argv[i]) == "--model" && i + 1 < argc) {
        model = std::string(argv[i + 1]);
      }else if (std::string(argv[i]) == "--type" && i + 1 < argc) {
        type =  vpDetectorDNNOpenCV::dnnResultsParsingTypeFromString(std::string(argv[i + 1]));
      } else if (std::string(argv[i]) == "--config" && i + 1 < argc) {
        config = std::string(argv[i + 1]);
        if(config.find("none") != std::string::npos)
        {
          config = std::string();
        }
      } else if (std::string(argv[i]) == "--framework" && i + 1 < argc) {
        framework = std::string(argv[i + 1]);
        if(framework.find("none") != std::string::npos)
        {
          framework = std::string();
        }
      } else if (std::string(argv[i]) == "--width" && i + 1 < argc) {
        inputWidth = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "--height" && i + 1 < argc) {
        inputHeight = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "--mean" && i + 3 < argc) {
        meanR = atof(argv[i + 1]);
        meanG = atof(argv[i + 2]);
        meanB = atof(argv[i + 3]);
      } else if (std::string(argv[i]) == "--scale" && i + 1 < argc) {
        scaleFactor = atof(argv[i + 1]);
      } else if (std::string(argv[i]) == "--swapRB") {
        swapRB = true;
      } else if (std::string(argv[i]) == "--confThresh" && i + 1 < argc) {
        confThresh = (float)atof(argv[i + 1]);
      } else if (std::string(argv[i]) == "--nmsThresh" && i + 1 < argc) {
        nmsThresh = (float)atof(argv[i + 1]);
      } else if (std::string(argv[i]) == "--filterThresh" && i + 1 < argc) {
        detectionFilter = atof(argv[i + 1]);
      } else if (std::string(argv[i]) == "--labels" && i + 1 < argc) {
        labelFile = std::string(argv[i + 1]);
      } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << argv[0]
                  << " --device <camera device number> --waitForClick --input <path to image or video>"
                     " (camera is used if input is empty) --model <path to net trained weights>"
                     " --type <type of DNN in " + vpDetectorDNNOpenCV::getAvailableDnnResultsParsingTypes() + 
                     "> --config <path to net config file or \"none\" not to use one> --framework <framework name or \"none\" not to specify one>"
                     " --width <blob width> --height <blob height>"
                     " -- mean <meanR meanG meanB> --scale <scale factor>"
                     " --swapRB --confThresh <confidence threshold>"
                     " --nmsThresh <NMS threshold> --filterThresh <threshold > 0., 0. to disable> --labels <path to label file>"
                  << std::endl;
        return EXIT_SUCCESS;
      }
    }

    std::cout << "Model: " << model << std::endl;
    std::cout << "Type: " << vpDetectorDNNOpenCV::dnnResultsParsingTypeToString(type) << std::endl;
    std::cout << "Config: " << (config.empty() ? "\"None\"" : config) << std::endl;
    std::cout << "Framework: " << (framework.empty() ? "\"None\"" : framework) << std::endl;
    std::cout << "Width: " << inputWidth << std::endl;
    std::cout << "Height: " << inputHeight << std::endl;
    std::cout << "Mean: " << meanR << ", " << meanG << ", " << meanB << std::endl;
    std::cout << "Scale: " << scaleFactor << std::endl;
    std::cout << "Swap RB? " << swapRB << std::endl;
    std::cout << "Confidence threshold: " << confThresh << std::endl;
    std::cout << "NMS threshold: " << nmsThresh << std::endl;
    std::cout << "Filter threshold: " << (detectionFilter > std::numeric_limits<double>::epsilon() ? std::to_string(detectionFilter) : "disabled") << std::endl;

    cv::VideoCapture capture;
    bool hasCaptureOpeningSucceeded;
    if (input.empty()) {
      hasCaptureOpeningSucceeded = capture.open(opt_device);
    } else {
      hasCaptureOpeningSucceeded = capture.open(input);
    }

    if(!hasCaptureOpeningSucceeded)
    {
      std::cout << "Capture from camera #" <<  (input.empty() ? std::to_string(opt_device) : input) << " didn't work" << std::endl;
      return 1;
    }

    vpImage<vpRGBa> I;
#if defined(VISP_HAVE_X11)
    vpDisplayX d;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d;
#endif

    if (labelFile.empty()) {
      throw(vpException(vpException::fatalError, "The path to the file containing the classes labels is empty !"));
    }
    else if(!vpIoTools::checkFilename(labelFile))
    {
      throw(vpException(vpException::fatalError, "The file containing the classes labels \"" + labelFile + "\" does not exist !"));
    }

    //! [DNN params]
    vpDetectorDNNOpenCV::NetConfig netConfig(confThresh, nmsThresh, labelFile, cv::Size(inputWidth, inputHeight), detectionFilter);
    vpDetectorDNNOpenCV dnn(netConfig, type);
    dnn.readNet(model, config, framework);
    dnn.setMean(meanR, meanG, meanB);
    dnn.setScaleFactor(scaleFactor);
    dnn.setSwapRB(swapRB);
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
      } else {
        vpImageConvert::convert(frame, I);
      }

      double t = vpTime::measureTimeMs();
      //! [DNN object detection]
      std::map<std::string, std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D>> detections;
      dnn.detect(frame, detections);
      //! [DNN object detection]
      t = vpTime::measureTimeMs() - t;

      vpDisplay::display(I);

      for( auto key_val : detections)
      {
        for(vpDetectorDNNOpenCV::DetectedFeatures2D detection : key_val.second )
        {
          detection.display(I);
        }
      }
      
      std::ostringstream oss;
      oss << "Detection time: " << t << " ms";
      if(hasToWaitClick)
      {
        // hasToWaitClick => we are displaying images
        vpDisplay::displayText(I, 20, 20, "Left click to display next image", vpColor::red);
      }
      vpDisplay::displayText(I, 40, 20, "Right click to quit", vpColor::red);
      vpDisplay::displayText(I, 60, 20, oss.str(), vpColor::red);

      vpDisplay::flush(I);
      vpMouseButton::vpMouseButtonType button;
    
      if (vpDisplay::getClick(I, button, hasToWaitClick))
      {
        if (button == vpMouseButton::button1)
        {
          // Left click => next image
          continue;
        }
        else if(button == vpMouseButton::button3)
        {
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
