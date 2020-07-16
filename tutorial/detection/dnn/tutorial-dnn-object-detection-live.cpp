//! \example tutorial-dnn-object-detection-live.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorDNN.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

int main(int argc, const char *argv[])
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(VISP_HAVE_OPENCV_DNN)
  try {
    int opt_device = 0;
    std::string input = "";
    //! [OpenCV DNN face detector]
    std::string model = "opencv_face_detector_uint8.pb";
    std::string config = "opencv_face_detector.pbtxt";
    //! [OpenCV DNN face detector]
    int inputWidth = 300, inputHeight = 300;
    double meanR = 104.0, meanG = 177.0, meanB = 123.0;
    double scaleFactor = 1.0;
    bool swapRB = false;
    float confThresh = 0.5f;
    float nmsThresh = 0.4f;
    std::string labelFile = "";
    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--device" && i+1 < argc) {
        opt_device = atoi(argv[i+1]);
      } else if (std::string(argv[i]) == "--input" && i+1 < argc) {
        input = std::string(argv[i+1]);
      } else if (std::string(argv[i]) == "--model" && i+1 < argc) {
        model = std::string(argv[i+1]);
      } else if (std::string(argv[i]) == "--config" && i+1 < argc) {
        config = std::string(argv[i+1]);
      } else if (std::string(argv[i]) == "--width" && i+1 < argc) {
        inputWidth = atoi(argv[i+1]);
      } else if (std::string(argv[i]) == "--height" && i+1 < argc) {
        inputHeight = atoi(argv[i+1]);
      } else if (std::string(argv[i]) == "--mean" && i+3 < argc) {
        meanR = atof(argv[i+1]);
        meanG = atof(argv[i+2]);
        meanB = atof(argv[i+3]);
      } else if (std::string(argv[i]) == "--scale" && i+1 < argc) {
        scaleFactor = atof(argv[i+1]);
      } else if (std::string(argv[i]) == "--swapRB") {
        swapRB = true;
      } else if (std::string(argv[i]) == "--confThresh" && i+1 < argc) {
        confThresh = (float)atof(argv[i+1]);
      } else if (std::string(argv[i]) == "--nmsThresh" && i+1 < argc) {
        nmsThresh = (float)atof(argv[i+1]);
      } else if (std::string(argv[i]) == "--labels" && i+1 < argc) {
        labelFile = std::string(argv[i+1]);
      } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << argv[0] << " --device <camera device number> --input <path to image or video>"
                                " (camera is used if input is empty) --model <path to net trained weights>"
                                " --config <path to net config file>"
                                " --width <blob width> --height <blob height>"
                                " -- mean <meanR meanG meanB> --scale <scale factor>"
                                " --swapRB --confThresh <confidence threshold>"
                                " --nmsThresh <NMS threshold> --labels <path to label file>" << std::endl;
        return EXIT_SUCCESS;
      }
    }

    std::cout << "Model: " << model << std::endl;
    std::cout << "Config: " << config << std::endl;
    std::cout << "Width: " << inputWidth << std::endl;
    std::cout << "Height: " << inputHeight << std::endl;
    std::cout << "Mean: " << meanR << ", " << meanG << ", " << meanB << std::endl;
    std::cout << "Scale: " << scaleFactor << std::endl;
    std::cout << "Swap RB? " << swapRB << std::endl;
    std::cout << "Confidence threshold: " << confThresh << std::endl;
    std::cout << "NMS threshold: " << nmsThresh << std::endl;

    cv::VideoCapture capture;
    if (input.empty()) {
      capture.open(opt_device);
    } else {
      capture.open(input);
    }

    vpImage<vpRGBa> I;
#if defined(VISP_HAVE_X11)
    vpDisplayX d;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d;
#endif

    //! [DNN params]
    vpDetectorDNN dnn;
    dnn.readNet(model, config);
    dnn.setInputSize(inputWidth, inputHeight);
    dnn.setMean(meanR, meanG, meanB);
    dnn.setScaleFactor(scaleFactor);
    dnn.setSwapRB(swapRB);
    dnn.setConfidenceThreshold(confThresh);
    dnn.setNMSThreshold(nmsThresh);
    //! [DNN params]

    std::vector<std::string> labels;
    if (!labelFile.empty()) {
      std::ifstream f_label(labelFile);
      std::string line;
      while (std::getline(f_label, line)) {
        labels.push_back(line);
      }
    }

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
      std::vector<vpRect> boundingBoxes;
      dnn.detect(I, boundingBoxes);
      //! [DNN object detection]
      t = vpTime::measureTimeMs() - t;

      vpDisplay::display(I);

      //! [DNN class ids and confidences]
      std::vector<int> classIds = dnn.getDetectionClassIds();
      std::vector<float> confidences = dnn.getDetectionConfidence();
      //! [DNN class ids and confidences]
      for (size_t i = 0; i < boundingBoxes.size(); i++) {
        vpDisplay::displayRectangle(I, boundingBoxes[i], vpColor::red, false, 2);

        std::ostringstream oss;
        if (labels.empty())
          oss << "class: " << classIds[i];
        else
          oss << labels[classIds[i]];
        oss << " - conf: " << confidences[i];

        vpDisplay::displayText(I, (int)boundingBoxes[i].getTop()-10, (int)boundingBoxes[i].getLeft()+10,
                               oss.str(), vpColor::red);
      }
      std::ostringstream oss;
      oss << "Detection time: " << t << " ms";
      vpDisplay::displayText(I, 20, 20, oss.str(), vpColor::red);

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
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
