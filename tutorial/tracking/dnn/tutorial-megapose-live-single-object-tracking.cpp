//! \example tutorial-megapose-live-single-object-tracking.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L))) && \
  defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_VIDEOIO) && \
  defined(HAVE_OPENCV_DNN) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(HAVE_OPENCV_HIGHGUI)) && \
  defined(VISP_HAVE_THREADS)

#include <optional>

#include <visp3/core/vpIoTools.h>
#include <visp3/detection/vpDetectorDNNOpenCV.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/dnn_tracker/vpMegaPose.h>
#include <visp3/dnn_tracker/vpMegaPoseTracker.h>
#include <visp3/io/vpJsonArgumentParser.h>

#include <nlohmann/json.hpp>

#include <opencv2/videoio.hpp>


using json = nlohmann::json; //! json namespace shortcut

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/*
 * Interpolate two vpColors. Linear interpolation between each components (R, G, B)
 *
 * low starting color
 * high ending color
 * f interpolation factor, between 0 and 1
 * Returns the interpolated color
 */
vpColor interpolate(const vpColor &low, const vpColor &high, const float f)
{
  const float r = ((float)high.R - (float)low.R) * f;
  const float g = ((float)high.G - (float)low.G) * f;
  const float b = ((float)high.B - (float)low.B) * f;
  return vpColor((unsigned char)r, (unsigned char)g, (unsigned char)b);
}

/*
 * Display the Megapose confidence score as a rectangle in the image.
 * This rectangle becomes green when Megapose is "confident" about its prediction
 * The confidence score measures whether Megapose can, from its pose estimation, recover the true pose in future pose refinement iterations
 *
 * \param[in] I : The image in which to display the confidence.
 * \param[in] score : The confidence score of Megapose, between 0 and 1.
 */
void displayScore(const vpImage<vpRGBa> &I, float score)
{
  const unsigned top = static_cast<unsigned>(I.getHeight() * 0.85f);
  const unsigned height = static_cast<unsigned>(I.getHeight() * 0.1f);
  const unsigned left = static_cast<unsigned>(I.getWidth() * 0.05f);
  const unsigned width = static_cast<unsigned>(I.getWidth() * 0.5f);
  vpRect full(left, top, width, height);
  vpRect scoreRect(left, top, width * score, height);
  const vpColor low = vpColor::red;
  const vpColor high = vpColor::green;
  const vpColor c = interpolate(low, high, score);

  vpDisplay::displayRectangle(I, full, c, false, 5);
  vpDisplay::displayRectangle(I, scoreRect, c, true, 1);
}

/*
 * Add the Megapose rendering on top of the actual image I.
 * Require I and overlay to be of the same size.
 * Note that a fully black object will not render
*/
void overlayRender(vpImage<vpRGBa> &I, const vpImage<vpRGBa> &overlay)
{
  const vpRGBa black = vpRGBa(0, 0, 0);
  for (unsigned int i = 0; i < I.getHeight(); ++i) {
    for (unsigned int j = 0; j < I.getWidth(); ++j) {
      if (overlay[i][j] != black) {
        I[i][j] = overlay[i][j];
      }
    }
  }
}

//! [Detect]
/*
 * Run the detection network on an image in order to find a specific object.
 * The best matching detection is returned:
 * - If a previous Megapose estimation is available, find the closest match in the image (Euclidean distance between centers)
 * - Otherwise, take the detection with highest confidence
 * If no detection corresponding to detectionLabel is found, then std::nullopt is returned
 */
std::optional<vpRect> detectObjectForInitMegaposeDnn(vpDetectorDNNOpenCV &detector, const cv::Mat &I,
  const std::string &detectionLabel,
  std::optional<vpMegaPoseEstimate> previousEstimate)
{
  std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D> detections_vec;
  detector.detect(I, detections_vec);
  std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D> matchingDetections;
  for (const auto &detection : detections_vec) {
    std::optional<std::string> classnameOpt = detection.getClassName();
    if (classnameOpt) {
      if (*classnameOpt == detectionLabel) {
        matchingDetections.push_back(detection);
      }
    }
  }
  if (matchingDetections.size() == 0) {
    return std::nullopt;
  }
  else if (matchingDetections.size() == 1) {
    return matchingDetections[0].getBoundingBox();
  }
  else {
    // Get detection that is closest to previous object bounding box estimated by Megapose
    if (previousEstimate) {
      vpRect best;
      double bestDist = 10000.f;
      const vpImagePoint previousCenter = (*previousEstimate).boundingBox.getCenter();
      for (const auto &detection : matchingDetections) {
        const vpRect detectionBB = detection.getBoundingBox();
        const vpImagePoint center = detectionBB.getCenter();
        const double matchDist = vpImagePoint::distance(center, previousCenter);
        if (matchDist < bestDist) {
          bestDist = matchDist;
          best = detectionBB;
        }
      }
      return best;

    }
    else { // Get detection with highest confidence
      vpRect best;
      double highestConf = 0.0;
      for (const auto &detection : matchingDetections) {
        const double conf = detection.getConfidenceScore();
        if (conf > highestConf) {
          highestConf = conf;
          best = detection.getBoundingBox();
        }
      }
      return best;
    }
  }
  return std::nullopt;
}

/*
 * Ask user to provide the detection themselves. They must click to start labelling, then click on the top left and bottom right corner to create the detection.
 */
std::optional<vpRect> detectObjectForInitMegaposeClick(const vpImage<vpRGBa> &I)
{
  const bool startLabelling = vpDisplay::getClick(I, false);

  const vpImagePoint textPosition(10.0, 20.0);

  if (startLabelling) {
    vpImagePoint topLeft, bottomRight;
    vpDisplay::displayText(I, textPosition, "Click the upper left corner of the bounding box", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I, topLeft, true);
    vpDisplay::display(I);
    vpDisplay::displayCross(I, topLeft, 5, vpColor::red, 2);
    vpDisplay::displayText(I, textPosition, "Click the bottom right corner of the bounding box", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I, bottomRight, true);
    vpRect bb(topLeft, bottomRight);
    return bb;
  }
  else {
    vpDisplay::display(I);
    vpDisplay::displayText(I, textPosition, "Click when the object is visible and static to start reinitializing megapose.", vpColor::red);
    vpDisplay::flush(I);
    return std::nullopt;
  }
}
//! [Detect]

enum DetectionMethod
{
  UNKNOWN,
  CLICK,
  DNN
};

NLOHMANN_JSON_SERIALIZE_ENUM(DetectionMethod, {
  {UNKNOWN, nullptr}, // Default value if the json string is not in "current", "desired" or "mean"
  {CLICK, "click"},
  {DNN, "dnn"} }
);


int main(int argc, const char *argv[])
{
  unsigned width = 640, height = 480;
  vpCameraParameters cam;
  std::string videoDevice = "0";
  std::string megaposeAddress = "127.0.0.1";
  unsigned megaposePort = 5555;
  int refinerIterations = 1, coarseNumSamples = 576;
  double reinitThreshold = 0.2;

  DetectionMethod detectionMethod = DetectionMethod::UNKNOWN;

  std::string detectorModelPath = "path/to/model.onnx", detectorConfig = "none";
  std::string detectorFramework = "onnx", detectorTypeString = "yolov7";
  std::string objectName = "cube";
  std::vector<std::string> labels = { "cube" };
  float detectorMeanR = 0.f, detectorMeanG = 0.f, detectorMeanB = 0.f;
  float detectorConfidenceThreshold = 0.65f, detectorNmsThreshold = 0.5f, detectorFilterThreshold = -0.25f;
  float detectorScaleFactor = 0.0039f;
  bool  detectorSwapRB = false;
  //! [Arguments]
  vpJsonArgumentParser parser("Single object tracking with Megapose", "--config", "/");
  parser.addArgument("width", width, true, "The image width")
    .addArgument("height", height, true, "The image height")
    .addArgument("camera", cam, true, "The camera intrinsic parameters. Should correspond to a perspective projection model without distortion.")
    .addArgument("video-device", videoDevice, true, "Video device")
    .addArgument("object", objectName, true, "Name of the object to track with megapose.")
    .addArgument("detectionMethod", detectionMethod, true, "How to perform detection of the object to get the bounding box:"
      " \"click\" for user labelling, \"dnn\" for dnn detection.")
    .addArgument("reinitThreshold", reinitThreshold, false, "If the Megapose score falls below this threshold, then a reinitialization is be required."
      " Should be between 0 and 1")
    .addArgument("megapose/address", megaposeAddress, true, "IP address of the Megapose server.")
    .addArgument("megapose/port", megaposePort, true, "Port on which the Megapose server listens for connections.")
    .addArgument("megapose/refinerIterations", refinerIterations, false, "Number of Megapose refiner model iterations."
      "A higher count may lead to better accuracy, at the cost of more processing time")
    .addArgument("megapose/initialisationNumSamples", coarseNumSamples, false, "Number of Megapose renderings used for the initial pose estimation.")

    .addArgument("detector/model-path", detectorModelPath, true, "Path to the model")
    .addArgument("detector/config", detectorConfig, true, "Path to the model configuration. Set to none if config is not required.")
    .addArgument("detector/framework", detectorFramework, true, "Detector framework")
    .addArgument("detector/type", detectorTypeString, true, "Detector type")
    .addArgument("detector/labels", labels, true, "Detection class labels")
    .addArgument("detector/mean/red", detectorMeanR, false, "Detector mean red component. Used to normalize image")
    .addArgument("detector/mean/green", detectorMeanG, false, "Detector mean green component. Used to normalize image")
    .addArgument("detector/mean/blue", detectorMeanB, false, "Detector mean red component. Used to normalize image")
    .addArgument("detector/confidenceThreshold", detectorConfidenceThreshold, false, "Detector confidence threshold. "
      "When a detection with a confidence below this threshold, it is ignored")
    .addArgument("detector/nmsThreshold", detectorNmsThreshold, false, "Detector non maximal suppression threshold.")
    .addArgument("detector/filterThreshold", detectorFilterThreshold, false)
    .addArgument("detector/scaleFactor", detectorScaleFactor, false, "Pixel intensity rescaling factor. If set to 1/255, then pixel values are between 0 and 1.")
    .addArgument("detector/swapRedAndBlue", detectorSwapRB, false, "Whether to swap red and blue channels before feeding the image to the detector.");

  parser.parse(argc, argv);
  //! [Arguments]

  if (cam.get_projModel() != vpCameraParameters::perspectiveProjWithoutDistortion) {
    throw vpException(vpException::badValue, "The camera projection model should be without distortion, as other models are ignored by Megapose");
  }

  if (detectionMethod == DetectionMethod::UNKNOWN) {
    throw vpException(vpException::badValue, "The specified detection method is incorrect: it should be either \"click\" or \"dnn\"");
  }

  cv::VideoCapture capture;
  bool isLiveCapture;
  bool hasCaptureOpeningSucceeded;
  double videoFrametime = 0; // Only for prerecorded videos
  if (vpMath::isNumber(videoDevice)) {
    hasCaptureOpeningSucceeded = capture.open(std::atoi(videoDevice.c_str()));
    isLiveCapture = true;
  }
  else {
    hasCaptureOpeningSucceeded = capture.open(videoDevice);
    isLiveCapture = false;
    double fps = capture.get(cv::CAP_PROP_FPS);
    videoFrametime = (1.0 / fps) * 1000.0;
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
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV d;
#endif
  //d.setDownScalingFactor(vpDisplay::SCALE_AUTO);
#if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(HAVE_OPENCV_DNN) && \
    ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
  vpDetectorDNNOpenCV::DNNResultsParsingType detectorType =
    vpDetectorDNNOpenCV::dnnResultsParsingTypeFromString(detectorTypeString);
  vpDetectorDNNOpenCV::NetConfig netConfig(detectorConfidenceThreshold, detectorNmsThreshold, labels,
    cv::Size(width, height), detectorFilterThreshold);
  vpDetectorDNNOpenCV dnn(netConfig, detectorType);
  if (detectionMethod == DetectionMethod::DNN) {
    dnn.readNet(detectorModelPath, detectorConfig, detectorFramework);
    dnn.setMean(detectorMeanR, detectorMeanG, detectorMeanB);
    dnn.setScaleFactor(detectorScaleFactor);
    dnn.setSwapRB(detectorSwapRB);
  }
#endif
  //! [Instantiate megapose]
  std::shared_ptr<vpMegaPose> megapose;
  try {
    megapose = std::make_shared<vpMegaPose>(megaposeAddress, megaposePort, cam, height, width);
  }
  catch (...) {
    throw vpException(vpException::ioError, "Could not connect to Megapose server at " + megaposeAddress + " on port " + std::to_string(megaposePort));
  }

  vpMegaPoseTracker megaposeTracker(megapose, objectName, refinerIterations);
  megapose->setCoarseNumSamples(coarseNumSamples);
  const std::vector<std::string> allObjects = megapose->getObjectNames();
  if (std::find(allObjects.begin(), allObjects.end(), objectName) == allObjects.end()) {
    throw vpException(vpException::badValue, "Object " + objectName + " is not known by the Megapose server!");
  }
  std::future<vpMegaPoseEstimate> trackerFuture;
  //! [Instantiate megapose]

  cv::Mat frame;
  vpMegaPoseEstimate megaposeEstimate; // last Megapose estimation
  vpRect lastDetection; // Last detection (initialization)
  bool callMegapose = true; // Whether we should call Megapose this iteration
  bool initialized = false; // Whether tracking should be initialized or reinitialized
  bool tracking = false;

  bool overlayModel = true;
  vpImage<vpRGBa> overlayImage(height, width);
  std::string overlayMode = "full";

  std::vector<double> megaposeTimes;
  std::vector<double> frameTimes;

  double megaposeStartTime = 0.0;

  //! [Acquisition]
  while (true) {
    const double frameStart = vpTime::measureTimeMs();
    capture >> frame;
    if (frame.empty())
      break;

    if (I.getSize() == 0) {
      vpImageConvert::convert(frame, I);
      d.init(I);
      vpDisplay::setTitle(I, "Megapose object pose estimation");
    }
    else {
      vpImageConvert::convert(frame, I);
    }
    vpDisplay::display(I);
    //! [Acquisition]
    // Check whether Megapose is still running
    //! [Check megapose]
    if (!callMegapose && trackerFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      megaposeEstimate = trackerFuture.get();
      if (tracking) {
        megaposeTimes.push_back(vpTime::measureTimeMs() - megaposeStartTime);
      }
      callMegapose = true;
      tracking = true;

      if (overlayModel) {
        overlayImage = megapose->viewObjects({ objectName }, { megaposeEstimate.cTo }, overlayMode);
      }

      if (megaposeEstimate.score < reinitThreshold) { // If confidence is low, require a reinitialisation with 2D detection
        initialized = false;
      }
    }
    //! [Check megapose]
    //! [Call MegaPose]
    if (callMegapose) {
      if (!initialized) {
        tracking = false;
        std::optional<vpRect> detection = std::nullopt;
#if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(HAVE_OPENCV_DNN) && \
    ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
        if (detectionMethod == DetectionMethod::DNN) {
          detection = detectObjectForInitMegaposeDnn(
            dnn, frame, objectName, initialized ? std::optional(megaposeEstimate) : std::nullopt);
        }
#endif
        if (detectionMethod == DetectionMethod::CLICK) {
          detection = detectObjectForInitMegaposeClick(I);
        }

        if (detection) {
          initialized = true;
          lastDetection = *detection;
          trackerFuture = megaposeTracker.init(I, lastDetection);
          callMegapose = false;

        }
      }
      else {
        trackerFuture = megaposeTracker.track(I);
        callMegapose = false;
        megaposeStartTime = vpTime::measureTimeMs();
      }
    }
    //! [Call MegaPose]

    //! [Display]
    std::string keyboardEvent;
    const bool keyPressed = vpDisplay::getKeyboardEvent(I, keyboardEvent, false);
    if (keyPressed) {
      if (keyboardEvent == "t") {
        overlayModel = !overlayModel;
      }
      else if (keyboardEvent == "w") {
        overlayMode = overlayMode == "full" ? "wireframe" : "full";
      }
    }

    if (tracking) {
      if (overlayModel) {
        overlayRender(I, overlayImage);
        vpDisplay::display(I);
      }
      vpDisplay::displayText(I, 20, 20, "Right click to quit", vpColor::red);
      vpDisplay::displayText(I, 30, 20, "Press T: Toggle overlay", vpColor::red);
      vpDisplay::displayText(I, 40, 20, "Press W: Toggle wireframe", vpColor::red);
      vpDisplay::displayFrame(I, megaposeEstimate.cTo, cam, 0.05, vpColor::none, 3);
      //vpDisplay::displayRectangle(I, lastDetection, vpColor::red);
      displayScore(I, megaposeEstimate.score);
    }
    //! [Display]

    vpDisplay::flush(I);

    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I, button, false)) {
      if (button == vpMouseButton::button3) {
        break; // Right click to stop
      }
    }
    const double frameEnd = vpTime::measureTimeMs();
    if (!isLiveCapture) {
      vpTime::wait(std::max<double>(0.0, videoFrametime - (frameEnd - frameStart)));
    }
    frameTimes.push_back(vpTime::measureTimeMs() - frameStart);
  }
  std::cout << "Average frame time: " << vpMath::getMean(frameTimes) << std::endl;
  std::cout << "Average time between Megapose calls: " << vpMath::getMean(megaposeTimes) << std::endl;
}

#else
int main()
{
  std::cout << "Compile ViSP with the DNN tracker module, the JSON 3rd party library and the OpenCV detection module" << std::endl;
  return EXIT_SUCCESS;
}

#endif
