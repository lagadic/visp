//! \example tutorial-dnn-object-detection-live.cpp
#include <visp3/core/vpConfig.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) && defined(VISP_HAVE_NLOHMANN_JSON) && (VISP_HAVE_OPENCV_VERSION >= 0x030403)
#include <visp3/core/vpIoTools.h>
#include <visp3/detection/vpDetectorDNNOpenCV.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/dnn_tracker/vpMegaPose.h>
#include <visp3/dnn_tracker/vpMegaPoseTracker.h>
#include <optional>
#include <visp3/io/vpJsonArgumentParser.h>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

/*
 * Interpolate two vpColors. Linear interpolation between each components (R, G, B)
 *
 * low starting color
 * high ending color
 * f interpolation factor, between 0 and 1
 * Returns the interpolated color
 */
vpColor interpolate(const vpColor &low, const vpColor &high, const double f)
{
  const float r = ((float)high.R - (float)low.R) * f;
  const float g = ((float)high.G - (float)low.G) * f;
  const float b = ((float)high.B - (float)low.B) * f;
  return vpColor((unsigned char)r, (unsigned char)g, (unsigned char)b);
}
/*
 * Display the megapose confidence score as a rectangle in the image.
 * This rectangle becomes green when megapose is "confident" about its prediction
 * The confidence score measures whether megapose can, from its pose estimation, recover the true pose in future pose refinement iterations
 *
 * I The image in which to display the confidence
 * score The confidence score of megapose, between 0 and 1
 */
void displayScore(const vpImage<vpRGBa> &I, double score)
{
  const unsigned top = I.getHeight() * 0.85;
  const unsigned height = I.getHeight() * 0.1;
  const unsigned left = I.getWidth() * 0.05;
  const unsigned width = I.getWidth() * 0.5;
  vpRect full(left, top, width, height);
  vpRect scoreRect(left, top, width * score, height);
  const vpColor low = vpColor::red;
  const vpColor high = vpColor::green;
  const vpColor c = interpolate(low, high, score);

  vpDisplay::displayRectangle(I, full, c, false, 5);
  vpDisplay::displayRectangle(I, scoreRect, c, true, 1);
}

/*
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

#if defined(VISP_HAVE_OPENCV_DNN)
//! [Detect]
/*
 * Run the detection network on an image in order to find a specific object.
 * The best matching detection is returned:
 *  If a previous megapose estimation is available, find the closest match in the image (Euclidean distance between centers)
 *  Otherwise, take the detection with highest confidence
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
    // Get detection that is closest to previous object bounding box estimated by megapose
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
#endif
/*
* Ask user to provide the detection themselves. They must click to start labelling, then click on the top left and bottom right corner to create the detection.
*/
std::optional<vpRect> detectObjectForInitMegaposeClick(const vpImage<vpRGBa> &I)
{
  const bool startLabelling = vpDisplay::getClick(I, false);

  const vpImagePoint textPosition(10.0, 50.0);

  if (startLabelling) {
    vpImagePoint topLeft, bottomRight;
    vpDisplay::displayText(I, textPosition, "Click the upper left corner of the bounding box", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I, topLeft, true);
    vpDisplay::displayCross(I, topLeft, 5, vpColor::red, 2);
    vpDisplay::displayText(I, textPosition, "Click the bottom right corner of the bounding box", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I, bottomRight, true);
    vpRect bb(topLeft, bottomRight);
    return bb;
  }
  else {
    vpDisplay::displayText(I, textPosition, "Click when the object is visible and static to start reinitializing megapose.", vpColor::red);
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


int main(int argc, const char *argv [])
{
  unsigned width = 640, height = 480;
  vpCameraParameters cam;
  std::string videoDevice = "0";
  std::string megaposeAddress = "127.0.0.1";
  unsigned megaposePort = 5555;
  int refinerIterations = 1, coarseNumSamples = 576;
  double reinitThreshold = 0.5;

  DetectionMethod detectionMethod = DetectionMethod::UNKNOWN;

  std::string detectorModelPath = "path/to/model.onnx", detectorConfig = "none";
  std::string detectorFramework = "onnx", detectorTypeString = "yolov7";
  std::string objectName = "cube";
  std::vector<std::string> labels = {"cube"};
  double detectorMeanR = 0.f, detectorMeanG = 0.f, detectorMeanB = 0.f;
  double detectorConfidenceThreshold = 0.65, detectorNmsThreshold = 0.5, detectorFilterThreshold = -0.25;
  double detectorScaleFactor = 0.0039;
  bool  detectorSwapRB = false;
  vpJsonArgumentParser parser("Single object tracking with Megapose", "--config", "/");
  parser.addArgument("width", width, true, "The image width")
    .addArgument("height", height, true, "The image height")
    .addArgument("camera", cam, true, "The camera intrinsic parameters. Should correspond to a perspective projection model without distortion.")
    .addArgument("video-device", videoDevice, true, "Video device")
    .addArgument("object", objectName, true, "Name of the object to track with megapose.")
    .addArgument("detectionMethod", detectionMethod, true, "How to perform detection of the object to get the bounding box:"
      " \"click\" for user labelling, \"dnn\" for dnn detection.")
    .addArgument("reinitThreshold", reinitThreshold, false, "If the megapose score falls below this threshold, then a reinitialization is be required."
      " Should be between 0 and 1")
    .addArgument("megapose/address", megaposeAddress, true, "IP address of the megapose server.")
    .addArgument("megapose/port", megaposePort, true, "Port on which the megapose server listens for connections.")
    .addArgument("megapose/refinerIterations", refinerIterations, false, "Number of megapose refiner model iterations."
      "A higher count may lead to better accuracy, at the cost of more processing time")
    .addArgument("megapose/initialisationNumSamples", coarseNumSamples, false, "Number of megapose renderings used for the initial pose estimation.")

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

  if (cam.get_projModel() != vpCameraParameters::perspectiveProjWithoutDistortion) {
    throw vpException(vpException::badValue, "The camera projection model should be without distortion, as other models are ignored by Megapose");
  }

  if (detectionMethod == DetectionMethod::UNKNOWN) {
    throw vpException(vpException::badValue, "The specified detection method is incorrect: it should be either \"click\" or \"dnn\"");
  }

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
#if defined(VISP_HAVE_OPENCV_DNN)
  vpDetectorDNNOpenCV::DNNResultsParsingType detectorType = vpDetectorDNNOpenCV::dnnResultsParsingTypeFromString(detectorTypeString);
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
  std::shared_ptr<vpMegaPose> megapose = std::make_shared<vpMegaPose>(megaposeAddress, megaposePort, cam, height, width);
  megapose->setCoarseNumSamples(coarseNumSamples);
  vpMegaPoseTracker megaposeTracker(megapose, objectName, refinerIterations);
  //! [Instantiate megapose]

  cv::Mat frame;
  vpMegaPoseEstimate megaposeEstimate;
  vpRect lastDetection;
  bool callMegapose = true;
  bool initialized = false;
  bool requiresReinit = false;

  std::future<vpMegaPoseEstimate> trackerFuture;
  const auto waitTime = std::chrono::milliseconds(0);

  double megaposeTime = 0.0;
  std::vector<std::pair<unsigned, double>> megaposeRunTimes;
  std::vector<std::pair<unsigned, vpMegaPoseEstimate>> iterMegaposeResults;
  std::vector<bool> iterRequiresInit;
  unsigned iter = 0;

  while (true) {
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
    //Check whether megapose is still running
    //! [Check megapose]
    if (!callMegapose && trackerFuture.wait_for(waitTime) == std::future_status::ready) {
      megaposeEstimate = trackerFuture.get();
      iterMegaposeResults.push_back(std::make_pair(iter, megaposeEstimate));
      megaposeRunTimes.push_back(std::make_pair(iter, vpTime::measureTimeMs() - megaposeTime));
      callMegapose = true;

      if (megaposeEstimate.score < reinitThreshold) { // If confidence is low, require a reinitialisation with 2D detection
        requiresReinit = true;
      }
    }
    //! [Check megapose]
    iterRequiresInit.push_back(requiresReinit || !initialized);
    //! [Call megapose]
    if (callMegapose) {
      if (!initialized || requiresReinit) {
        std::optional<vpRect> detection = std::nullopt;
#if defined(VISP_HAVE_OPENCV_DNN)
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
          requiresReinit = false;
          lastDetection = *detection;
          trackerFuture = megaposeTracker.init(I, lastDetection);
          callMegapose = false;
          megaposeTime = vpTime::measureTimeMs();
        }
      }
      else {
        trackerFuture = megaposeTracker.track(I);
        callMegapose = false;
      }
    }
    //! [Call megapose]
    vpDisplay::display(I);
    if (initialized && !requiresReinit) {
      vpDisplay::displayText(I, 100, 10, "Right click to quit", vpColor::red);
      vpDisplay::displayFrame(I, megaposeEstimate.cTo, cam, 0.05, vpColor::none, 3);
      vpDisplay::displayRectangle(I, lastDetection, vpColor::red);
      displayScore(I, megaposeEstimate.score);
    }

    vpDisplay::flush(I);
    ++iter;

    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I, button, false)) {
      if (button == vpMouseButton::button3) {
        break; // Right click to stop
      }
    }
  }
}
#else
int main()
{
  std::cout << "Compile ViSP with the DNN tracker module, the JSON 3rd party library and the OpenCV detection module" << std::endl;
  return EXIT_SUCCESS;
}

#endif