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
#include <visp3/io/vpJsonArgumentParser.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;



/**
 * @brief Interpolate two vpColors. Linear interpolation between each components (R, G, B)
 *
 * @param low starting color
 * @param high ending color
 * @param f interpolation factor, between 0 and 1
 * @return vpColor The interpolated color
 */
vpColor interpolate(const vpColor &low, const vpColor &high, const double f)
{
  const float r = ((float)high.R - (float)low.R) * f;
  const float g = ((float)high.G - (float)low.G) * f;
  const float b = ((float)high.B - (float)low.B) * f;
  return vpColor((unsigned char)r, (unsigned char)g, (unsigned char)b);
}
/**
 * @brief Display the megapose confidence score as a rectangle in the image.
 * This rectangle becomes green when megapose is "confident" about its prediction
 * The confidence score measures whether megapose can, from its pose estimation, recover the true pose in future pose refinement iterations
 *
 * @param I The image in which to display the confidence
 * @param score The confidence score of megapose, between 0 and 1
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

std::optional<vpRect> detectObjectForInitMegapose(vpDetectorDNNOpenCV &detector, const cv::Mat &I,
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
    if (previousEstimate) { // Get detection that is closest to previous object bounding box estimated by megapose
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

int main(int argc, const char *argv [])
{
  unsigned width = 640, height = 480;
  vpCameraParameters cam;
  std::string detectorModelPath = "path/to/model.onnx";
  std::string detectorFramework = "onnx";
  std::string detectorConfig = "none";
  std::string objectName = "cube";
  std::vector<std::string> labels = {"cube"};
  double detectorMeanR = 104.f, detectorMeanG = 177.f, detectorMeanB = 123.f;
  double detectorConfidenceThreshold = 0.5, detectorNmsThreshold = 0.4, detectorFilterThreshold = 0.25;
  double detectorScaleFactor = 0.0039;
  bool  detectorSwapRB = false;

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
      .addArgument("object", objectName, true, "Name of the object to track with megapose.")


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
      .addArgument("detector/scaleFactor", detectorScaleFactor, false)
      .addArgument("detector/swapRedAndBlue", detectorSwapRB, false)

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
    dnn.setScaleFactor(detectorScaleFactor);
    dnn.setSwapRB(detectorSwapRB);
    //! [DNN params]

    //! [Instantiate megapose]
    std::shared_ptr<vpMegaPose> megapose = std::make_shared<vpMegaPose>(megaposeAddress, megaposePort, cam, height, width);
    megapose->setCoarseNumSamples(coarseNumSamples);
    vpMegaPoseTracker megaposeTracker(megapose, objectName, refinerIterations);
    //! [Instantiate megapose]
    std::cout << "After megapose" << std::endl;

    vpMegaPoseEstimate megaposeEstimate;
    bool initialized = false;
    bool requiresReinit = false;
    vpRect lastDnnDetection;
    cv::Mat frame;

    std::future<vpMegaPoseEstimate> trackerFuture;
    const auto waitTime = std::chrono::milliseconds(0);
    bool callMegapose = true;

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
      if (!callMegapose && trackerFuture.wait_for(waitTime) == std::future_status::ready) {
        megaposeEstimate = trackerFuture.get();
        callMegapose = true;

        if (megaposeEstimate.score < 0.5) {
          requiresReinit = true;
        }
      }
      if (callMegapose) {
        if (!initialized || requiresReinit) {
          std::optional<vpRect> detection = detectObjectForInitMegapose(
            dnn, frame, objectName, initialized ? std::optional(megaposeEstimate) : std::nullopt);
          if (detection) {
            initialized = true;
            requiresReinit = false;
            lastDnnDetection = *detection;
            trackerFuture = megaposeTracker.init(I, lastDnnDetection);
            callMegapose = false;
          }
        }
        else {
          trackerFuture = megaposeTracker.track(I);
          callMegapose = false;
        }
      }

      vpDisplay::display(I);
      if (initialized && !requiresReinit) {
        vpDisplay::displayFrame(I, megaposeEstimate.cTo, cam, 0.05, vpColor::none, 3);
        vpDisplay::displayRectangle(I, lastDnnDetection, vpColor::red);
        displayScore(I, megaposeEstimate.score);

      }
      double t_vector = vpTime::measureTimeMs();
      //! [DNN object detection vector mode]

      //! [DNN object detection vector mode]
      t_vector = vpTime::measureTimeMs() - t_vector;


      vpDisplay::flush(I);

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