/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * DNN object detection using OpenCV DNN module.
 */
#ifndef _vpDetectorDNN_h_
#define _vpDetectorDNN_h_

#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher.
// Here we cannot use (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) in the declaration of the class
#if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(HAVE_OPENCV_DNN) && \
    ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

#include <map>
#include <string>
#include <vector>

#include <opencv2/dnn.hpp>

#include <visp3/core/vpColor.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRect.h>

#include <optional>

#ifdef ENABLE_VISP_NAMESPACE
namespace VISP_NAMESPACE_NAME
{
#endif
class vpDetectorDNNOpenCV;
#ifdef ENABLE_VISP_NAMESPACE
}
#endif

// Forward declaration to have the operator in the global namespace
std::ostream &operator<<(std::ostream &os, const VISP_NAMESPACE_ADDRESSING vpDetectorDNNOpenCV &network);

#ifdef VISP_HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
// Forward declaration to have the methods in the global namespace
void from_json(const nlohmann::json &j, VISP_NAMESPACE_ADDRESSING vpDetectorDNNOpenCV &network);
void to_json(nlohmann::json &j, const VISP_NAMESPACE_ADDRESSING vpDetectorDNNOpenCV &network);
#endif

#ifdef ENABLE_VISP_NAMESPACE
namespace VISP_NAMESPACE_NAME
{
#endif
/*!
 * \class vpDetectorDNNOpenCV
 * \ingroup group_detection_dnn
 * This class is a wrapper over the <a href="https://docs.opencv.org/master/d6/d0f/group__dnn.html">
 * OpenCV DNN module</a> and specialized to handle object detection task.
 *
 * This class supports the following networks dedicated to object detection:
 *
 * - Faster-RCNN, see usage to detect objects belonging to the COCO dataset using \ref dnn_supported_faster_rcnn network
 * - SSD MobileNet, see usage to detect objects belonging to the COCO dataset using \ref dnn_supported_mobilenet_ssd network
 * - ResNet 10, see usage for \ref dnn_usecase_face_detection
 * - Yolo v3, see usage to detect objects belonging to the COCO dataset using \ref dnn_supported_yolov3 network
 * - Yolo v4, see usage to detect objects belonging to the COCO dataset using \ref dnn_supported_yolov4 network
 * - Yolo v5, see usage to detect objects belonging to the COCO dataset using \ref dnn_supported_yolov5 network
 * - Yolo v7, see usage to detect objects belonging to the COCO dataset using \ref dnn_supported_yolov7 network
 * - Yolo v8, see usage to detect objects belonging to the COCO dataset using \ref dnn_supported_yolov8 network
 *
 * This class can be initialized from a JSON file if ViSP has been compiled with NLOHMANN JSON (see \ref soft_tool_json to see how to do it).
 * Examples of such JSON files can be found in the tutorial folder.
 *
 * \sa \ref tutorial-detection-dnn
 */
class VISP_EXPORT vpDetectorDNNOpenCV
{
public:
  /**
   * \enum DNNResultsParsingType
   * \brief Enumeration listing the types of DNN for which the \b vpDetectorDNNOpenCV furnishes the methods
   * permitting to parse the raw detection data into \b vpDetectorDNNOpenCV::DetectedFeatures2D .
   */
  typedef enum DNNResultsParsingType
  {
    USER_SPECIFIED = 0, /*!< The user will give a pointer towards the parsing method to use to parse the raw data resulting from the detection step.*/
    FASTER_RCNN = 1, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a Faster-RCNN DNN. See \b vpDetectorDNNOpenCV::postProcess_FasterRCNN for more information.*/
    SSD_MOBILENET = 2, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a SSD MobileNet DNN. See \b vpDetectorDNNOpenCV::postProcess_SSD_MobileNet for more information.*/
    RESNET_10 = 3, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a ResNet 10 DNN. See \b vpDetectorDNNOpenCV::postProcess_ResNet_10 for more information.*/
    YOLO_V3 = 4, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a YoloV3 DNN. See \b vpDetectorDNNOpenCV::postProcess_YoloV3_V4 for more information.*/
    YOLO_V4 = 5, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a YoloV4 DNN. See \b vpDetectorDNNOpenCV::postProcess_YoloV3_V4 for more information.*/
    YOLO_V5 = 6, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a YoloV5 DNN. See \b vpDetectorDNNOpenCV::postProcess_YoloV5_V7 for more information.*/
    YOLO_V7 = 7, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a YoloV7 DNN. See \b vpDetectorDNNOpenCV::postProcess_YoloV5_V7 for more information.*/
    YOLO_V8 = 8, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a YoloV8 DNN. See \b vpDetectorDNNOpenCV::postProcess_YoloV8 for more information.*/
    COUNT = 9 /*!< The number of parsing method that come along with the \b vpDetectorDNNOpenCV class.*/
  } DNNResultsParsingType;

  typedef struct DetectionCandidates
  {
    std::vector< float > m_confidences; /*!< Vector containing the detection confidence of each \b vpDetectorDNNOpenCV::DetectionCandidates::m_boxes.*/
    std::vector< cv::Rect > m_boxes; /*!< The bounding box of each detection candidate.*/
    std::vector< int > m_classIds; /*!< The class ID of each detection candidate.*/
  } DetectionCandidates;

  /**
   * \class DetectedFeatures2D
   * \brief Structure containing the bounding box, expressed in pixels, confidence and class information
   * about an object detected in a image.
   */
  typedef class DetectedFeatures2D
  {
  protected:
    vpRect m_bbox; /*!< The bounding box of the detected object.*/
    double m_score; /*!< The confidence in the detection.*/
    unsigned int m_cls; /*!< The class ID.*/
    std::optional<std::string> m_classname; /*!< The class name, if the class names were given to the \b vpDetectorDNNOpenCV::NetConfig used to configure the \b vpDetectorDNNOpenCV object.*/
  public:
    /**
     * \brief Construct a new Detected Features 2 D object
     *
     * \param u_min The left coordinate of the bounding box, expressed in pixel.
     * \param u_max The right coordinate of the bounding box, expressed in pixel.
     * \param v_min The top coordinate of the bounding box, expressed in pixel.
     * \param v_max The bottom coordinate of the bounding box, expressed in pixel.
     * \param cls The class ID
     * \param score The confidence in the detection.
     * \param classname The class name, if the class names were given to the \b vpDetectorDNNOpenCV::NetConfig used to configure the \b vpDetectorDNNOpenCV object.
     */
    inline explicit DetectedFeatures2D(double u_min, double u_max
      , double v_min, double v_max
      , unsigned int cls, double score
      , const std::optional<std::string> &classname
    )
      : m_bbox(vpImagePoint(v_min, u_min), vpImagePoint(v_max, u_max))
      , m_score(score)
      , m_cls(cls)
    {
      if (classname) {
        m_classname = classname;
      }
      else {
        m_classname = std::nullopt;
      }
    };

    /*!
    * Return the bounding box of the detected object.
    */
    inline vpRect getBoundingBox() const { return m_bbox; }
    /*!
    * Return the confidence score of the detected object, a value between 0 and 1.
    */
    inline double getConfidenceScore() const { return m_score; }
    /*!
    * Return the class ID of the detected object.
    */
    inline unsigned int getClassId() const { return m_cls; }
    /*!
    * Return the class name of the detected object.
    */
    inline std::optional<std::string> getClassName() const { return m_classname; }

    template < typename Type >
    void display(const vpImage< Type > &img, const vpColor &color = vpColor::blue, unsigned int thickness = 1) const;

    friend vpDetectorDNNOpenCV;
  } DetectedFeatures2D;

  /**
   * \struct NetConfig
   * \brief Structure containing some information required for the configuration of a \b vpDetectorDNNOpenCV object.
   */
  class NetConfig;

  static std::string getAvailableDnnResultsParsingTypes();
  static std::string dnnResultsParsingTypeToString(const DNNResultsParsingType &type);
  static DNNResultsParsingType dnnResultsParsingTypeFromString(const std::string &name);
  static std::vector<std::string> parseClassNamesFile(const std::string &filename);
  vpDetectorDNNOpenCV();
  vpDetectorDNNOpenCV(const NetConfig &config, const DNNResultsParsingType &typeParsingMethod, void (*parsingMethod)(DetectionCandidates &, std::vector<cv::Mat> &, const NetConfig &) = postProcess_unimplemented);
#ifdef VISP_HAVE_NLOHMANN_JSON
  vpDetectorDNNOpenCV(const std::string &jsonPath, void (*parsingMethod)(DetectionCandidates &, std::vector<cv::Mat> &, const NetConfig &) = postProcess_unimplemented);
  void initFromJSON(const std::string &jsonPath);
  void saveConfigurationInJSON(const std::string &jsonPath) const;
#endif
  virtual ~vpDetectorDNNOpenCV();

  virtual bool detect(const vpImage<unsigned char> &I, std::vector<DetectedFeatures2D> &output);
  virtual bool detect(const vpImage<unsigned char> &I, std::map< std::string, std::vector<DetectedFeatures2D>> &output);
  virtual bool detect(const vpImage<unsigned char> &I, std::vector< std::pair<std::string, std::vector<DetectedFeatures2D>>> &output);
  virtual bool detect(const vpImage<vpRGBa> &I, std::vector<DetectedFeatures2D> &output);
  virtual bool detect(const vpImage<vpRGBa> &I, std::map< std::string, std::vector<DetectedFeatures2D>> &output);
  virtual bool detect(const vpImage<vpRGBa> &I, std::vector< std::pair<std::string, std::vector<DetectedFeatures2D>>> &output);
  virtual bool detect(const cv::Mat &I, std::vector<DetectedFeatures2D> &output);
  virtual bool detect(const cv::Mat &I, std::map< std::string, std::vector<DetectedFeatures2D>> &output);
  virtual bool detect(const cv::Mat &I, std::vector< std::pair<std::string, std::vector<DetectedFeatures2D>>> &output);

  void readNet(const std::string &model, const std::string &config = "", const std::string &framework = "");

  void setNetConfig(const NetConfig &config);
  void setConfidenceThreshold(const float &confThreshold);
  void setNMSThreshold(const float &nmsThreshold);
  void setDetectionFilterSizeRatio(const double &sizeRatio);
  void setInputSize(const int &width, const int &height);
  void setMean(const double &meanR, const double &meanG, const double &meanB);
  void setPreferableBackend(const int &backendId);
  void setPreferableTarget(const int &targetId);
  void setScaleFactor(const double &scaleFactor);
  void setSwapRB(const bool &swapRB);
  void setParsingMethod(const DNNResultsParsingType &typeParsingMethod, void (*parsingMethod)(DetectionCandidates &, std::vector<cv::Mat> &, const NetConfig &) = postProcess_unimplemented);
  inline const NetConfig &getNetConfig() const
  {
    return m_netConfig;
  }

#ifdef VISP_HAVE_NLOHMANN_JSON
  /**
   * \brief Read the network configuration from JSON. All values are optional and if an argument is not present,
   * the default value defined in the constructor is kept
   *
   * \param j The JSON object, resulting from the parsing of a JSON file.
   * \param network The network, that will be initialized from the JSON data.
   */
  friend inline void ::from_json(const nlohmann::json &j, vpDetectorDNNOpenCV &network);

  /**
   * \brief Parse the network configuration into JSON format.
   *
   * \param j The JSON parser.
   * \param network  The network we want to parse the configuration.
   */
  friend inline void ::to_json(nlohmann::json &j, const vpDetectorDNNOpenCV &network);
#endif

  friend inline std::ostream &::operator<<(std::ostream &os, const vpDetectorDNNOpenCV &network);

protected:
#if (VISP_HAVE_OPENCV_VERSION == 0x030403)
  std::vector<cv::String> getOutputsNames();
#endif
  std::vector<DetectedFeatures2D>
    filterDetectionSingleClassInput(const std::vector<DetectedFeatures2D> &detected_features, const double minRatioOfAreaOk);

  std::vector<DetectedFeatures2D>
    filterDetectionMultiClassInput(const std::vector<DetectedFeatures2D> &detected_features, const double minRatioOfAreaOk);

  std::map<std::string, std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D>>
    filterDetectionMultiClassInput(const std::map< std::string, std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D>> &detected_features, const double minRatioOfAreaOk);

  void postProcess(DetectionCandidates &proposals);

  void postProcess_YoloV3_V4(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

  void postProcess_YoloV5_V7(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

  void postProcess_YoloV8(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

  void postProcess_FasterRCNN(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  void postProcess_SSD_MobileNet(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);
#endif

  void postProcess_ResNet_10(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

  static void postProcess_unimplemented(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

  //! If true, filter the detections removing the ones for which the bbox does not respect area(bbox) € [mean_class(area) * ratio; mean_class(area) / ratio]
  bool m_applySizeFilterAfterNMS;
  //! Buffer for the blob in input net
  cv::Mat m_blob;
  //! Buffer for gray to RGBa image conversion
  vpImage<vpRGBa> m_I_color;
  //! Buffer for the input image
  cv::Mat m_img;
  //! Indices for NMS
  std::vector<int> m_indices;
  //! DNN network
  cv::dnn::Net m_net;
  //! Configuration of the DNN
  NetConfig m_netConfig;
  //! Names of layers with unconnected outputs
  std::vector<cv::String> m_outNames;
  //! Contains all output blobs for each layer specified in m_outNames
  std::vector<cv::Mat> m_dnnRes;
  //! Pointer towards the parsing method, used if \b m_parsingMethodType is equal to \b m_parsingMethodType::USER_SPECIFIED
  void (*m_parsingMethod)(DetectionCandidates &, std::vector<cv::Mat> &, const NetConfig &);
};

/*!
 * Display the bbox and score of the detected object in an image.
 *
 * \param[in] img : Image used as background.
 * \param[in] color : Color used to draw the CAD model.
 * \param[in] thickness : Thickness used to draw the CAD model.
 */
template < typename Type >
inline void
vpDetectorDNNOpenCV::DetectedFeatures2D::display(const vpImage< Type > &img, const vpColor &color, unsigned int thickness) const
{
  vpDisplay::displayRectangle(img, m_bbox, color, false, thickness);

  std::stringstream ss;
  if (m_classname) {
    ss << *m_classname;
  }
  else {
    ss << m_cls;
  }
  ss << "(" << std::setprecision(4) << m_score * 100. << "%)";
  vpDisplay::displayText(img, m_bbox.getTopRight(), ss.str(), color);
}
#ifdef ENABLE_VISP_NAMESPACE
}
#endif

inline std::ostream &::operator<<(std::ostream &os, const VISP_NAMESPACE_ADDRESSING vpDetectorDNNOpenCV &network)
{
  os << network.m_netConfig;
  return os;
}

#ifdef VISP_HAVE_NLOHMANN_JSON
  /**
   * \brief Read the network configuration from JSON. All values are optional and if an argument is not present,
   * the default value defined in the constructor is kept
   *
   * \param j The JSON object, resulting from the parsing of a JSON file.
   * \param network The network, that will be initialized from the JSON data.
   */
inline void from_json(const nlohmann::json &j, VISP_NAMESPACE_ADDRESSING vpDetectorDNNOpenCV &network)
{
  network.m_netConfig = j.value("networkSettings", network.m_netConfig);
}

/**
 * \brief Parse the network configuration into JSON format.
 *
 * \param j The JSON parser.
 * \param network  The network we want to parse the configuration.
 */
inline void to_json(nlohmann::json &j, const VISP_NAMESPACE_ADDRESSING vpDetectorDNNOpenCV &network)
{
  j = nlohmann::json {
    {"networkSettings", network.m_netConfig}
  };
}
#endif

#endif
#endif
