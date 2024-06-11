/****************************************************************************
 *
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
 *
*****************************************************************************/
#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher
#if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(HAVE_OPENCV_DNN) && \
    ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorDNNOpenCV.h>
#include <visp3/core/vpIoTools.h>

#include<algorithm>

BEGIN_VISP_NAMESPACE
/**
 * \brief Get the list of the parsing methods / types of DNNs supported by the \b vpDetectorDNNOpenCV class.
 *
 * \return std::string The list of the supported parsing methods / types of DNNs.
 */
std::string vpDetectorDNNOpenCV::getAvailableDnnResultsParsingTypes()
{
  std::string list = "[";
  for (unsigned int i = 0; i < vpDetectorDNNOpenCV::COUNT - 1; i++) {
    list += "\"" + dnnResultsParsingTypeToString((vpDetectorDNNOpenCV::DNNResultsParsingType)i) + "\", ";
  }
  list += "\"" + dnnResultsParsingTypeToString((vpDetectorDNNOpenCV::DNNResultsParsingType)(vpDetectorDNNOpenCV::COUNT - 1)) + "\"]";
  return list;
}

/*!
 * Cast a \b vpDetectorDNNOpenCV::DNNResultsParsingType into its name, in \b std::string format.
 * The naming convention is the following:
 * - only lowercases
 * - the underscores '_' are replaced by hyphens '-'.
 *
 * \param type: the type of parsing method to apply to interpret the DNN inference raw results.
 * \return std::string: the name of the type of parsing method to apply.
 */
std::string vpDetectorDNNOpenCV::dnnResultsParsingTypeToString(const DNNResultsParsingType &type)
{
  std::string name;
  switch (type) {
  case YOLO_V3:
    name = "yolov3";
    break;
  case YOLO_V4:
    name = "yolov4";
    break;
  case YOLO_V5:
    name = "yolov5";
    break;
  case YOLO_V7:
    name = "yolov7";
    break;
  case YOLO_V8:
    name = "yolov8";
    break;
  case FASTER_RCNN:
    name = "faster-rcnn";
    break;
  case SSD_MOBILENET:
    name = "ssd-mobilenet";
    break;
  case RESNET_10:
    name = "resnet-10";
    break;
  case USER_SPECIFIED:
    name = "user-specified";
    break;
  case COUNT:
    name = "unknown";
    break;
  }
  return name;
}

/*!
 * Cast a name of class of parsing method into a \b vpDetectorDNNOpenCV::DNNResultsParsingType.
 * The naming convention is the following:
 * - only lowercases
 * - the underscores '_' are replaced by hyphens '-'.
 * \param name: the name of the type of parsing method to apply.
 * \return vpDetectorDNNOpenCV::DNNResultsParsingType: the corresponding \b vpDetectorDNNOpenCV::DNNResultsParsingType.
 */
vpDetectorDNNOpenCV::DNNResultsParsingType vpDetectorDNNOpenCV::dnnResultsParsingTypeFromString(const std::string &name)
{
  vpDetectorDNNOpenCV::DNNResultsParsingType res(COUNT);
  bool hasFoundMatch = false;
  std::string name_lowercase = vpIoTools::toLowerCase(name);
  for (int id = 0; id < COUNT && !hasFoundMatch; id++) {
    vpDetectorDNNOpenCV::DNNResultsParsingType temp = (vpDetectorDNNOpenCV::DNNResultsParsingType)id;
    if (dnnResultsParsingTypeToString(temp) == name_lowercase) {
      res = temp;
      hasFoundMatch = true;
    }
  }
  return res;
}

/*!
 * \brief Parse the designated file that contains the list of the classes the network can detect.
 * The class names must either be indicated in an array of string in YAML format, or one name
 * by row (without quotes) as described in NetConfig::parseClassNamesFile().
 *
 * \param filename The path towards the file.
 * \return std::vector<std::string> The list of class names.
 *
 * \sa NetConfig::parseClassNamesFile().
 */
std::vector<std::string> vpDetectorDNNOpenCV::parseClassNamesFile(const std::string &filename)
{
  return NetConfig::parseClassNamesFile(filename);
}

vpDetectorDNNOpenCV::vpDetectorDNNOpenCV()
  : m_applySizeFilterAfterNMS(false), m_blob(), m_I_color(), m_img(),
  m_net(), m_netConfig(), m_outNames(), m_dnnRes(),
  m_parsingMethod(vpDetectorDNNOpenCV::postProcess_unimplemented)
{
  setDetectionFilterSizeRatio(m_netConfig.m_filterSizeRatio);
}

/**
 * \brief Construct a new \b vpDetectorDNNOpenCV object and, if the model file has been given to the \b config object, read the net by calling \b vpDetectorDNNOpenCV::readNet
 *
 * \param config The network configuration.
 * \param typeParsingMethod The type of parsing method that must be used to parse the raw results of the DNN detection step.
 * \param parsingMethod If \b typeParsingMethod is set to \b vpDetectorDNNOpenCV::DNNResultsParsingType::USER_SPECIFIED, the parsing method that must be used to parse the raw results of the DNN detection step.
 */
vpDetectorDNNOpenCV::vpDetectorDNNOpenCV(const NetConfig &config, const DNNResultsParsingType &typeParsingMethod, void (*parsingMethod)(DetectionCandidates &, std::vector<cv::Mat> &, const NetConfig &))
  : m_applySizeFilterAfterNMS(false), m_blob(), m_I_color(), m_img(),
  m_net(), m_netConfig(config), m_outNames(), m_dnnRes()
{
  setDetectionFilterSizeRatio(m_netConfig.m_filterSizeRatio);
  setParsingMethod(typeParsingMethod, parsingMethod);
  if (!m_netConfig.m_modelFilename.empty()) {
    readNet(m_netConfig.m_modelFilename, m_netConfig.m_modelConfigFilename, m_netConfig.m_framework);
  }
}

#ifdef VISP_HAVE_NLOHMANN_JSON

using json = nlohmann::json;

/**
 * \brief Construct a new vpDetectorDNNOpenCV object from a JSON file and a potential parsing method.
 *
 * \param jsonPath The JSON file permitting to initialize the detector.
 * \param parsingMethod If the user chose to use a user-specified parsing method, the parsing method that must be used to parse the raw results of the DNN detection step.
 */
vpDetectorDNNOpenCV::vpDetectorDNNOpenCV(const std::string &jsonPath, void (*parsingMethod)(DetectionCandidates &, std::vector<cv::Mat> &, const NetConfig &))
  : m_applySizeFilterAfterNMS(false), m_blob(), m_I_color(), m_img(),
  m_net(), m_netConfig(), m_outNames(), m_dnnRes()
{
  initFromJSON(jsonPath);
  setDetectionFilterSizeRatio(m_netConfig.m_filterSizeRatio);
  setParsingMethod(m_netConfig.m_parsingMethodType, parsingMethod);
}

/**
 * \brief
 *
 * \param jsonPath
 */
void vpDetectorDNNOpenCV::initFromJSON(const std::string &jsonPath)
{
  std::ifstream file(jsonPath);
  if (!file.good()) {
    std::stringstream ss;
    ss << "Problem opening file " << jsonPath << ". Make sure it exists and is readable" << std::endl;
    throw vpException(vpException::ioError, ss.str());
  }
  json j;
  try {
    j = json::parse(file);
  }
  catch (json::parse_error &e) {
    std::stringstream msg;
    msg << "Could not parse JSON file : \n";

    msg << e.what() << std::endl;
    msg << "Byte position of error: " << e.byte;
    throw vpException(vpException::ioError, msg.str());
  }
  *this = j; // Call from_json(const json& j, vpDetectorDNN& *this) to read json
  file.close();
  readNet(m_netConfig.m_modelFilename, m_netConfig.m_modelConfigFilename, m_netConfig.m_framework);
}

/**
 * \brief Save the network configuration in a JSON file.
 *
 * \param jsonPath Path towards the output JSON file .
 */
void vpDetectorDNNOpenCV::saveConfigurationInJSON(const std::string &jsonPath) const
{
  std::ofstream file(jsonPath);
  const json j = *this;
  file << j.dump(4);
  file.close();
}
#endif

/**
 * \brief Destroy the \b vpDetectorDNNOpenCV object
 */
vpDetectorDNNOpenCV::~vpDetectorDNNOpenCV() { }

/**
 * \brief Object detection using OpenCV DNN module.
 * \warning Classical object detection network uses as input 3-channels.
 * Grayscale image is converted to color image.
 *
 * \param I Input image.
 * \param output Vector of detections, whichever class they belong to.
 * \return false if there is no detection, true otherwise.
 */
bool vpDetectorDNNOpenCV::detect(const vpImage<unsigned char> &I, std::vector<DetectedFeatures2D> &output)
{
  vpImageConvert::convert(I, m_I_color);

  return detect(m_I_color, output);
}

/*!
  Object detection using OpenCV DNN module.
  \warning Classical object detection network uses as input 3-channels.
  Grayscale image is converted to color image.

  \param I : Input image.
  \param output: map where the name of the class is used as key and whose value is a vector of detected 2D features that belong to the class.
  \return false if there is no detection.
*/
bool vpDetectorDNNOpenCV::detect(const vpImage<unsigned char> &I, std::map< std::string, std::vector<DetectedFeatures2D>> &output)
{
  vpImageConvert::convert(I, m_I_color);

  return detect(m_I_color, output);
}

/*!
  Object detection using OpenCV DNN module.
  \warning Classical object detection network uses as input 3-channels.
  Grayscale image is converted to color image.

  \param I : Input image.
  \param output: vector of pairs <name_of_the_class, vector_of_detections>
  \return false if there is no detection.
*/
bool vpDetectorDNNOpenCV::detect(const vpImage<unsigned char> &I, std::vector< std::pair<std::string, std::vector<DetectedFeatures2D>>> &output)
{
  vpImageConvert::convert(I, m_I_color);

  return detect(m_I_color, output);
}

/**
 * \brief Object detection using OpenCV DNN module.
 * \warning OpenCV DNN module uses \b cv::Mat as input.
 * \b vpImage<vpRGBa> is converted to \b cv::Mat.
 *
 * \param I : Input image.
 * \param output Vector of detections, whichever class they belong to.
 * \return false if there is no detection, true otherwise.
 */
bool vpDetectorDNNOpenCV::detect(const vpImage<vpRGBa> &I, std::vector<DetectedFeatures2D> &output)
{
  vpImageConvert::convert(I, m_img);

  return detect(m_img, output);
}

/**
* \brief Object detection using OpenCV DNN module.
* \warning OpenCV DNN module uses \b cv::Mat as input.
* Grayscale image is converted to color image.
*
* \param I : Input image.
* \param output: map where the name of the class is used as key and whose value is a vector of detected 2D features that belong to the class.
* \return false if there is no detection.
*/
bool vpDetectorDNNOpenCV::detect(const vpImage<vpRGBa> &I, std::map< std::string, std::vector<DetectedFeatures2D>> &output)
{
  vpImageConvert::convert(I, m_img);

  return detect(m_img, output);
}

/*!
  Object detection using OpenCV DNN module.

  \param I : Input image.
  \param output: vector of pairs <name_of_the_class, vector_of_detections>
  \return false if there is no detection, true otherwise.
*/
bool vpDetectorDNNOpenCV::detect(const vpImage<vpRGBa> &I, std::vector< std::pair<std::string, std::vector<DetectedFeatures2D>>> &output)
{
  vpImageConvert::convert(I, m_img);

  return detect(m_img, output);
}

/*!
  Object detection using OpenCV DNN module.

  \param I : Input image.
  \param output : Vector of detections, whichever class they belong to.
  \return false if there is no detection.
*/
bool vpDetectorDNNOpenCV::detect(const cv::Mat &I, std::vector<DetectedFeatures2D> &output)
{
  m_img = I;
  output.clear();

  cv::Size inputSize(m_netConfig.m_inputSize.width > 0 ? m_netConfig.m_inputSize.width : m_img.cols,
    m_netConfig.m_inputSize.height > 0 ? m_netConfig.m_inputSize.height : m_img.rows);
  cv::dnn::blobFromImage(m_img, m_blob, m_netConfig.m_scaleFactor, inputSize, m_netConfig.m_mean, m_netConfig.m_swapRB, false);

  m_net.setInput(m_blob);
  try {
    m_net.forward(m_dnnRes, m_outNames);
  }
  catch (const cv::Exception &e) {
    std::cerr << "Caught an exception trying to run inference:" << std::endl << "\t"
      << e.what()
      << "\nCuda and/or GPU driver might not be correctly installed. Setting preferable backend to CPU and trying again." << std::endl;
    m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
    m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    m_net.forward(m_dnnRes, m_outNames);
  }

  DetectionCandidates proposals;
  postProcess(proposals);
  size_t nbClassNames = m_netConfig.m_classNames.size();
  for (size_t i = 0; i < m_indices.size(); ++i) {
    int idx = m_indices[i];
    cv::Rect box = proposals.m_boxes[idx];
    std::optional<std::string> classname_opt;
    if (nbClassNames > 0) {
      classname_opt = m_netConfig.m_classNames[proposals.m_classIds[idx]];
    }
    output.emplace_back(box.x, box.x + box.width, box.y, box.y + box.height
      , proposals.m_classIds[idx], proposals.m_confidences[idx]
      , classname_opt
    );
  }

  if (m_applySizeFilterAfterNMS) {
    // removing false detections, based on the bbox sizes
    output = filterDetectionMultiClassInput(output, m_netConfig.m_filterSizeRatio);
  }

  return !output.empty();
}

/*!
  Object detection using OpenCV DNN module.

  \param I : Input image.
  \param output : map where the name of the class is used as key and whose value is a vector of detected 2D features that belong to the class.
  \return false if there is no detection.
*/
bool vpDetectorDNNOpenCV::detect(const cv::Mat &I, std::map< std::string, std::vector<DetectedFeatures2D>> &output)
{
  m_img = I;
  output.clear();

  cv::Size inputSize(m_netConfig.m_inputSize.width > 0 ? m_netConfig.m_inputSize.width : m_img.cols,
    m_netConfig.m_inputSize.height > 0 ? m_netConfig.m_inputSize.height : m_img.rows);
  cv::dnn::blobFromImage(m_img, m_blob, m_netConfig.m_scaleFactor, inputSize, m_netConfig.m_mean, m_netConfig.m_swapRB, false);

  m_net.setInput(m_blob);
  try {
    m_net.forward(m_dnnRes, m_outNames);
  }
  catch (const cv::Exception &e) {
    std::cerr << "Caught an exception trying to run inference:" << std::endl << "\t"
      << e.what()
      << "\nCuda and/or GPU driver might not be correctly installed. Setting preferable backend to CPU and trying again." << std::endl;
    m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
    m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    m_net.forward(m_dnnRes, m_outNames);
  }

  DetectionCandidates proposals;
  postProcess(proposals);
  size_t nbClassNames = m_netConfig.m_classNames.size();
  for (size_t i = 0; i < m_indices.size(); ++i) {
    int idx = m_indices[i];
    cv::Rect box = proposals.m_boxes[idx];
    std::string classname;
    if (nbClassNames > 0) {
      classname = m_netConfig.m_classNames[proposals.m_classIds[idx]];
    }
    else {
      classname = std::to_string(proposals.m_classIds[idx]);
    }
    std::optional<std::string> classname_opt = std::optional<std::string>(classname);
    output[classname].emplace_back(box.x, box.x + box.width, box.y, box.y + box.height
      , proposals.m_classIds[idx], proposals.m_confidences[idx]
      , classname_opt
    );
  }

  if (m_applySizeFilterAfterNMS) {
    output = filterDetectionMultiClassInput(output, m_netConfig.m_filterSizeRatio);
  }

  return !output.empty();
}

/*!
  Object detection using OpenCV DNN module.

  \param I : Input image.
  \param output : vector of pairs <name_of_the_class, vector_of_detections>
  \return false if there is no detection.
*/
bool vpDetectorDNNOpenCV::detect(const cv::Mat &I, std::vector< std::pair<std::string, std::vector<DetectedFeatures2D>>> &output)
{
  std::map< std::string, std::vector<DetectedFeatures2D>> map_output;
  bool returnStatus = detect(I, map_output);
  for (auto key_val : map_output) {
    output.push_back(key_val);
  }
  return returnStatus;
}

#if (VISP_HAVE_OPENCV_VERSION == 0x030403)
/**
 * \brief Get the names of the output layers of the DNN.
 *
 * \return std::vector<cv::String>
 */
std::vector<cv::String> vpDetectorDNNOpenCV::getOutputsNames()
{
  static std::vector<cv::String> names;
  if (names.empty()) {
    std::vector<int> outLayers = m_net.getUnconnectedOutLayers();
    std::vector<cv::String> layersNames = m_net.getLayerNames();
    names.resize(outLayers.size());
    for (size_t i = 0; i < outLayers.size(); ++i)
      names[i] = layersNames[outLayers[i] - 1];
  }
  return names;
}
#endif

/*!
  Post-process the raw results of the DNN.
  Call the post-process method corresponding to the \b vpDetectorDNNOpenCV::DNNResultsParsingType that will
  extract the data stored as a matrix.
  Then, perform Non-Maximum Suppression to remove overlapping detections.

  \param[inout] proposals : Input/output that will contains all the detection candidates.
*/
void vpDetectorDNNOpenCV::postProcess(DetectionCandidates &proposals)
{
  switch (m_netConfig.m_parsingMethodType) {
  case YOLO_V3:
  case YOLO_V4:
    postProcess_YoloV3_V4(proposals, m_dnnRes, m_netConfig);
    break;
  case YOLO_V5:
  case YOLO_V7:
    postProcess_YoloV5_V7(proposals, m_dnnRes, m_netConfig);
    break;
  case YOLO_V8:
    postProcess_YoloV8(proposals, m_dnnRes, m_netConfig);
    break;
  case FASTER_RCNN:
    postProcess_FasterRCNN(proposals, m_dnnRes, m_netConfig);
    break;
  case SSD_MOBILENET:
#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
    void postProcess_SSD_MobileNet(DetectionCandidates & proposals, std::vector<cv::Mat> &dnnRes, const NetConfig & netConfig);
#else
    // NB: the two SSD-MobileNet DNNs that have been tested worked only
    // using the ResNet-10 parsing method
    postProcess_ResNet_10(proposals, m_dnnRes, m_netConfig);
#endif
    break;
  case RESNET_10:
    postProcess_ResNet_10(proposals, m_dnnRes, m_netConfig);
    break;
  case USER_SPECIFIED:
    m_parsingMethod(proposals, m_dnnRes, m_netConfig);
    break;
  default:
    throw(vpException(vpException::badValue, "Type of DNN post-processing method not handled."));
  }

  m_indices.clear();
  cv::dnn::NMSBoxes(proposals.m_boxes, proposals.m_confidences, m_netConfig.m_confThreshold, m_netConfig.m_nmsThreshold, m_indices);
}

/*!
 * \brief Return a new vector of detected features whose area is greater
 *  or equal to the average area x \b minRatioOfAreaOk. This method assumes that
 * \b detected_features contains detections of a single class.
 *
 * \param detected_features The original list of detected features, belonging to the same class.
 * \param minRatioOfAreaOk The minimum ratio of area a feature bounding box must reach to be kept
 * in the resulting list of features. Value between 0 and 1.0.
 * \return std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D> The resulting list of features, that only contains the features
 * whose area is in the range [average area x \b minRatioOfAreaOk ; average area / \b minRatioOfAreaOk ].
 */
std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D>
vpDetectorDNNOpenCV::filterDetectionSingleClassInput(const std::vector<DetectedFeatures2D> &detected_features, const double minRatioOfAreaOk)
{
  double meanArea(0.);
  double originalNumberOfObj = static_cast<double>(detected_features.size());
  double meanFactor = 1. / originalNumberOfObj;

  // Computing the average area of the class
  for (DetectedFeatures2D feature : detected_features) {
    meanArea += feature.m_bbox.getArea();
  }
  meanArea *= meanFactor;

  // Keeping only the detections that respect the area criterion
  std::vector<DetectedFeatures2D> filtered_features;
  for (DetectedFeatures2D feature : detected_features) {
    if (feature.m_bbox.getArea() >= minRatioOfAreaOk * meanArea && feature.m_bbox.getArea() < meanArea / minRatioOfAreaOk) {
      filtered_features.push_back(feature);
    }
  }

  return filtered_features;
}

/*!
 * \brief Return a new vector, ordered by \b vpDetectorDNNOpenCV::DetectedFeatures2D::m_cls ,
 * where the area of each detection belonging to one class is in the range
 * [average_area(class) x \b minRatioOfAreaOk ; average_area(class) / \b minRatioOfAreaOk ] .
 *
 * \param detected_features The original list of detected features, that can contains several classes.
 * \param minRatioOfAreaOk The minimum ratio of area a feature bounding box must reach to be kept in the resulting list of features. Value between 0 and 1.0.
 * \return std::vector<vpDetectorDNNOpenCV::DetectedFeatures2> The filtered list of features, ordered by \b vpDetectorDNNOpenCV::DetectedFeatures2D::m_cls in ascending order, where
 * only the features respecting the area criterion are kept.
 */
std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D>
vpDetectorDNNOpenCV::filterDetectionMultiClassInput(const std::vector<DetectedFeatures2D> &detected_features, const double minRatioOfAreaOk)
{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  /**
   * \class MeanAreaComputer
   * \brief Helper to compute the average area of the detections belonging to the same class.
   */
  class MeanAreaComputer
  {
  private:
    std::map<int, std::pair<int, double>> m_map_id_pairOccurrencesAreas; /*!< Uses the \b vpDetectorDNNOpenCV::DetectedFeatures2D::m_classIds as keys
                                                                           and pairs <nb_occurrences, summed_areas> as values.*/

    std::map<int, double> m_mapMeans; /*!< Map <class_id; average_area_class>.*/
    /**
     * \brief Compute the average area of the detections corresponding to \b class_id.
     *
     * \param class_id The numerical ID of the class.
     * \return double The average area of all the detections belonging to \b class_id .
     */
    double computeMeanArea(const int &class_id)
    {
      return m_map_id_pairOccurrencesAreas[class_id].second / (double)m_map_id_pairOccurrencesAreas[class_id].first;
    }

  public:
    /**
     * \brief Compute the average area of each class that were encountered.
     */
    void computeMeans()
    {
      for (const auto &classID_pair : m_map_id_pairOccurrencesAreas) {
        m_mapMeans[classID_pair.first] = computeMeanArea(classID_pair.first);
      }
    }

    double getMean(const int &class_id)
    {
      if (m_map_id_pairOccurrencesAreas.find(class_id) == m_map_id_pairOccurrencesAreas.end()) {
        throw(vpException(vpException::badValue, "[MeanAreaComputer::getMean] Asking for class_id \"" + std::to_string(class_id) + "\" that is not present in m_mapMeans. Did you call computeMeans ?"));
      }
      return m_mapMeans[class_id];
    }

    /**
     * \brief Increment the number of occurrences and the
     *
     * \param feature
     */
    void operator()(const DetectedFeatures2D &feature)
    {
      int class_id = feature.getClassId();
      double area = feature.getBoundingBox().getArea();
      if (m_map_id_pairOccurrencesAreas.find(class_id) == m_map_id_pairOccurrencesAreas.end()) {
        m_map_id_pairOccurrencesAreas[class_id] = std::pair<int, double>(1, area);
      }
      else {
        std::pair<int, double> prev_state = m_map_id_pairOccurrencesAreas[class_id];
        m_map_id_pairOccurrencesAreas[class_id] = std::pair<int, double>(prev_state.first + 1, prev_state.second + area);
      }
    }
  };
#endif // DOXYGEN_SHOULD_SKIP_THIS

  // Computing the average area of each class
  MeanAreaComputer meanComputer;
  std::for_each(detected_features.begin(), detected_features.end(), meanComputer);
  meanComputer.computeMeans();

  // Keeping only the detections that respect the area criterion
  std::vector<DetectedFeatures2D> filtered_features;
  for (DetectedFeatures2D feature : detected_features) {
    double meanArea = meanComputer.getMean(feature.getClassId());
    if (feature.m_bbox.getArea() >= minRatioOfAreaOk * meanArea
      && feature.m_bbox.getArea() < meanArea / minRatioOfAreaOk) {
      filtered_features.push_back(feature);
    }
  }

  return filtered_features;
}

/**
 * \brief Return a new map <class, vector_corresponding_detections> where the area
 * of each detection belonging to one class is in the range [average_area(class) x \b minRatioOfAreaOk ; average_area(class) / \b minRatioOfAreaOk ] .
 *
 * \param detected_features The original detected features, that can contains several classes.
 * \param minRatioOfAreaOk The minimum ratio of area a feature bounding box must reach to be kept in the resulting list of features. Value between 0 and 1.0.
 * \return std::map<std::string, std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D>> A new filtered map where each detection
 * belonging to a class respect the area criterion.
 */
std::map<std::string, std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D>>
vpDetectorDNNOpenCV::filterDetectionMultiClassInput(const std::map< std::string, std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D>> &detected_features, const double minRatioOfAreaOk)
{
  std::map<std::string, std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D>> output;
  for (auto keyval : detected_features) {
    output[keyval.first] = filterDetectionSingleClassInput(detected_features.at(keyval.first), minRatioOfAreaOk); // removing false detections
  }
  return output;
}

/*!
  Post-process the raw results of a Yolov3-type or YoloV4-type  DNN.
  Extract the data stored as a matrix. They are stored as follow:
  [batchsize][1:nb_proposals][1:5+nb_classes]
  Where a detection proposal consists of:
  [center_x; center_y; width; height; objectness; score_class_0; ...; score_last_class]
  where center_x € [0; 1] and center_y € [0; 1] which correspond to the ratio of the position
  of the center of the bbox with regard to the total width/height of the image.

  \param proposals : input/output that will contains all the detection candidates.
  \param dnnRes: raw results of the \b vpDetectorDNNOpenCV::detect step.
  \param netConfig: the configuration of the network, to know for instance the DNN input size.
*/
void vpDetectorDNNOpenCV::postProcess_YoloV3_V4(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig)
{
  size_t nbBatches = dnnRes.size();

  for (size_t i = 0; i < nbBatches; i++) {
    // Slightly modify from here: https://github.com/opencv/opencv/blob/8c25a8eb7b10fb50cda323ee6bec68aa1a9ce43c/samples/dnn/object_detection.cpp#L192-L221
     // Counts the number of proposed detections and the number of data corresponding to 1 detection
    int num_proposal = dnnRes[i].size[0]; // Number of detections
    int nout = dnnRes[i].size[1]; // Number of data for each detection
    if (dnnRes[i].dims > 2) {
      num_proposal = dnnRes[i].size[1];
      nout = dnnRes[i].size[2];
      dnnRes[i] = dnnRes[i].reshape(0, num_proposal);
    }

    int n = 0, row_ind = 0; /// cx,cy,w,h,box_score,class_score
    float *pdata = (float *)dnnRes[i].data;

    // Iterate on the detections to keep only the meaningful ones
    for (n = 0; n < num_proposal; n++) {
      float box_score = pdata[4];
      if (box_score > netConfig.m_confThreshold) {
        cv::Mat scores = dnnRes[i].row(row_ind).colRange(5, nout);
        cv::Point classIdPoint;
        double max_class_score;
        // Get the value and location of the maximum score
        cv::minMaxLoc(scores, 0, &max_class_score, 0, &classIdPoint);

        max_class_score *= box_score;

        // The detection is kept only if the confidence is greater than the threshold
        if (max_class_score > netConfig.m_confThreshold) {
          const int class_idx = classIdPoint.x;
          float cx = pdata[0] * m_img.cols; /// cx
          float cy = pdata[1] * m_img.rows; /// cy
          float w = pdata[2] * m_img.cols; /// w
          float h = pdata[3] * m_img.rows; /// h

          int left = int(cx - 0.5 * w);
          int top = int(cy - 0.5 * h);

          proposals.m_confidences.push_back((float)max_class_score);
          proposals.m_boxes.push_back(cv::Rect(left, top, (int)(w), (int)(h)));
          proposals.m_classIds.push_back(class_idx);
        }
      }
      row_ind++;
      pdata += nout;
    }
  }
}

/*!
  Post-process the raw results of a YoloV5-type or a YoloV7-type  DNN.
  Extract the data stored as a matrix. They are stored as follow:
  [batchsize][1:num_proposals][1:5+nb_classes]
  Where a detection proposal consists of:
  [center_x; center_y; width; height; objectness; score_class_0; ...; score_last_class]

  \param proposals : input/output that will contains all the detection candidates.
  \param dnnRes: raw results of the \b vpDetectorDNNOpenCV::detect step.
  \param netConfig: the configuration of the network, to know for instance the DNN input size.
*/
void vpDetectorDNNOpenCV::postProcess_YoloV5_V7(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig)
{
  // Compute the ratio between the original size of the image and the network size to translate network coordinates into
  // image coordinates
  float ratioh = (float)m_img.rows / netConfig.m_inputSize.height, ratiow = (float)m_img.cols / netConfig.m_inputSize.width;
  size_t nbBatches = dnnRes.size();

  for (size_t i = 0; i < nbBatches; i++) {
    // Counts the number of proposed detections and the number of data corresponding to 1 detection
    int num_proposal = dnnRes[i].size[0]; // Number of detections
    int nout = dnnRes[i].size[1]; // Number of data for each detection
    if (dnnRes[i].dims > 2) {
      num_proposal = dnnRes[i].size[1];
      nout = dnnRes[i].size[2];
      dnnRes[i] = dnnRes[i].reshape(0, num_proposal);
    }

    int n = 0, row_ind = 0; /// cx,cy,w,h,box_score,class_score
    float *pdata = (float *)dnnRes[i].data;

    // Iterate on the detections to keep only the meaningful ones
    for (n = 0; n < num_proposal; n++) {
      float box_score = pdata[4];

      if (box_score > netConfig.m_confThreshold) {
        cv::Mat scores = dnnRes[i].row(row_ind).colRange(5, nout);
        cv::Point classIdPoint;
        double max_class_score;
        // Get the value and location of the maximum score
        cv::minMaxLoc(scores, 0, &max_class_score, 0, &classIdPoint);
        max_class_score *= box_score;

        // The detection is kept only if the confidence is greater than the threshold
        if (max_class_score > netConfig.m_confThreshold) {
          const int class_idx = classIdPoint.x;
          float cx = pdata[0] * ratiow; /// cx
          float cy = pdata[1] * ratioh; /// cy
          float w = pdata[2] * ratiow; /// w
          float h = pdata[3] * ratioh; /// h

          int left = int(cx - 0.5 * w);
          int top = int(cy - 0.5 * h);

          proposals.m_confidences.push_back((float)max_class_score);
          proposals.m_boxes.push_back(cv::Rect(left, top, (int)(w), (int)(h)));
          proposals.m_classIds.push_back(class_idx);
        }
      }
      row_ind++;
      pdata += nout;
    }
  }
}

/*!
  Post-process the raw results of a YoloV8-type  DNN.
  Extract the data stored as a matrix. They are stored as follow:
  [batchsize][1:5+nb_classes][1:8400]
  Where 8400 is the number of detection proposals and a detection proposal consists of:
  [center_x; center_y; width; height; score_class_0; ...; score_last_class]^T

  \param proposals : input/output that will contains all the detection candidates.
  \param dnnRes: raw results of the \b vpDetectorDNNOpenCV::detect step.
  \param netConfig: the configuration of the network, to know for instance the DNN input size.
*/
void vpDetectorDNNOpenCV::postProcess_YoloV8(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig)
{
  // Code adapted from here: https://github.com/JustasBart/yolov8_CPP_Inference_OpenCV_ONNX/blob/minimalistic/inference.cpp
  // Compute the ratio between the original size of the image and the network size to translate network coordinates into
  // image coordinates
  float ratioh = (float)m_img.rows / netConfig.m_inputSize.height, ratiow = (float)m_img.cols / netConfig.m_inputSize.width;
  size_t nbBatches = dnnRes.size();

  for (size_t i = 0; i < nbBatches; i++) {
    // Counts the number of proposed detections and the number of data corresponding to 1 detection
    int num_proposal = dnnRes[i].size[1]; // Number of detections
    int nout = dnnRes[i].size[0]; // Number of data for each detection
    if (dnnRes[i].dims > 2) {
      num_proposal = dnnRes[i].size[2];
      nout = dnnRes[i].size[1];
      dnnRes[i] = dnnRes[i].reshape(0, nout);
    }
    cv::transpose(dnnRes[i], dnnRes[i]); // Organise data as YoloV5 i.e. [batchsize][1:num_proposals][1:4+nb_classes]

    int n = 0, row_ind = 0; /// cx,cy,w,h,box_score,class_score
    float *pdata = (float *)dnnRes[i].data;

    // Iterate on the detections to keep only the meaningful ones
    for (n = 0; n < num_proposal; n++) {
      cv::Mat scores = dnnRes[i].row(row_ind).colRange(4, nout);
      cv::Point classIdPoint;
      double max_class_score;
      // Get the value and location of the maximum score
      cv::minMaxLoc(scores, 0, &max_class_score, 0, &classIdPoint);

      // The detection is kept only if the confidence is greater than the threshold
      if (max_class_score > netConfig.m_confThreshold) {
        const int class_idx = classIdPoint.x;
        float cx = pdata[0] * ratiow; /// cx
        float cy = pdata[1] * ratioh; /// cy
        float w = pdata[2] * ratiow; /// w
        float h = pdata[3] * ratioh; /// h

        int left = int(cx - 0.5 * w);
        int top = int(cy - 0.5 * h);

        proposals.m_confidences.push_back((float)max_class_score);
        proposals.m_boxes.push_back(cv::Rect(left, top, (int)(w), (int)(h)));
        proposals.m_classIds.push_back(class_idx);
      }

      row_ind++;
      pdata += nout;
    }
  }
}

/*!
  Post-process the raw results of a Faster-RCNN-type DNN.
  Extract the data stored as a matrix.
  The network produces output blob with a shape 1x1xNx7 where N is a number of
  detections and an every detection is a vector of values
  [batchId, classId, confidence, left, top, right, bottom]

  \param proposals : input/output that will contains all the detection candidates.
  \param dnnRes: raw results of the \b vpDetectorDNNOpenCV::detect step.
  \param netConfig: the configuration of the network, to know for instance the DNN input size.
*/
void vpDetectorDNNOpenCV::postProcess_FasterRCNN(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig)
{
  // Direct copy from object_detection.cpp OpenCV sample
  // Faster-RCNN

  // Network produces output blob with a shape 1x1xNx7 where N is a number of
  // detections and an every detection is a vector of values
  // [batchId, classId, confidence, left, top, right, bottom]
  size_t nbBatches = dnnRes.size();
  for (size_t j = 0; j < nbBatches; j++) {
    float *data = (float *)dnnRes[j].data;
    for (size_t i = 0; i < dnnRes[j].total(); i += 7) {
      float confidence = data[i + 2];
      if (confidence > netConfig.m_confThreshold) {
        int left = (int)(data[i + 3] * m_img.cols);
        int top = (int)(data[i + 4] * m_img.rows);
        int right = (int)(data[i + 5] * m_img.cols);
        int bottom = (int)(data[i + 6] * m_img.rows);
        int classId = (int)(data[i + 1]);

        proposals.m_confidences.push_back((float)confidence);
        proposals.m_boxes.push_back(cv::Rect(left, top, right - left + 1, bottom - top + 1));
        proposals.m_classIds.push_back(classId);
      }
    }
  }

}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  Post-process the raw results of a SSD-MobileNet-type  DNN.
  Extract the data stored as a matrix.
  The network produces 2 outputs blob:
  - `scores` with dimensions 1xNxC
  - 'boxes'  with dimensions 1xNx4
  where `N` is a number of detections and `C` is the number of classes (with `BACKGROUND` as classId = 0).

  \param proposals : input/output that will contains all the detection candidates.
  \param dnnRes: raw results of the \b vpDetectorDNNOpenCV::detect step.
  \param netConfig: the configuration of the network, to know for instance the DNN input size.
*/
void vpDetectorDNNOpenCV::postProcess_SSD_MobileNet(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig)
{
  // Network produces 2 outputs blob:
  // - `scores` with dimensions 1xNxC
  // - 'boxes'  with dimensions 1xNx4
  // where `N` is a number of detections and `C` is the number of classes (with `BACKGROUND` as classId = 0).

  int scores_index = m_outNames[0] == "scores" ? 0 : 1; // scores output index.
  int boxes_index = m_outNames[0] == "boxes" ? 0 : 1;   // boxes output index.

  int N = dnnRes[scores_index].size[1], C = dnnRes[scores_index].size[2];

  float *confidence = (float *)dnnRes[scores_index].data;
  float *bbox = (float *)dnnRes[boxes_index].data;

  // Loop over all guesses on the output of the network.
  for (int i = 0; i < N; i++) {
    uint32_t maxClass = 0;
    float maxScore = -1000.0f;

    for (int j = 1; j < C; j++) // ignore background (classId = 0).
    {
      const float score = confidence[i * C + j];

      if (score < netConfig.m_confThreshold)
        continue;

      if (score > maxScore) {
        maxScore = score;
        maxClass = j;
      }
    }

    if (maxScore > netConfig.m_confThreshold) {
      int left = (int)(bbox[4 * i] * m_img.cols);
      int top = (int)(bbox[4 * i + 1] * m_img.rows);
      int right = (int)(bbox[4 * i + 2] * m_img.cols);
      int bottom = (int)(bbox[4 * i + 3] * m_img.rows);
      int width = right - left + 1;
      int height = bottom - top + 1;

      int classId = maxClass;
      proposals.m_confidences.push_back(maxScore);
      proposals.m_boxes.push_back(cv::Rect(left, top, width, height));
      proposals.m_classIds.push_back(classId);
    }
  }
}
#endif

/*!
  Post-process the raw results of a ResNet-10-type  DNN.
  Extract the data stored as a matrix.
  The network produces output blob with a shape 1x1xNx7 where N is a number of
  detections and an every detection is a vector of values
  [batchId, classId, confidence, left, top, right, bottom]

  \param proposals : input/output that will contains all the detection candidates.
  \param dnnRes: raw results of the \b vpDetectorDNNOpenCV::detect step.
  \param netConfig: the configuration of the network, to know for instance the DNN input size.
*/
void vpDetectorDNNOpenCV::postProcess_ResNet_10(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig)
{
  // Direct copy from object_detection.cpp OpenCV sample

  // Network produces output blob with a shape 1x1xNx7 where N is a number of
  // detections and an every detection is a vector of values
  // [batchId, classId, confidence, left, top, right, bottom]
  CV_Assert(dnnRes.size() == 1);
  float *data = (float *)dnnRes[0].data;
  for (size_t i = 0; i < dnnRes[0].total(); i += 7) {
    float confidence = data[i + 2];
    if (confidence > netConfig.m_confThreshold) {
      int left = (int)(data[i + 3] * m_img.cols);
      int top = (int)(data[i + 4] * m_img.rows);
      int right = (int)(data[i + 5] * m_img.cols);
      int bottom = (int)(data[i + 6] * m_img.rows);
      int classId = (int)(data[i + 1]) - 1;

      proposals.m_confidences.push_back((float)confidence);
      proposals.m_boxes.push_back(cv::Rect(left, top, right - left + 1, bottom - top + 1));
      proposals.m_classIds.push_back(classId);
    }
  }
}

/*!
  Method throwing a \b vpException::functionNotImplementedError . It is called if the user set the
  \b vpDetectorDNNOpenCV::NetConfig::m_parsingMethodType to \b USER_DEFINED but didn't set the parsing method.

  \param proposals : input/output that will contains all the detection candidates.
  \param dnnRes: raw results of the \b vpDetectorDNNOpenCV::detect step.
  \param netConfig: the configuration of the network, to know for instance the DNN input size.
*/
void vpDetectorDNNOpenCV::postProcess_unimplemented(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig)
{
  (void)proposals;
  (void)dnnRes;
  (void)netConfig;
  throw(vpException(vpException::functionNotImplementedError, "vpDetectorDNNOpenCV::postProcess was called with a USER_SPECIFIED DNN but not post processing method was given."));
}

/*!
  Read a network, see OpenCV readNet documentation for more information.

  \param model Path to a binary file of model containing trained weights.
  The following file extensions are expected for models from different frameworks:
    - `*.caffemodel` (Caffe, http://caffe.berkeleyvision.org/)
    - `*.pb` (TensorFlow, https://www.tensorflow.org/)
    - `*.t7` | `*.net` (Torch, http://torch.ch/)
    - `*.weights` (Darknet, https://pjreddie.com/darknet/)
    - `*.bin` (DLDT, https://software.intel.com/openvino-toolkit)
    - `*.onnx` (ONNX, https://onnx.ai/)
  \param config Path to a text file of model containing network configuration.
  It could be a file with the following extensions:
    - `*.prototxt` (Caffe, http://caffe.berkeleyvision.org/)
    - `*.pbtxt` (TensorFlow, https://www.tensorflow.org/)
    - `*.cfg` (Darknet, https://pjreddie.com/darknet/)
    - `*.xml` (DLDT, https://software.intel.com/openvino-toolkit)
  \param framework Optional name of an origin framework of the model. Automatically detected if it is not set.
*/
void vpDetectorDNNOpenCV::readNet(const std::string &model, const std::string &config, const std::string &framework)
{
  m_netConfig.m_modelFilename = model;
  m_netConfig.m_modelConfigFilename = config;
  m_netConfig.m_framework = framework;
  m_net = cv::dnn::readNet(model, config, framework);
#if (VISP_HAVE_OPENCV_VERSION == 0x030403)
  m_outNames = getOutputsNames();
#else
  m_outNames = m_net.getUnconnectedOutLayersNames();
#endif
}

/*!
 * Configure the DNN (thresholds, input size, ...).
 * If the DNN weights file is known, initialize the \b m_net by reading the weights.
 *
 * \param config: the desired configuration of the network
 */
void vpDetectorDNNOpenCV::setNetConfig(const NetConfig &config)
{
  m_netConfig = config;
  setDetectionFilterSizeRatio(m_netConfig.m_filterSizeRatio);
  setParsingMethod(m_netConfig.m_parsingMethodType);
  if (!m_netConfig.m_modelFilename.empty()) {
    readNet(m_netConfig.m_modelFilename, m_netConfig.m_modelConfigFilename, m_netConfig.m_framework);
  }
}

/*!
  Set confidence threshold to filter the detections.

  \param confThreshold Confidence threshold between [0, 1]
*/
void vpDetectorDNNOpenCV::setConfidenceThreshold(const float &confThreshold) { m_netConfig.m_confThreshold = confThreshold; }

/*!
  Set Non-Maximum Suppression threshold, used to filter multiple detections at approximatively
  the same location.

  \param nmsThreshold Non-Maximum Suppression threshold between [0, 1]
*/
void vpDetectorDNNOpenCV::setNMSThreshold(const float &nmsThreshold) { m_netConfig.m_nmsThreshold = nmsThreshold; }

/*!
  Set the size ratio used in the \b filterDetection method. If <= 0., \b filterDetection is not used.
  The detections of class \b c for which the bbox area does not belong to [mean_class_c(Area) * \b sizeRatio ; mean_class_c(Area) / \b sizeRatio ]
  are removed from the list of detections.

  \param sizeRatio the size ratio used in the \b filterDetection method. If <= 0., \b filterDetection is not used.
*/
void vpDetectorDNNOpenCV::setDetectionFilterSizeRatio(const double &sizeRatio)
{
  m_netConfig.m_filterSizeRatio = sizeRatio;
  if (m_netConfig.m_filterSizeRatio > std::numeric_limits<double>::epsilon()) {
    m_applySizeFilterAfterNMS = true;
  }
  else {
    m_applySizeFilterAfterNMS = false;
  }
}

/*!
  Set dimension to resize the image to the input blob.

  \param width If <= 0, blob width is set to image width
  \param height If <= 0, blob height is set to image height
*/
void vpDetectorDNNOpenCV::setInputSize(const int &width, const int &height)
{
  m_netConfig.m_inputSize.width = width;
  m_netConfig.m_inputSize.height = height;
}

/*!
  Set mean subtraction values.

  \param meanR Mean value for R-channel
  \param meanG Mean value for G-channel
  \param meanB Mean value for R-channel
*/
void vpDetectorDNNOpenCV::setMean(const double &meanR, const double &meanG, const double &meanB) { m_netConfig.m_mean = cv::Scalar(meanR, meanG, meanB); }

/*!
  Set preferable backend for inference computation.
  See OpenCV setPreferableBackend documentation for more information.

  \param backendId Backend identifier
*/
void vpDetectorDNNOpenCV::setPreferableBackend(const int &backendId) { m_net.setPreferableBackend(backendId); }

/*!
  Set preferable target for inference computation.
  See OpenCV setPreferableTarget documentation for more information.

  \param targetId Target identifier
*/
void vpDetectorDNNOpenCV::setPreferableTarget(const int &targetId) { m_net.setPreferableTarget(targetId); }

/*!
  Set scale factor to normalize the range of pixel values.
*/
void vpDetectorDNNOpenCV::setScaleFactor(const double &scaleFactor)
{
  m_netConfig.m_scaleFactor = scaleFactor;
  if ((m_netConfig.m_parsingMethodType == YOLO_V7 || m_netConfig.m_parsingMethodType == YOLO_V8) && m_netConfig.m_scaleFactor != 1 / 255.) {
    std::cout << "[vpDetectorDNNOpenCV::setParsingMethod] WARNING: scale factor should be 1/255. to normalize pixels value." << std::endl;
  }
}

/*!
  If true, swap R and B channel for mean subtraction. For instance
  when the network has been trained on RGB image format (OpenCV uses
  BGR convention).
*/
void vpDetectorDNNOpenCV::setSwapRB(const bool &swapRB) { m_netConfig.m_swapRB = swapRB; }

/*!
  Set the type of parsing method that must be used to interpret the raw results of the DNN detection.

  \param typeParsingMethod: the type of parsing method that must be used to interpret the raw results of the DNN detection.
  \param parsingMethod: if \b typeParsingMethod is equal to \b vpDetectorDNNOpenCV::USER_DEFINED , a function permitting to interpret the \b cv::Mat
  resulting from the DNN inference.
*/
void vpDetectorDNNOpenCV::setParsingMethod(const DNNResultsParsingType &typeParsingMethod, void (*parsingMethod)(DetectionCandidates &, std::vector<cv::Mat> &, const NetConfig &))
{
  m_netConfig.m_parsingMethodType = typeParsingMethod;
  m_parsingMethod = parsingMethod;
  if ((m_netConfig.m_parsingMethodType == YOLO_V7 || m_netConfig.m_parsingMethodType == YOLO_V8) && m_netConfig.m_scaleFactor != 1 / 255.) {
    m_netConfig.m_scaleFactor = 1 / 255.;
    std::cout << "[vpDetectorDNNOpenCV::setParsingMethod] NB: scale factor changed to 1/255. to normalize pixels value." << std::endl;
  }

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  if (m_netConfig.m_parsingMethodType == SSD_MOBILENET) {
    std::cout << "[vpDetectorDNNOpenCV::setParsingMethod] WARNING: The chosen type of network is " << dnnResultsParsingTypeToString(m_netConfig.m_parsingMethodType) << " VISP_BUILD_DEPRECATED_FUNCTIONS is set to true." << std::endl;
    std::cout << "\tThe parsing method that worked with  the networks quoted in the ViSP documentation was postProcess_ResNet_10 instead of postProcess_SSD_MobileNet." << std::endl;
    std::cout << "\tIf the SSD-MobileNet network does not seem to work, please try to recompile ViSP setting VISP_BUILD_DEPRECATED_FUNCTIONS as false." << std::endl << std::flush;
  }
#endif
}

END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_core.a(vpDetectorDNNOpenCV.cpp.o) has no symbols
void dummy_vpDetectorDNN() { };
#endif
