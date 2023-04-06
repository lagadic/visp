/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
#ifndef _vpDetectorDNN_h_
#define _vpDetectorDNN_h_

#include <visp3/core/vpConfig.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(VISP_HAVE_OPENCV_DNN) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
#include <map>
#include <string>
#include <vector>

#include <opencv2/dnn.hpp>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRect.h>

#include <optional>

/*!
  \class vpDetectorDNNOpenCV
  \ingroup group_detection_dnn
  This class is a wrapper over the <a href="https://docs.opencv.org/master/d6/d0f/group__dnn.html">
  OpenCV DNN module</a> and specialized to handle object detection task.

  Example is provided in tutorial-dnn-object-detection-live.cpp
*/
class VISP_EXPORT vpDetectorDNNOpenCV
{
public:
  typedef enum DNNResultsParsingType
  {
    USER_SPECIFIED =  0,
    FASTER_RCNN    =  1,
    R_FCN          =  2,
    SSD_MOBILENET  =  3,
    RESNET_10      =  4,
    YOLO_V3        =  5,
    YOLO_V4        =  6,
    YOLO_V7        =  7,
    YOLO_V8        =  8,
    COUNT          =  9
  } DNNResultsParsingType;

  typedef struct DetectionCandidates
  {
    std::vector< float > m_confidences;
    std::vector< cv::Rect > m_boxes;
    std::vector< int > m_classIds;
  } DetectionCandidates;

  typedef struct DetectedFeatures2D
  {
    vpRect m_bbox;
    double m_score;
    unsigned int m_cls;
    std::optional<std::string> m_classname;

    inline explicit DetectedFeatures2D( double u_min, double u_max
                                      , double v_min, double v_max
                                      , unsigned int cls, double score
                                      , const std::optional<std::string> &classname
                                      )
    : m_bbox( vpImagePoint(v_min, u_min), vpImagePoint(v_max, u_max))
    , m_score(score) 
    , m_cls(cls)
    {
      if(classname)
      {
        m_classname = classname;
      }
      else
      {
        m_classname = std::nullopt;
      }
    };

    template < typename Type >
    void display( const vpImage< Type > &img, const vpColor &color = vpColor::blue, unsigned int thickness = 1 ) const;

  } DetectedFeatures2D;

  typedef struct NetConfig
  {
    double m_confThreshold; //! Threshold to filter detections by confidence
    double m_nmsThreshold;  //! Threshold for Non-Maximum Suppression
    std::vector<std::string> m_classNames; //! Vector containing the names of the different classes the DNN can detect
    cv::Size m_inputSize; //! Size of the images the DNN can manipulate. The input images will be resized to match these dimensions.
    double m_filterSizeRatio; //! Size ratio used by the \b filterDetection method. If <= 0., the \b filterDetection method is not used.

    inline static std::vector<std::string> parseClassNamesFile(const std::string &filename)
    {
      std::vector<std::string> classNames;
      std::ifstream ifs(filename);
      std::string line;
      while (getline(ifs, line)) {
        if (line.find("[") == std::string::npos) {
          classNames.push_back(line);
        } else {
          std::string lineWithoutBracket;
          if (line.find("[") != std::string::npos) {
            lineWithoutBracket = line.substr(line.find("[") + 1, line.size() - 2); // Remove opening and closing brackets
          }

          while (!lineWithoutBracket.empty()) {
            std::string className;
            auto start_pos = lineWithoutBracket.find("\"");
            auto end_pos = lineWithoutBracket.find("\"", start_pos + 1);
            className = lineWithoutBracket.substr(start_pos + 1, end_pos - (start_pos + 1));
            if (!className.empty()) {
              classNames.push_back(className);
              lineWithoutBracket = lineWithoutBracket.substr(end_pos + 1);
            }
          }
        }
      }
      return classNames;
    }

    inline NetConfig(double confThresh, const double &nmsThresh, const std::vector<std::string> & classNames, const cv::Size &dnnInputSize, const double &filterSizeRatio = 0.)
      : m_confThreshold(confThresh)
      , m_nmsThreshold(nmsThresh)
      , m_classNames(classNames)
      , m_inputSize(dnnInputSize)
      , m_filterSizeRatio(filterSizeRatio)
    {
    }

    inline NetConfig(double confThresh, const double &nmsThresh, const std::string &classNamesFile, const cv::Size &dnnInputSize, const double &filterSizeRatio = 0.)
      : m_confThreshold(confThresh)
      , m_nmsThreshold(nmsThresh)
      , m_inputSize(dnnInputSize)
      , m_filterSizeRatio(filterSizeRatio)
    {
      m_classNames = parseClassNamesFile(classNamesFile);
    }
  } NetConfig;

  static std::string getAvailableDnnResultsParsingTypes();
  static std::string dnnResultsParsingTypeToString(const DNNResultsParsingType &type);
  static DNNResultsParsingType dnnResultsParsingTypeFromString(const std::string &name);
  static std::vector<std::string> parseClassNamesFile(const std::string &filename);
  vpDetectorDNNOpenCV();
  vpDetectorDNNOpenCV(const NetConfig &config,const DNNResultsParsingType &typeParsingMethod, void (*parsingMethod)(DetectionCandidates &, std::vector<cv::Mat>&, const NetConfig &) = postProcess_unimplemented);
  virtual ~vpDetectorDNNOpenCV();

  virtual bool detect(const vpImage<unsigned char> &I, std::map< std::string, std::vector<DetectedFeatures2D>> &output);
  virtual bool detect(const vpImage<unsigned char> &I, std::vector< std::pair<std::string, std::vector<DetectedFeatures2D>>> &output);
  virtual bool detect(const vpImage<vpRGBa> &I, std::map< std::string, std::vector<DetectedFeatures2D>> &output);
  virtual bool detect(const vpImage<vpRGBa> &I, std::vector< std::pair<std::string, std::vector<DetectedFeatures2D>>> &output);
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
  void setParsingMethod(const DNNResultsParsingType &typeParsingMethod, void (*parsingMethod)(DetectionCandidates &, std::vector<cv::Mat>&, const NetConfig &) = postProcess_unimplemented);

protected:
#if (VISP_HAVE_OPENCV_VERSION == 0x030403)
  std::vector<cv::String> getOutputsNames();
#endif
  std::vector<DetectedFeatures2D>
  filterDetection(const std::vector<DetectedFeatures2D>& detected_features, const double minRatioOfAreaOk);

  void postProcess(std::map< std::string, std::vector<DetectedFeatures2D>> &output);

  void postProcess_YoloV3_V4(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

  void postProcess_YoloV7(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

  void postProcess_YoloV8(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

  void postProcess_FasterRCNN_RFCN(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

  void postProcess_SSD_MobileNet(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

  void postProcess_ResNet_10(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);
  
  void postProcess_OldMethod(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig);

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
  //! Values for mean subtraction
  cv::Scalar m_mean;
  //! DNN network
  cv::dnn::Net m_net;
  //! Configuration of the DNN
  NetConfig m_netConfig;
  //! Names of layers with unconnected outputs
  std::vector<cv::String> m_outNames;
  //! Contains all output blobs for each layer specified in m_outNames
  std::vector<cv::Mat> m_dnnRes;
  //! Scale factor to normalize pixel values
  double m_scaleFactor;
  //! If true, swap R and B for mean subtraction, e.g. when a model has been trained on BGR image format
  bool m_swapRB;
  //! Parsing method that should be used to parse the cv::Mat returned by the cv::dnn::Net::forward method
  DNNResultsParsingType m_parsingMethodType;
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
vpDetectorDNNOpenCV::DetectedFeatures2D::display( const vpImage< Type > &img, const vpColor &color, unsigned int thickness ) const
{
  vpDisplay::displayRectangle( img, m_bbox, color, false, thickness );

  std::stringstream ss;
  if(m_classname)
  {
    ss << *m_classname;
  }
  else
  {
    ss << m_cls;
  }
  ss << "(" << std::setprecision( 4 ) << m_score * 100. << "%)";
  vpDisplay::displayText( img, m_bbox.getTopRight(), ss.str(), color );
}
#endif
#endif