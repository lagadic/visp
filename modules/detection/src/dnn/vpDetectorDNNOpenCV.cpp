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
#include <visp3/core/vpConfig.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x030403) && defined(VISP_HAVE_OPENCV_DNN)
#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorDNNOpenCV.h>


std::string vpDetectorDNNOpenCV::getAvailableDnnResultsParsingTypes()
{
  std::string list = "[";
  for(unsigned int i = 0; i < vpDetectorDNNOpenCV::COUNT - 1; i++)
  {
    list += "\"" + dnnResultsParsingTypeToString((vpDetectorDNNOpenCV::DNNResultsParsingType) i) + "\", ";
  }
  list += "\"" + dnnResultsParsingTypeToString((vpDetectorDNNOpenCV::DNNResultsParsingType) (vpDetectorDNNOpenCV::COUNT - 1)) + "\"]";
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
  switch(type)
  {
    case YOLO_V7:
      name = "yolov7";
      break;
    case YOLO_V8:
      name = "yolov8";
      break;
    case FASTER_RCNN:
      name = "faster-rcnn";
      break;
    case R_FCN:
      name = "r-fcn";
      break;
    case SSD_MOBILENET:
      name = "ssd-mobilenet";
      break;
    case RESNET_10:
      name = "resnet-10";
      break;
    case OLD_METHOD:
      name = "old-method";
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
  for(int id = 0; id < COUNT && !hasFoundMatch; id++)
  {
    vpDetectorDNNOpenCV::DNNResultsParsingType temp = (vpDetectorDNNOpenCV::DNNResultsParsingType) id;
    if(dnnResultsParsingTypeToString(temp) == name)
    {
      res = temp;
      hasFoundMatch = true;
    }
  }
  return res;
}

/*!
 *  
 * 
 * \param filename 
 * \return std::vector<std::string> 
 */
std::vector<std::string> vpDetectorDNNOpenCV::parseClassNamesFile(const std::string &filename)
{
  return NetConfig::parseClassNamesFile(filename);
}

vpDetectorDNNOpenCV::vpDetectorDNNOpenCV()
  : m_blob(), m_classIds(), m_I_color(), m_img(),
    m_mean(127.5, 127.5, 127.5), m_net(), m_netConfig(0.5, 0.4, std::vector<std::string>(), cv::Size(300, 300)), m_outNames(), m_dnnRes(),
    m_scaleFactor(2.0 / 255.0), m_swapRB(true), m_parsingMethodType(USER_SPECIFIED), m_parsingMethod(vpDetectorDNNOpenCV::postProcess_unimplemented)
{
}


vpDetectorDNNOpenCV::vpDetectorDNNOpenCV(const DNNResultsParsingType &typeParsingMethod, void (*parsingMethod)(DetectionCandidates &, std::vector<cv::Mat> &, const NetConfig &))
  : m_blob(), m_classIds(), m_I_color(), m_img(),
    m_mean(127.5, 127.5, 127.5), m_net(), m_netConfig(0.5, 0.4, std::vector<std::string>(), cv::Size(300, 300)), m_outNames(), m_dnnRes(),
    m_scaleFactor(2.0 / 255.0), m_swapRB(true), m_parsingMethodType(typeParsingMethod), m_parsingMethod(parsingMethod)
{
}

vpDetectorDNNOpenCV::~vpDetectorDNNOpenCV() {}

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

/*!
  Object detection using OpenCV DNN module.
  
  \param I : Input image.
  \param output: map where the name of the class is used as key and whose value is a vector of detected 2D features that belong to the class.
  \return false if there is no detection.
*/
bool vpDetectorDNNOpenCV::detect(const vpImage<vpRGBa> &I, std::map< std::string, std::vector<DetectedFeatures2D>> &output)
{
  vpImageConvert::convert(I, m_img);
  
  return detect(m_I_color, output);
}

/*!
  Object detection using OpenCV DNN module.
  
  \param I : Input image.
  \param output: vector of pairs <name_of_the_class, vector_of_detections>
  \return false if there is no detection.
*/
bool vpDetectorDNNOpenCV::detect(const vpImage<vpRGBa> &I, std::vector< std::pair<std::string, std::vector<DetectedFeatures2D>>> &output)
{
  vpImageConvert::convert(I, m_img);
  
  return detect(m_I_color, output);
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

  cv::Size inputSize(m_netConfig.m_inputSize.width  > 0 ? m_netConfig.m_inputSize.width : m_img.cols,
                     m_netConfig.m_inputSize.height > 0 ? m_netConfig.m_inputSize.height : m_img.rows);
  cv::dnn::blobFromImage(m_img, m_blob, m_scaleFactor, inputSize, m_mean, m_swapRB, false);

  m_net.setInput(m_blob);
  m_net.forward(m_dnnRes, m_outNames);

  postProcess(output);

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
  for(std::map<std::string, std::vector<DetectedFeatures2D>>::iterator it = map_output.begin(); it != map_output.end(); it++)
  {
    
    output.push_back(std::pair<std::string, std::vector<DetectedFeatures2D>>(it->first, it->second));
  }
  return returnStatus;
}


#if (VISP_HAVE_OPENCV_VERSION == 0x030403)
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

  \param output : map of pairs <name_of_the_class, vector_of_detections>
*/
void vpDetectorDNNOpenCV::postProcess(std::map< std::string, std::vector<DetectedFeatures2D>> &output)
{
  /////Detection proposals
  DetectionCandidates proposals;

  switch(m_parsingMethodType)
  {
    case YOLO_V7:
      postProcess_YoloV7(proposals, m_dnnRes, m_netConfig);
      break;
    case YOLO_V8:
      postProcess_YoloV8(proposals, m_dnnRes, m_netConfig);
      break;
    case FASTER_RCNN:
    case R_FCN:
      postProcess_YoloV8(proposals, m_dnnRes, m_netConfig);
      break;
    case SSD_MOBILENET:
      postProcess_SSD_MobileNet(proposals, m_dnnRes, m_netConfig);
      break;
    case RESNET_10:
      postProcess_ResNet_10(proposals, m_dnnRes, m_netConfig);
      break;
    case OLD_METHOD:
      postProcess_OldMethod(proposals, m_dnnRes, m_netConfig);
      break;
    case USER_SPECIFIED:
      m_parsingMethod(proposals, m_dnnRes, m_netConfig);
      break;
    default:
      throw(vpException(vpException::badValue, "Type of DNN post-processing method not handled."));
  }

  m_indices.clear();
  cv::dnn::NMSBoxes(proposals.m_boxes, proposals.m_confidences, m_netConfig.m_confThreshold, m_netConfig.m_nmsThreshold, m_indices);
  
  for ( size_t i = 0; i < m_indices.size(); ++i )
  {
    int idx      = m_indices[i];
    cv::Rect box = proposals.m_boxes[idx];
    std::string classname = m_netConfig.m_classNames[proposals.m_classIds[idx]];
    #if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
    std::optional<std::string> classname_opt = std::optional<std::string>(classname);
    #endif 
    output[classname].emplace_back( box.x, box.x + box.width, box.y, box.y + box.height
                                  , proposals.m_classIds[idx], proposals.m_confidences[idx]
                                  #if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
                                  , classname_opt
                                  #else
                                  , classname
                                  #endif  
                                  );
  }
}

/*!
  Post-process the raw results of a YoloV7-type  DNN.
  Extract the data stored as a matrix. They are stored as follow: 
  [batchsize][1:25200][1:5+nb_classes]
  Where 25200 is the number of detection proposals and a detection proposal consists of:
  [center_x; center_y; width; height; objectness; score_class_0; ...; score_last_class] 

  \param proposals : input/output that will contains all the detection candidates.
  \param dnnRes: raw results of the \b vpDetectorDNNOpenCV::detect step.
  \param netConfig: the configuration of the network, to know for instance the DNN input size.
*/
void vpDetectorDNNOpenCV::postProcess_YoloV7(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig)
{
  // Counts the number of proposed detections and the number of data corresponding to 1 detection
  int num_proposal = dnnRes[0].size[0]; // Number of detections
  int nout         = dnnRes[0].size[1]; // Number of data for each detection
  if ( dnnRes[0].dims > 2 )
  {
    num_proposal = dnnRes[0].size[1];
    nout         = dnnRes[0].size[2];
    dnnRes[0]      = dnnRes[0].reshape( 0, num_proposal );
  }

  // Compute the ratio between the original size of the image and the network size to translate network coordinates into
  // image coordinates
  float ratioh = (float)m_img.rows / netConfig.m_inputSize.height, ratiow = (float)m_img.cols / netConfig.m_inputSize.width;
  int n = 0, row_ind = 0; /// cx,cy,w,h,box_score,class_score
  float *pdata = (float *)dnnRes[0].data;

  // Iterate on the detections to keep only the meaningful ones
  for ( n = 0; n < num_proposal; n++ )
  {
    float box_score = pdata[4];
    if ( box_score > m_netConfig.m_confThreshold )
    {
      cv::Mat scores = dnnRes[0].row( row_ind ).colRange( 5, nout );
      cv::Point classIdPoint;
      double max_class_score;
      // Get the value and location of the maximum score
      cv::minMaxLoc( scores, 0, &max_class_score, 0, &classIdPoint );
      max_class_score *= box_score;

      // The detection is kept only if the confidence is greater than the threshold
      if ( max_class_score > m_netConfig.m_confThreshold )
      {
        const int class_idx = classIdPoint.x;
        float cx            = pdata[0] * ratiow; /// cx
        float cy            = pdata[1] * ratioh; /// cy
        float w             = pdata[2] * ratiow; /// w
        float h             = pdata[3] * ratioh; /// h

        int left = int( cx - 0.5 * w );
        int top  = int( cy - 0.5 * h );

        proposals.m_confidences.push_back( (float)max_class_score );
        proposals.m_boxes.push_back( cv::Rect( left, top, (int)( w ), (int)( h ) ) );
        proposals.m_classIds.push_back( class_idx );
      }
    }
    row_ind++;
    pdata += nout;
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
  // Counts the number of proposed detections and the number of data corresponding to 1 detection
  int num_proposal = dnnRes[0].size[1]; // Number of detections
  int nout         = dnnRes[0].size[0]; // Number of data for each detection
  if ( dnnRes[0].dims > 2 )
  {
    num_proposal = dnnRes[0].size[2];
    nout         = dnnRes[0].size[1];
    dnnRes[0]      = dnnRes[0].reshape( 0, num_proposal );
  }

  // Compute the ratio between the original size of the image and the network size to translate network coordinates into
  // image coordinates
  float ratioh = (float)m_img.rows / netConfig.m_inputSize.height, ratiow = (float)m_img.cols / netConfig.m_inputSize.width;

  // Iterate on the detections to keep only the meaningful ones
  for (int n = 0; n < num_proposal; n++ )
  {
    /// proposal = [cx,cy,w,h,classes_score]^T
    cv::Mat proposal = dnnRes[0].col( n );
    cv::Point classIdPoint;
    double max_class_score;
    // Get the value and location of the maximum score
    cv::minMaxLoc( proposal.rowRange( 4, nout ), 0, &max_class_score, 0, &classIdPoint );
      
    // The detection is kept only if the confidence is greater than the threshold
    if ( max_class_score > m_netConfig.m_confThreshold )
    {
      const int class_idx = classIdPoint.x;
      float cx            = proposal.at<double>(0,0) * ratiow; /// cx
      float cy            = proposal.at<double>(1,0) * ratioh; /// cy
      float w             = proposal.at<double>(2,0) * ratiow; /// w
      float h             = proposal.at<double>(3,0) * ratioh; /// h

      int left = int( cx - 0.5 * w );
      int top  = int( cy - 0.5 * h );

      proposals.m_confidences.push_back( (float)max_class_score );
      proposals.m_boxes.push_back( cv::Rect( left, top, (int)( w ), (int)( h ) ) );
      proposals.m_classIds.push_back( class_idx );
    }
  }
}

/*!
  Post-process the raw results of a Faster-RCNN-type or a R-FCN-type DNN.
  Extract the data stored as a matrix.
  The network produces output blob with a shape 1x1xNx7 where N is a number of
  detections and an every detection is a vector of values
  [batchId, classId, confidence, left, top, right, bottom]

  \param proposals : input/output that will contains all the detection candidates.
  \param dnnRes: raw results of the \b vpDetectorDNNOpenCV::detect step.
  \param netConfig: the configuration of the network, to know for instance the DNN input size.
*/
void vpDetectorDNNOpenCV::postProcess_FasterRCNN_RFCN(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig)
{
  // Direct copy from object_detection.cpp OpenCV sample
  // Faster-RCNN or R-FCN
  
  // Network produces output blob with a shape 1x1xNx7 where N is a number of
  // detections and an every detection is a vector of values
  // [batchId, classId, confidence, left, top, right, bottom]
  CV_Assert(dnnRes.size() == 1);
  float *data = (float *)dnnRes[0].data;
  for (size_t i = 0; i < dnnRes[0].total(); i += 7) 
  {
    float confidence = data[i + 2];
    if (confidence > netConfig.m_confThreshold) 
    {
      int left = (int)data[i + 3];
      int top = (int)data[i + 4];
      int right = (int)data[i + 5];
      int bottom = (int)data[i + 6];
      int classId = (int)(data[i + 1]) - 1;
        
      proposals.m_confidences.push_back( (float)confidence );
      proposals.m_boxes.push_back( cv::Rect( left, top, right - left + 1, bottom - top + 1 ) );
      proposals.m_classIds.push_back( classId );
    }
  }
}

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
  for (int i = 0; i < N; i++) 
  {
    uint32_t maxClass = 0;
    float maxScore = -1000.0f;

    for (int j = 1; j < C; j++) // ignore background (classId = 0).
    {
      const float score = confidence[i * C + j];

      if (score < netConfig.m_confThreshold)
        continue;

      if (score > maxScore) 
      {
        maxScore = score;
        maxClass = j;
      }
    }

    if (maxScore > netConfig.m_confThreshold) 
    {
      int left = (int)(bbox[4 * i] * m_img.cols);
      int top = (int)(bbox[4 * i + 1] * m_img.rows);
      int right = (int)(bbox[4 * i + 2] * m_img.cols);
      int bottom = (int)(bbox[4 * i + 3] * m_img.rows);
      int width = right - left + 1;
      int height = bottom - top + 1;

      int classId = maxClass;
      proposals.m_confidences.push_back( maxScore );
      proposals.m_boxes.push_back( cv::Rect( left, top, width, height ) );
      proposals.m_classIds.push_back( classId );
    }
  }
}

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
  for (size_t i = 0; i < dnnRes[0].total(); i += 7) 
  {
    float confidence = data[i + 2];
    if (confidence > netConfig.m_confThreshold) 
    {
      int left = (int)(data[i + 3] * m_img.cols);
      int top = (int)(data[i + 4] * m_img.rows);
      int right = (int)(data[i + 5] * m_img.cols);
      int bottom = (int)(data[i + 6] * m_img.rows);
      int classId = (int)(data[i + 1]) - 1;
        
      proposals.m_confidences.push_back( (float)confidence );
      proposals.m_boxes.push_back( cv::Rect( left, top, right - left + 1, bottom - top + 1 ) );
      proposals.m_classIds.push_back( classId );
    }
  }
}

/*!
  Post-process the raw results of a YoloV7-type  DNN.
  Extract the data stored as a matrix.
  The network produces output blob with a shape NxC where N is a number of
  detected objects and C is a number of classes + 4 where the first 4
  numbers are [center_x, center_y, width, height]

  \param proposals : input/output that will contains all the detection candidates.
  \param dnnRes: raw results of the \b vpDetectorDNNOpenCV::detect step.
  \param netConfig: the configuration of the network, to know for instance the DNN input size.
*/
void vpDetectorDNNOpenCV::postProcess_OldMethod(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig)
{
  // Direct copy from object_detection.cpp OpenCV sample
  static std::vector<int> outLayers = m_net.getUnconnectedOutLayers();
  static std::string outLayerType = m_net.getLayer(outLayers[0])->type;

  if (outLayerType == "Region") 
  {
    for (size_t i = 0; i < dnnRes.size(); ++i) 
    {
      // Network produces output blob with a shape NxC where N is a number of
      // detected objects and C is a number of classes + 4 where the first 4
      // numbers are [center_x, center_y, width, height]
      float *data = (float *)dnnRes[i].data;
      for (int j = 0; j < dnnRes[i].rows; ++j, data += dnnRes[i].cols) 
      {
        cv::Mat scores = dnnRes[i].row(j).colRange(5, dnnRes[i].cols);
        cv::Point classIdPoint;
        double confidence;
        cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
        if (confidence > netConfig.m_confThreshold) 
        {
          int centerX = (int)(data[0] * m_img.cols);
          int centerY = (int)(data[1] * m_img.rows);
          int width = (int)(data[2] * m_img.cols);
          int height = (int)(data[3] * m_img.rows);
          int left = centerX - width / 2;
          int top = centerY - height / 2;
          int classId = classIdPoint.x;
          
          proposals.m_confidences.push_back( (float)confidence );
          proposals.m_boxes.push_back( cv::Rect( left, top, width, height ) );
          proposals.m_classIds.push_back( classId );
        }
      }
    }
  } else
    CV_Error(cv::Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);
}

/*!
  Method throwing a \b vpException::functionNotImplementedError . It is called if the user set the
  \b m_parsingMethodType to \b USER_DEFINED but didn't set the parsing method.

  \param proposals : input/output that will contains all the detection candidates.
  \param dnnRes: raw results of the \b vpDetectorDNNOpenCV::detect step.
  \param netConfig: the configuration of the network, to know for instance the DNN input size.
*/
void vpDetectorDNNOpenCV::postProcess_unimplemented(DetectionCandidates &proposals, std::vector<cv::Mat> &dnnRes, const NetConfig &netConfig)
{
  (void) proposals;
  (void) dnnRes;
  (void) netConfig;
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
  \param classFile: path towards the file containing the list of classes detected by the DNN, in json format.
  \param config Path to a text file of model containing network configuration.
  It could be a file with the following extensions:
    - `*.prototxt` (Caffe, http://caffe.berkeleyvision.org/)
    - `*.pbtxt` (TensorFlow, https://www.tensorflow.org/)
    - `*.cfg` (Darknet, https://pjreddie.com/darknet/)
    - `*.xml` (DLDT, https://software.intel.com/openvino-toolkit)
  \param framework Optional name of an origin framework of the model. Automatically detected if it is not set.
*/
void vpDetectorDNNOpenCV::readNet(const std::string &model, const std::string &classFile, const std::string &config, const std::string &framework)
{
  m_net = cv::dnn::readNet(model, config, framework);
#if (VISP_HAVE_OPENCV_VERSION == 0x030403)
  m_outNames = getOutputsNames();
#else
  m_outNames = m_net.getUnconnectedOutLayersNames();
#endif

  m_netConfig.m_classNames = parseClassNamesFile(classFile);
}

/*!
  Set confidence threshold to filter the detections.

  \param confThreshold Confidence threshold between [0, 1]
*/
void vpDetectorDNNOpenCV::setConfidenceThreshold(float confThreshold) { m_netConfig.m_confThreshold = confThreshold; }

/*!
  Set dimension to resize the image to the input blob.

  \param width If <= 0, blob width is set to image width
  \param height If <= 0, blob height is set to image height
*/
void vpDetectorDNNOpenCV::setInputSize(int width, int height)
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
void vpDetectorDNNOpenCV::setMean(double meanR, double meanG, double meanB) { m_mean = cv::Scalar(meanR, meanG, meanB); }

/*!
  Set Non-Maximum Suppression threshold, used to filter multiple detections at approximatively
  the same location.

  \param nmsThreshold Non-Maximum Suppression threshold between [0, 1]
*/
void vpDetectorDNNOpenCV::setNMSThreshold(float nmsThreshold) { m_netConfig.m_nmsThreshold = nmsThreshold; }

/*!
  Set preferable backend for inference computation.
  See OpenCV setPreferableBackend documentation for more information.

  \param backendId Backend identifier
*/
void vpDetectorDNNOpenCV::setPreferableBackend(int backendId) { m_net.setPreferableBackend(backendId); }

/*!
  Set preferable target for inference computation.
  See OpenCV setPreferableTarget documentation for more information.

  \param targetId Target identifier
*/
void vpDetectorDNNOpenCV::setPreferableTarget(int targetId) { m_net.setPreferableTarget(targetId); }

/*!
  Set scale factor to normalize the range of pixel values.
*/
void vpDetectorDNNOpenCV::setScaleFactor(double scaleFactor) { m_scaleFactor = scaleFactor; }

/*!
  If true, swap R and B channel for mean subtraction. For instance
  when the network has been trained on RGB image format (OpenCV uses
  BGR convention).
*/
void vpDetectorDNNOpenCV::setSwapRB(bool swapRB) { m_swapRB = swapRB; }

/*!
  Set the type of parsing method that must be used to interpret the raw results of the DNN detection.

  \param typeParsingMethod: the type of parsing method that must be used to interpret the raw results of the DNN detection.
  \param parsingMethod: if \b typeParsingMethod is equal to \b vpDetectorDNNOpenCV::USER_DEFINED , a function permitting to interpret the \b cv::Mat
  resulting from the DNN inference. 
*/
void vpDetectorDNNOpenCV::setParsingMethod(const DNNResultsParsingType &typeParsingMethod, void (*parsingMethod)(DetectionCandidates &, std::vector<cv::Mat>&, const NetConfig &))
{
  m_parsingMethodType = typeParsingMethod;
  m_parsingMethod = parsingMethod;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_core.a(vpDetectorDNNOpenCV.cpp.o) has no
// symbols
void dummy_vpDetectorDNN(){};
#endif
