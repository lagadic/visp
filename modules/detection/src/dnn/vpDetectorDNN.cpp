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

#if (VISP_HAVE_OPENCV_VERSION >= 0x030403)
#include <visp3/detection/vpDetectorDNN.h>
#include <visp3/core/vpImageConvert.h>

vpDetectorDNN::vpDetectorDNN() : m_blob(), m_boxes(), m_classIds(), m_confidences(),
    m_confidenceThreshold(0.5), m_I_color(), m_img(), m_inputSize(300,300), m_mean(127.5, 127.5, 127.5),
    m_net(), m_nmsThreshold(0.4f), m_outNames(), m_outs(), m_scaleFactor(2.0/255.0), m_swapRB(true) {}

vpDetectorDNN::~vpDetectorDNN() {}

/*!
  Object detection using OpenCV DNN module.
  \warning Classical object detection network uses as input 3-channels.
  Grayscale image is converted to color image.

  \param I : Input image.
  \return false if there is no detection.
*/
bool vpDetectorDNN::detect(const vpImage<unsigned char> &I) {
  vpImageConvert::convert(I, m_I_color);

  std::vector<vpRect> boundingBoxes;
  return detect(m_I_color, boundingBoxes);
}

/*!
  Object detection using OpenCV DNN module.

  \param I : Input image.
  \param boundingBoxes : Vector of detection bounding boxes.
  \return false if there is no detection.
*/
bool vpDetectorDNN::detect(const vpImage<vpRGBa> &I, std::vector<vpRect> &boundingBoxes) {
  vpImageConvert::convert(I, m_img);

  cv::Size inputSize(m_inputSize.width > 0 ? m_inputSize.width : m_img.cols,
                     m_inputSize.height > 0 ? m_inputSize.height : m_img.rows);
  cv::dnn::blobFromImage(m_img, m_blob, m_scaleFactor, inputSize, m_mean, m_swapRB, false);

  m_net.setInput(m_blob);
  m_net.forward(m_outs, m_outNames);

  postProcess();

  boundingBoxes.resize(m_boxesNMS.size());
  for (size_t i = 0; i < m_boxesNMS.size(); i++) {
    cv::Rect box = m_boxesNMS[i];
    boundingBoxes[i] = vpRect(box.x, box.y, box.width, box.height);
  }

  m_nb_objects = boundingBoxes.size();
  m_polygon.resize(boundingBoxes.size());
  m_message.resize(boundingBoxes.size());
  for (size_t i = 0; i < boundingBoxes.size(); i++) {
    std::vector<vpImagePoint> polygon;

    double x = boundingBoxes[i].getLeft();
    double y = boundingBoxes[i].getTop();
    double w = boundingBoxes[i].getWidth();
    double h = boundingBoxes[i].getHeight();

    polygon.push_back(vpImagePoint(y, x));
    polygon.push_back(vpImagePoint(y + h, x));
    polygon.push_back(vpImagePoint(y + h, x + w));
    polygon.push_back(vpImagePoint(y, x + w));

    m_polygon[i] = polygon;

    std::ostringstream oss;
    oss << m_classIds[i] << " ; " << m_confidences[i] << " ; " << m_boxes[i];
    m_message[i] = oss.str();
  }

  return !boundingBoxes.empty();
}

/*!
  Get raw detection bounding boxes.

  \param afterNMS If true, return detection bounding boxes after NMS
*/
std::vector<vpRect> vpDetectorDNN::getDetectionBBs(bool afterNMS) const {
  std::vector<vpRect> bbs;
  if (afterNMS) {
    bbs.reserve(m_boxesNMS.size());
    for (size_t i = 0; i < m_boxesNMS.size(); i++) {
      cv::Rect box = m_boxes[i];
      bbs.push_back(vpRect(box.x, box.y, box.width, box.height));
    }
  } else {
    bbs.reserve(m_boxes.size());
    for (size_t i = 0; i < m_boxes.size(); i++) {
      cv::Rect box = m_boxes[i];
      bbs.push_back(vpRect(box.x, box.y, box.width, box.height));
    }
  }

  return bbs;
}

/*!
  Get detection class ids.

  \param afterNMS If true, returns class ids after NMS
*/
std::vector<int> vpDetectorDNN::getDetectionClassIds(bool afterNMS) const {
  if (afterNMS) {
    std::vector<int> classIds;
    for (size_t i = 0; i < m_indices.size(); i++) {
      int idx = m_indices[i];
      classIds.push_back(m_classIds[idx]);
    }
    return classIds;
  }

  return m_classIds;
}

/*!
  Get detection confidences.
*/
std::vector<float> vpDetectorDNN::getDetectionConfidence(bool afterNMS) const {
  if (afterNMS) {
    std::vector<float> confidences;
    for (size_t i = 0; i < m_indices.size(); i++) {
      int idx = m_indices[i];
      confidences.push_back(m_confidences[idx]);
    }
    return confidences;
  }

  return m_confidences;
}

#if (VISP_HAVE_OPENCV_VERSION == 0x030403)
std::vector<cv::String> vpDetectorDNN::getOutputsNames() {
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

void vpDetectorDNN::postProcess() {
  //Direct copy from object_detection.cpp OpenCV sample
  static std::vector<int> outLayers = m_net.getUnconnectedOutLayers();
  static std::string outLayerType = m_net.getLayer(outLayers[0])->type;

  m_classIds.clear();
  m_confidences.clear();
  m_boxes.clear();
  if (m_net.getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
  {
    // Network produces output blob with a shape 1x1xNx7 where N is a number of
    // detections and an every detection is a vector of values
    // [batchId, classId, confidence, left, top, right, bottom]
    CV_Assert(m_outs.size() == 1);
    float* data = (float*)m_outs[0].data;
    for (size_t i = 0; i < m_outs[0].total(); i += 7)
    {
      float confidence = data[i + 2];
      if (confidence > m_confidenceThreshold)
      {
        int left = (int)data[i + 3];
        int top = (int)data[i + 4];
        int right = (int)data[i + 5];
        int bottom = (int)data[i + 6];
        int width = right - left + 1;
        int height = bottom - top + 1;
        m_classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
        m_boxes.push_back(cv::Rect(left, top, width, height));
        m_confidences.push_back(confidence);
      }
    }
  }
  else if (outLayerType == "DetectionOutput")
  {
    // Network produces output blob with a shape 1x1xNx7 where N is a number of
    // detections and an every detection is a vector of values
    // [batchId, classId, confidence, left, top, right, bottom]
    CV_Assert(m_outs.size() == 1);
    float* data = (float*)m_outs[0].data;
    for (size_t i = 0; i < m_outs[0].total(); i += 7)
    {
      float confidence = data[i + 2];
      if (confidence > m_confidenceThreshold)
      {
        int left = (int)(data[i + 3] * m_img.cols);
        int top = (int)(data[i + 4] * m_img.rows);
        int right = (int)(data[i + 5] * m_img.cols);
        int bottom = (int)(data[i + 6] * m_img.rows);
        int width = right - left + 1;
        int height = bottom - top + 1;
        m_classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
        m_boxes.push_back(cv::Rect(left, top, width, height));
        m_confidences.push_back(confidence);
      }
    }
  }
  else if (outLayerType == "Region")
  {
    for (size_t i = 0; i < m_outs.size(); ++i)
    {
      // Network produces output blob with a shape NxC where N is a number of
      // detected objects and C is a number of classes + 4 where the first 4
      // numbers are [center_x, center_y, width, height]
      float* data = (float*)m_outs[i].data;
      for (int j = 0; j < m_outs[i].rows; ++j, data += m_outs[i].cols)
      {
        cv::Mat scores = m_outs[i].row(j).colRange(5, m_outs[i].cols);
        cv::Point classIdPoint;
        double confidence;
        cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
        if (confidence > m_confidenceThreshold)
        {
          int centerX = (int)(data[0] * m_img.cols);
          int centerY = (int)(data[1] * m_img.rows);
          int width = (int)(data[2] * m_img.cols);
          int height = (int)(data[3] * m_img.rows);
          int left = centerX - width / 2;
          int top = centerY - height / 2;

          m_classIds.push_back(classIdPoint.x);
          m_confidences.push_back((float)confidence);
          m_boxes.push_back(cv::Rect(left, top, width, height));
        }
      }
    }
  }
  else
    CV_Error(cv::Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);

  cv::dnn::NMSBoxes(m_boxes, m_confidences, m_confidenceThreshold, m_nmsThreshold, m_indices);
  m_boxesNMS.resize(m_indices.size());
  for (size_t i = 0; i < m_indices.size(); ++i)
  {
      int idx = m_indices[i];
      m_boxesNMS[i] = m_boxes[idx];
  }
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
  \param config Path to a text file of model containing network configuration.
  It could be a file with the following extensions:
    - `*.prototxt` (Caffe, http://caffe.berkeleyvision.org/)
    - `*.pbtxt` (TensorFlow, https://www.tensorflow.org/)
    - `*.cfg` (Darknet, https://pjreddie.com/darknet/)
    - `*.xml` (DLDT, https://software.intel.com/openvino-toolkit)
  \param framework Optional name of an origin framework of the model. Automatically detected if it is not set.
*/
void vpDetectorDNN::readNet(const std::string &model, const std::string &config, const std::string &framework) {
  m_net = cv::dnn::readNet(model, config, framework);
#if (VISP_HAVE_OPENCV_VERSION == 0x030403)
  m_outNames = getOutputsNames();
#else
  m_outNames = m_net.getUnconnectedOutLayersNames();
#endif
}

/*!
  Set confidence threshold to filter the detections.

  \param confThreshold Confidence threshold between [0, 1]
*/
void vpDetectorDNN::setConfidenceThreshold(float confThreshold) {
  m_confidenceThreshold = confThreshold;
}

/*!
  Set dimension to resize the image to the input blob.

  \param width If <= 0, blob width is set to image width
  \param height If <= 0, blob height is set to image height
*/
void vpDetectorDNN::setInputSize(int width, int height) {
  m_inputSize.width = width;
  m_inputSize.height = height;
}

/*!
  Set mean subtraction values.

  \param meanR Mean value for R-channel
  \param meanG Mean value for G-channel
  \param meanB Mean value for R-channel
*/
void vpDetectorDNN::setMean(double meanR, double meanG, double meanB) {
  m_mean = cv::Scalar(meanR, meanG, meanB);
}

/*!
  Set Non-Maximum Suppression threshold, used to filter multiple detections at approximatively
  the same location.

  \param nmsThreshold Non-Maximum Suppression threshold between [0, 1]
*/
void vpDetectorDNN::setNMSThreshold(float nmsThreshold) {
  m_nmsThreshold = nmsThreshold;
}

/*!
  Set preferable backend for inference computation.
  See OpenCV setPreferableBackend documentation for more information.

  \param backendId Backend identifier
*/
void vpDetectorDNN::setPreferableBackend(int backendId) {
  m_net.setPreferableBackend(backendId);
}

/*!
  Set preferable target for inference computation.
  See OpenCV setPreferableTarget documentation for more information.

  \param targetId Target identifier
*/
void vpDetectorDNN::setPreferableTarget(int targetId) {
  m_net.setPreferableTarget(targetId);
}

/*!
  Set scale factor to normalize the range of pixel values.
*/
void vpDetectorDNN::setScaleFactor(double scaleFactor) {
  m_scaleFactor = scaleFactor;
}

/*!
  If true, swap R and B channel for mean subtraction. For instance
  when the network has been trained on RGB image format (OpenCV uses
  BGR convention).
*/
void vpDetectorDNN::setSwapRB(bool swapRB) {
  m_swapRB = swapRB;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDetectorDNN.cpp.o) has no
// symbols
void dummy_vpDetectorDNN(){};
#endif
