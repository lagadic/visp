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

#if (VISP_HAVE_OPENCV_VERSION >= 0x030403)
#include <opencv2/dnn.hpp>
#include <visp3/detection/vpDetectorBase.h>

/*!
  \class vpDetectorDNN
  \ingroup group_detection_dnn
  This class is a wrapper over the <a href="https://docs.opencv.org/master/d6/d0f/group__dnn.html">
  OpenCV DNN module</a> and specialized to handle object detection task.

  Example is provided in tutorial-dnn-object-detection-live.cpp
*/
class VISP_EXPORT vpDetectorDNN : public vpDetectorBase
{
public:
  vpDetectorDNN();
  virtual ~vpDetectorDNN();

  virtual bool detect(const vpImage<unsigned char> &I);
  virtual bool detect(const vpImage<vpRGBa> &I, std::vector<vpRect> &boundingBoxes);

  std::vector<vpRect> getDetectionBBs(bool afterNMS=true) const;
  std::vector<int> getDetectionClassIds(bool afterNMS=true) const;
  std::vector<float> getDetectionConfidence(bool afterNMS=true) const;

  void readNet(const std::string &model, const std::string &config="", const std::string &framework="");
  void setConfidenceThreshold(float confThreshold);
  void setInputSize(int width, int height);
  void setMean(double meanR, double meanG, double meanB);
  void setNMSThreshold(float nmsThreshold);
  void setPreferableBackend(int backendId);
  void setPreferableTarget(int targetId);
  void setScaleFactor(double scaleFactor);
  void setSwapRB(bool swapRB);

private:
#if (VISP_HAVE_OPENCV_VERSION == 0x030403)
  std::vector<cv::String> getOutputsNames();
#endif
  void postProcess();

  //! Buffer for the blob in input net
  cv::Mat m_blob;
  //! Detection bounding boxes
  std::vector<cv::Rect> m_boxes;
  //! Detection bounding boxes after Non-Maximum Suppression
  std::vector<cv::Rect> m_boxesNMS;
  //! Detection class ids
  std::vector<int> m_classIds;
  //! Detection confidence
  std::vector<float> m_confidences;
  //! Threshold to filter detections by confidence
  float m_confidenceThreshold;
  //! Buffer for gray to RGBa image conversion
  vpImage<vpRGBa> m_I_color;
  //! Buffer for the input image
  cv::Mat m_img;
  //! Indices for NMS
  std::vector<int> m_indices;
  //! Blob size
  cv::Size m_inputSize;
  //! Values for mean subtraction
  cv::Scalar m_mean;
  //! DNN network
  cv::dnn::Net m_net;
  //! Threshold for Non-Maximum Suppression
  float m_nmsThreshold;
  //! Names of layers with unconnected outputs
  std::vector<cv::String> m_outNames;
  //! Contains all output blobs for each layer specified in m_outNames
  std::vector<cv::Mat> m_outs;
  //! Scale factor to normalize pixel values
  double m_scaleFactor;
  //! If true, swap R and B for mean subtraction, e.g. when a model has been trained on BGR image format
  bool m_swapRB;
};
#endif
#endif
