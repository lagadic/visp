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

  //! [vpDetectorDNNOpenCV supported models]
  \section dnn_supported_models Supported detection models
  \subsection dnn_supported_faster_rcnn Faster-RCNN

  You can find the config file [here](https://github.com/opencv/opencv_extra/blob/master/testdata/dnn/faster_rcnn_inception_v2_coco_2018_01_28.pbtxt) and the weights [there](http://download.tensorflow.org/models/object_detection/faster_rcnn_inception_v2_coco_2018_01_28.tar.gz).

  To run the tutorial with the `Faster-RCNN` network, please run the following commands:
  ```
  $ DNN_PATH=/path/to/my/dnn/folder \
    MODEL=${DNN_PATH}/Faster-RCNN/weights/faster_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb \
    LABELS=${DNN_PATH}/Faster-RCNN/cfg/coco_classes.txt \
    CONFIG=${DNN_PATH}/Faster-RCNN/cfg/config.pbtxt \
    TYPE=faster-rcnn \
    FRAMEWORK=none \
    WIDTH=450 HEIGHT=258 
  $ ./tutorial-dnn-object-detection-live --model $MODEL --labels $LABELS --config $CONFIG --type $TYPE \
      --framework $FRAMEWORK --width $WIDTH --height $HEIGHT --nmsThresh 0.5 --mean 0 0 0 \
      --confThresh 0.35 --filterThresh -0.25 --scale 1 
  ```

  \subsection dnn_supported_mobilenet_ssd MobileNet SSD

  If you want to use `Mobilenet V1`, you can find the config file [here](https://raw.githubusercontent.com/Qengineering/MobileNet_SSD_OpenCV_TensorFlow/master/ssd_mobilenet_v1_coco_2017_11_17.pbtxt) and the weights [there](http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_2017_11_17.tar.gz) (you need to use the `frozen_inference_graph.pb` file).
  The parameters to use with this network were found [there](https://github.com/opencv/opencv/blob/0052d46b8e33c7bfe0e1450e4bff28b88f455570/samples/dnn/models.yml#L68).

  To run the tutorial with the `Mobilenet V1` network, please run the following commands:
  ```
  $ DNN_PATH=/path/to/my/dnn/folder \
    MODEL=${DNN_PATH}/MobileNet-SSD/weights/ssd-mobilenet.pb \
    LABELS=${DNN_PATH}/MobileNet-SSD/cfg/coco_classes.txt \
    TYPE=ssd-mobilenet \
    FRAMEWORK=none \
    CONFIG=${DNN_PATH}/MobileNet-SSD/cfg/ssd-mobilenet.pbtxt \
    WIDTH=300 HEIGHT=300
  $ ./tutorial-dnn-object-detection-live --model $MODEL --labels $LABELS --config $CONFIG --type $TYPE \
        --framework $FRAMEWORK --width $WIDTH --height $HEIGHT --nmsThresh 0.5 --mean 0 0 0 \
        --filterThresh -0.25 --scale 1
  ```

  If you would rather use the v3 of Mobilenet-SSD, please download the config file [here](https://gist.github.com/dkurt/54a8e8b51beb3bd3f770b79e56927bd7) and the weights [there](http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v3_large_coco_2020_01_14.tar.gz).
  Then, to run the tutorial with the `Mobilenet V3` network, please run the following commands:
  ```
  $ DNN_PATH=/path/to/my/dnn/folder \
    MODEL=${DNN_PATH}/MobileNet-SSD/weights/ssd-mobilenet_v3.pb \
    LABELS=${DNN_PATH}/MobileNet-SSD/cfg/coco_classes.txt \
    TYPE=ssd-mobilenet \ 
    FRAMEWORK=none \ 
    CONFIG=${DNN_PATH}/MobileNet-SSD/cfg/ssd-mobilenet_v3.pbtxt \
    WIDTH=320 HEIGHT=320  
  $ ./tutorial-dnn-object-detection-live --model $MODEL --labels $LABELS --config $CONFIG --type $TYPE \
        --framework $FRAMEWORK --width $WIDTH --height $HEIGHT --nmsThresh 0.5 --mean 0.0019 0.0019 0.0019 \
        --filterThresh -0.25 --scale 0.00389
  ```

  \subsection dnn_supported_yolov3 YoloV3

  You can find the config file [here](https://github.com/AlexeyAB/darknet/blob/master/cfg/yolov3.cfg) and the
  weights [there](https://pjreddie.com/media/files/yolov3.weights).

  To run the tutorial program `tutorial-dnn-object-detection-live`, use the following commands:
  ```
  $ DNN_PATH=/path/to/my/dnn/folder \
    MODEL=${DNN_PATH}/yolov3/weights/yolov3.weights \
    LABELS=${DNN_PATH}/yolov3/cfg/coco_classes.txt \
    TYPE=yolov3 \
    FRAMEWORK=darknet \
    CONFIG=${DNN_PATH}/yolov3/cfg/yolov3.cfg \
    WIDTH=416 HEIGHT=416
  $ ./tutorial-dnn-object-detection-live --model $MODEL --labels $LABELS --config $CONFIG --type $TYPE \
      --framework $FRAMEWORK --width $WIDTH --height $HEIGHT --nmsThresh 0.5 --mean 0 0 0 \
      --filterThresh -0.25 --scale 0.0039
  ```

  \subsection dnn_supported_yolov4 YoloV4

  You can find the the config file [here}(https://github.com/AlexeyAB/darknet/blob/master/cfg/yolov4-tiny.cfg) and weights [there](https://github.com/AlexeyAB/darknet/releases/download/yolov4/yolov4-tiny.weights).

  To run the tutorial program `tutorial-dnn-object-detection-live`, use the following commands:
  ```
  $ DNN_PATH=/path/to/my/dnn/folder \
    MODEL=${DNN_PATH}/yolov4/weights/yolov4-tiny.weights \
    LABELS=${DNN_PATH}/yolov4/cfg/coco_classes.txt \
    TYPE=yolov4 \
    FRAMEWORK=darknet \
    CONFIG=${DNN_PATH}/yolov4/cfg/yolov4-tiny.cfg \
    WIDTH=416 HEIGHT=416  
  $ ./tutorial-dnn-object-detection-live --model $MODEL --labels $LABELS --config $CONFIG --type $TYPE \
      --framework $FRAMEWORK --width $WIDTH --height $HEIGHT --nmsThresh 0.5 --mean 0 0 0 \
      --filterThresh -0.25 --scale 0.0039
  ```

  \subsection dnn_supported_yolov5 YoloV5

  You can find the weights in ONNX format [here](https://github.com/doleron/yolov5-opencv-cpp-python/blob/main/config_files/yolov5n.onnx).
  <b>NB</b>: You do not need a config file when using a network saved in ONNX format.

  To run the tutorial program `tutorial-dnn-object-detection-live`, use the following commands:
  ```
  $ DNN_PATH=/path/to/my/dnn/folder \
    MODEL=${DNN_PATH}/yolov5/weights/yolov5n.onnx \
    LABELS=${DNN_PATH}/yolov5/cfg/coco_classes.txt \
    TYPE=yolov5 \
    FRAMEWORK=onnx \
    CONFIG=none \
    WIDTH=640 HEIGHT=640
  $ ./tutorial-dnn-object-detection-live --model $MODEL --labels $LABELS --config $CONFIG --type $TYPE \
      --framework $FRAMEWORK --width $WIDTH --height $HEIGHT --nmsThresh 0.5 --mean 0 0 0 \
      --filterThresh -0.25 --scale 0.0039
  ```

  \subsection dnn_supported_yolov7 YoloV7

  To be able to use `YoloV7` with the class `vpDetectorDNNOpenCV`, you must first download the weights in the Pytorch format from [here](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-tiny.pt).

  Then, convert it in ONNX format using the scripts that you can find on the [YoloV7 repo](https://github.com/WongKinYiu/yolov7) with the following arguments:
  ```
  python3 export.py --weights ../weights/yolov7.pt --grid --simplify --topk-all 100 --iou-thres 0.65 --conf-thres 0.35 --img-size 640 640  --max-wh 640
  ```

  Finally, please use the following commands to run the tutorial program:
  ```
  $ DNN_PATH=/path/to/my/dnn/folder \
    MODEL=${DNN_PATH}/yolov7/weights/yolov7-tiny.onnx \
    LABELS=${DNN_PATH}/yolov7/cfg/coco_classes.txt \
    TYPE=yolov7 \
    FRAMEWORK=onnx \
    CONFIG=none \
    WIDTH=640 HEIGHT=640  
  $ ./tutorial-dnn-object-detection-live --model $MODEL --labels $LABELS --config $CONFIG --type $TYPE \
      --framework $FRAMEWORK --width $WIDTH --height $HEIGHT --nmsThresh 0.5 --mean 0 0 0 \
      --filterThresh -0.25 --scale 0.0039
  ```

  <b>NB</b>: You do not need a config file when using a network saved in ONNX format.

  \subsection dnn_supported_yolov8 YoloV8

  You can find the weights in ONNX format [here](https://github.com/JustasBart/yolov8_CPP_Inference_OpenCV_ONNX/blob/minimalistic/source/models/yolov8s.onnx).

  Please use the following commands to run the tutorial program:
  ```
  $ DNN_PATH=/path/to/my/dnn/folder \
    MODEL=${DNN_PATH}/yolov8/weights/yolov8s.onnx \
    LABELS=${DNN_PATH}/yolov8/cfg/coco_classes.txt \
    TYPE=yolov8 \
    FRAMEWORK=onnx \
    CONFIG=none \
    WIDTH=640 HEIGHT=480  
  $ ./tutorial-dnn-object-detection-live --model $MODEL --labels $LABELS --config $CONFIG --type $TYPE \
      --framework $FRAMEWORK --width $WIDTH --height $HEIGHT --nmsThresh 0.5 --mean 0 0 0 \
      --filterThresh -0.25 --scale 0.0039
  ```

  <b>NB</b>: You do not need a config file when using a network saved in ONNX format.

  \subsection dnn_troubleshootings Troubleshootings

  When using the `vpDetectorDNNOpenCV` class, you may face the following errors:

  \subsection dnn_error_size Error in the DNN input size 

  <blockquote>
  [ERROR:0@1.338] global net_impl.cpp:1161 getLayerShapesRecursively OPENCV/DNN: [Reshape]:(onnx_node!Reshape_219): getMemoryShapes() throws exception. inputs=1 outputs=1/1 blobs=0
  [ERROR:0@1.338] global net_impl.cpp:1167 getLayerShapesRecursively     input[0] = [ 1 64 8400 ]
  [ERROR:0@1.338] global net_impl.cpp:1171 getLayerShapesRecursively     output[0] = [ ]
  [ERROR:0@1.338] global net_impl.cpp:1177 getLayerShapesRecursively Exception message: OpenCV(4.7.0) ${HOME}/visp_ws/3rdparty/opencv/modules/dnn/src/layers/reshape_layer.cpp:109: error: (-215:Assertion failed) total(srcShape, srcRange.start, srcRange.end) == maskTotal in function 'computeShapeByReshapeMask'

  terminate called after throwing an instance of 'cv::Exception'
    what():  OpenCV(4.7.0) ${HOME}/visp_ws/3rdparty/opencv/modules/dnn/src/layers/reshape_layer.cpp:109: error: (-215:Assertion failed) total(srcShape, srcRange.start, srcRange.end) == maskTotal in function 'computeShapeByReshapeMask'
  </blockquote>

  This error may occur if you mistook the input size of the DNN (i.e. if you are asking to resize the input images to a size
  that does not match the one expected by the DNN).

  \subsection dnn_error_unimplemented YoloV3: transpose weights is not functionNotImplementedError

  ```
  terminate called after throwing an instance of 'cv::Exception'
  what(): OpenCV(4.7.0) error: (-213:The function/feature is not implemented) Transpose the weights (except for convolutional) is not implemented in function 'ReadDarknetFromWeightsStream'
  ```

  Following the proposition found [here](https://github.com/opencv/opencv/issues/15502#issuecomment-531755462) to download once 
  again the weights from [here](https://pjreddie.com/media/files/yolov3.weights) permitted to solve this error.
  //! [vpDetectorDNNOpenCV supported models]

  \section Example
  
  Example is provided in tutorial-dnn-object-detection-live.cpp
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
    USER_SPECIFIED =  0, /*!< The user will give a pointer towards the parsing method to use to parse the raw data resulting from the detection step.*/
    FASTER_RCNN    =  1, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a Faster-RCNN DNN. See \b vpDetectorDNNOpenCV::postProcess_FasterRCNN for more information.*/
    SSD_MOBILENET  =  2, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a SSD MobileNet DNN. See \b vpDetectorDNNOpenCV::postProcess_SSD_MobileNet for more information.*/
    RESNET_10      =  3, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a ResNet 10 DNN. See \b vpDetectorDNNOpenCV::postProcess_ResNet_10 for more information.*/
    YOLO_V3        =  4, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a YoloV3 DNN. See \b vpDetectorDNNOpenCV::postProcess_YoloV3_V4 for more information.*/
    YOLO_V4        =  5, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a YoloV4 DNN. See \b vpDetectorDNNOpenCV::postProcess_YoloV3_V4 for more information.*/
    YOLO_V5        =  6, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a YoloV5 DNN. See \b vpDetectorDNNOpenCV::postProcess_YoloV5_V7 for more information.*/
    YOLO_V7        =  7, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a YoloV7 DNN. See \b vpDetectorDNNOpenCV::postProcess_YoloV5_V7 for more information.*/
    YOLO_V8        =  8, /*!< The \b vpDetectorDNNOpenCV object will use the parsing method corresponding to a YoloV8 DNN. See \b vpDetectorDNNOpenCV::postProcess_YoloV8 for more information.*/
    COUNT          =  9 /*!< The number of parsing method that come along with the \b vpDetectorDNNOpenCV class.*/
  } DNNResultsParsingType;

  typedef struct DetectionCandidates
  {
    std::vector< float > m_confidences; /*!< Vector containing the detection confidence of each \b vpDetectorDNNOpenCV::DetectionCandidates::m_boxes.*/
    std::vector< cv::Rect > m_boxes; /*!< The bounding box of each detection candidate.*/
    std::vector< int > m_classIds; /*!< The class ID of each detection candidate.*/
  } DetectionCandidates;

  /**
   * \struct DetectedFeatures2D
   * \brief Structure containing the bounding box, expressed in pixels, confidence and class information
   * about an object detected in a image.
   */
  typedef struct DetectedFeatures2D
  {
    vpRect m_bbox; /*!< The bounding box of the detected object.*/
    double m_score; /*!< The confidence in the detection.*/
    unsigned int m_cls; /*!< The class ID.*/
    std::optional<std::string> m_classname; /*!< The class name, if the class names were given to the \b vpDetectorDNNOpenCV::NetConfig used to configure the \b vpDetectorDNNOpenCV object.*/

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

  /**
   * \struct NetConfig
   * \brief Structure containing some information required for the configuration of a \b vpDetectorDNNOpenCV object.
   */
  typedef struct NetConfig
  {
    double m_confThreshold;               /*!< Threshold to filter detections by confidence.*/
    double m_nmsThreshold;                /*!< Threshold for Non-Maximum Suppression.*/
    std::vector<std::string> m_classNames;/*!< Vector containing the names of the different classes the DNN can detect.*/
    cv::Size m_inputSize;                 /*!<  Size of the images the DNN can manipulate. The input images will be resized to match these dimensions.*/
    double m_filterSizeRatio;             /*!<  Size ratio used by the \b vpDetectorDNNOpenCV::filterDetection method. If <= 0., the \b filterDetection method is not used.*/

    /**
     * \brief Parse the file containing the list of classes the DNN can detect.
     * These classes can be written either as a YAML array (i.e. ["classname_0", ... ,"classname_last"])
     * or with one classname by row (without quotes).
     * 
     * \param filename The path towards the file containing the list of classes the DNN can detect.
     * \return std::vector<std::string> The list of classes the DNN can detect.
     */
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

    /**
     * \brief Construct a new Net Config object
     * 
     * \param confThresh The confidence threshold to keep a detection.
     * \param nmsThresh The Non-Maximum Suppression threshold to merge overlapping detections.
     * \param classNames A vector containing the list of classes the DNN can detect.
     * \param dnnInputSize The size of the input that the DNN is expecting ().
     * \param filterSizeRatio The threshold for the size filter that the user can chose to activate or not (see \b vpDetectorDNNOpenCV::filterDetection method for more information).
     */
    inline NetConfig(double confThresh, const double &nmsThresh, const std::vector<std::string> & classNames, const cv::Size &dnnInputSize, const double &filterSizeRatio = 0.)
      : m_confThreshold(confThresh)
      , m_nmsThreshold(nmsThresh)
      , m_classNames(classNames)
      , m_inputSize(dnnInputSize)
      , m_filterSizeRatio(filterSizeRatio)
    {
    }

    /**
     * \brief Construct a new Net Config object
     * 
     * \param confThresh The confidence threshold to keep a detection.
     * \param nmsThresh The Non-Maximum Suppression threshold to merge overlapping detections.
     * \param classNamesFile The path towards the file containing the classes names, written as a YAML string array or one class name by line.
     * \param dnnInputSize The size of the input that the DNN is expecting ().
     * \param filterSizeRatio The threshold for the size filter that the user can chose to activate or not (see \b vpDetectorDNNOpenCV::filterDetection method for more information).
     */
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