// This tutorial uses NVIDIA TensorRT inference framework to perform object detection.
// The object detection model is provided as a `.onnx` file. The model will be parsed
// and a GPU Inference Engine (GIE) will be created if it doesn't exist. This GIE is
// specific to the platform you're using.
//
// This tutorial was tested on NVIDIA Jetson TX2.
//
// The object detection model used is `SSD_Mobilenet V1` (Single Shot MultiBox Detector)
// pre-trained on PASCAL VOC dataset.  It can detect 20 classes.
// For more information about the model, see this link:
//
// https://github.com/qfgaohao/pytorch-ssd
//

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_TENSORRT) && defined(VISP_HAVE_OPENCV)

#if defined(HAVE_OPENCV_CUDEV) && defined(HAVE_OPENCV_CUDAWARPING) && defined(HAVE_OPENCV_CUDAARITHM) && \
    defined(HAVE_OPENCV_DNN) && defined(HAVE_OPENCV_VIDEOIO)
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayX.h>

#include <opencv2/videoio.hpp>

//! [OpenCV CUDA header files]
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/dnn.hpp>
//! [OpenCV CUDA header files]

//! [CUDA header files]
#include <cuda_runtime_api.h>
//! [CUDA header files]

//! [TRT header files]
#include <NvInfer.h>
#include <NvOnnxParser.h>
//! [TRT header files]

#include <sys/stat.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

//! [Preprocess image]
void preprocessImage(cv::Mat &img, float *gpu_input, const nvinfer1::Dims &dims, float meanR, float meanG, float meanB)
{
  if (img.empty()) {
    std::cerr << "Image is empty." << std::endl;
    return;
  }

  cv::cuda::GpuMat gpu_frame;
  // Upload image to GPU
  gpu_frame.upload(img);

  // input_dims is in NxCxHxW format.
  auto input_width = dims.d[3];
  auto input_height = dims.d[2];
  auto channels = dims.d[1];
  auto input_size = cv::Size(input_width, input_height);

  // Resize
  cv::cuda::GpuMat resized;
  cv::cuda::resize(gpu_frame, resized, input_size, 0, 0, cv::INTER_NEAREST);

  // Normalize
  cv::cuda::GpuMat flt_image;
  resized.convertTo(flt_image, CV_32FC3);
  cv::cuda::subtract(flt_image, cv::Scalar(meanR, meanG, meanB), flt_image, cv::noArray(), -1);
  cv::cuda::divide(flt_image, cv::Scalar(127.5f, 127.5f, 127.5f), flt_image, 1, -1);

  // To tensor
  std::vector<cv::cuda::GpuMat> chw;
  for (int i = 0; i < channels; ++i)
    chw.emplace_back(cv::cuda::GpuMat(input_size, CV_32FC1, gpu_input + i * input_width * input_height));
  cv::cuda::split(flt_image, chw);
}
//! [Preprocess image]

//! [getSizeByDim function]
size_t getSizeByDim(const nvinfer1::Dims &dims)
{
  size_t size = 1;
  for (int i = 0; i < dims.nbDims; ++i)
    size *= dims.d[i];
  return size;
}
//! [getSizeByDim function]

//! [PostProcess results]
std::vector<cv::Rect> postprocessResults(std::vector<void *> buffers, const std::vector<nvinfer1::Dims> &output_dims,
                                         int batch_size, int image_width, int image_height, float confThresh,
                                         float nmsThresh, std::vector<int> &classIds)
{
  // private variables of vpDetectorDNN
  std::vector<cv::Rect> m_boxes, m_boxesNMS;
  std::vector<int> m_classIds;
  std::vector<float> m_confidences;
  std::vector<int> m_indices;

  // copy results from GPU to CPU
  std::vector<std::vector<float> > cpu_outputs;
  for (size_t i = 0; i < output_dims.size(); i++) {
    cpu_outputs.push_back(std::vector<float>(getSizeByDim(output_dims[i]) * batch_size));
    cudaMemcpy(cpu_outputs[i].data(), (float *)buffers[1 + i], cpu_outputs[i].size() * sizeof(float),
               cudaMemcpyDeviceToHost);
  }

  // post process
  int N = output_dims[0].d[1], C = output_dims[0].d[2]; // (1 x N x C format); N: Number of output detection boxes
  // (fixed in the model), C: Number of classes.
  for (int i = 0; i < N; i++)                           // for all N (boxes)
  {
    uint32_t maxClass = 0;
    float maxScore = -1000.0f;

    for (int j = 1; j < C; j++) // ignore background (classId = 0).
    {
      const float score = cpu_outputs[0][i * C + j];

      if (score < confThresh)
        continue;

      if (score > maxScore) {
        maxScore = score;
        maxClass = j;
      }
    }

    if (maxScore > confThresh) {
      int left = (int)(cpu_outputs[1][4 * i] * image_width);
      int top = (int)(cpu_outputs[1][4 * i + 1] * image_height);
      int right = (int)(cpu_outputs[1][4 * i + 2] * image_width);
      int bottom = (int)(cpu_outputs[1][4 * i + 3] * image_height);
      int width = right - left + 1;
      int height = bottom - top + 1;

      m_boxes.push_back(cv::Rect(left, top, width, height));
      m_classIds.push_back(maxClass);
      m_confidences.push_back(maxScore);
    }
  }

  cv::dnn::NMSBoxes(m_boxes, m_confidences, confThresh, nmsThresh, m_indices);
  m_boxesNMS.resize(m_indices.size());
  for (size_t i = 0; i < m_indices.size(); ++i) {
    int idx = m_indices[i];
    m_boxesNMS[i] = m_boxes[idx];
  }

  classIds = m_classIds; // Returning detected objects class Ids.
  return m_boxesNMS;
}
//! [PostProcess results]

class Logger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char *msg) noexcept // override
  {
    if ((severity == Severity::kERROR) || (severity == Severity::kINTERNAL_ERROR) || (severity == Severity::kVERBOSE))
      std::cout << msg << std::endl;
  }
} gLogger;

// destroy TensoRT objects if something goes wrong
struct TRTDestroy
{
  template <class T> void operator()(T *obj) const
  {
    if (obj)
      obj->destroy();
  }
};

template <class T> using TRTUniquePtr = std::unique_ptr<T, TRTDestroy>;

//! [ParseOnnxModel]
bool parseOnnxModel(const std::string &model_path, TRTUniquePtr<nvinfer1::ICudaEngine> &engine,
                    TRTUniquePtr<nvinfer1::IExecutionContext> &context)
  //! [ParseOnnxModel]
{
  // this section of code is from jetson-inference's `tensorNet`, to test if the GIE already exists.
  char cache_prefix[FILENAME_MAX];
  char cache_path[FILENAME_MAX];

  snprintf(cache_prefix, FILENAME_MAX, "%s", model_path.c_str());
  snprintf(cache_path, FILENAME_MAX, "%s.engine", cache_prefix);

  std::cout << "attempting to open engine cache file " << cache_path << std::endl;

  //! [ParseOnnxModel engine exists]
  if (vpIoTools::checkFilename(cache_path)) {
    char *engineStream = nullptr;
    size_t engineSize = 0;

    // determine the file size of the engine
    struct stat filestat;
    stat(cache_path, &filestat);
    engineSize = filestat.st_size;

    // allocate memory to hold the engine
    engineStream = (char *)malloc(engineSize);

    // open the engine cache file from disk
    FILE *cacheFile = nullptr;
    cacheFile = fopen(cache_path, "rb");

    // read the serialized engine into memory
    const size_t bytesRead = fread(engineStream, 1, engineSize, cacheFile);

    if (bytesRead != engineSize) // Problem while deserializing.
    {
      std::cerr << "Error reading serialized engine into memory." << std::endl;
      return false;
    }

    // close the plan cache
    fclose(cacheFile);

    // Recreate the inference runtime
    TRTUniquePtr<nvinfer1::IRuntime> infer { nvinfer1::createInferRuntime(gLogger) };
    engine.reset(infer->deserializeCudaEngine(engineStream, engineSize, nullptr));
    context.reset(engine->createExecutionContext());

    return true;
  }
  //! [ParseOnnxModel engine exists]

  //! [ParseOnnxModel engine does not exist]
  else {
    if (!vpIoTools::checkFilename(model_path)) {
      std::cerr << "Could not parse ONNX model. File not found" << std::endl;
      return false;
    }

    TRTUniquePtr<nvinfer1::IBuilder> builder { nvinfer1::createInferBuilder(gLogger) };
    TRTUniquePtr<nvinfer1::INetworkDefinition> network {
        builder->createNetworkV2(1U << (uint32_t)nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH) };
    TRTUniquePtr<nvonnxparser::IParser> parser { nvonnxparser::createParser(*network, gLogger) };

    // parse ONNX
    if (!parser->parseFromFile(model_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kINFO))) {
      std::cerr << "ERROR: could not parse the model." << std::endl;
      return false;
    }

    TRTUniquePtr<nvinfer1::IBuilderConfig> config { builder->createBuilderConfig() };
    // allow TRT to use up to 1GB of GPU memory for tactic selection
    config->setMaxWorkspaceSize(32 << 20);
    // use FP16 mode if possible
    if (builder->platformHasFastFp16()) {
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }

    builder->setMaxBatchSize(1);

    engine.reset(builder->buildEngineWithConfig(*network, *config));
    context.reset(engine->createExecutionContext());

    TRTUniquePtr<nvinfer1::IHostMemory> serMem { engine->serialize() };

    if (!serMem) {
      std::cout << "Failed to serialize CUDA engine." << std::endl;
      return false;
    }

    const char *serData = (char *)serMem->data();
    const size_t serSize = serMem->size();

    // allocate memory to store the bitstream
    char *engineMemory = (char *)malloc(serSize);

    if (!engineMemory) {
      std::cout << "Failed to allocate memory to store CUDA engine." << std::endl;
      return false;
    }

    memcpy(engineMemory, serData, serSize);

    // write the cache file
    FILE *cacheFile = nullptr;
    cacheFile = fopen(cache_path, "wb");

    fwrite(engineMemory, 1, serSize, cacheFile);
    fclose(cacheFile);

    return true;
  }
  //! [ParseOnnxModel engine does not exist]
}

int main(int argc, char **argv)
{
  int opt_device = 0;
  unsigned int opt_scale = 1;
  std::string input = "";
  std::string modelFile = vpIoTools::getViSPImagesDataPath() + "/dnn/object_detection/ssd-mobilenet.onnx";
  std::string labelFile = vpIoTools::getViSPImagesDataPath() + "/dnn/object_detection/pascal-voc-labels.txt";
  std::string config = "";
  float meanR = 127.5f, meanG = 127.5f, meanB = 127.5f;
  float confThresh = 0.5f;
  float nmsThresh = 0.4f;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--device" && i + 1 < argc) {
      opt_device = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      input = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--model" && i + 1 < argc) {
      modelFile = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--config" && i + 1 < argc) {
      config = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--input-scale" && i + 1 < argc) {
      opt_scale = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--mean" && i + 3 < argc) {
      meanR = atof(argv[i + 1]);
      meanG = atof(argv[i + 2]);
      meanB = atof(argv[i + 3]);
    }
    else if (std::string(argv[i]) == "--confThresh" && i + 1 < argc) {
      confThresh = (float)atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--nmsThresh" && i + 1 < argc) {
      nmsThresh = (float)atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--labels" && i + 1 < argc) {
      labelFile = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0]
        << " [--device <camera device number>] [--input <path to image or video>"
        " (camera is used if input is empty)] [--model <path to net trained weights>]"
        " [--config <path to net config file>]"
        " [--input-scale <input scale factor>] [--mean <meanR meanG meanB>]"
        " [--confThresh <confidence threshold>]"
        " [--nmsThresh <NMS threshold>] [--labels <path to label file>]"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  std::string model_path(modelFile);
  int batch_size = 1;

  std::vector<std::string> labels;
  if (!labelFile.empty()) {
    std::ifstream f_label(labelFile);
    std::string line;
    while (std::getline(f_label, line)) {
      labels.push_back(line);
    }
  }

  //! [Create GIE]
  // Parse the model and initialize the engine and the context.
  TRTUniquePtr<nvinfer1::ICudaEngine> engine { nullptr };
  TRTUniquePtr<nvinfer1::IExecutionContext> context { nullptr };
  if (!parseOnnxModel(model_path, engine, context)) // Problem parsing Onnx model
  {
    std::cout << "Make sure the model file exists. To see available models, plese visit: "
      "\n\twww.github.com/lagadic/visp-images/dnn/object_detection/"
      << std::endl;
    return EXIT_FAILURE;
  }
  //! [Create GIE]

  std::vector<nvinfer1::Dims> input_dims;
  std::vector<nvinfer1::Dims> output_dims;
  std::vector<void *> buffers(engine->getNbBindings()); // buffers for input and output data.

  //! [Get I/O dimensions]
  for (int i = 0; i < engine->getNbBindings(); ++i) {
    auto binding_size = getSizeByDim(engine->getBindingDimensions(i)) * batch_size * sizeof(float);
    cudaMalloc(&buffers[i], binding_size);

    if (engine->bindingIsInput(i)) {
      input_dims.emplace_back(engine->getBindingDimensions(i));
    }
    else {
      output_dims.emplace_back(engine->getBindingDimensions(i));
    }
  }

  if (input_dims.empty() || output_dims.empty()) {
    std::cerr << "Expect at least one input and one output for network" << std::endl;
    return EXIT_FAILURE;
  }
  //! [Get I/O dimensions]

  //! [OpenCV VideoCapture]
  cv::VideoCapture capture;

  if (input.empty()) {
    capture.open(opt_device);
  }
  else {
    capture.open(input);
  }

  if (!capture.isOpened()) { // check if we succeeded
    std::cout << "Failed to open the camera" << std::endl;
    return EXIT_FAILURE;
  }

  int cap_width = (int)capture.get(cv::CAP_PROP_FRAME_WIDTH);
  int cap_height = (int)capture.get(cv::CAP_PROP_FRAME_HEIGHT);
  capture.set(cv::CAP_PROP_FRAME_WIDTH, cap_width / opt_scale);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, cap_height / opt_scale);
  //! [OpenCV VideoCapture]

  vpImage<vpRGBa> I;
  cv::Mat frame;
  capture >> frame;

  if (input.empty()) {
    int i = 0;
    while ((i++ < 20) && !capture.read(frame)) {
    }; // warm up camera by skiping unread frames
  }

  vpImageConvert::convert(frame, I);
  int height = I.getHeight(), width = I.getWidth();

  std::cout << "Image size: " << width << " x " << height << std::endl;

  std::vector<cv::Rect> boxesNMS;
  std::vector<int> classIds;

  vpDisplayX d(I);

  double start, stop;
  //! [Main loop]
  while (!vpDisplay::getClick(I, false)) {
    // get frame.
    capture >> frame;

    vpImageConvert::convert(frame, I);

    start = vpTime::measureTimeMs();
    // preprocess
    preprocessImage(frame, (float *)buffers[0], input_dims[0], meanR, meanG, meanB);

    // inference.
    context->enqueue(batch_size, buffers.data(), 0, nullptr);

    // post-process
    boxesNMS = postprocessResults(buffers, output_dims, batch_size, width, height, confThresh, nmsThresh, classIds);

    stop = vpTime::measureTimeMs();
    // display.
    vpDisplay::display(I);
    vpDisplay::displayText(I, 10, 10, std::to_string(stop - start), vpColor::red);

    for (unsigned int i = 0; i < boxesNMS.size(); i++) {
      vpDisplay::displayRectangle(I, vpRect(boxesNMS[i].x, boxesNMS[i].y, boxesNMS[i].width, boxesNMS[i].height),
                                  vpColor::red, false, 2);
      vpDisplay::displayText(I, boxesNMS[i].y - 10, boxesNMS[i].x, labels[classIds[i]], vpColor::red);
    }

    vpDisplay::flush(I);
  }
  //! [Main loop]

  for (void *buf : buffers)
    cudaFree(buf);

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "OpenCV is not built with CUDA." << std::endl;

  return EXIT_SUCCESS;
}
#endif

#else
int main()
{
  std::cout << "ViSP is not built with TensorRT." << std::endl;

  return EXIT_SUCCESS;
}
#endif
