#####################################
## tensorrt specific configuration ##
#####################################
find_package(CUDA)
find_library(NVINFER  NAMES nvinfer)
find_library(NVINFERPLUGIN NAMES nvinfer_plugin)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVONNXPARSER NAMES nvonnxparser)
find_library(NVONNXPARSERRUNTIME NAMES nvonnxparser_runtime)

# If it is ALL there, export libraries as a single package
if(CUDA_FOUND AND NVINFER AND NVINFERPLUGIN AND NVPARSERS AND NVONNXPARSER)
  list(APPEND TENSORRT_LIBRARIES ${CUDA_LIBRARIES} nvinfer nvinfer_plugin nvparsers nvonnxparser)
  if(NVONNXPARSERRUNTIME)
    list(APPEND TENSORRT_LIBRARIES nvonnxparser_runtime)
  endif()
  list(APPEND TENSORRT_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS})
  set(TENSORRT_FOUND ON)
else()
  set(TENSORRT_FOUND OFF)
endif()
