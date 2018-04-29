## Building ViSP for Android

I'm beginning with comparing OpenCV's cmake files and that of ViSP. I guess I need to add
Android build flags in ViSP's cmake as there were in OpenCV.
Finally I'll try to build only one ViSP module(as of now) for android

#### OpenCV cmake files that have android build flags
Equivalent of these files (if exists) have to be edited for ViSP too 

| Cmake File | Status        | Note |
| ---------- | ------------- | ---- |
| CMakeLists.txt | to be edited | 
| cmake/OpenCVModule.cmake | **Edited**  | Added a few lines in cmake/GenAndroidMK.cmake 
| cmake/OpenCVGenConfig.cmake | to be edited | 
| cmake/OpenCVGenPkgConfig.cmake | to be edited | 
| cmake/OpenCVGenAndroidMK.cmake | **Created** | Named cmake/GenAndroidMK.cmake 
| cmake/OpenCVFindIPPIW.cmake | NA | 
| cmake/OpenCVFindIPP.cmake | NA | 
| cmake/OpenCVCompilerDefenses.cmake | NA | Has some *-D_FORTIFY_SOURCE=2* flag.  
| cmake/OpenCVCompilerOptions.cmake | later |had 3rd party (pthread) flags, may add later.Maybe this is visp/AddExtraCompilationsFlags.cmake 
| cmake/OpenCVFindLibsGrfmt.cmake | later | 3rd party 
| cmake/OpenCVFindVA_INTEL.cmake | NA | 
| cmake/OpenCVFindVA.cmake | NA | 
| cmake/OpenCVDetectAndroidSDK.cmake | **Created** | 
| cmake/OpenCVDetectCUDA.cmake | NA | ViSP has no integrations for CUDA support
| cmake/OpenCVDetectPython.cmake | NA | ViSP not in Python

A few templates to be created

| Template Name | Status   |
| ------------- | -------- |
| templates/OpenCV.mk.in | Added. Simply replaced all OPENCV instances by VISP. Don't know whether any error
| templates/OpenCVConfig-ANDROID.cmake.in |
| templates/OpenCVConfig-ANDROID.cmake.in |
| templates/OpenCVConfig-ANDROID.cmake.in |
| templates/OpenCVConfig.cmake.in |
| templates/OpenCVConfig.cmake.in |
| templates/OpenCVConfig.root-ANDROID.cmake.in |
| templates/OpenCVConfig.root-ANDROID.cmake.in |


#### ViSP cmake files (not found in OpenCV cmake folder) which will be edited
I haven't found such a file yet.  
But since ViSP has additional third party libraries, we may have to edit(or often remove) few 3rd party libraries. 

### Quick lookup table for equivalent variables  

| OpenCV variable | ViSP variable |Note | 
| --------------- | ------------- | ---- |
| OpenCV_SOURCE_DIR | VISP_SOURCE_DIR |
| OPENCV_INCLUDE_DIRS_CONFIGCMAKE | VISP_INCLUDE_DIRS_CONFIGCMAKE
| OPENCV_CONFIG_FILE_INCLUDE_DIR | VISP_CONFIG_FILE_INCLUDE_DIR | added in CMakeLists.txt
| Opencv_BINARY_DIR | VISP_BINARY_DIR |
| OPENCV_CONFIG_INSTALL_PATH | VISP_CONFIG_INSTALL_PATH | added in CMakeLists.txt
| OPENCV_THIS_DIR | VISP_THIS_DIR | added in VISP.mk.in
| OPENCV_TARGET_ARCH_ABI | VISP_TARGET_ARCH_ABI | added in VISP.mk.in  