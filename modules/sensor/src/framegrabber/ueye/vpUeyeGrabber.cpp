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
 * IDS uEye interface.
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_UEYE)

#include <string.h>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/sensor/vpUeyeGrabber.h>

#include <ueye.h>

#include "vpUeyeGrabber_impl.h"

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*!
 * \brief Number of image buffer to alloc
 */
#define IMAGE_COUNT 5

#define CAMINFO BOARDINFO
#define EVENTTHREAD_WAIT_TIMEOUT 1000

#define CAP(val, min, max)                                                                                             \
  {                                                                                                                    \
    if (val < min) {                                                                                                   \
      val = min;                                                                                                       \
    } else if (val > max) {                                                                                            \
      val = max;                                                                                                       \
    }                                                                                                                  \
  }

BEGIN_VISP_NAMESPACE
/*! \brief image buffer properties structure */
struct sBufferProps
{
  int width;
  int height;
  int bitspp;
};

/*! \brief camera feature properties structure */
struct sCameraProps
{
  bool bUsesImageFormats;
  int nImgFmtNormal;
  int nImgFmtDefaultNormal;
  int nImgFmtTrigger;
  int nImgFmtDefaultTrigger;
};

/*!
 * \brief uEye Image parameter structure
 */
typedef struct _UEYE_IMAGE
{
  char *pBuf;
  INT nImageID;
  INT nImageSeqNum;
  INT nBufferSize;
} UEYE_IMAGE, *PUEYE_IMAGE;

class vpUeyeGrabber::vpUeyeGrabberImpl
{
public:
  vpUeyeGrabberImpl()
    : m_hCamera((HIDS)0), m_activeCameraSelected(-1), m_pLastBuffer(nullptr), m_cameraList(nullptr), m_bLive(true),
    m_bLiveStarted(false), m_verbose(false), m_I_temp()
  {
    ZeroMemory(&m_SensorInfo, sizeof(SENSORINFO));
    ZeroMemory(&m_CamInfo, sizeof(CAMINFO));
    ZeroMemory(&m_CamListInfo, sizeof(UEYE_CAMERA_INFO));
    ZeroMemory(m_Images, sizeof(m_Images));

    m_BufferProps.width = 0;
    m_BufferProps.height = 0;
    m_BufferProps.bitspp = 8;

    m_event = 0;
#ifndef __linux__
    m_hEvent = 0;
#endif

    // Active camera is the first one that is found
    m_activeCameraSelected = setActiveCamera(0);
  }

  ~vpUeyeGrabberImpl() { close(); }

  void acquire(vpImage<unsigned char> &I, double *timestamp_camera, std::string *timestamp_system)
  {
    INT ret = IS_SUCCESS;

    if (!m_hCamera) {
      open(I);
    }

    if (m_hCamera) {
      if (!m_bLive) {
        ret = is_FreezeVideo(m_hCamera, IS_WAIT);
      }
      else {
        if (!m_bLiveStarted) {
          ret = is_CaptureVideo(m_hCamera, IS_DONT_WAIT);
          m_bLiveStarted = true;
        }
      }

      ret = waitEvent();

      if (ret == IS_SUCCESS) {
        INT dummy = 0;
        char *pLast = nullptr, *pMem = nullptr;

        is_GetActSeqBuf(m_hCamera, &dummy, &pMem, &pLast);
        m_pLastBuffer = pLast;

        if (!m_pLastBuffer || m_BufferProps.width < 1 || m_BufferProps.height < 1)
          return;

        int nNum = getImageNum(m_pLastBuffer);
        if (timestamp_camera != nullptr || timestamp_system != nullptr) {
          int nImageID = getImageID(m_pLastBuffer);
          UEYEIMAGEINFO ImageInfo;
          if (is_GetImageInfo(m_hCamera, nImageID, &ImageInfo, sizeof(ImageInfo)) == IS_SUCCESS) {
            if (timestamp_camera != nullptr) {
              *timestamp_camera = static_cast<double>(ImageInfo.u64TimestampDevice) / 10000.;
            }
            if (timestamp_system != nullptr) {
              std::stringstream ss;
              ss << ImageInfo.TimestampSystem.wYear << ":" << std::setfill('0') << std::setw(2)
                << ImageInfo.TimestampSystem.wMonth << ":" << std::setfill('0') << std::setw(2)
                << ImageInfo.TimestampSystem.wDay << ":" << std::setfill('0') << std::setw(2)
                << ImageInfo.TimestampSystem.wHour << ":" << std::setfill('0') << std::setw(2)
                << ImageInfo.TimestampSystem.wMinute << ":" << std::setfill('0') << std::setw(2)
                << ImageInfo.TimestampSystem.wSecond << ":" << std::setfill('0') << std::setw(3)
                << ImageInfo.TimestampSystem.wMilliseconds;
              *timestamp_system = ss.str();
            }
          }
        }

        helper::LockUnlockSeqBuffer lock(m_hCamera, nNum, m_pLastBuffer);

        if (lock.OwnsLock()) {
          // get current colormode
          int colormode = is_SetColorMode(m_hCamera, IS_GET_COLOR_MODE);

          switch (colormode) {
          default:
          case IS_CM_MONO8:
            memcpy(reinterpret_cast<unsigned char *>(I.bitmap), reinterpret_cast<unsigned char *>(m_pLastBuffer),
                   m_BufferProps.width * m_BufferProps.height * m_BufferProps.bitspp / 8);
            break;
          case IS_CM_SENSOR_RAW8:
            m_I_temp.resize(m_BufferProps.height, m_BufferProps.width),
              vpImageConvert::demosaicRGGBToRGBaBilinear(reinterpret_cast<unsigned char *>(m_pLastBuffer),
                                                         reinterpret_cast<unsigned char *>(m_I_temp.bitmap),
                                                         m_BufferProps.width, m_BufferProps.height);
            vpImageConvert::RGBaToGrey(reinterpret_cast<unsigned char *>(m_I_temp.bitmap),
                                       reinterpret_cast<unsigned char *>(I.bitmap), m_BufferProps.width,
                                       m_BufferProps.height);
            break;
          case IS_CM_BGR565_PACKED:
            throw(vpException(vpException::fatalError, "vpUeyeGrabber doesn't support BGR565 format"));

          case IS_CM_RGB8_PACKED:
            vpImageConvert::RGBToGrey(reinterpret_cast<unsigned char *>(m_pLastBuffer),
                                      reinterpret_cast<unsigned char *>(I.bitmap), m_BufferProps.width,
                                      m_BufferProps.height);
            break;
          case IS_CM_BGR8_PACKED:
            vpImageConvert::BGRToGrey(reinterpret_cast<unsigned char *>(m_pLastBuffer),
                                      reinterpret_cast<unsigned char *>(I.bitmap), m_BufferProps.width,
                                      m_BufferProps.height);
            break;
          case IS_CM_RGBA8_PACKED:
            vpImageConvert::RGBaToGrey(reinterpret_cast<unsigned char *>(m_pLastBuffer),
                                       reinterpret_cast<unsigned char *>(I.bitmap), m_BufferProps.width,
                                       m_BufferProps.height);
            break;
          case IS_CM_BGRA8_PACKED:
            vpImageConvert::BGRaToGrey(reinterpret_cast<unsigned char *>(m_pLastBuffer),
                                       reinterpret_cast<unsigned char *>(I.bitmap), m_BufferProps.width,
                                       m_BufferProps.height);
            break;
          }
        }
      }
    }
  }

  void acquire(vpImage<vpRGBa> &I, double *timestamp_camera, std::string *timestamp_system)
  {
    INT ret = IS_SUCCESS;

    if (!m_hCamera) {
      open(I);
    }

    if (m_hCamera) {
      if (!m_bLive) {
        ret = is_FreezeVideo(m_hCamera, IS_WAIT);
      }
      else {
        if (!m_bLiveStarted) {
          //          ret = is_CaptureVideo(m_hCamera, IS_DONT_WAIT);
          ret = is_CaptureVideo(m_hCamera, IS_WAIT);
          m_bLiveStarted = true;
        }
      }

      ret = waitEvent();

      if (ret == IS_SUCCESS) {
        INT dummy = 0;
        char *pLast = nullptr, *pMem = nullptr;

        is_GetActSeqBuf(m_hCamera, &dummy, &pMem, &pLast);
        m_pLastBuffer = pLast;

        if (!m_pLastBuffer || m_BufferProps.width < 1 || m_BufferProps.height < 1)
          return;

        int nNum = getImageNum(m_pLastBuffer);
        if (timestamp_camera != nullptr || timestamp_system != nullptr) {
          int nImageID = getImageID(m_pLastBuffer);
          UEYEIMAGEINFO ImageInfo;
          if (is_GetImageInfo(m_hCamera, nImageID, &ImageInfo, sizeof(ImageInfo)) == IS_SUCCESS) {
            if (timestamp_camera != nullptr) {
              *timestamp_camera = static_cast<double>(ImageInfo.u64TimestampDevice) / 10000.;
            }
            if (timestamp_system != nullptr) {
              std::stringstream ss;
              ss << ImageInfo.TimestampSystem.wYear << ":" << std::setfill('0') << std::setw(2)
                << ImageInfo.TimestampSystem.wMonth << ":" << std::setfill('0') << std::setw(2)
                << ImageInfo.TimestampSystem.wDay << ":" << std::setfill('0') << std::setw(2)
                << ImageInfo.TimestampSystem.wHour << ":" << std::setfill('0') << std::setw(2)
                << ImageInfo.TimestampSystem.wMinute << ":" << std::setfill('0') << std::setw(2)
                << ImageInfo.TimestampSystem.wSecond << ":" << std::setfill('0') << std::setw(3)
                << ImageInfo.TimestampSystem.wMilliseconds;
              *timestamp_system = ss.str();
            }
          }
        }

        helper::LockUnlockSeqBuffer lock(m_hCamera, nNum, m_pLastBuffer);

        if (lock.OwnsLock()) {
          // get current colormode
          int colormode = is_SetColorMode(m_hCamera, IS_GET_COLOR_MODE);

          switch (colormode) {
          default:
          case IS_CM_MONO8:
            vpImageConvert::GreyToRGBa(reinterpret_cast<unsigned char *>(m_pLastBuffer),
                                       reinterpret_cast<unsigned char *>(I.bitmap), m_BufferProps.width,
                                       m_BufferProps.height);
            break;
          case IS_CM_SENSOR_RAW8:
            vpImageConvert::demosaicRGGBToRGBaBilinear(reinterpret_cast<unsigned char *>(m_pLastBuffer),
                                                       reinterpret_cast<unsigned char *>(I.bitmap), m_BufferProps.width,
                                                       m_BufferProps.height);
            break;
          case IS_CM_BGR565_PACKED:
            throw(vpException(vpException::fatalError, "vpUeyeGrabber doesn't support BGR565 format"));

          case IS_CM_RGB8_PACKED:
            vpImageConvert::RGBToRGBa(reinterpret_cast<unsigned char *>(m_pLastBuffer),
                                      reinterpret_cast<unsigned char *>(I.bitmap), m_BufferProps.width,
                                      m_BufferProps.height);
            break;
          case IS_CM_BGR8_PACKED:
            vpImageConvert::BGRToRGBa(reinterpret_cast<unsigned char *>(m_pLastBuffer),
                                      reinterpret_cast<unsigned char *>(I.bitmap), m_BufferProps.width,
                                      m_BufferProps.height);
            break;
          case IS_CM_RGBA8_PACKED:
            memcpy(reinterpret_cast<unsigned char *>(I.bitmap), reinterpret_cast<unsigned char *>(m_pLastBuffer),
                   m_BufferProps.width * m_BufferProps.height * m_BufferProps.bitspp / 8);
            break;
          case IS_CM_BGRA8_PACKED:
            vpImageConvert::BGRaToRGBa(reinterpret_cast<unsigned char *>(m_pLastBuffer),
                                       reinterpret_cast<unsigned char *>(I.bitmap), m_BufferProps.width,
                                       m_BufferProps.height);
            break;
          }
        }
      }
    }
  }

  bool allocImages()
  {
    m_pLastBuffer = nullptr;
    int nWidth = 0;
    int nHeight = 0;

    UINT nAbsPosX;
    UINT nAbsPosY;

    is_AOI(m_hCamera, IS_AOI_IMAGE_GET_POS_X_ABS, (void *)&nAbsPosX, sizeof(nAbsPosX));
    is_AOI(m_hCamera, IS_AOI_IMAGE_GET_POS_Y_ABS, (void *)&nAbsPosY, sizeof(nAbsPosY));

    is_ClearSequence(m_hCamera);
    freeImages();

    for (unsigned int i = 0; i < sizeof(m_Images) / sizeof(m_Images[0]); i++) {
      nWidth = m_BufferProps.width;
      nHeight = m_BufferProps.height;

      if (nAbsPosX) {
        m_BufferProps.width = nWidth = m_SensorInfo.nMaxWidth;
      }
      if (nAbsPosY) {
        m_BufferProps.height = nHeight = m_SensorInfo.nMaxHeight;
      }

      if (is_AllocImageMem(m_hCamera, nWidth, nHeight, m_BufferProps.bitspp, &m_Images[i].pBuf,
                           &m_Images[i].nImageID) != IS_SUCCESS)
        return false;
      if (is_AddToSequence(m_hCamera, m_Images[i].pBuf, m_Images[i].nImageID) != IS_SUCCESS)
        return false;

      m_Images[i].nImageSeqNum = i + 1;
      m_Images[i].nBufferSize = nWidth * nHeight * m_BufferProps.bitspp / 8;
    }

    return true;
  }

  int cameraInitialized()
  {
    int ret = 0;
    unsigned int uInitialParameterSet = IS_CONFIG_INITIAL_PARAMETERSET_NONE;

    if ((ret = is_GetCameraInfo(m_hCamera, &m_CamInfo)) != IS_SUCCESS) {
      throw(vpException(vpException::fatalError, "uEye error: GetCameraInfo failed"));
    }
    else if ((ret = is_GetSensorInfo(m_hCamera, &m_SensorInfo)) != IS_SUCCESS) {
      throw(vpException(vpException::fatalError, "uEye error: GetSensorInfo failed"));
    }
    else if ((ret = is_Configuration(IS_CONFIG_INITIAL_PARAMETERSET_CMD_GET, &uInitialParameterSet,
                                     sizeof(unsigned int))) != IS_SUCCESS) {
      throw(vpException(vpException::fatalError, "uEye error: querying 'initial parameter set' failed"));
    }
    else {
   // m_nWidth = m_SensorInfo.nMaxWidth;
   // m_nHeight = m_SensorInfo.nMaxHeight;

   // restore all defaults
   // do this only if there is no 'initial parameter set' installed.
   // if an 'initial parameter set' is installed we must not overwrite this setup!
      if (uInitialParameterSet == IS_CONFIG_INITIAL_PARAMETERSET_NONE) {
        ret = is_ResetToDefault(m_hCamera);
      }

      int colormode = 0;
      if (m_SensorInfo.nColorMode >= IS_COLORMODE_BAYER) {
        colormode = IS_CM_BGRA8_PACKED;
      }
      else {
        colormode = IS_CM_MONO8;
      }

      if (is_SetColorMode(m_hCamera, colormode) != IS_SUCCESS) {
        throw(vpException(vpException::fatalError, "uEye error: SetColorMode failed"));
      }

      /* get some special camera properties */
      ZeroMemory(&m_CameraProps, sizeof(m_CameraProps));

      // If the camera does not support a continuous AOI -> it uses special image formats
      m_CameraProps.bUsesImageFormats = false;
      INT nAOISupported = 0;
      if (is_ImageFormat(m_hCamera, IMGFRMT_CMD_GET_ARBITRARY_AOI_SUPPORTED, (void *)&nAOISupported,
                         sizeof(nAOISupported)) == IS_SUCCESS) {
        m_CameraProps.bUsesImageFormats = (nAOISupported == 0);
      }

      /* set the default image format, if used */
      if (m_CameraProps.bUsesImageFormats) {
        // search the default formats
        m_CameraProps.nImgFmtNormal = searchDefImageFormats(CAPTMODE_FREERUN | CAPTMODE_SINGLE);
        m_CameraProps.nImgFmtDefaultNormal = m_CameraProps.nImgFmtNormal;
        m_CameraProps.nImgFmtTrigger = searchDefImageFormats(CAPTMODE_TRIGGER_SOFT_SINGLE);
        m_CameraProps.nImgFmtDefaultTrigger = m_CameraProps.nImgFmtTrigger;
        // set the default formats
        if ((ret = is_ImageFormat(m_hCamera, IMGFRMT_CMD_SET_FORMAT, (void *)&m_CameraProps.nImgFmtNormal,
                                  sizeof(m_CameraProps.nImgFmtNormal))) == IS_SUCCESS) {
          // m_nImageFormat = nFormat;
          // bRet = TRUE;
        }
      }

      /* setup the capture parameter */
      setupCapture();

      enableEvent(IS_SET_EVENT_FRAME);
    }

    m_pLastBuffer = nullptr;

    return ret;
  }

  void close()
  {
    if (m_hCamera == IS_INVALID_HIDS)
      return;

    if (m_hCamera) {
      if (m_bLive && m_bLiveStarted) {
        INT nRet = 1;
        double t = vpTime::measureTimeSecond();
        while (nRet != IS_SUCCESS && (vpTime::measureTimeSecond() - t) <= 2.) {
          nRet = is_StopLiveVideo(m_hCamera, IS_WAIT);
        }
        m_bLiveStarted = false;
      }

      is_ClearSequence(m_hCamera);
      freeImages();

      if (is_ExitCamera(m_hCamera) != IS_SUCCESS) {
        throw(vpException(vpException::fatalError, "Cannot logoff camera"));
      }

      disableEvent();

      m_hCamera = (HIDS)0;
    }
  }

  void disableEvent()
  {
    is_DisableEvent(m_hCamera, m_event);
#ifndef __linux__
    is_ExitEvent(m_hCamera, m_event);
    CloseHandle(m_hEvent);
#endif
  }

  int enableEvent(int event)
  {
    int ret = 0;
    m_event = event;
#ifndef __linux__
    m_hEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    if (m_hEvent == nullptr) {
      return -1;
    }
    ret = is_InitEvent(m_hCamera, m_hEvent, m_event);
#endif
    ret = is_EnableEvent(m_hCamera, m_event);

    return ret;
  }

  int waitEvent()
  {
#ifdef __linux__
    if (is_WaitEvent(m_hCamera, m_event, EVENTTHREAD_WAIT_TIMEOUT) == IS_SUCCESS) {
#else
    if (WaitForSingleObject(m_hEvent, EVENTTHREAD_WAIT_TIMEOUT) == WAIT_OBJECT_0) {
#endif
      return IS_SUCCESS;
    }
    else {
      return IS_TIMED_OUT;
    }
    }

  void freeImages()
  {
    m_pLastBuffer = nullptr;
    // printf ("freeing image buffers\n");
    for (unsigned int i = 0; i < sizeof(m_Images) / sizeof(m_Images[0]); i++) {
      if (nullptr != m_Images[i].pBuf) {
        is_FreeImageMem(m_hCamera, m_Images[i].pBuf, m_Images[i].nImageID);
      }

      m_Images[i].pBuf = nullptr;
      m_Images[i].nImageID = 0;
      m_Images[i].nImageSeqNum = 0;
    }
  }

  std::string getActiveCameraModel() const { return m_CamListInfo.Model; }

  std::string getActiveCameraSerialNumber() const { return m_CamListInfo.SerNo; }

  int getBitsPerPixel(int colormode)
  {
    switch (colormode) {
    default:
    case IS_CM_MONO8:
    case IS_CM_SENSOR_RAW8:
      return 8; // occupies 8 Bit
    case IS_CM_MONO12:
    case IS_CM_MONO16:
    case IS_CM_SENSOR_RAW12:
    case IS_CM_SENSOR_RAW16:
    case IS_CM_BGR5_PACKED:
    case IS_CM_BGR565_PACKED:
    case IS_CM_UYVY_PACKED:
    case IS_CM_CBYCRY_PACKED:
      return 16; // occupies 16 Bit
    case IS_CM_RGB8_PACKED:
    case IS_CM_BGR8_PACKED:
      return 24;
    case IS_CM_RGBA8_PACKED:
    case IS_CM_BGRA8_PACKED:
    case IS_CM_RGBY8_PACKED:
    case IS_CM_BGRY8_PACKED:
    case IS_CM_RGB10_PACKED:
    case IS_CM_BGR10_PACKED:
      return 32;
    }
  }

  std::vector<unsigned int> getCameraIDList() const
  {
    CameraList camera_list;
    return camera_list.getCameraIDList();
  }

  std::vector<std::string> getCameraModelList() const
  {
    CameraList camera_list;
    return camera_list.getCameraModelList();
  }

  std::vector<std::string> getCameraSerialNumberList() const
  {
    CameraList camera_list;
    return camera_list.getCameraSerialNumberList();
  }

  unsigned int getFrameHeight() const
  {
    if (!isConnected()) {
      throw(vpException(vpException::fatalError, "Unable to get frame height. Camera connexion is not opened"));
    }
    return static_cast<unsigned int>(m_BufferProps.height);
  }

  unsigned int getFrameWidth() const
  {
    if (!isConnected()) {
      throw(vpException(vpException::fatalError, "Unable to get frame width. Camera connexion is not opened"));
    }
    return static_cast<unsigned int>(m_BufferProps.width);
  }

  double getFramerate() const
  {
    if (!m_hCamera) {
      return 0;
    }
    double fps;

    // Get framerate
    if (is_GetFramesPerSecond(m_hCamera, &fps) != IS_SUCCESS) {
      if (m_verbose) {
        std::cout << "Unable to get acquisition frame rate" << std::endl;
      }
    }
    return fps;
  }

  INT getImageID(char *pbuf)
  {
    if (!pbuf)
      return 0;

    for (unsigned int i = 0; i < sizeof(m_Images) / sizeof(m_Images[0]); i++)
      if (m_Images[i].pBuf == pbuf)
        return m_Images[i].nImageID;

    return 0;
  }

  INT getImageNum(char *pbuf)
  {
    if (!pbuf)
      return 0;

    for (unsigned int i = 0; i < sizeof(m_Images) / sizeof(m_Images[0]); i++)
      if (m_Images[i].pBuf == pbuf)
        return m_Images[i].nImageSeqNum;

    return 0;
  }

  bool isConnected() const { return (m_hCamera != (HIDS)0); }

  void loadParameters(const std::string &filename)
  {
    if (!vpIoTools::checkFilename(filename)) {
      throw(vpException(vpException::fatalError, "Camera parameters file doesn't exist: %s", filename.c_str()));
    }

    const std::wstring filename_(filename.begin(), filename.end());
    int ret = is_ParameterSet(m_hCamera, IS_PARAMETERSET_CMD_LOAD_FILE, (void *)filename_.c_str(), 0);

    if (ret == IS_INVALID_CAMERA_TYPE) {
      throw(vpException(vpException::fatalError, "The camera parameters file %s belong to a different camera",
                        filename.c_str()));
    }
    else if (ret == IS_INCOMPATIBLE_SETTING) {
      throw(vpException(vpException::fatalError,
                        "Because of incompatible settings, cannot load parameters from file %s", filename.c_str()));
    }
    else if (ret != IS_SUCCESS) {
      throw(vpException(vpException::fatalError, "Cannot load parameters from file %s", filename.c_str()));
    }
    else {
      std::cout << "Parameters loaded sucessfully" << std::endl;
    }

    setupCapture();
  }

  void open()
  {
    if (m_hCamera) {
      if (is_ExitCamera(m_hCamera) != IS_SUCCESS) {
        throw(vpException(vpException::fatalError, "Cannot logoff camera"));
      }
    }

    // open the selected camera
    m_hCamera = (HIDS)(m_CamListInfo.dwDeviceID | IS_USE_DEVICE_ID); // open camera

    if (is_InitCamera(&m_hCamera, 0) != IS_SUCCESS) { // init camera - no window handle required
      throw(vpException(vpException::fatalError, "Cannot open connexion with IDS uEye camera"));
    }

    int ret = cameraInitialized();
    if (ret != IS_SUCCESS) {
      throw(vpException(vpException::fatalError, "Unable to initialize uEye camera"));
    }
  }

  template <class Type> void open(vpImage<Type> &I)
  {
    open();
    I.resize(m_SensorInfo.nMaxHeight, m_SensorInfo.nMaxWidth);
  }

  /*! \brief search the right default image format of a camera
   *
   * \param suppportMask
   * \return found imageformat
   */
  int searchDefImageFormats(int suppportMask)
  {
    int ret = IS_SUCCESS;
    int nNumber;
    int format = 0;
    IMAGE_FORMAT_LIST *pFormatList;
    IS_RECT rectAOI;

    if ((ret = is_ImageFormat(m_hCamera, IMGFRMT_CMD_GET_NUM_ENTRIES, (void *)&nNumber, sizeof(nNumber))) ==
            IS_SUCCESS &&
        (ret = is_AOI(m_hCamera, IS_AOI_IMAGE_GET_AOI, (void *)&rectAOI, sizeof(rectAOI))) == IS_SUCCESS) {
      int i = 0;
      int nSize = sizeof(IMAGE_FORMAT_LIST) + (nNumber - 1) * sizeof(IMAGE_FORMAT_LIST);
      pFormatList = (IMAGE_FORMAT_LIST *)(new char[nSize]);
      pFormatList->nNumListElements = nNumber;
      pFormatList->nSizeOfListEntry = sizeof(IMAGE_FORMAT_INFO);

      if ((ret = is_ImageFormat(m_hCamera, IMGFRMT_CMD_GET_LIST, (void *)pFormatList, nSize)) == IS_SUCCESS) {
        for (i = 0; i < nNumber; i++) {
          if ((pFormatList->FormatInfo[i].nSupportedCaptureModes & suppportMask) &&
              pFormatList->FormatInfo[i].nHeight == (UINT)rectAOI.s32Height &&
              pFormatList->FormatInfo[i].nWidth == (UINT)rectAOI.s32Width) {
            format = pFormatList->FormatInfo[i].nFormatID;
            break;
          }
        }
      }
      else {
        throw(vpException(vpException::fatalError, "uEye error: is_ImageFormat returned %d", ret));
      }

      delete (pFormatList);
    }
    else {
      throw(vpException(vpException::fatalError, "uEye error: is_ImageFormat returned %d", ret));
    }
    return format;
  }

  int setActiveCamera(unsigned int cam_index)
  {
    m_cameraList = new CameraList;
    m_activeCameraSelected = m_cameraList->setActiveCamera(cam_index);
    if (!m_activeCameraSelected) {
      m_CamListInfo = m_cameraList->getCameraInfo();
    }
    delete m_cameraList;
    return m_activeCameraSelected;
  }

  std::string toUpper(const std::basic_string<char> &s)
  {
    std::string s_upper = s;
    for (std::basic_string<char>::iterator p = s_upper.begin(); p != s_upper.end(); ++p) {
      *p = toupper(*p);
    }
    return s_upper;
  }

  int setColorMode(const std::string &color_mode)
  {
    if (!isConnected()) {
      throw(vpException(vpException::fatalError,
                        "Cannot set color mode. Connection to active uEye camera is not opened"));
    }

    std::string color_mode_upper = toUpper(color_mode);
    int cm = IS_CM_MONO8;
    if (color_mode_upper == "MONO8") {
      cm = IS_CM_MONO8;
    }
    else if (color_mode_upper == "RGB24") {
      cm = IS_CM_BGR8_PACKED;
    }
    else if (color_mode_upper == "RGB32") {
      cm = IS_CM_RGBA8_PACKED;
    }
    else if (color_mode_upper == "BAYER8") {
      cm = IS_CM_SENSOR_RAW8;
    }
    else {
      throw(vpException(vpException::fatalError, "Unsupported color mode %s", color_mode.c_str()));
    }

    INT ret = IS_SUCCESS;
    if ((ret = is_SetColorMode(m_hCamera, cm)) != IS_SUCCESS) {
      std::cout << "Could not set color mode of " << m_CamListInfo.Model << " to " << color_mode << std::endl;
    }
    else {
      setupCapture();
    }
    return ret;
  }

  int setFrameRate(bool auto_frame_rate, double frame_rate_hz)
  {
    if (!isConnected()) {
      throw(vpException(vpException::fatalError,
                        "Cannot set frame rate. Connection to active uEye camera is not opened"));
    }

    INT ret = IS_SUCCESS;

    // Auto
    if (auto_frame_rate) {
      double pval1 = 0, pval2 = 0;

      // Make sure that auto shutter is enabled before enabling auto frame rate
      bool autoShutterOn = false;
      is_SetAutoParameter(m_hCamera, IS_GET_ENABLE_AUTO_SENSOR_SHUTTER, &pval1, &pval2);
      autoShutterOn |= (pval1 != 0);
      is_SetAutoParameter(m_hCamera, IS_GET_ENABLE_AUTO_SHUTTER, &pval1, &pval2);
      autoShutterOn |= (pval1 != 0);
      if (!autoShutterOn) {
        if (m_verbose) {
          std::cout << "Auto shutter mode is not supported for " << m_CamListInfo.Model << std::endl;
        }
        return IS_NO_SUCCESS;
      }

      // Set frame rate / auto
      pval1 = auto_frame_rate;
      if ((ret = is_SetAutoParameter(m_hCamera, IS_SET_ENABLE_AUTO_SENSOR_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS) {
        if ((ret = is_SetAutoParameter(m_hCamera, IS_SET_ENABLE_AUTO_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS) {
          if (m_verbose) {
            std::cout << "Auto frame rate mode is not supported for " << m_CamListInfo.Model << std::endl;
          }
          return IS_NO_SUCCESS;
        }
      }
    }
    else { // Manual
      double minFrameTime, maxFrameTime, intervalFrameTime, newFrameRate;
      // Make sure that user-requested frame rate is achievable
      if ((ret = is_GetFrameTimeRange(m_hCamera, &minFrameTime, &maxFrameTime, &intervalFrameTime)) != IS_SUCCESS) {
        if (m_verbose) {
          std::cout << "Failed to query valid frame rate range from " << m_CamListInfo.Model << std::endl;
        }
        return ret;
      }
      CAP(frame_rate_hz, 1.0 / maxFrameTime, 1.0 / minFrameTime);

      // Update frame rate
      if ((ret = is_SetFrameRate(m_hCamera, frame_rate_hz, &newFrameRate)) != IS_SUCCESS) {
        if (m_verbose) {
          std::cout << "Failed to set frame rate to " << frame_rate_hz << " MHz for " << m_CamListInfo.Model
            << std::endl;
        }
        return ret;
      }
      else if (frame_rate_hz != newFrameRate) {
        frame_rate_hz = newFrameRate;
      }
    }

    if (m_verbose) {
      std::cout << "Updated frame rate for " << m_CamListInfo.Model << ": "
        << ((auto_frame_rate) ? "auto" : std::to_string(frame_rate_hz)) << " Hz" << std::endl;
    }

    return ret;
  }

  int setExposure(bool auto_exposure, double exposure_ms)
  {
    if (!isConnected()) {
      throw(
          vpException(vpException::fatalError, "Cannot set exposure. Connection to active uEye camera is not opened"));
    }

    INT err = IS_SUCCESS;

    double minExposure, maxExposure;

    // Set auto exposure
    if (auto_exposure) {
      double pval1 = auto_exposure, pval2 = 0;
      if ((err = is_SetAutoParameter(m_hCamera, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER, &pval1, &pval2)) != IS_SUCCESS) {
        if ((err = is_SetAutoParameter(m_hCamera, IS_SET_ENABLE_AUTO_SHUTTER, &pval1, &pval2)) != IS_SUCCESS) {
          std::cout << "Auto exposure mode is not supported for " << m_CamListInfo.Model << std::endl;
          return IS_NO_SUCCESS;
        }
      }
    }

    else { // Set manual exposure timing
      // Make sure that user-requested exposure rate is achievable
      if (((err = is_Exposure(m_hCamera, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN, (void *)&minExposure,
                              sizeof(minExposure))) != IS_SUCCESS) ||
          ((err = is_Exposure(m_hCamera, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX, (void *)&maxExposure,
                              sizeof(maxExposure))) != IS_SUCCESS)) {
        std::cout << "Failed to query valid exposure range from " << m_CamListInfo.Model << std::endl;
        return err;
      }
      CAP(exposure_ms, minExposure, maxExposure);

      // Update exposure
      if ((err = is_Exposure(m_hCamera, IS_EXPOSURE_CMD_SET_EXPOSURE, (void *)&(exposure_ms), sizeof(exposure_ms))) !=
          IS_SUCCESS) {
        std::cout << "Failed to set exposure to " << exposure_ms << " ms for " << m_CamListInfo.Model << std::endl;
        return err;
      }
    }

    if (m_verbose) {
      std::cout << "Updated exposure: " << ((auto_exposure) ? "auto" : std::to_string(exposure_ms) + " ms") << " for "
        << m_CamListInfo.Model << std::endl;
    }

    return err;
  }

  int setGain(bool auto_gain, int master_gain, bool gain_boost)
  {
    if (!isConnected()) {
      throw(vpException(vpException::fatalError, "Cannot set gain. Connection to active uEye camera is not opened"));
    }

    INT err = IS_SUCCESS;

    // Validate arguments
    CAP(master_gain, 0, 100);

    double pval1 = 0, pval2 = 0;

    if (auto_gain) {
      // Set auto gain
      pval1 = 1;
      if ((err = is_SetAutoParameter(m_hCamera, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &pval1, &pval2)) != IS_SUCCESS) {
        if ((err = is_SetAutoParameter(m_hCamera, IS_SET_ENABLE_AUTO_GAIN, &pval1, &pval2)) != IS_SUCCESS) {
          if (m_verbose) {
            std::cout << m_CamListInfo.Model << " does not support auto gain mode" << std::endl;
          }
          return IS_NO_SUCCESS;
        }
      }
    }
    else {
   // Disable auto gain
      if ((err = is_SetAutoParameter(m_hCamera, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &pval1, &pval2)) != IS_SUCCESS) {
        if ((err = is_SetAutoParameter(m_hCamera, IS_SET_ENABLE_AUTO_GAIN, &pval1, &pval2)) != IS_SUCCESS) {
          std::cout << m_CamListInfo.Model << " does not support auto gain mode" << std::endl;
        }
      }

      // Set gain boost
      if (is_SetGainBoost(m_hCamera, IS_GET_SUPPORTED_GAINBOOST) != IS_SET_GAINBOOST_ON) {
        gain_boost = false;
      }
      else {
        if ((err = is_SetGainBoost(m_hCamera, (gain_boost) ? IS_SET_GAINBOOST_ON : IS_SET_GAINBOOST_OFF)) !=
            IS_SUCCESS) {
          std::cout << "Failed to " << ((gain_boost) ? "enable" : "disable") << " gain boost for "
            << m_CamListInfo.Model << std::endl;
        }
      }

      // Set manual gain parameters
      if ((err = is_SetHardwareGain(m_hCamera, master_gain, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER,
                                    IS_IGNORE_PARAMETER)) != IS_SUCCESS) {
        std::cout << "Failed to set manual master gain: " << master_gain << " for " << m_CamListInfo.Model << std::endl;
        return IS_NO_SUCCESS;
      }
    }

    if (m_verbose) {
      if (auto_gain) {
        std::cout << "Updated gain for " << m_CamListInfo.Model << ": auto" << std::endl;
      }
      else {
        std::cout << "Updated gain for " << m_CamListInfo.Model << ": manual master gain " << master_gain << std::endl;
      }
      std::cout << "\n  gain boost: " << (gain_boost ? "enabled" : "disabled") << std::endl;
      ;
    }

    return err;
  }

  void applySubsamplingSettings(int subsamplingMode, int nMode)
  {
    INT ret = IS_SUCCESS;
    int currentSubsampling = is_SetSubSampling(m_hCamera, IS_GET_SUBSAMPLING);
    if ((ret = is_SetSubSampling(m_hCamera, subsamplingMode | nMode)) != IS_SUCCESS) {
      throw(vpException(vpException::fatalError, "Unable to apply subsampling factor"));
    }

    int newSubsampling = is_SetSubSampling(m_hCamera, IS_GET_SUBSAMPLING);
    if ((nMode == IS_SUBSAMPLING_DISABLE) && (currentSubsampling == newSubsampling)) {
      // the subsampling nMode IS_SUBSAMPLING_DISABLE was expected, but the device
      // did not changed the format, disable subsampling.
      if ((ret = is_SetSubSampling(m_hCamera, IS_SUBSAMPLING_DISABLE)) != IS_SUCCESS) {
        throw(vpException(vpException::fatalError, "Unable to apply subsampling factor"));
      }
    }
  }

  void setSubsampling(int factor)
  {
    if (!isConnected()) {
      throw(vpException(vpException::fatalError,
                        "Cannot set sub sampling factor. Connection to active uEye camera is not opened"));
    }

    INT hMode = IS_SUBSAMPLING_DISABLE, vMode = IS_SUBSAMPLING_DISABLE;

    switch (factor) {
    case 2:
      hMode = IS_SUBSAMPLING_2X_HORIZONTAL;
      vMode = IS_SUBSAMPLING_2X_VERTICAL;
      break;
    case 3:
      hMode = IS_SUBSAMPLING_3X_HORIZONTAL;
      vMode = IS_SUBSAMPLING_3X_VERTICAL;
      break;
    case 4:
      hMode = IS_SUBSAMPLING_4X_HORIZONTAL;
      vMode = IS_SUBSAMPLING_4X_VERTICAL;
      break;
    case 5:
      hMode = IS_SUBSAMPLING_5X_HORIZONTAL;
      vMode = IS_SUBSAMPLING_5X_VERTICAL;
      break;
    case 6:
      hMode = IS_SUBSAMPLING_6X_HORIZONTAL;
      vMode = IS_SUBSAMPLING_6X_VERTICAL;
      break;
    case 8:
      hMode = IS_SUBSAMPLING_8X_HORIZONTAL;
      vMode = IS_SUBSAMPLING_8X_VERTICAL;
      break;
    case 16:
      hMode = IS_SUBSAMPLING_16X_HORIZONTAL;
      vMode = IS_SUBSAMPLING_16X_VERTICAL;
      break;
    default:
      hMode = IS_SUBSAMPLING_DISABLE;
      vMode = IS_SUBSAMPLING_DISABLE;
    }

    if (m_bLive && m_bLiveStarted) {
      is_StopLiveVideo(m_hCamera, IS_WAIT);
    }

    INT subsamplingModeH = is_SetSubSampling(m_hCamera, IS_GET_SUBSAMPLING) & IS_SUBSAMPLING_MASK_VERTICAL;
    applySubsamplingSettings(subsamplingModeH, hMode);

    INT subsamplingModeV = is_SetSubSampling(m_hCamera, IS_GET_SUBSAMPLING) & IS_SUBSAMPLING_MASK_HORIZONTAL;
    applySubsamplingSettings(subsamplingModeV, vMode);

    setupCapture();
  }

  void setWhiteBalance(bool auto_wb)
  {
    if (!isConnected()) {
      throw(vpException(vpException::fatalError,
                        "Cannot set white balance. Connection to active uEye camera is not opened"));
    }

    double dblAutoWb = 0.0;

    if (auto_wb) {
      dblAutoWb = 0.0;
      is_SetAutoParameter(m_hCamera, IS_SET_AUTO_WB_ONCE, &dblAutoWb, nullptr);

      dblAutoWb = 1.0;
      is_SetAutoParameter(m_hCamera, IS_SET_ENABLE_AUTO_WHITEBALANCE, &dblAutoWb, nullptr);
    }
    else {
      dblAutoWb = 0.0;
      is_SetAutoParameter(m_hCamera, IS_SET_AUTO_WB_ONCE, &dblAutoWb, nullptr);
      is_SetAutoParameter(m_hCamera, IS_SET_ENABLE_AUTO_WHITEBALANCE, &dblAutoWb, nullptr);
    }
  }

  int setupCapture()
  {
    int width, height;
    // init the memorybuffer properties
    ZeroMemory(&m_BufferProps, sizeof(m_BufferProps));

    IS_RECT rectAOI;
    INT nRet = is_AOI(m_hCamera, IS_AOI_IMAGE_GET_AOI, (void *)&rectAOI, sizeof(rectAOI));

    if (nRet == IS_SUCCESS) {
      width = rectAOI.s32Width;
      height = rectAOI.s32Height;

      // get current colormode
      int colormode = is_SetColorMode(m_hCamera, IS_GET_COLOR_MODE);

      if (colormode == IS_CM_BGR5_PACKED) {
        is_SetColorMode(m_hCamera, IS_CM_BGR565_PACKED);
        colormode = IS_CM_BGR565_PACKED;
        std::cout << "uEye color format 'IS_CM_BGR5_PACKED' actually not supported by vpUeyeGrabber, patched to "
          "'IS_CM_BGR565_PACKED'"
          << std::endl;
      }

      // fill memorybuffer properties
      ZeroMemory(&m_BufferProps, sizeof(m_BufferProps));
      m_BufferProps.width = width;
      m_BufferProps.height = height;
      m_BufferProps.bitspp = getBitsPerPixel(colormode);

      // Reallocate image buffers
      allocImages();

      if (m_verbose) {
        std::cout << "Capture ready set up." << std::endl;
      }
    }
    return 0;
  }

  void setVerbose(bool verbose) { m_verbose = verbose; }

private:
  HIDS m_hCamera; // handle to camera
  int m_activeCameraSelected;
  SENSORINFO m_SensorInfo; // sensor information struct
  CAMINFO m_CamInfo;       // Camera (Board)info
  UEYE_CAMERA_INFO m_CamListInfo;
  char *m_pLastBuffer;
  CameraList *m_cameraList;
  struct sBufferProps m_BufferProps;
  struct sCameraProps m_CameraProps;
  UEYE_IMAGE m_Images[IMAGE_COUNT]; // uEye frame buffer array
  bool m_bLive;                     // live or snapshot indicator
  bool m_bLiveStarted;              // live mode is started
  bool m_verbose;
  /* event waiting for */
  int m_event;
#ifndef __linux__
  /* on windows we need an Event handle member */
  HANDLE m_hEvent;
#endif
  vpImage<vpRGBa> m_I_temp; // Temp image used for Bayer conversion
  };
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*
 **********************************************************************************************
 */

/*!
 * Default constructor.
 * By default, the active camera is the first one that is found.
 * To select a specific camera use setActiveCamera().
 */
vpUeyeGrabber::vpUeyeGrabber() : m_impl(new vpUeyeGrabberImpl()) { }

/*!
 * Destructor.
 */
vpUeyeGrabber::~vpUeyeGrabber() { delete m_impl; }

/*!
 * Capture a new grayscale image.
 *
 * \param[out] I : Captured image.
 * \param[out] timestamp_camera : Time of image capture in milli-seconds with a resolution of 0.1 us, or nullptr if not
 * wanted. The time of image capture is defined as:
 * - The time when a (hardware or software) trigger event is received by the camera in trigger mode.
 *   The delay between the receipt of the trigger signal and the start of exposure depends on the sensor.
 * - The time when the sensor starts to output image data in freerun mode. A rolling shutter sensors
 *   starts to output image data after exposure of the first row. With a global shutter sensor, image data
 *   is output after exposure of all rows.
 * \param[out] timestamp_system : Time with a resolution of 1 ms synchronized with the PC's system time,
 * and resynchronized every 60 seconds. This may cause minor time shifts (average time about 3 ms).
 * The format of the string is the following: `YYYY:MM:DD:HH:MM:SS:mmm` for
 * year, month, day, hour, minute, second, millisecond.
 *
 * To determine the exact interval between two image captures, it is therefore recommended to read out the camera
 * timestamp provided in `timestamp_camera`.
 *
 * \sa setColorMode()
 */
void vpUeyeGrabber::acquire(vpImage<unsigned char> &I, double *timestamp_camera, std::string *timestamp_system)
{
  m_impl->acquire(I, timestamp_camera, timestamp_system);
}

/*!
 * Capture a new color image.
 * \param[out] I : Captured image.
 * \param[out] timestamp_camera : Time of image capture in milli-seconds with a resolution of 0.1 us, or nullptr if not
 * wanted. The time of image capture is defined as:
 * - The time when a (hardware or software) trigger event is received by the camera in trigger mode.
 *   The delay between the receipt of the trigger signal and the start of exposure depends on the sensor.
 * - The time when the sensor starts to output image data in freerun mode. A rolling shutter sensors
 *   starts to output image data after exposure of the first row. With a global shutter sensor, image data
 *   is output after exposure of all rows.
 * \param[out] timestamp_system : Time with a resolution of 1 ms synchronized with the PC's system time,
 * and resynchronized every 60 seconds. This may cause minor time shifts (average time about 3 ms).
 * The format of the string is the following: `YYYY:MM:DD:HH:MM:SS:mmm` for
 * year, month, day, hour, minute, second, millisecond.
 *
 * To determine the exact interval between two image captures, it is therefore recommended to read out the camera
 * timestamp provided in `timestamp_camera`.
 *
 * \sa setColorMode()
 */
void vpUeyeGrabber::acquire(vpImage<vpRGBa> &I, double *timestamp_camera, std::string *timestamp_system)
{
  m_impl->acquire(I, timestamp_camera, timestamp_system);
}

/*!
 * Get active camera model.
 *
 * \sa setActiveCamera(), getActiveCameraSerialNumber()
 */
std::string vpUeyeGrabber::getActiveCameraModel() const { return m_impl->getActiveCameraModel(); }

/*!
 * Get active camera serial number.
 *
 * \sa setActiveCamera(), getActiveCameraModel()
 */
std::string vpUeyeGrabber::getActiveCameraSerialNumber() const { return m_impl->getActiveCameraSerialNumber(); }

/*!
 * Get camera ID list.
 * \return Vector of camera ID corresponding to each camera index. The size of this vector gives correponds
 * also to the number of connected cameras.
 *
 * \sa getCameraModelList(), getCameraSerialNumberList()
 */
std::vector<unsigned int> vpUeyeGrabber::getCameraIDList() const { return m_impl->getCameraIDList(); }

/*!
 * Get camera model list.
 * \return Vector of camera models corresponding to each camera index. The size of this vector gives correponds
 * also to the number of connected cameras.
 *
 * \sa getCameraIDList(), getCameraSerialNumberList()
 */
std::vector<std::string> vpUeyeGrabber::getCameraModelList() const { return m_impl->getCameraModelList(); }

/*!
 * Get camera serial number list.
 * \return Vector of camera serial numbers corresponding to each camera index. The size of this vector gives correponds
 * also to the number of connected cameras.
 *
 * \sa getCameraIDList(), getCameraModelList()
 */
std::vector<std::string> vpUeyeGrabber::getCameraSerialNumberList() const
{
  return m_impl->getCameraSerialNumberList();
}

/*!
 * Returns the current number of frames actually captured per second.
 * This function needs to be called after acquire().
 *
 * \sa acquire()
 */
double vpUeyeGrabber::getFramerate() const { return m_impl->getFramerate(); }

/*!
 * Return image width captured by the active camera.
 * \warning Image width could differ from the one returned by open()
 * depending on the subsampling settings loaded by loadParameters()
 *
 * \sa open(), getFrameWidth()
 */
unsigned int vpUeyeGrabber::getFrameHeight() const { return m_impl->getFrameHeight(); }

/*!
 * Return image height captured by the active camera.
 * \warning Image width could differ from the one returned by open()
 * depending on the subsampling settings loaded by loadParameters()
 *
 * \sa open(), getFrameHeight()
 */
unsigned int vpUeyeGrabber::getFrameWidth() const { return m_impl->getFrameWidth(); }

/*!
 * Return true if a camera is connected, false otherwise.
 */
bool vpUeyeGrabber::isConnected() const { return m_impl->isConnected(); }

/*!
 * Load camera parameters from an `.ini` file.
 * \param[in] filename : Camera parameters file that contains camera settings. Such a file could be produced using
 * `ueyedemo` binary provided with IDS uEye Software Suite
 */
void vpUeyeGrabber::loadParameters(const std::string &filename) { m_impl->loadParameters(filename); }

/*!
 * Starts the driver and establishes the connection to the camera.
 * \param[out] I : Grayscale image that is resized to match the capture settings.
 */
void vpUeyeGrabber::open(vpImage<unsigned char> &I) { m_impl->open(I); }

/*!
 * Starts the driver and establishes the connection to the camera.
 * \param[out] I : Color image that is resized to match the capture settings.
 */
void vpUeyeGrabber::open(vpImage<vpRGBa> &I) { m_impl->open(I); }

/*!
 * Select a camera from the camera list.
 * \param cam_index : Camera index.
 * \return True if successful, false otherwise.
 */
bool vpUeyeGrabber::setActiveCamera(unsigned int cam_index)
{
  return (m_impl->setActiveCamera(cam_index) ? false : true);
}

/*!
 * Updates active camera color mode.
 *
 * \warning Before calling this function the connexion with the active camera should be opened.
 *
 * \param[in] color_mode : Desired color mode. Admissible values are "MONO8", "RGB8", "RGB32" or "BAYER8".
 * \note - When acquiring gray level images using acquire(vpImage<unsigned char> &) we strongly recommend to
 * set color mode to "MONO8".
 * \note - When acquiring color level images using acquire(vpImage<vpRGBa> &) we strongly recommend to
 * set color mode to "RGB32".
 * \return true if color mode is applied, false if the color mode is unsupported.
 *
 *  \sa open(), setExposure(), setFrameRate(), setGain(), setSubsampling(), setWhiteBalance()
 *
 */
bool vpUeyeGrabber::setColorMode(const std::string &color_mode)
{
  return (m_impl->setColorMode(color_mode) ? false : true);
}

/*!
 * Updates active camera exposure / shutter either to auto mode, or to specified manual parameters.
 *
 * \warning Before calling this function the connexion with the active camera should be opened.
 *
 * \param[in] auto_exposure : When true enable camera's hardware auto exposure / shutter.
 * This function returns false if the camera does not support auto exposure mode. When set to false, set manual exposure
 * time. \param[in] exposure_ms : Manual exposure setting time in ms. Valid value range depends on active camera pixel
 * clock rate. \return True if successful, false otherwise.
 *
 * \sa open(), setColorMode(), setFrameRate(), setGain(), setSubsampling(), setWhiteBalance()
 *
 */
bool vpUeyeGrabber::setExposure(bool auto_exposure, double exposure_ms)
{
  return (m_impl->setExposure(auto_exposure, exposure_ms) ? false : true);
}

/*!
 * Updates active camera frame rate either to auto mode, or to a specified manual value.
 *
 * Enabling auto frame rate mode requires to enable auto shutter mode.
 * Enabling auto frame rate mode will disable auto gain mode.
 *
 * \warning Before calling this function the connexion with the active camera should be opened.
 *
 * \param[in] auto_frame_rate : Updates camera's hardware auto frame rate mode. When true enable auto frame rate mode.
 * When set to false, enables manual frame rate mode.
 * \param[in] manual_frame_rate_hz : Desired manual frame rate in Hz. Valid value range depends on current camera pixel
 * clock rate. This parameter is only used when auto frame rate is disabled. \return True if successful, false
 * otherwise.
 *
 * A typical usage is the following:
 * \code
 * vpUeyeGrabber g;
 * vpImage<unsigned char> I;
 * g.open(I);
 * if (! g.setFrameRate(true)) {
 *   std::cout << "Unable to set auto frame rate" << std::endl;
 *   double manual_fps = 10.; // 10 Hz is required if auto frame rate is not possible
 *   if (g.setFrameRate(false, manual_fps)) {
 *     std::cout << "Framerate set to: " << manual_fps << std::endl;
 *   }
 * }
 * \endcode
 *
 * \sa open(), setColorMode(), setExposure(), setGain(), setSubsampling(), setWhiteBalance()
 */
bool vpUeyeGrabber::setFrameRate(bool auto_frame_rate, double manual_frame_rate_hz)
{
  return (m_impl->setFrameRate(auto_frame_rate, manual_frame_rate_hz) ? false : true);
}

/*!
 * Updates active camera gain either to auto mode, or to specified manual parameters.
 *
 * Auto gain mode is disabled if auto frame rate mode is enabled.
 *
 * \warning Before calling this function the connexion with the active camera should be opened.
 *
 * \param[in] auto_gain : Updates camera's hardware auto gain mode.
 * - Set to true to enable auto gain.  If this mode is not supported, returns false.
 * - Set to false, to set manual gain and enable/disable gain boost/
 * \param[in] master_gain : Manual master gain percentage in range 0 - 100.
 * \param[in] gain_boost : Only in manual mode, enable/disable gain boost.
 * \return True if successful, false otherwise.
 *
 * \sa open(), setColorMode(), setExposure(), setFrameRate(), setSubsampling(), setWhiteBalance()
 */
bool vpUeyeGrabber::setGain(bool auto_gain, int master_gain, bool gain_boost)
{
  return (m_impl->setGain(auto_gain, master_gain, gain_boost) ? false : true);
}

/*!
 * Updates active camera image subsampling factor to reduce image size.
 *
 * \warning Before calling this function the connexion with the active camera should be opened.
 *
 * \param[in] factor : Desired subsampling factor. The number of rows and columns
 * of the resulting image corresponds to the full resolution image size divided by this factor.
 *
 * \sa open(), setColorMode(), setExposure(), setFrameRate(), setGain(), setWhiteBalance()
 */
void vpUeyeGrabber::setSubsampling(int factor) { m_impl->setSubsampling(factor); }

/*!
 * Enables or disables the active camera auto white balance mode.
 *
 * \warning Before calling this function the connexion with the active camera should be opened.
 *
 * \param auto_wb : If true enable auto white balance mode. If false, disable auto white balance mode.
 *
 * \sa open(), setColorMode(), setExposure(), setFrameRate(), setGain(), setSubsampling()
 *
 */
void vpUeyeGrabber::setWhiteBalance(bool auto_wb) { m_impl->setWhiteBalance(auto_wb); }

/*!
 * Enable/disable verbose mode.
 *
 * \param[in] verbose : true to enable, false to disable verbose mode.
 */
void vpUeyeGrabber::setVerbose(bool verbose) { m_impl->setVerbose(verbose); }
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_sensor.a(vpUeyeGrabber.cpp.o) has no symbols
void dummy_vpUeyeGrabber() { };

#endif
