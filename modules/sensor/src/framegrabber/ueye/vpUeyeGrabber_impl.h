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

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_VISP_NAMESPACE
class CameraList
{
public:
  CameraList() : m_pCamList(nullptr), m_CamInfo()
  {
    // init the internal camera info structure
    ZeroMemory(&m_CamInfo, sizeof(UEYE_CAMERA_INFO));
    // get the cameralist from SDK
    initCameraList();

    m_CamInfo.dwDeviceID = -1;
  }

  ~CameraList() { deleteCameraList(); }

  bool initCameraList()
  {
    /*!
     * \par Retrieve the camera list from libueye_api.so
     *
     * According to the libueye_api documentation, camera list query is performed in two steps:
     * \li Query the number of available cameras by calling is_GetCameralist() with one
     * UEYE_CAMERA_LIST structure as argument whose member dwCount ist zero. The number of cameras
     * may then be retrieved from the data returned by is_GetCameralist() by reading
     * m_pCamList->dwCount.
     * \li Allocate the number of elements of type UEYE_CAMERA_LIST and call is_GetCameraList()
     * again according to the uEye SDK documentation. The number of elements to retrieve must be
     * given in member m_pCamList->dwCount of the first array element.
     */
    bool ret;
    deleteCameraList();
    m_pCamList = new UEYE_CAMERA_LIST;

    m_pCamList->dwCount = 0;

    if (is_GetCameraList(m_pCamList) == IS_SUCCESS) {
      DWORD dw = m_pCamList->dwCount;
      delete m_pCamList;
      m_pCamList = nullptr;

      if (dw) {
        // Reallocate the required camera list size
        m_pCamList = (PUEYE_CAMERA_LIST) new char[sizeof(DWORD) + dw * sizeof(UEYE_CAMERA_INFO)];
        m_pCamList->dwCount = dw;

        // Get CameraList and store it ...
        if (is_GetCameraList(m_pCamList) == IS_SUCCESS) {
          // SelectCamera (0);
          ret = true;
        }
        else {
          ret = false;
        }
      }
      else {
        ret = false;
      }
    }
    else {
      ret = false;
    }

    return ret;
  }

  void deleteCameraList()
  {
    if (m_pCamList)
      delete m_pCamList;
    m_pCamList = nullptr;

    ZeroMemory(&m_CamInfo, sizeof(UEYE_CAMERA_INFO));
  }

  std::vector<unsigned int> getCameraIDList()
  {
    std::vector<unsigned int> ids;
    for (unsigned int i = 0; i < size(); i++) {
      ids.push_back(m_pCamList->uci[i].dwDeviceID);
    }
    return ids;
  }

  std::vector<std::string> getCameraModelList()
  {
    std::vector<std::string> models;
    for (unsigned int i = 0; i < size(); i++) {
      models.push_back(m_pCamList->uci[i].Model);
    }
    return models;
  }

  std::vector<std::string> getCameraSerialNumberList()
  {
    std::vector<std::string> serials;
    for (unsigned int i = 0; i < size(); i++) {
      serials.push_back(m_pCamList->uci[i].SerNo);
    }
    return serials;
  }

  UEYE_CAMERA_INFO getCameraInfo() { return m_CamInfo; }

  /*!
   * Select a camera from the camera list.
   * \param cam_index : Camera index.
   * \return Zero if successful, nonzero otherwise.
   */
  int setActiveCamera(unsigned int cam_index)
  {
    if (!m_pCamList) {
      return -1;
    }
    if (cam_index >= m_pCamList->dwCount)
      return -1;
    // copy current camera info
    memcpy(&m_CamInfo, &m_pCamList->uci[cam_index], sizeof(UEYE_CAMERA_INFO));
    return 0;
  }

  /*!
   * Select a camera from the camera list by her ID.
   * \param cam_id :  Camera ID.
   * \return Zero if successful and camera exist, nonzero otherwise
   */
  int selectCameraByID(unsigned int cam_id)
  {
    for (unsigned int i = 0; i < size(); i++) {
      if (m_pCamList->uci[i].dwDeviceID == cam_id) {
        // copy current camera info
        memcpy(&m_CamInfo, &m_pCamList->uci[i], sizeof(UEYE_CAMERA_INFO));
        return 0;
      }
    }
    return -1;
  }

  unsigned int size()
  {
    if (m_pCamList) {
      return (unsigned int)m_pCamList->dwCount;
    }
    else {
      return 0;
    }
  }

private:
  PUEYE_CAMERA_LIST m_pCamList;
  UEYE_CAMERA_INFO m_CamInfo;
};
END_VISP_NAMESPACE

/*
 **********************************************************************************************
 */

namespace helper
{
class LockUnlockSeqBuffer
{
public:
  LockUnlockSeqBuffer(HIDS hCam, INT nSeqNum, char *pcMem);
  ~LockUnlockSeqBuffer(void);

  bool OwnsLock(void) const { return m_bOwnsLock; }
  void Unlock(void);

private:
  LockUnlockSeqBuffer(void);
  LockUnlockSeqBuffer(const LockUnlockSeqBuffer &);
  void operator=(LockUnlockSeqBuffer);

private:
  HIDS m_hCam;
  INT m_nSeqNum;
  char *m_pcMem;
  bool m_bOwnsLock;
};

LockUnlockSeqBuffer::LockUnlockSeqBuffer(HIDS hCam, INT nSeqNum, char *pcMem)
  : m_hCam(hCam), m_nSeqNum(nSeqNum), m_pcMem(pcMem), m_bOwnsLock(false)
{
  INT nRet = is_LockSeqBuf(m_hCam, m_nSeqNum, m_pcMem);

  m_bOwnsLock = (IS_SUCCESS == nRet);
}

LockUnlockSeqBuffer::~LockUnlockSeqBuffer(void) { Unlock(); }

void LockUnlockSeqBuffer::Unlock(void)
{
  if (m_bOwnsLock) {
    is_UnlockSeqBuf(m_hCam, m_nSeqNum, m_pcMem);
    m_bOwnsLock = false;
  }
}
} // namespace helper

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
