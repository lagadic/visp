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
 * MegaPose wrapper.
 *
*****************************************************************************/
#ifndef _vpMegaPose_h_
#define _vpMegaPose_h_

#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_THREADS)

#include <vector>
#include <string>
#include <unordered_map>
#include <mutex>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpRect.h>

#include <nlohmann/json.hpp>

BEGIN_VISP_NAMESPACE
/**
 * \class vpMegaPoseEstimate
 * \ingroup module_dnn_tracker
 * Result from a pose estimation performed by MegaPose.
 * Contains:
 * - the estimated pose as a vpHomogeneousMatrix
 * - The confidence score between 0-1.
 * This score is defined as 1 if the current pose is in the basin of attraction of the true pose
 * (whether the MegaPose refiner can converge to the true pose). As such, it should be used to detect divergence.
 * - The bounding box of the object in image space.
*/
class vpMegaPoseEstimate
{
public:
  vpMegaPoseEstimate() : score(0.f) { }
  vpHomogeneousMatrix cTo;
  float score;
  vpRect boundingBox;
};

// Don't use the default ViSP JSON conversion to avoid potential regressions (that wouldn't be detected)
// and very specific parsing on the server side
inline void from_megapose_json(const nlohmann::json &j, vpHomogeneousMatrix &T)
{
  std::vector<double> values = j;
  assert(values.size() == 16);
  std::copy(values.begin(), values.end(), T.data);
}

inline void to_megapose_json(nlohmann::json &j, const vpHomogeneousMatrix &T)
{
  std::vector<double> values;
  values.reserve(16);
  for (unsigned i = 0; i < 16; ++i) {
    values.push_back(T.data[i]);
  }
  j = values;
}

inline void to_megapose_json(nlohmann::json &j, const vpRect &d)
{
  std::vector<double> values = {
    d.getLeft(), d.getTop(), d.getRight(), d.getBottom()
  };
  j = values;
}

inline void from_megapose_json(const nlohmann::json &j, vpRect &d)
{
  std::vector<double> values = j.get<std::vector<double>>();
  assert((values.size() == 4));
  d.setLeft(values[0]);
  d.setTop(values[1]);
  d.setRight(values[2]);
  d.setBottom(values[3]);
}

inline void from_json(const nlohmann::json &j, vpMegaPoseEstimate &m)
{
  m.score = j["score"];
  from_megapose_json(j.at("cTo"), m.cTo);
  from_megapose_json(j.at("boundingBox"), m.boundingBox);
}

/**
  * \class vpMegaPose
  * \ingroup module_dnn_tracker
  * Class to communicate with a MegaPose server.
  * MegaPose is a deep learning-based method to estimate the 6D pose of novel objects, meaning that it does not require training for a specific object.
  * MegaPose is a multistage method and can be used either as a full pose estimation algorithm or as a tracker.
  * MegaPose works by render and compare: a synthetized view of an object is compared with a real-world image to see if the poses match.
  * Behind the scene, there are two main models:
  * - The coarse model: given an image, multiple synthetic view are compared with the "true" object image and a model outputs the probability of a render corresponding to the same object as the true image. This model requires the object to be detected (bounding box) in the image.
  * - The refiner model: Given an initial pose estimate, this model predicts a pose displacement to best align render and true image. It is called iteratively, for a fixed number of iterations.
  *
  * This can be best visualized in the figure below (taken from \cite Labbe2022Megapose):
  * \image html megapose_architecture.jpg
  *
  * For more information on how the model works, see <a href="https://megapose6d.github.io/">The MegaPose Github page</a> or the paper \cite Labbe2022Megapose.
  *
  * For instructions on how to install the Python server and an example usage, see \ref tutorial-tracking-megapose.
*/
class VISP_EXPORT vpMegaPose
{
public:
  /**
  * Enum to communicate with python server.
  * Used to map to a string, which the client and server check to see what message they are getting/expecting.
  */
  enum ServerMessage
  {
    UNKNOWN = 0,
    ERR = 1, //! An error occurred server side
    OK = 2, //! Server has successfully completed operation, no return value expected
    GET_POSE = 3, //! Ask the server to estimate poses
    RET_POSE = 4, //! Code sent when server returns pose estimates
    GET_VIZ = 5, //! Ask the server for a rendering of the object
    RET_VIZ = 6, //! Code sent when server returns the rendering of an object
    SET_INTR = 7, //! Set the intrinsics for the MegaPose server
    GET_SCORE = 8, //! Ask the server to score a pose estimate
    RET_SCORE = 9, //! Code sent when server returns a pose score
    SET_SO3_GRID_SIZE = 10, //! Ask the server to set the number of samples for coarse estimation
    GET_LIST_OBJECTS = 11,
    RET_LIST_OBJECTS = 12,
    EXIT = 13
  };
  /**
  * Instantiates a connection to a MegaPose server.
  * The server should already be started  and listening at host:port.
  * \param[in] host : The host to connect to (IP address).
  * \param[in] port : The port on which the server is listening for incoming connections.
  * \param[in] cam : Intrinsics of the camera with which the images sent to the MegaPose server are acquired.
  * \param[in] height : Height of the images sent to the server.
  * \param[in] width : Width of the images sent to the server.
  */
  vpMegaPose(const std::string &host, int port, const vpCameraParameters &cam, unsigned height, unsigned width);

  /**
  * Estimate the poses of objects (in the frame of the camera c) with MegaPose.
  * The object origins used to estimate the poses are those used by the MegaPose server.
  * \param[in] image : The image, acquired by camera c, used to estimate the pose of the objects.
  * \param[in] objectNames : Names of the objects for which to estimate the pose. The name of the object should be known by the MegaPose server.
  * An object name can appear multiple times if multiple instances of the object are in the image and their pose should estimated.
  * \param[in] depth : An optional depth image, that must be aligned with the RGB image. If provided, the MegaPose server should be configure to use the depth.
  * Note that the using depth may lead to a noisy estimation and should not always be preferred.
  * \param[in] depthToM : A scale factor that is used to convert the depth map into meters. If depth is null, the value is ignored.
  * \param[in] detections : The bounding boxes of the objects \e objectNames. Used only for the coarse model, which uses the size of the bounding box to generate initial guesses.
  * If specified, should be the same size as \e objectNames. The bounding box at index i will be for the object i.
  * \param[in] initial_cTos : An optional initial pose estimate for each object present in the image. If not null, then the MegaPose server only runs the refiner model,
  * which is faster but only allows for smaller corrections (depending on the number of refiner iterations).
  * If specified, should be the same size as \e objectNames. The initial pose estimate at index i will be for the object at index i.
  * \param[in] refinerIterations : Number of MegaPose refiner iterations to be performed.
  *
  * \return A list of vpMegaPoseEstimate, one for each input object, (same length as \e objectNames)
  */
  std::vector<vpMegaPoseEstimate> estimatePoses(const vpImage<vpRGBa> &image, const std::vector<std::string> &objectNames,
                                                const vpImage<uint16_t> *const depth = nullptr, const double depthToM = 0.f,
                                                const std::vector<vpRect> *const detections = nullptr,
                                                const std::vector<vpHomogeneousMatrix> *const initial_cTos = nullptr,
                                                int refinerIterations = -1);
  /**
  * Score the input poses with MegaPose. The score, between 0 and 1, indicates whether the refiner model can converge to the correct pose.
  * As such, it should mainly be used to detect divergence, and not the quality of the pose estimate.
  *
  * \param[in] image : The input RGB image in which the objects are located.
  * \param[in] objectNames : The detected objects. Names should be known by the MegaPose server.
  * \param[in] cTos : The object poses to be scored.
  *
  * \return A list of scores, each between 0 and 1. Returns one score per object (the result has the same length as objectNames)
  */
  std::vector<double> scorePoses(const vpImage<vpRGBa> &image, const std::vector<std::string> &objectNames,
                                  const std::vector<vpHomogeneousMatrix> &cTos);

  /**
  * Set the camera parameters for the MegaPose server.
  * \param[in] cam : The camera intrinsics.
  * \param[in] height : The incoming images' height.
  * \param[in] width : The incoming images' width.
  * Note that the height and width should be equal to the width of the images used in estimatePoses or scorePoses. Otherwise an exception will be thrown.
  */
  void setIntrinsics(const vpCameraParameters &cam, unsigned height, unsigned width);

  vpImage<vpRGBa> viewObjects(const std::vector<std::string> &objectNames,
                              const std::vector<vpHomogeneousMatrix> &poses, const std::string &viewType);

  /**
  * Set the number of renders used for coarse pose estimation by MegaPose.
  * \param[in] num : The number of renders for full pose estimation by MegaPose. This number should be equal to 72, 512, 576 or 4608.
  */
  void setCoarseNumSamples(const unsigned num);

  /**
   * @brief Query the server to find the name of all of the objects it knows.
   * @return The names of the objects known by the server
   */
  std::vector<std::string> getObjectNames();

  ~vpMegaPose();

private:
  // Server connection data
  int m_serverSocket;
  int m_fd;

  std::mutex m_mutex; // Since client-server communications are synchronous, avoid multiple parallel communications

  void makeMessage(const vpMegaPose::ServerMessage messageType, std::vector<uint8_t> &data) const;
  std::pair<vpMegaPose::ServerMessage, std::vector<uint8_t>> readMessage() const;

  const static std::unordered_map<vpMegaPose::ServerMessage, std::string> m_codeMap;
  static std::string messageToString(const vpMegaPose::ServerMessage messageType);
  static vpMegaPose::ServerMessage stringToMessage(const std::string &s);
};

END_VISP_NAMESPACE
#endif // VISP_HAVE_NLOHMANN_JSON
#endif
