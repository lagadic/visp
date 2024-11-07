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
 * Tracker based on MegaPose.
 *
*****************************************************************************/

#ifndef _vpMegaPoseTracker_h_
#define _vpMegaPoseTracker_h_

#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_THREADS)

#include <future>
#include <memory>
#include <visp3/dnn_tracker/vpMegaPose.h>

BEGIN_VISP_NAMESPACE
/**
 * \class vpMegaPoseTracker
 * \ingroup module_dnn_tracker
 *
 * @brief A simplified interface to track a single object with MegaPose.
 * This tracker works asynchronously: A call to init or track will not stop the current thread. Rather, an std::future object is returned, and its result should be acquired when it is ready.
 *
 * To instantiate and use the tracker:
 *
 * \code{.cpp}
 * #include <visp3/dnn_tracker/vpMegaPoseTracker.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpCameraParameters cam;
 *   cam.initPersProjWithoutDistortion(500.0, 500.0, 320.0, 240.0);
 *   std::shared_ptr<vpMegaPose> megapose;
 *   try {
 *     megapose = std::make_shared<vpMegaPose>("127.0.0.1", 5555, cam, 480, 640);
 *   }
 *   catch (vpException &e) {
 *     throw vpException(vpException::ioError, "Could not connect to MegaPose server.");
 *   }
 *   vpMegaPoseTracker megaposeTracker(megapose, "my_object_name", 1);
 *   //...
 *   vpRect detection;
 *   vpImage<vpRGBa> I(480, 640);
 *   // Perform object detection for init
 *   // detection = ...;
 *   // Acquire Image I
 *   // I = ...;
 *   std::future<vpMegaPoseEstimate> futurePoseEstimate;
 *
 *   futurePoseEstimate = megaposeTracker.init(I, detection);
 *   // Do something else
 *   vpMegaPoseEstimate estimate = futurePoseEstimate.get(); // Block and await result
 *   bool callMegapose = true; // True when we should call megapose
 *   while(true) { // Run continuously, update results when MegaPose returns new results
 *     // I = ...;
 *     if (!callMegapose && futurePosEstimate.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
 *       // process result
 *       callMegapose = true;
 *     }
 *     if (callMegapose) {
 *       futurePoseEstimate = megaposeTracker.track(I);
 *       callMegapose = false;
 *     }
 *   }
 * }
 * \endcode
 * For a more detailed usage see \ref tutorial-tracking-megapose.
*/
class VISP_EXPORT vpMegaPoseTracker
{
public:
  /**
   * @brief Construct a new MegaPose tracker.
   *
   * @param[in] megapose : A valid connection to the MegaPose server.
   * @param[in] objectLabel : The name of the object to track.
   * @param[in] refinerIterations : Number of refiner iterations to perform every time init or track is called. Impacts performance.
   */
  vpMegaPoseTracker(std::shared_ptr<vpMegaPose> megapose, const std::string &objectLabel, const int refinerIterations) :
    m_megapose(megapose), m_objectLabel(objectLabel), m_refinerIterations(refinerIterations), m_initialized(false)
  { }
  /**
   * @brief Initialize tracking. Performs a full object pose estimation with megapose.
   *
   * This is slower than tracking.
   *
   * Requires a bounding box of the object to track.
   *
   * @param[in] I : The image in which the object is located.
   * @param[in] bb : The bounding box of the object.
   * @return A future object that will contain the result of the pose estimation.
   */
  std::future<vpMegaPoseEstimate> init(const vpImage<vpRGBa> &I, const vpRect &bb);
  /**
   * @brief Initialize tracking from an initial pose. The initial pose should be in the neighborhood of the true pose.
   * The pose should be expressed in the camera frame.
   * This method will call MegaPose to correct the initial estimate and initialize the tracking.
   *
   * @param[in] I : The image in which the object is located.
   * @param[in] cTo : An initial, coarse, estimate of the object pose.
   * @return A future object that will contain the result of the pose estimation.
   */
  std::future<vpMegaPoseEstimate> init(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cTo);


  /**
   * @brief Track the object in the image. Requires the tracker to be initialized by calling init.
   *
   * @param[in] I : The image containing the object to track.
   * @return A future object that will contain the result of the pose estimation.
   */
  std::future<vpMegaPoseEstimate> track(const vpImage<vpRGBa> &I);

  /**
   * @brief Update the current pose estimate with a new one, provided by an external source.
   * No operation (such as init or track) should be running.
   *
   * @param[in] cTo : The new pose estimate.
   */
  void updatePose(const vpHomogeneousMatrix &cTo);

private:
  std::shared_ptr<vpMegaPose> m_megapose;
  vpMegaPoseEstimate m_poseEstimate;
  std::string m_objectLabel;
  int m_refinerIterations;
  bool m_initialized;
};
END_VISP_NAMESPACE
#endif // VISP_HAVE_NLOHMANN_JSON

#endif
