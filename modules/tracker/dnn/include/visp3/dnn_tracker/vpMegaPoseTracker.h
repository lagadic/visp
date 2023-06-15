#ifndef VPMEGAPOSETRACKER_HPP
#define VPMEGAPOSETRACKER_HPP

#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_NLOHMANN_JSON)

#include <future>
#include<memory>
#include <visp3/dnn_tracker/vpMegaPose.h>
/**
 * \class vpMegaPoseTracker
 * \ingroup module_dnn_tracker
 *
 * @brief A simplified interface to track a single object with megapose.
 * This tracker works asynchronously: A call to init or track will not stop the current thread. Rather, an std::future object is returned, and its result should be acquired when it is ready.
 *
 * To instantiate and use the tracker:
 *
 * \code{.cpp}
 * #include <visp3/dnn_tracker/vpMegaposeTracker.h>
 *
 * int main()
 * {
 *    vpCameraParameters cam;
 *    cam.initPersProjWithoutDistortion(500.0, 500.0, 320.0, 240.0);
 *    std::shared_ptr<vpMegaPose> megapose;
 *    try {
 *      megapose = std::make_shared<vpMegaPose>("127.0.0.1", 5555, cam, 480, 640);
 *    }
 *    catch (vpException &e) {
 *      throw vpException(vpException::ioError, "Could not connect to megapose server.");
 *    }
 *    vpMegaPoseTracker megaposeTracker(megapose, "my_object_name", 1);
 *    //...
 *    vpRect detection;
 *    vpImage<vpRGBa> I(480, 640);
 *    // Perform object detection for init
 *    // detection = ...;
 *    // Acquire Image I
 *    // I = ...;
 *    std::future<vpMegaPoseEstimate> futurePoseEstimate;
 *
 *    futurePoseEstimate = megaposeTracker.init(I, detection);
 *    // Do something else
 *    vpMegaPoseEstimate estimate = futurePoseEstimate.get(); // Block and await result
 *    bool callMegapose = true; // True when we should call megapose
 *    while(true) { // Run continuously, update results when megapose returns new results
 *      // I = ...;
 *      if (!callMegapose && futurePosEstimate.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
 *        // process result
 *        callMegapose = true;
 *      }
 *      if (callMegapose) {
 *        futurePoseEstimate = megaposeTracker.track(I);
 *        callMegapose = false;
 *      }
 *    }
 *
 * }
 * \endcode
 * For a more detailed usage see \ref tutorial-tracking-megapose
 */
class VISP_EXPORT vpMegaPoseTracker
{
public:
  /**
   * @brief Construct a new Megapose tracker.
   *
   * @param megapose a valid connection to the megapose server
   * @param objectLabel The name of the object to track
   * @param refinerIterations Number of refiner iterations to perform every time init or track is called. Impacts performance.
   */
  vpMegaPoseTracker(std::shared_ptr<vpMegaPose> megapose, const std::string &objectLabel, const int refinerIterations) :
    megapose(megapose), objectLabel(objectLabel), refinerIterations(refinerIterations), initialized(false)
  { }
  /**
   * @brief Initialize tracking. Performs a full object pose estimation with megapose.
   *
   * This is slower than tracking.
   *
   * Requires a bounding box of the object to track.
   *
   * @param I The image in which the object is located
   * @param bb The bounding box of the object
   * @return std::future<vpMegaPoseEstimate> A future object that will contain the result of the pose estimation
   */
  std::future<vpMegaPoseEstimate> init(const vpImage<vpRGBa> &I, const vpRect & bb);
  /**
   * @brief Initialize tracking from an initial pose. The initial pose should be in the neighborhood of the true pose.
   * The pose should be expressed in the camera frame.
   * This method will call megapose to correct the initial estimate and initialize the tracking.
   *
   * @param I The image in which the object is located
   * @param cTo An initial, coarse, estimate of the object pose
   * @return std::future<vpMegaPoseEstimate> A future object that will contain the result of the pose estimation
   */
  std::future<vpMegaPoseEstimate> init(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cTo);


  /**
   * @brief Track the object in the image. Requires the tracker to be initialized by calling init.
   *
   * @param I The image containing the object to track
   * @return std::future<vpMegaPoseEstimate> A future object that will contain the result of the pose estimation
   */
  std::future<vpMegaPoseEstimate> track(const vpImage<vpRGBa> &I);

private:
  std::shared_ptr<vpMegaPose> megapose;
  vpMegaPoseEstimate poseEstimate;
  std::string objectLabel;
  int refinerIterations;
  bool initialized;
};

#endif // VISP_HAVE_NLOHMANN_JSON

#endif