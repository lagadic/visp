#include <visp3/dnn_tracker/vpMegaPoseTracker.h>
#include <future>

std::future<vpMegaPoseEstimate> vpMegaPoseTracker::init(const vpImage<vpRGBa> &I, const vpRect &bb)
{
  return std::async(std::launch::async, [&I, &bb, this]() -> vpMegaPoseEstimate {
    std::vector<vpRect> bbs = {bb};
    this->poseEstimate = megapose->estimatePoses(I, {this->objectLabel}, nullptr, 0.0, &bbs, nullptr)[0];
    this->initialized = true;
    return this->poseEstimate;
                    });
}
std::future<vpMegaPoseEstimate> vpMegaPoseTracker::init(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cTo)
{
  return std::async(std::launch::async, [&I, &cTo, this]() -> vpMegaPoseEstimate {
    std::vector<vpHomogeneousMatrix> poses = { cTo };
    this->poseEstimate = megapose->estimatePoses(I, { this->objectLabel }, nullptr, 0.0, nullptr, &poses)[0];
    this->initialized = true;
    return this->poseEstimate;
  });
}

std::future<vpMegaPoseEstimate> vpMegaPoseTracker::track(const vpImage<vpRGBa> &I)
{
  if (!initialized) {
    throw vpException(vpException::notInitialized, "Megapose tracker was not initialized. Call init before calling track.");
  }
  return std::async(std::launch::async, [&I, this]() -> vpMegaPoseEstimate {
    std::vector<vpHomogeneousMatrix> poses = {this->poseEstimate.cTo};
    this->poseEstimate = megapose->estimatePoses(I, {this->objectLabel}, nullptr, 0.0, nullptr, &poses, refinerIterations)[0];
    return this->poseEstimate;
  });
}