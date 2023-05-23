#ifndef VPMEGAPOSETRACKER_HPP
#define VPMEGAPOSETRACKER_HPP

#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_NLOHMANN_JSON)

#include <future>
#include<memory>
#include <visp3/dnn_tracker/vpMegaPose.h>

class vpMegaPoseTracker {
public:
  vpMegaPoseTracker(std::shared_ptr<vpMegaPose> megapose, const std::string& objectLabel, const int refinerIterations) :
    megapose(megapose), objectLabel(objectLabel), refinerIterations(refinerIterations)
  {
  }
  std::future<vpMegaPoseEstimate> init(const vpImage<vpRGBa>& I, const vpRect& bb);
  std::future<vpMegaPoseEstimate> track(const vpImage<vpRGBa>& I);

private:
  std::shared_ptr<vpMegaPose> megapose;
  vpMegaPoseEstimate poseEstimate;
  std::string objectLabel;
  int refinerIterations;
};

#endif // VISP_HAVE_NLOHMANN_JSON

#endif