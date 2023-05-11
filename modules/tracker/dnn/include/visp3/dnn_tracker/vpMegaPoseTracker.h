#ifndef VPMEGAPOSETRACKER_HPP
#define VPMEGAPOSETRACKER_HPP

#include <visp3/dnn_tracker/vpMegaPose.h>
#include <future>



class vpMegaPoseTracker {
public:
  vpMegaPoseTracker(vpMegaPose& megapose, const std::string& objectLabel, const int refinerIterations) :
    megapose(megapose), objectLabel(objectLabel), refinerIterations(refinerIterations)
  {
  }
  std::future<vpMegaPoseEstimate> init(const vpImage<vpRGBa>& I, const vpRect& bb);
  std::future<vpMegaPoseEstimate> track(const vpImage<vpRGBa>& I);

private:
  vpMegaPose& megapose;
  vpMegaPoseEstimate poseEstimate;
  std::string objectLabel;
  int refinerIterations;
};
#endif