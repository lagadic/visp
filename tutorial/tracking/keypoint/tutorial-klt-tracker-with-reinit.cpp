//! \example tutorial-klt-tracker-with-reinit.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/klt/vpKltOpencv.h>

int main()
{
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO) && defined(HAVE_OPENCV_VIDEOIO)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  try {
    vpVideoReader reader;
    reader.setFileName("video-postcard.mp4");

    vpImage<unsigned char> I;
    reader.acquire(I);

    cv::Mat cvI;

    vpImageConvert::convert(I, cvI);

    // Display initialisation
    vpDisplayOpenCV d(I, 0, 0, "Klt tracking");
    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpKltOpencv tracker;
    // Set tracker parameters
    tracker.setMaxFeatures(200);
    tracker.setWindowSize(10);
    tracker.setQuality(0.01);
    tracker.setMinDistance(15);
    tracker.setHarrisFreeParameter(0.04);
    tracker.setBlockSize(9);
    tracker.setUseHarris(1);
    tracker.setPyramidLevels(3);

    // Initialise the tracking
    tracker.initTracking(cvI);

    while (!reader.end()) {
      reader.acquire(I);
      std::cout << "Process image " << reader.getFrameIndex() << std::endl;
      vpDisplay::display(I);

      vpImageConvert::convert(I, cvI);

      //! [Re-init tracker]
      // Restart the initialization to detect new keypoints
      if (reader.getFrameIndex() == 25) {
        std::cout << "Re initialize the tracker" << std::endl;

        // Save of previous features
        std::vector<cv::Point2f> prev_features = tracker.getFeatures();

        // Start a new feature detection
        tracker.initTracking(cvI);
        std::vector<cv::Point2f> new_features = tracker.getFeatures();

        // Add previous features if they are not to close to detected one
        double distance, minDistance_ = tracker.getMinDistance();
        for (size_t i = 0; i < prev_features.size(); i++) {
          // Test if a previous feature is not redundant with one of the newly
          // detected
          bool is_redundant = false;
          for (size_t j = 0; j < new_features.size(); j++) {
            distance = sqrt(vpMath::sqr(new_features[j].x - prev_features[i].x) +
                            vpMath::sqr(new_features[j].y - prev_features[i].y));
            if (distance < minDistance_) {
              is_redundant = true;
              break;
            }
          }
          if (is_redundant) {
            continue;
          }
          // std::cout << "Add previous feature with index " << i <<
          // std::endl;
          tracker.addFeature(prev_features[i]);
        }
      }
      // Track the features
      tracker.track(cvI);
      //! [Re-init tracker]

      std::cout << "tracking of " << tracker.getNbFeatures() << " features" << std::endl;

      tracker.display(I, vpColor::red);
      vpDisplay::flush(I);
    }

    vpDisplay::getClick(I);

  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
#endif
  return EXIT_SUCCESS;
}
