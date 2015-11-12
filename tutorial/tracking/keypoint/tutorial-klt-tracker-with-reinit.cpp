//! \example tutorial-klt-tracker-with-reinit.cpp
#include <visp3/core/vpImageConvert.h>
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpVideoReader.h>

int main()
{
#ifdef VISP_HAVE_OPENCV
  try {
    vpVideoReader reader;
    reader.setFileName("video-postcard.mpeg");

    vpImage<unsigned char> I;
    reader.acquire(I);

#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    IplImage * cvI = NULL;
#else
    cv::Mat cvI;
#endif
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

    while ( ! reader.end() )
    {
      reader.acquire(I);
      std::cout << "acquire image " << reader.getFrameIndex() << std::endl;
      vpDisplay::display(I);

      vpImageConvert::convert(I, cvI);

      //! [Re-init tracker]
      // Restart the initialization to detect new keypoints
      if (reader.getFrameIndex() == 25) {
        std::cout << "Re initialize the tracker" << std::endl;
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
        // Save of previous features
        std::vector<cv::Point2f> prev_features = tracker.getFeatures();

        // Start a new feature detection
        tracker.initTracking(cvI);
        std::vector<cv::Point2f> new_features = tracker.getFeatures();

        // Add previous features if they are not to close to detected one
        double distance, minDistance_ = tracker.getMinDistance();
        bool is_redundant;
        for (size_t i=0; i < prev_features.size(); i++) {
          // Test if a previous feature is not redundant with one of the newly detected
          is_redundant = false;
          for (size_t j=0; j < new_features.size(); j++){
            distance = sqrt(vpMath::sqr(new_features[j].x-prev_features[i].x) + vpMath::sqr(new_features[j].y-prev_features[i].y));
            if(distance < minDistance_){
              is_redundant = true;
              break;
            }
          }
          if(is_redundant){
            continue;
          }
          //std::cout << "Add previous feature with index " << i << std::endl;
          tracker.addFeature(prev_features[i]);
        }
#else
        // Save of previous features
        int prev_nfeatures = tracker.getNbFeatures();
        float x,y;
        int id, j=0;

        CvPoint2D32f *prev_features = (CvPoint2D32f*)cvAlloc(prev_nfeatures*sizeof(CvPoint2D32f));

        for (int i=0; i <prev_nfeatures ; i ++) {
          tracker.getFeature(i, id, x, y);
          prev_features[i].x=x;
          prev_features[i].y=y;
          //printf("prev feature %d: id %d coord: %g %g\n", i, id, x, y);
        }

        // Start a new feature detection
        tracker.initTracking(cvI);
        std::cout << "Detection of " << tracker.getNbFeatures() << " new features" << std::endl;

        // Add previous features if they are not to close to detected one
        double distance, minDistance_ = tracker.getMinDistance();
        bool is_redundant;
        for(int i = tracker.getNbFeatures() ;
            j<prev_nfeatures && i<tracker.getMaxFeatures() ;
            j++){
          // Test if a previous feature is not redundant with new the one that are newly detected
          is_redundant = false;
          for(int k=0; k<tracker.getNbFeatures(); k++){
            tracker.getFeature(k,id,x,y);
            //printf("curr feature %d: id %d coord: %g %g\n", k, id, x, y);
            distance = sqrt(vpMath::sqr(x-prev_features[j].x) + vpMath::sqr(y-prev_features[j].y));
            if(distance < minDistance_){
              is_redundant = true;
              break;
            }
          }
          if(is_redundant){
            continue;
          }
          //std::cout << "Add previous feature with index " << i << std::endl;
          tracker.addFeature(i, prev_features[j].x, prev_features[j].y);
          i++;
        }
        cvFree(&prev_features);
#endif
      }
      // Track the features
      tracker.track(cvI);
      //! [Re-init tracker]

      std::cout << "tracking of " << tracker.getNbFeatures() << " features" << std::endl;

      tracker.display(I, vpColor::red);
      vpDisplay::flush(I);
    }

    vpDisplay::getClick(I);

#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    cvReleaseImage(&cvI);
#endif

    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
