/*! \example tutorial-klt-tracker-with-reinit.cpp */
#include <visp/vpImageConvert.h>
#include <visp/vpKltOpencv.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpVideoReader.h>

int main()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x010100) && defined(VISP_HAVE_FFMPEG)
  try {
    vpVideoReader reader;
    reader.setFileName("video-postcard.mpeg");

    vpImage<unsigned char> I;
    reader.acquire(I);

    IplImage * cvI = NULL;
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

      // Restart the initialization to detect new keypoints
      if (reader.getFrameIndex() == 25) {
        std::cout << "Re initialize the tracker" << std::endl;

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
        float distance, minDistance_ = 2.f;// choose to keep old points distant of 2 pixels
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
      }
      // Track the features
      tracker.track(cvI);
      std::cout << "tracking of " << tracker.getNbFeatures() << " features" << std::endl;

      tracker.display(I, vpColor::red);
      vpDisplay::flush(I);
    }

    vpDisplay::getClick(I);

    cvReleaseImage(&cvI);

    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
