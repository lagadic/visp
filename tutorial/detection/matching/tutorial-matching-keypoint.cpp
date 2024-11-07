//! \example tutorial-matching-keypoint.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpVideoReader.h>
//! [Include]
#include <visp3/vision/vpKeyPoint.h>
//! [Include]

int main()
{
//! [Define]
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_FEATURES2D)
  //! [Define]
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  vpImage<unsigned char> I;

  vpVideoReader reader;
  reader.setFileName("video-postcard.mp4");
  reader.acquire(I);

  //! [Construction]
  const std::string detectorName = "ORB";
  const std::string extractorName = "ORB";
  // Hamming distance must be used with ORB
  const std::string matcherName = "BruteForce-Hamming";
  vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
  vpKeyPoint keypoint(detectorName, extractorName, matcherName, filterType);
  std::cout << "Reference keypoints=" << keypoint.buildReference(I) << std::endl;
  //! [Construction]

  //! [Create image]
  vpImage<unsigned char> Idisp;
  Idisp.resize(I.getHeight(), 2 * I.getWidth());
  Idisp.insert(I, vpImagePoint(0, 0));
  Idisp.insert(I, vpImagePoint(0, I.getWidth()));
  //! [Create image]
  //! [Init display]
  vpDisplayOpenCV d(Idisp, 0, 0, "Matching keypoints with ORB keypoints");
  vpDisplay::display(Idisp);
  vpDisplay::flush(Idisp);
  //! [Init display]

  while (!reader.end()) {
    //! [Acquisition]
    reader.acquire(I);
    Idisp.insert(I, vpImagePoint(0, I.getWidth()));
    //! [Acquisition]

    //! [Display]
    vpDisplay::display(Idisp);
    vpDisplay::displayLine(Idisp, vpImagePoint(0, I.getWidth()), vpImagePoint(I.getHeight(), I.getWidth()),
                           vpColor::white, 2);
    //! [Display]

    //! [Matching]
    unsigned int nbMatch = keypoint.matchPoint(I);
    //! [Matching]

    std::cout << "Matches=" << nbMatch << std::endl;

    //! [Get matches]
    vpImagePoint iPref, iPcur;
    for (unsigned int i = 0; i < nbMatch; i++) {
      keypoint.getMatchedPoints(i, iPref, iPcur);
      //! [Get matches]
      //! [Display matches]
      vpDisplay::displayLine(Idisp, iPref, iPcur + vpImagePoint(0, I.getWidth()), vpColor::green);
      //! [Display matches]
    }
    //! [Display flush]
    vpDisplay::flush(Idisp);
    //! [Display flush]

    if (vpDisplay::getClick(Idisp, false))
      break;
  }

  vpDisplay::getClick(Idisp);
#endif

  return EXIT_SUCCESS;
}
