//! \example tutorial-matching-keypoint-SIFT.cpp
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpVideoReader.h>
#include <visp/vpImageIo.h>
//! [Include]
#include <visp/vpKeyPoint.h>
//! [Include]

int main() {
  //! [Define]
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101) && (defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D))
  //! [Define]
  vpImage<unsigned char> I;

  vpVideoReader reader;
  reader.setFileName("video-postcard.mpeg");
  reader.acquire(I);

  //! [Construction]
  const std::string detectorName = "SIFT";
  const std::string extractorName = "SIFT";
  //Use L2 distance with a matching done using FLANN (Fast Library for Approximate Nearest Neighbors)
  const std::string matcherName = "FlannBased";
  vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
  vpKeyPoint keypoint(detectorName, extractorName, matcherName, filterType);
  //! [Construction]

  //! [Build Reference]
  std::cout << "Reference keypoints=" << keypoint.buildReference(I) << std::endl;
  //! [Build Reference]

  //! [Create image]
  vpImage<unsigned char> Idisp;
  Idisp.resize(I.getHeight(), 2*I.getWidth());
  Idisp.insert(I, vpImagePoint(0, 0));
  Idisp.insert(I, vpImagePoint(0, I.getWidth()));
  //! [Create image]
  //! [Init display]
  vpDisplayOpenCV d(Idisp, 0, 0, "Matching keypoints with SIFT keypoints") ;
  vpDisplay::display(Idisp);
  vpDisplay::flush(Idisp);
  //! [Init display]

  while ( ! reader.end() )
  {
    //! [Acquisition]
    reader.acquire(I);
    Idisp.insert(I, vpImagePoint(0, I.getWidth()));
    //! [Acquisition]

    //! [Display]
    vpDisplay::display(Idisp);
    vpDisplay::displayLine(Idisp, vpImagePoint(0, I.getWidth()), vpImagePoint(I.getHeight(), I.getWidth()), vpColor::white, 2);
    //! [Display]

    //! [Matching]
    int nbMatch = keypoint.matchPoint(I);
    //! [Matching]

    std::cout << "Matches=" << nbMatch << std::endl;

    //! [Get matches]
    vpImagePoint iPref, iPcur;
    for (int i = 0; i < nbMatch; i++)
    {
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

  return 0;
}
