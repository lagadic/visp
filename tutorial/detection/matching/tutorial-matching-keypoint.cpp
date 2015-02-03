//! \example tutorial-matching-keypoint.cpp
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpVideoReader.h>
#include <visp/vpImageIo.h>
//! [Include]
#include <visp/vpKeyPoint.h>
//! [Include]

int main() {
  //! [Define]
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  //! [Define]
  vpImage<unsigned char> I;

  vpVideoReader reader;
  reader.setFileName("video-postcard.mpeg");
  reader.acquire(I);

  //! [Construction]
  const std::string detectorName = "ORB";
  const std::string extractorName = "ORB";
  //Hamming distance must be used with ORB
  const std::string matcherName = "BruteForce-Hamming";
  vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
  vpKeyPoint keypoint(detectorName, extractorName, matcherName, filterType);
  std::cout << "Reference keypoints=" << keypoint.buildReference(I) << std::endl;
  //! [Construction]

  //! [Create image]
  vpImage<unsigned char> Idisp;
  Idisp.resize(I.getHeight(), 2*I.getWidth());
  Idisp.insert(I, vpImagePoint(0, 0));
  Idisp.insert(I, vpImagePoint(0, I.getWidth()));
  //! [Create image]
  //! [Init display]
  vpDisplayOpenCV d(Idisp, 0, 0, "Matching keypoints with ORB keypoints") ;
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
    unsigned int nbMatch = keypoint.matchPoint(I);
    //! [Matching]

    std::cout << "Matches=" << nbMatch << std::endl;

    //! [Get matches]
    vpImagePoint iPref, iPcur;
    for (unsigned int i = 0; i < nbMatch; i++)
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
