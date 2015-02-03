//! \example tutorial-matching-surf-deprecated.cpp
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpVideoReader.h>
//! [Include]
#include <visp/vpKeyPointSurf.h>
//! [Include]

int main()
{  
  //! [Define]
#if defined(VISP_HAVE_OPENCV_NONFREE) && (VISP_HAVE_OPENCV_VERSION < 0x030000)
  //! [Define]
  vpImage<unsigned char> I;

  vpVideoReader reader;
  reader.setFileName("video-postcard.mpeg");
  reader.acquire(I);

  //! [Construction]
  vpKeyPointSurf surf;
  surf.buildReference(I);
  //! [Construction]

  //! [Create image]
  vpImage<unsigned char> Idisp;
  Idisp.resize(I.getHeight(), 2*I.getWidth());
  Idisp.insert(I, vpImagePoint(0, 0));
  Idisp.insert(I, vpImagePoint(0, I.getWidth()));
  //! [Create image]
  //! [Init display]
  vpDisplayOpenCV d(Idisp, 0, 0, "Matching surf keypoints") ;
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
    int nbMatch = surf.matchPoint(I);
    //! [Matching]

    //! [Get matches]
    vpImagePoint iPref, iPcur;
    for (int i = 0; i < nbMatch; i++)
    {
      surf.getMatchedPoints(i, iPref, iPcur);
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
