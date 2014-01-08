/*! \example tutorial-matching-surf.cpp */
#include <visp/vpKeyPointSurf.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpVideoReader.h>

int main()
{  
#if defined(VISP_HAVE_OPENCV_NONFREE) && defined(VISP_HAVE_FFMPEG)
  vpImage<unsigned char> I;

  vpVideoReader reader;
  reader.setFileName("video-postcard.mpeg");
  reader.acquire(I);

  vpKeyPointSurf surf;
  surf.buildReference(I);

  vpImage<unsigned char> Idisp;
  Idisp.resize(I.getHeight(), 2*I.getWidth());
  Idisp.insert(I, vpImagePoint(0, 0));
  Idisp.insert(I, vpImagePoint(0, I.getWidth()));

  vpDisplayOpenCV d(Idisp, 0, 0, "Matching surf keypoints") ;
  vpDisplay::display(Idisp);
  vpDisplay::flush(Idisp);

  while ( ! reader.end() )
  {
    reader.acquire(I);
    Idisp.insert(I, vpImagePoint(0, I.getWidth()));

    vpDisplay::display(Idisp);
    vpDisplay::displayLine(Idisp, vpImagePoint(0, I.getWidth()), vpImagePoint(I.getHeight(), I.getWidth()), vpColor::white, 2);

    int nbMatch = surf.matchPoint(I);

    vpImagePoint iPref, iPcur;
    for (int i = 0; i < nbMatch; i++)
    {
      surf.getMatchedPoints(i, iPref, iPcur);
      vpDisplay::displayLine(Idisp, iPref, iPcur + vpImagePoint(0, I.getWidth()), vpColor::green);
    }
    vpDisplay::flush(Idisp);

    if (vpDisplay::getClick(Idisp, false))
      break;
  }

  vpDisplay::getClick(Idisp);
#endif

  return 0;
}
