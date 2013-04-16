/*! \example tutorial-blob-auto-tracker.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpImageIo.h>

int main()
{
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
  bool learn = true;
  vpImage<unsigned char> I; // Create a gray level image container

  vpImageIo::read(I, "./target.pgm");

#if defined UNIX
  vpDisplayX d(I, 0, 0, "Camera view");
#else
  vpDisplayGDI d(I, 0, 0, "Camera view");
#endif
  vpDisplay::display(I);
  vpDisplay::flush(I);

  vpDot2 blob;
  if (learn) {
    // Learn the characteristics of the blob to auto detect
    blob.setGraphics(true);
    blob.setGraphicsThickness(1);
    blob.initTracking(I);
    blob.track(I);
    std::cout << "Blob characteristics: " << std::endl;
    std::cout << " width : " << blob.getWidth() << std::endl;
    std::cout << " height: " << blob.getHeight() << std::endl;
    std::cout << " area: " << blob.getArea() << std::endl;
    std::cout << " gray level min: " << blob.getGrayLevelMin() << std::endl;
    std::cout << " gray level max: " << blob.getGrayLevelMax() << std::endl;
    std::cout << " grayLevelPrecision: " << blob.getGrayLevelPrecision() << std::endl;
    std::cout << " sizePrecision: " << blob.getSizePrecision() << std::endl;
    std::cout << " ellipsoidShapePrecision: " << blob.getEllipsoidShapePrecision() << std::endl;
  }
  else {
    // Set blob characteristics for the auto detection
    blob.setWidth(50);
    blob.setHeight(50);
    blob.setArea(1700);
    blob.setGrayLevelMin(0);
    blob.setGrayLevelMax(30);
    blob.setGrayLevelPrecision(0.8);
    blob.setSizePrecision(0.65);
    blob.setEllipsoidShapePrecision(0.65);
  }

  std::list<vpDot2> blob_list;
  blob.searchDotsInArea(I, 0, 0, I.getWidth(), I.getHeight(), blob_list);

  if (learn) {
    // The blob that is tracked by initTracking() is not in the list of auto detected blobs
    // We add it:
    blob_list.push_back(blob);
  }
  std::cout << "Number of auto detected blob: " << blob_list.size() << std::endl;

  while(1) {
    vpDisplay::display(I);

    for(std::list<vpDot2>::iterator it=blob_list.begin(); it != blob_list.end(); ++it) {
      (*it).setGraphics(true);
      (*it).setGraphicsThickness(3);
      (*it).track(I);
    }

    vpDisplay::flush(I);

    if (vpDisplay::getClick(I, false)) {
      vpImage<vpRGBa> Ic;
      vpDisplay::getImage(I, Ic);
      vpImageIo::write(Ic, "output.png");
      break;
    }
    vpTime::wait(40);
  }
#endif
}
