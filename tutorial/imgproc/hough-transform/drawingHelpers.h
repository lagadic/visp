#ifndef _drawingHelpers_h_
#define _drawingHelpers_h_

#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

namespace drawingHelpers
{
  #if defined(VISP_HAVE_X11)
    extern vpDisplayX d;
  #elif defined(VISP_HAVE_OPENCV)
    extern vpDisplayOpenCV d;
  #elif defined(VISP_HAVE_GTK)
    extern vpDisplayGTK d;
  #elif defined(VISP_HAVE_GDI)
    extern vpDisplayGDI d;
  #elif defined(VISP_HAVE_D3D9)
    extern vpDisplayD3D d;
  #endif

  extern vpImage<vpRGBa> I_disp;

  bool display(vpImage<vpRGBa> &I, const std::string &title, const bool &blockingMode);
  bool display(vpImage<unsigned char> &I, const std::string &title, const bool &blockingMode);
  bool display(vpImage<double> &D, const std::string &title, const bool &blockingMode);
}

#endif