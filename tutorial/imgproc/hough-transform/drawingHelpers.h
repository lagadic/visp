#ifndef DRAWING_HELPERS_H
#define DRAWING_HELPERS_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace drawingHelpers
{
#if defined(VISP_HAVE_X11)
extern VISP_NAMESPACE_ADDRESSING vpDisplayX d;
#elif defined(VISP_HAVE_OPENCV)
extern VISP_NAMESPACE_ADDRESSING vpDisplayOpenCV d;
#elif defined(VISP_HAVE_GTK)
extern VISP_NAMESPACE_ADDRESSING vpDisplayGTK d;
#elif defined(VISP_HAVE_GDI)
extern VISP_NAMESPACE_ADDRESSING vpDisplayGDI d;
#elif defined(VISP_HAVE_D3D9)
extern VISP_NAMESPACE_ADDRESSING vpDisplayD3D d;
#endif

extern VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> I_disp;

bool display(VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I, const std::string &title, const bool &blockingMode);
bool display(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, const std::string &title, const bool &blockingMode);
bool display(VISP_NAMESPACE_ADDRESSING vpImage<double> &D, const std::string &title, const bool &blockingMode);
}

#endif
#endif
