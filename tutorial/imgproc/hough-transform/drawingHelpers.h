#ifndef DRAWING_HELPERS_H
#define DRAWING_HELPERS_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace drawingHelpers
{
bool display(VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I, const std::string &title, const bool &blockingMode);
bool display(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &Idisp, const std::string &title, const bool &blockingMode);
bool display(VISP_NAMESPACE_ADDRESSING vpImage<double> &D, VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &Idisp, const std::string &title, const bool &blockingMode);
}

#endif
#endif
