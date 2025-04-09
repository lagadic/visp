//! \example tutorial-barcode-detector.cpp
//! [Include]
#include <visp3/detection/vpDetectorDataMatrixCode.h>
#include <visp3/detection/vpDetectorQRCode.h>
//! [Include]
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>

int main(int argc, const char **argv)
{
//! [Macro defined]
#if (defined(VISP_HAVE_ZBAR) || defined(VISP_HAVE_DMTX)) && \
    defined(VISP_HAVE_DISPLAY)
  //! [Macro defined]
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display;
#else
  vpDisplay *display = nullptr;
#endif
  try {
    vpImage<unsigned char> I;
    vpImageIo::read(I, "bar-code.jpg");

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    display = vpDisplayFactory::createDisplay(I);
#else
    display = vpDisplayFactory::allocateDisplay(I);
#endif

    //! [Create base detector]
    vpDetectorBase *detector = nullptr;
    //! [Create base detector]

#if (defined(VISP_HAVE_ZBAR) && defined(VISP_HAVE_DMTX))
    int opt_barcode = 0; // 0=QRCode, 1=DataMatrix

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--code-type" && i + 1 < argc) {
        opt_barcode = atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "Usage: " << argv[0] << " [--code-type <0 for QR code | 1 for DataMatrix code>] [--help] [-h]"
          << std::endl;
        return EXIT_SUCCESS;
      }
    }
    //! [Create detector]
    if (opt_barcode == 0)
      detector = new vpDetectorQRCode;
    else
      detector = new vpDetectorDataMatrixCode;
//! [Create detector]
#elif defined(VISP_HAVE_ZBAR)
    detector = new vpDetectorQRCode;
    (void)argc;
    (void)argv;
#elif defined(VISP_HAVE_DMTX)
    detector = new vpDetectorDataMatrixCode;
    (void)argc;
    (void)argv;
#endif

    vpDisplay::display(I);

    //! [Detection]
    bool status = detector->detect(I);
    //! [Detection]
    std::ostringstream legend;
    legend << detector->getNbObjects() << " bar code detected";
    vpDisplay::displayText(I, static_cast<int>(I.getHeight()) - 30, 10, legend.str(), vpColor::red);

    //! [Parse detected codes]
    if (status) {
      for (size_t i = 0; i < detector->getNbObjects(); i++) {
        //! [Parse detected codes]
        //! [Get location]
        std::vector<vpImagePoint> p = detector->getPolygon(i);
        vpRect bbox = detector->getBBox(i);
        //! [Get location]
        vpDisplay::displayRectangle(I, bbox, vpColor::green);
        //! [Get message]
        vpDisplay::displayText(I, static_cast<int>(bbox.getTop() - 10), static_cast<int>(bbox.getLeft()),
                               "Message: \"" + detector->getMessage(i) + "\"", vpColor::red);
        //! [Get message]
        for (size_t j = 0; j < p.size(); j++) {
          vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
          std::ostringstream number;
          number << j;
          vpDisplay::displayText(I, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
        }
      }

      vpDisplay::displayText(I, static_cast<int>(I.getHeight()) - 15, 10, "A click to quit...", vpColor::red);
      vpDisplay::flush(I);
      vpDisplay::getClick(I);
    }
    delete detector;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
#endif
#else
  (void)argc;
  (void)argv;
#endif
}
