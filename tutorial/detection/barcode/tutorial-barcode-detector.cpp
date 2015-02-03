//! \example tutorial-barcode-detector.cpp
//! [Include]
#include <visp/vpDetectorDataMatrixCode.h>
#include <visp/vpDetectorQRCode.h>
//! [Include]
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>

int main(int argc, const char** argv)
{
  //! [Macro defined]
#if (defined(VISP_HAVE_ZBAR) || defined(VISP_HAVE_DMTX)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  //! [Macro defined]
  try {
    vpImage<unsigned char> I;
    vpImageIo::read(I, "bar-code.pgm");

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#endif

    //! [Create base detector]
    vpDetectorBase *detector = NULL;
    //! [Create base detector]

#if (defined(VISP_HAVE_ZBAR) && defined(VISP_HAVE_DMTX))
    int opt_barcode = 0; // 0=QRCode, 1=DataMatrix

    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--code-type")
        opt_barcode = atoi(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0]
                  << " [--code-type <0 for QRcode | 1 for DataMatrix>] [--help]"
                  << std::endl;
        return 0;
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
    vpDisplay::displayText(I, (int)I.getHeight()-30, 10, legend.str(), vpColor::red);

    //! [Parse detected codes]
    if (status) {
      for(size_t i=0; i < detector->getNbObjects(); i++) {
        //! [Parse detected codes]
        //! [Get location]
        std::vector<vpImagePoint> p = detector->getPolygon(i);
        vpRect bbox = detector->getBBox(i);
        //! [Get location]
        vpDisplay::displayRectangle(I, bbox, vpColor::green);
        //! [Get message]
        vpDisplay::displayText(I, (int)(bbox.getTop()-10), (int)bbox.getLeft(),
                               "Message: \"" + detector->getMessage(i) + "\"",
                               vpColor::red);
        //! [Get message]
        for(size_t j=0; j < p.size(); j++) {
          vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
          std::ostringstream number;
          number << j;
          vpDisplay::displayText(I, p[j]+vpImagePoint(10,0), number.str(), vpColor::blue);
        }
      }

      vpDisplay::displayText(I, (int)I.getHeight()-15, 10, "A click to quit...", vpColor::red);
      vpDisplay::flush(I);
      vpDisplay::getClick(I);
    }
    delete detector;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#endif
}
