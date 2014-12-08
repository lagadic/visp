/*! \example tutorial-barcode-detector-live.cpp */
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpImageConvert.h>
#include <visp/vpDetectorDataMatrixCode.h>
#include <visp/vpDetectorQRCode.h>


int main(int argc, const char** argv)
{
#if defined(VISP_HAVE_OPENCV) && (defined(VISP_HAVE_ZBAR) || defined(VISP_HAVE_DMTX))
  int opt_device = 0;
  int opt_barcode = 0; // 0=QRCode, 1=DataMatrix

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--device")
      opt_device = atoi(argv[i+1]);
    else if (std::string(argv[i]) == "--code-type")
      opt_barcode = atoi(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0]
                << " [--device <camera number>] [--code-type <0 for QRcode | 1 for DataMatrix>] [--help]"
                << std::endl;
      return 0;
    }
  }
  std::cout << "Use device: " << opt_device << std::endl;

  try {
    cv::VideoCapture cap(opt_device); // open the default camera
    if(!cap.isOpened()) { // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return -1;
    }
    cv::Mat frame;
    cap >> frame; // get a new frame from camera
    std::cout << "Image size: " << frame.rows << " " << frame.cols << std::endl;

    vpImage<unsigned char> I; // for gray images
    vpImageConvert::convert(frame, I);

    vpDisplayOpenCV d(I);

    vpDetectorBarCodeBase *detector;
#if (defined(VISP_HAVE_ZBAR) && defined(VISP_HAVE_DMTX))
    if (opt_barcode == 0)
      detector = new vpDetectorQRCode;
    else
      detector = new vpDetectorDataMatrixCode;
#elif defined(VISP_HAVE_ZBAR)
    detector = new vpDetectorQRCode;
#elif defined(VISP_HAVE_DMTX)
    detector = new vpDetectorDataMatrixCode;
#endif

    for(;;) {
      cap >> frame; // get a new frame from camera
      // Convert the image in ViSP format and display it
      vpImageConvert::convert(frame, I);
      vpDisplay::display(I);

      bool status = detector->detect(I);
      std::ostringstream legend;
      legend << detector->getNbObjects() << " bar code detected";
      vpDisplay::displayText(I, 10, 10, legend.str(), vpColor::red);

      if (status) {
        for(size_t i=0; i<detector->getNbObjects(); i++) {
          std::vector<vpImagePoint> p = detector->getPolygon(i);
          vpRect bbox = detector->getBBox(i);
          vpDisplay::displayRectangle(I, bbox, vpColor::green);
          vpDisplay::displayText(I, bbox.getTop()-20, bbox.getLeft(), "Message: \"" + detector->getMessage(i) + "\"", vpColor::red);
          for(size_t j=0; j < p.size(); j++) {
            vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
            std::ostringstream number;
            number << j;
            vpDisplay::displayText(I, p[j]+vpImagePoint(10,0), number.str(), vpColor::blue);
          }
        }
      }

      vpDisplay::displayText(I, I.getHeight()-25, 10, "Click to quit...", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) // a click to exit
        break;
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
