/*! \example tutorial-video-recorder.cpp */
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpTime.h>
#include <visp/vpVideoWriter.h>

/*!
 This example allows to record a video from a camera.
 It only requires that ViSP is build with OpenCV.

 Example: ./tutorial-video-recorder --device 0 --name myvideo.mpeg
 */
int main(int argc, char** argv)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  std::string videoname = "video-recorded.mpg";
  int device = 0;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--device")
      device = atoi(argv[i+1]);
    else if (std::string(argv[i]) == "--name")
      videoname = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--device <device number>] [--name <video name>] [--help]\n" << std::endl;
      return 0;
    }
  }

  std::cout << "Use device: " << device << std::endl;
  std::cout << "Record video in: " << videoname << std::endl;

  try {
    cv::VideoCapture cap(device); // open the default camera
    if(!cap.isOpened()) { // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return -1;
    }
    cv::Mat frame;
    cap >> frame; // get a new frame from camera
    std::cout << "Image size: " << frame.rows << " " << frame.cols << std::endl;

    //vpImage<vpRGBa> I; // for color images
    vpImage<unsigned char> I; // for gray images
    vpImageConvert::convert(frame, I);

    vpDisplayOpenCV d(I);

    vpVideoWriter writer;

#ifdef VISP_HAVE_FFMPEG
    // Set up the bit rate
    writer.setBitRate(1000000);
    // Set up the codec to use
#  if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54,51,110) // libavcodec 54.51.100
    writer.setCodec(CODEC_ID_MPEG2VIDEO);
#  else
    writer.setCodec(AV_CODEC_ID_MPEG2VIDEO);
#  endif
#elif VISP_HAVE_OPENCV_VERSION >= 0x030000
    writer.setCodec( cv::VideoWriter::fourcc('P','I','M','1') ); // MPEG-1 codec
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
    writer.setCodec( CV_FOURCC('P','I','M','1') );
#endif
    writer.setFileName(videoname);
    writer.open(I);
    bool recording = false;

    for(;;) {
      cap >> frame; // get a new frame from camera
      // Convert the image in ViSP format and display it
      vpImageConvert::convert(frame, I);
      vpDisplay::display(I);
      if (recording == false) {
        vpDisplay::displayText(I, 10, 10, "A click to start recording", vpColor::green);
        if (vpDisplay::getClick(I, false))
          recording = true;
      }
      else {
        writer.saveFrame(I);
        vpDisplay::displayText(I, 10, 10, "Recording: A click to stop and exit", vpColor::red);
        if (vpDisplay::getClick(I, false))
          break;
      }

      vpDisplay::flush(I);
    }
    std::cout << "The video was recorded in \"" << videoname << "\"" << std::endl;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
}
