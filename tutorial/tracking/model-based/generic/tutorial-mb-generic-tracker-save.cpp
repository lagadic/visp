#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/io/vpVideoReader.h>


#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_MINIZ)
#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
std::vector<double> poseToVec(const vpHomogeneousMatrix &cMo)
{
  vpThetaUVector tu = cMo.getThetaUVector();
  vpTranslationVector t = cMo.getTranslationVector();

  std::vector<double> vec { t[0], t[1], t[2], tu[0], tu[1], tu[2] };

  return vec;
}

// https://en.cppreference.com/w/cpp/io/c/fprintf
std::string toString(const std::string &name, int val)
{
  auto fmt = name.c_str();
  int sz = std::snprintf(nullptr, 0, fmt, val);
  std::vector<char> buf(sz + 1);      // note +1 for null terminator
  std::sprintf(buf.data(), fmt, val);
  std::string str(buf.begin(), buf.end());

  return str;
}
}

int main(int argc, char **argv)
{
  bool opencv_backend = false;
  std::string npz_filename = "npz_tracking_teabox.npz";
  bool color_mode = false;
  bool save_alpha = false;
  bool print_cMo = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--cv-backend") {
      opencv_backend = true;
    }
    else if ((std::string(argv[i]) == "--save" || std::string(argv[i]) == "-o") && i+1 < argc) {
      npz_filename = argv[i+1];
    }
    else if (std::string(argv[i]) == "--color" || std::string(argv[i]) == "-c") {
      color_mode = true;
    }
    else if (std::string(argv[i]) == "--alpha" || std::string(argv[i]) == "-a") {
      save_alpha = true;
    }
    else if (std::string(argv[i]) == "--print-cMo" && i+1 < argc) {
      print_cMo = true;
    }
    else {
      std::cout << "Options:" << std::endl;
      std::cout << "  --cv-backend   use OpenCV if available for in-memory PNG encoding" << std::endl;
      std::cout << "  --save / -o    output filename" << std::endl;
      std::cout << "  --color        save RGB data" << std::endl;
      std::cout << "  --alpha        if --color opton, save RGBa data" << std::endl;
      std::cout << "  --print-cMo    print cMo" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  std::cout << "Save file: " << npz_filename << std::endl;
  std::cout << "OpenCV backend? " << opencv_backend << std::endl;
  std::cout << "Color image? " << color_mode << std::endl;
  std::cout << "Save alpha channel? " << save_alpha << std::endl;

  std::string opt_videoname = "model/teabox/teabox.mp4";
  std::string opt_modelname = "model/teabox/teabox.cao";

  std::string parentname = vpIoTools::getParent(opt_modelname);
  std::string objectname = vpIoTools::getNameWE(opt_modelname);

  if (!parentname.empty())
    objectname = parentname + "/" + objectname;

  std::cout << "Video name: " << opt_videoname << std::endl;

  vpImage<unsigned char> I;
  vpImage<vpRGBa> I_color;

  vpVideoReader g;
  g.setFileName(opt_videoname);
  g.open(I);

  vpMbGenericTracker tracker;
  tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER);

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(839, 839, 325, 243);
  tracker.setCameraParameters(cam);
  tracker.loadModel(objectname + ".cao");

  vpHomogeneousMatrix cMo;
  cMo[0][0] = 0.4889237963; cMo[0][1] = 0.8664706489; cMo[0][2] = 0.1009065709; cMo[0][3] = -0.07010159786;
  cMo[1][0] = 0.4218451176; cMo[1][1] = -0.1335995053; cMo[1][2] = -0.8967708007; cMo[1][3] = -0.08363026223;
  cMo[2][0] = -0.7635445096; cMo[2][1] = 0.4810195286; cMo[2][2] = -0.4308363901; cMo[2][3] = 0.4510066725;
  tracker.initFromPose(I, cMo);

  const int height = I.getRows();
  const int width = I.getCols();
  int channel = 1;
  if (color_mode) {
    channel = save_alpha ? 4 : 3;
  }

  const vpImageIo::vpImageIoBackendType backend =
    opencv_backend ? vpImageIo::IO_OPENCV_BACKEND : vpImageIo::IO_STB_IMAGE_BACKEND;

  // vector of byte data to store encoded image data using PNG format
  std::vector<unsigned char> img_buffer;

  const std::string camera_name = "Camera";
  std::vector<char> vec_camera_name(camera_name.begin(), camera_name.end());

  visp::cnpy::npz_save(npz_filename, "camera_name", &vec_camera_name[0], { vec_camera_name.size() }, "w"); // overwrite
  visp::cnpy::npz_save(npz_filename, "height", &height, { 1 }, "a"); // append
  visp::cnpy::npz_save(npz_filename, "width", &width, { 1 }, "a");
  visp::cnpy::npz_save(npz_filename, "channel", &channel, { 1 }, "a");

  const double cam_px = cam.get_px(), cam_py = cam.get_py(), cam_u0 = cam.get_u0(), cam_v0 = cam.get_v0();
  visp::cnpy::npz_save(npz_filename, "cam_px", &cam_px, { 1 }, "a");
  visp::cnpy::npz_save(npz_filename, "cam_py", &cam_py, { 1 }, "a");
  visp::cnpy::npz_save(npz_filename, "cam_u0", &cam_u0, { 1 }, "a");
  visp::cnpy::npz_save(npz_filename, "cam_v0", &cam_v0, { 1 }, "a");

  std::vector<double> vec_poses;
  vec_poses.reserve(g.getLastFrameIndex() * 6);

  std::vector<int> vec_img_data_size;
  vec_img_data_size.reserve(g.getLastFrameIndex());
  std::vector<unsigned char> vec_img_data;
  vec_img_data.reserve(g.getLastFrameIndex() * height * width);

  std::vector<double> times;
  size_t iter = 0;
  while (!g.end()) {
    g.acquire(I);
    tracker.track(I);
    tracker.getPose(cMo);
    if (print_cMo) {
      std::cout << "\ncMo:\n" << cMo << std::endl;
    }

    vpImageConvert::convert(I, I_color);
    double start = vpTime::measureTimeMs();
    if (color_mode) {
      vpImageIo::writePNGtoMem(I_color, img_buffer, backend, save_alpha);
    }
    else {
      vpImageIo::writePNGtoMem(I, img_buffer, backend);
    }
    double end = vpTime::measureTimeMs();
    times.push_back(end-start);
    vec_img_data_size.push_back(static_cast<int>(img_buffer.size()));
    vec_img_data.insert(vec_img_data.end(), img_buffer.begin(), img_buffer.end());

    std::vector<double> vec_pose = poseToVec(cMo);
    vec_poses.insert(vec_poses.end(), vec_pose.begin(), vec_pose.end());

    std::map<std::string, std::vector<std::vector<double> > > mapOfModels;
    std::map<std::string, unsigned int> mapOfW;
    mapOfW[camera_name] = I.getWidth();
    std::map<std::string, unsigned int> mapOfH;
    mapOfH[camera_name] = I.getHeight();
    std::map<std::string, vpHomogeneousMatrix> mapOfcMos;
    mapOfcMos[camera_name] = cMo;
    std::map<std::string, vpCameraParameters> mapOfCams;
    mapOfCams[camera_name] = cam;
    tracker.getModelForDisplay(mapOfModels, mapOfW, mapOfH, mapOfcMos, mapOfCams);

    std::vector<std::vector<double>> model = mapOfModels[camera_name];
    const std::string model_iter = toString("model_%06zu", static_cast<int>(iter));
    const std::string model_iter_sz = model_iter + "_sz";
    const size_t model_size = model.size();
    visp::cnpy::npz_save(npz_filename, model_iter_sz, &model_size, { 1 }, "a");

    for (size_t i = 0; i < model.size(); i++) {
      char buffer[100];
      int res = snprintf(buffer, 100, "model_%06zu_%06zu", iter, i);
      if (res > 0 && res < 100) {
        const std::string model_iter_data = buffer;
        std::vector<double> &vec_line = model[i];
        visp::cnpy::npz_save(npz_filename, model_iter_data, &vec_line[0], { vec_line.size() }, "a");
      }
    }

    iter++;
  }

  std::cout << "Mean time for image encoding: " << vpMath::getMean(times) << " ms ; Median time: "
    << vpMath::getMedian(times) << " ms ; Std: " << vpMath::getStdev(times) << " ms" << std::endl;

  visp::cnpy::npz_save(npz_filename, "vec_img_data_size", vec_img_data_size.data(), { vec_img_data_size.size() }, "a");
  visp::cnpy::npz_save(npz_filename, "vec_img", vec_img_data.data(), { vec_img_data.size() }, "a");
  // Show how to save a "multidimensional" array
  assert(iter == vec_poses.size()/6);
  visp::cnpy::npz_save(npz_filename, "vec_poses", vec_poses.data(), { static_cast<size_t>(iter), 6 }, "a");

  visp::cnpy::npz_save(npz_filename, "nb_data", &iter, { 1 }, "a");

  return EXIT_SUCCESS;
}

#else
#include <iostream>

int main()
{
  std::cout << "This tutorial needs c++11 flags" << std::endl;
#ifndef VISP_HAVE_MINIZ
  std::cerr << "You also need to enable npz I/O functions" << std::endl;
#endif
}
#endif
