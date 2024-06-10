#include <memory>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageDraw.h>

#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(HAVE_OPENCV_HIGHGUI)) \
  && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
namespace
{
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

template<typename T, typename... Args>
std::unique_ptr<T> make_unique_compat(Args&&... args)
{
#if ((__cplusplus >= 201402L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201402L)))
  return std::make_unique<T>(args...);
#else
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
#endif
}
}
#endif

int main(int argc, char *argv[])
{
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(HAVE_OPENCV_HIGHGUI)) \
  && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_MINIZ)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  bool opencv_backend = false;
  std::string npz_filename = "npz_tracking_teabox.npz";
  bool print_cMo = false;
  bool dump_infos = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--cv-backend") {
      opencv_backend = true;
    }
    else if ((std::string(argv[i]) == "--read" || std::string(argv[i]) == "-i") && i+1 < argc) {
      npz_filename = argv[i+1];
    }
    else if (std::string(argv[i]) == "--print-cMo" && i+1 < argc) {
      print_cMo = true;
    }
    else if (std::string(argv[i]) == "--dump" && i+1 < argc) {
      dump_infos = true;
    }
    else {
      std::cout << "Options:" << std::endl;
      std::cout << "  --cv-backend   use OpenCV if available for in-memory PNG decoding" << std::endl;
      std::cout << "  --read / -i    input filename" << std::endl;
      std::cout << "  --print-cMo    print cMo" << std::endl;
      std::cout << "  --dump         print all the data name in the file" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  std::cout << "Read file: " << npz_filename << std::endl;
  std::cout << "OpenCV backend? " << opencv_backend << std::endl;

  const vpImageIo::vpImageIoBackendType backend =
    opencv_backend ? vpImageIo::IO_OPENCV_BACKEND : vpImageIo::IO_STB_IMAGE_BACKEND;

  visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
  if (dump_infos) {
    std::cout << npz_filename << " file contains the following data:" << std::endl;
    for (visp::cnpy::npz_t::const_iterator it = npz_data.begin(); it != npz_data.end(); ++it) {
      std::cout << "  " << it->first << std::endl;
    }
  }

  visp::cnpy::NpyArray arr_height = npz_data["height"];
  visp::cnpy::NpyArray arr_width = npz_data["width"];
  visp::cnpy::NpyArray arr_channel = npz_data["channel"];
  int height = *arr_height.data<int>();
  int width = *arr_width.data<int>();
  int channel = *arr_channel.data<int>();
  std::cout << "height: " << height << std::endl;
  std::cout << "width: " << width << std::endl;
  std::cout << "channel: " << channel << std::endl;
  std::cout << "Color mode? " << (channel > 1) << std::endl;

  visp::cnpy::NpyArray arr_camera_name = npz_data["camera_name"];
  // For null-terminated character handling, see:
  // https://stackoverflow.com/a/8247804
  // https://stackoverflow.com/a/45491652
  std::vector<char> vec_arr_camera_name = arr_camera_name.as_vec<char>();
  const std::string camera_name = std::string(vec_arr_camera_name.begin(), vec_arr_camera_name.end());
  std::cout << "Camera name: " << camera_name << std::endl;

  visp::cnpy::NpyArray arr_px = npz_data["cam_px"];
  visp::cnpy::NpyArray arr_py = npz_data["cam_py"];
  visp::cnpy::NpyArray arr_u0 = npz_data["cam_u0"];
  visp::cnpy::NpyArray arr_v0 = npz_data["cam_v0"];
  vpCameraParameters cam(*arr_px.data<double>(), *arr_py.data<double>(), *arr_u0.data<double>(), *arr_v0.data<double>());
  std::cout << "Cam: " << cam << std::endl;

  vpImage<unsigned char> I(height, width);
  vpImage<vpRGBa> I_display(height, width);

  std::unique_ptr<vpDisplay> display;
#if defined(VISP_HAVE_X11)
  display = make_unique_compat<vpDisplayX>(I_display, 100, 100, "Model-based tracker");
#elif defined(VISP_HAVE_GDI)
  display = make_unique_compat<vpDisplayGDI>(I_display, 100, 100, "Model-based tracker");
#elif defined(HAVE_OPENCV_HIGHGUI)
  display = make_unique_compat<vpDisplayOpenCV>(I_display, 100, 100, "Model-based tracker");
#endif

  visp::cnpy::NpyArray arr_nb_data = npz_data["nb_data"];
  int nb_data = *arr_nb_data.data<int>();
  std::cout << "Number of images: " << nb_data << std::endl;

  // Load all the images data
  visp::cnpy::NpyArray arr_vec_img_data_size = npz_data["vec_img_data_size"];
  int *vec_img_data_size_ptr = arr_vec_img_data_size.data<int>();
  visp::cnpy::NpyArray arr_vec_img = npz_data["vec_img"];
  unsigned char *vec_img_ptr = arr_vec_img.data<unsigned char>();
  std::vector<unsigned char> vec_img;
  size_t img_data_offset = 0;

  // Load all the poses
  visp::cnpy::NpyArray arr_vec_poses = npz_data["vec_poses"];
  double *vec_poses_ptr = arr_vec_poses.data<double>();
  assert(arr_vec_poses.shape.size() == 2);
  assert(arr_vec_poses.shape[1] == 6);
  size_t pose_size = arr_vec_poses.shape[1];

  std::vector<double> times;

  for (int iter = 0; iter < nb_data; iter++) {
    // std::copy(vec_img_ptr + img_data_offset, vec_img_ptr + img_data_offset + vec_img_data_size_ptr[iter],
    //     std::back_inserter(vec_img));
    vec_img = std::vector<unsigned char>(vec_img_ptr + img_data_offset, vec_img_ptr + img_data_offset + vec_img_data_size_ptr[iter]);
    double start = vpTime::measureTimeMs(), end = -1;
    if (channel > 1) {
      vpImageIo::readPNGfromMem(vec_img, I_display, backend);
      end = vpTime::measureTimeMs();
    }
    else {
      vpImageIo::readPNGfromMem(vec_img, I, backend);
      end = vpTime::measureTimeMs();
      vpImageConvert::convert(I, I_display);
    }
    times.push_back(end-start);
    img_data_offset += vec_img_data_size_ptr[iter];

    const std::string str_model_iter_sz = toString("model_%06d", iter) + "_sz";
    visp::cnpy::NpyArray arr_model_iter_sz = npz_data[str_model_iter_sz];
    size_t model_sz = *arr_model_iter_sz.data<size_t>();

    for (size_t i = 0; i < model_sz; i++) {
      char buffer[100];
      int res = snprintf(buffer, 100, "model_%06d_%06zu", iter, i);
      if (res > 0 && res < 100) {
        std::string str_model_iter_data = buffer;
        visp::cnpy::NpyArray arr_model_iter_data = npz_data[str_model_iter_data];

        if (arr_model_iter_data.shape[0] >= 5) {
          if (std::fabs(arr_model_iter_data.data<double>()[0]) <= std::numeric_limits<double>::epsilon()) {  // line feature
            vpImageDraw::drawLine(I_display,
              vpImagePoint(arr_model_iter_data.data<double>()[1], arr_model_iter_data.data<double>()[2]),
              vpImagePoint(arr_model_iter_data.data<double>()[3], arr_model_iter_data.data<double>()[4]), vpColor::red, 3);
          }
        }
      }
    }

    vpHomogeneousMatrix cMo(vpTranslationVector(vec_poses_ptr[pose_size*iter], vec_poses_ptr[pose_size*iter + 1], vec_poses_ptr[pose_size*iter + 2]),
      vpThetaUVector(vec_poses_ptr[pose_size*iter + 3], vec_poses_ptr[pose_size*iter + 4], vec_poses_ptr[pose_size*iter + 5])
    );

    if (print_cMo) {
      std::cout << "\ncMo:\n" << cMo << std::endl;
    }

    vpDisplay::display(I_display);
    vpDisplay::displayFrame(I_display, cMo, cam, 0.025, vpColor::none, 3);
    vpDisplay::flush(I_display);

    vpTime::wait(30);
  }

  std::cout << "Mean time for image decoding: " << vpMath::getMean(times) << " ms ; Median time: "
    << vpMath::getMedian(times) << " ms ; Std: " << vpMath::getStdev(times) << " ms" << std::endl;

  vpDisplay::getClick(I_display, true);

#else
  (void)argc;
  (void)argv;
  std::cerr << "Error, a missing display library is needed (X11, GDI or OpenCV built with HighGUI module)." << std::endl;
#ifndef VISP_HAVE_MINIZ
  std::cerr << "You also need to enable npz I/O functions" << std::endl;
#endif
#endif

  return EXIT_SUCCESS;
}
