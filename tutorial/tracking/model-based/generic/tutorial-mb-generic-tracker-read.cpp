#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageDraw.h>

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
}

int main(int argc, char **argv)
{
  const std::string npz_filename = "npz_tracking_teabox.npz";
  visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);

  visp::cnpy::NpyArray arr_height = npz_data["height"];
  visp::cnpy::NpyArray arr_width = npz_data["width"];
  visp::cnpy::NpyArray arr_channel = npz_data["channel"];
  int height = *arr_height.data<int>();
  int width = *arr_width.data<int>();
  int channel = *arr_channel.data<int>();
  std::cout << "height: " << height << std::endl;
  std::cout << "width: " << width << std::endl;
  std::cout << "channel: " << channel << std::endl;

  visp::cnpy::NpyArray arr_camera_name = npz_data["camera_name"];
  const std::string camera_name = std::string(arr_camera_name.data<char>());
  std::cout << "Camera name: " << camera_name << std::endl;

  visp::cnpy::NpyArray arr_px = npz_data["cam_px"];
  visp::cnpy::NpyArray arr_py = npz_data["cam_py"];
  visp::cnpy::NpyArray arr_u0 = npz_data["cam_u0"];
  visp::cnpy::NpyArray arr_v0 = npz_data["cam_v0"];
  vpCameraParameters cam(*arr_px.data<double>(), *arr_py.data<double>(), *arr_u0.data<double>(), *arr_v0.data<double>());
  std::cout << "Cam: " << cam << std::endl;

  vpImage<unsigned char> I(height, width);
  vpImage<vpRGBa> I_display;
  vpImageConvert::convert(I, I_display);
  vpDisplayX d(I_display, 100, 100, "Model-based tracker");

  visp::cnpy::NpyArray arr_nb_data = npz_data["nb_data"];
  int nb_data = *arr_nb_data.data<int>();
  std::cout << "Number of images: " << nb_data << std::endl;

  // Load all the poses
  visp::cnpy::NpyArray arr_vec_poses = npz_data["vec_poses"];
  double* vec_poses_ptr = arr_vec_poses.data<double>();
  assert(arr_vec_poses.shape.size() == 2);
  assert(arr_vec_poses.shape[1] == 6);
  size_t pose_size = arr_vec_poses.shape[1];

  for (int iter = 0; iter < nb_data; iter++) {
    std::string img_data_str = toString("png_image_%06d", iter);
     visp::cnpy::NpyArray arr_nb_data = npz_data[img_data_str];

    vpImageIo::readPNGfromMem(arr_nb_data.data<unsigned char>(), arr_nb_data.shape[0], I);
    vpImageConvert::convert(I, I_display);

    const std::string str_model_iter_sz = toString("model_%06d", iter) + "_sz";
    visp::cnpy::NpyArray arr_model_iter_sz = npz_data[str_model_iter_sz];
    size_t model_sz = *arr_model_iter_sz.data<size_t>();

    for (size_t i = 0; i < model_sz; i++) {
      char buffer[100];
      int res = snprintf(buffer, 100, "model_%06d_%06ld", iter, i);
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

    // const std::string vec_pose_str = toString("vec_pose_%06d", iter);
    // visp::cnpy::NpyArray arr_vec_pose = npz_data[vec_pose_str];
    // vpHomogeneousMatrix cMo(vpTranslationVector(arr_vec_pose.data<double>()[3], arr_vec_pose.data<double>()[4], arr_vec_pose.data<double>()[5]),
    //   vpThetaUVector(arr_vec_pose.data<double>()[0], arr_vec_pose.data<double>()[1], arr_vec_pose.data<double>()[2])
    // );
    vpHomogeneousMatrix cMo(vpTranslationVector(vec_poses_ptr[pose_size*iter + 3], vec_poses_ptr[pose_size*iter + 4], vec_poses_ptr[pose_size*iter + 5]),
      vpThetaUVector(vec_poses_ptr[pose_size*iter], vec_poses_ptr[pose_size*iter + 1], vec_poses_ptr[pose_size*iter + 2])
    );
    std::cout << "\ncMo:\n" << cMo << std::endl;

    vpDisplay::display(I_display);
    vpDisplay::displayFrame(I_display, cMo, cam, 0.025, vpColor::none, 3);
    vpDisplay::flush(I_display);

    vpTime::wait(30);
  }

  vpDisplay::getClick(I_display, true);

  return EXIT_SUCCESS;
}
