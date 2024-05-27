//! \example tutorial-pose-from-planar-object.cpp

// Core
#include <visp3/core/vpColorDepthConversion.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpXmlParserCamera.h>

// Vision
#include <visp3/vision/vpPlaneEstimation.h>
#include <visp3/vision/vpPose.h>

// IO
#include <visp3/io/vpImageIo.h>

// GUI
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

// Check if std:c++17 or higher
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L))) && defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_PUGIXML)

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

// Local helper
namespace
{
// Display
#if defined(VISP_HAVE_X11)
using Display = vpDisplayX;
#elif defined(VISP_HAVE_GDI)
using Display = vpDisplayGDI;
#elif defined(VISP_HAVE_OPENCV)
using Display = vpDisplayOpenCV;
#elif defined(VISP_HAVE_GTK)
using Display = vpDisplayGTK;
#elif defined(VISP_HAVE_D3D9)
using Display = vpDisplayD3D;
#endif

constexpr auto DispScaleType { vpDisplay::SCALE_AUTO };

// Model
constexpr auto ModelCommentHeader { "#" };
constexpr auto ModelKeypointsHeader { "Keypoints" };
constexpr auto ModelBoundsHeader { "Bounds" };
constexpr auto ModelDataHeader { "data:" };

// Depth
constexpr auto DepthScale { 0.001 };
} // namespace

#ifndef DOXYGEN_SHOULD_SKIP_THIS
class Model
{
public:
  Model() = delete;
  ~Model() = default;
  Model(const Model &) = default;
  Model(Model &&) = default;
  Model &operator=(const Model &) = default;
  Model &operator=(Model &&) = default;

  explicit Model(const std::string &model_filename);

public:
  using Id = unsigned long int;
  static inline std::string to_string(const Id &id) { return std::to_string(id); };

  std::map<Id, vpPoint> keypoints(const vpHomogeneousMatrix &cMo = {}) const;
  std::map<Id, vpPoint> bounds(const vpHomogeneousMatrix &cMo = {}) const;

private:
  std::map<Id, vpPoint> m_keypoints {};
  std::map<Id, vpPoint> m_bounds {};
};

inline Model::Model(const std::string &model_filename)
{
  std::fstream file;
  file.open(model_filename.c_str(), std::fstream::in);

  std::string line {}, subs {};
  bool in_model_bounds { false };
  bool in_model_keypoints { false };
  unsigned int data_curr_line { 0 };
  unsigned int data_line_start_pos { 0 };

  auto reset = [&]() {
    in_model_bounds = false;
    in_model_keypoints = false;
    data_curr_line = 0;
    data_line_start_pos = 0;
    };

  while (getline(file, line)) {
    if (line.substr(0, std::string(ModelCommentHeader).size()) == ModelCommentHeader || line == ModelDataHeader) {
      continue;
    }
    else if (line == ModelBoundsHeader) {
      reset();
      in_model_bounds = true;
      continue;
    }
    else if (line == ModelKeypointsHeader) {
      reset();
      in_model_keypoints = true;
      continue;
    }

    if (data_curr_line == 0) {
      // Get indentation level which is common to all lines
      data_line_start_pos = (unsigned int)line.find("[") + 1;
    }

    try {
      std::stringstream ss(line.substr(data_line_start_pos, line.find("]") - data_line_start_pos));
      unsigned int data_on_curr_line = 0;
      vpColVector oXYZ({ 0, 0, 0, 1 });
      while (getline(ss, subs, ',')) {
        oXYZ[data_on_curr_line++] = std::atof(subs.c_str());
      }
      if (in_model_bounds) {
        m_bounds.try_emplace(data_curr_line, oXYZ[0], oXYZ[1], oXYZ[2]);
      }
      else if (in_model_keypoints) {
        m_keypoints.try_emplace(data_curr_line, oXYZ[0], oXYZ[1], oXYZ[2]);
      }
      data_curr_line++;
    }
    catch (...) {
   // Line is empty or incomplete. We skeep it
    }
  }

  file.close();
}

inline std::map<Model::Id, vpPoint> Model::bounds(const vpHomogeneousMatrix &cMo) const
{
  auto bounds = m_bounds;
  std::for_each(begin(bounds), end(bounds), [&cMo](auto &bound) { bound.second.project(cMo); });

  return bounds;
}

inline std::map<Model::Id, vpPoint> Model::keypoints(const vpHomogeneousMatrix &cMo) const
{
  auto keypoints = m_keypoints;
  std::for_each(begin(keypoints), end(keypoints), [&cMo](auto &keypoint) { keypoint.second.project(cMo); });

  return keypoints;
}

std::ostream &operator<<(std::ostream &os, const Model &model)
{
  os << "-Bounds:" << std::endl;
  for (const auto &[id, bound] : model.bounds()) {
    // clang-format off
    os << std::setw(4) << std::setfill(' ') << id << ": "
      << std::setw(6) << std::setfill(' ') << bound.get_X() << ", "
      << std::setw(6) << std::setfill(' ') << bound.get_Y() << ", "
      << std::setw(6) << std::setfill(' ') << bound.get_Z() << std::endl;
   // clang-format on
  }

  os << "-Keypoints:" << std::endl;
  for (const auto &[id, keypoint] : model.keypoints()) {
    // clang-format off
    os << std::setw(4) << std::setfill(' ') << id << ": "
      << std::setw(6) << std::setfill(' ') << keypoint.get_X() << ", "
      << std::setw(6) << std::setfill(' ') << keypoint.get_Y() << ", "
      << std::setw(6) << std::setfill(' ') << keypoint.get_Z() << std::endl;
   // clang-format on
  }

  return os;
}

std::tuple<vpImage<vpRGBa>, vpImage<uint16_t>, vpCameraParameters, vpCameraParameters, vpHomogeneousMatrix>
readData(const std::string &input_directory, const unsigned int cpt = 0)
{
  char buffer[FILENAME_MAX];
  std::stringstream ss;
  ss << input_directory << "/color_image_%04d.jpg";
  snprintf(buffer, FILENAME_MAX, ss.str().c_str(), cpt);
  const std::string filename_color = buffer;

  ss.str("");
  ss << input_directory << "/depth_image_%04d.bin";
  snprintf(buffer, FILENAME_MAX, ss.str().c_str(), cpt);
  const std::string filename_depth = buffer;

  // Read color
  vpImage<vpRGBa> I_color {};
  vpImageIo::read(I_color, filename_color);

  // Read raw depth
  vpImage<uint16_t> I_depth_raw {};
  std::ifstream file_depth(filename_depth.c_str(), std::ios::in | std::ios::binary);
  if (file_depth.is_open()) {
    unsigned int height = 0, width = 0;
    vpIoTools::readBinaryValueLE(file_depth, height);
    vpIoTools::readBinaryValueLE(file_depth, width);
    I_depth_raw.resize(height, width);

    for (auto i = 0u; i < height; i++) {
      for (auto j = 0u; j < width; j++) {
        vpIoTools::readBinaryValueLE(file_depth, I_depth_raw[i][j]);
      }
    }
  }

  // Read camera parameters (intrinsics)
  ss.str("");
  ss << input_directory << "/camera.xml";

  vpXmlParserCamera parser {};
  vpCameraParameters color_param {}, depth_param {};
  parser.parse(color_param, ss.str(), "color_camera", vpCameraParameters::perspectiveProjWithDistortion);
  parser.parse(depth_param, ss.str(), "depth_camera", vpCameraParameters::perspectiveProjWithDistortion);

  // Read camera parameters (extrinsics)
  ss.str("");
  ss << input_directory << "/depth_M_color.txt";
  std::ifstream file_depth_M_color(ss.str().c_str(), std::ios::in | std::ios::binary);

  vpHomogeneousMatrix depth_M_color {};
  depth_M_color.load(file_depth_M_color);

  return { I_color, I_depth_raw, color_param, depth_param, depth_M_color };
}

std::vector<vpImagePoint> getRoiFromUser(vpImage<vpRGBa> color_img)
{
  // Init displays
  Display disp_color(color_img, 0, 0, "Roi bounds", DispScaleType);
  disp_color.display(color_img);
  disp_color.flush(color_img);

  std::vector<vpImagePoint> v_ip {};
  do {
    // Prepare display
    disp_color.display(color_img);
    auto disp_lane { 0 };

    vpDisplay::displayText(color_img, 15 * ++disp_lane, 15, "Select point along the d435 box boundary", vpColor::green);
    vpDisplay::displayText(color_img, 15 * ++disp_lane, 15, "Left click to select a point", vpColor::green);
    vpDisplay::displayText(color_img, 15 * ++disp_lane, 15, "Middle click to remove the last point", vpColor::green);
    vpDisplay::displayText(color_img, 15 * ++disp_lane, 15, "Right click to finish/quit", vpColor::green);

    // Display already selected points
    for (auto j = 0u; j < v_ip.size(); j++) {
      vpDisplay::displayCross(color_img, v_ip.at(j), 15, vpColor::green);
      vpDisplay::displayText(color_img, v_ip.at(j) + vpImagePoint(10, 10), std::to_string(j), vpColor::green);
    }
    disp_color.flush(color_img);

    // Wait for new point
    vpImagePoint ip {};
    vpMouseButton::vpMouseButtonType button {};
    vpDisplay::getClick(color_img, ip, button, true);

    switch (button) {
    case vpMouseButton::button2: {
      if (v_ip.size() > 0) {
        v_ip.erase(std::prev(end(v_ip)));
      }
      break;
    }

    case vpMouseButton::button3: {
      return v_ip;
    }

    default: {
      v_ip.push_back(ip);
      break;
    }
    }

  } while (1);
}

std::map<Model::Id, vpImagePoint> getKeypointsFromUser(vpImage<vpRGBa> color_img, const Model &model,
                                                       const std::string &parent_data)
{
  // Init displays
  Display disp_color(color_img, 0, 0, "Keypoints", DispScaleType);
  disp_color.display(color_img);
  disp_color.flush(color_img);

  vpImage<vpRGBa> I_help {};
  vpImageIo::read(I_help, parent_data + "/data/d435_box_keypoints_user_helper.png");
  Display disp_help(I_help, disp_color.getWindowXPosition() + color_img.getWidth(), disp_color.getWindowYPosition(),
                    "Keypoints [help]", DispScaleType);
  disp_help.display(I_help);
  disp_help.flush(I_help);

  std::map<Model::Id, vpImagePoint> keypoints {};
  // - The next line produces an internal compiler error with Visual Studio 2017:
  //   tutorial-pose-from-planar-object.cpp(304): fatal error C1001: An internal error has occurred in the compiler.
  //   [C:\projects\visp\build\tutorial\computer-vision\tutorial-pose-from-planar-object.vcxproj] (compiler file
  //   'd:\agent\_work\8\s\src\vctools\compiler\cxxfe\sl\p1\cxx\grammar.y', line 12721)
  //    To work around this problem, try simplifying or changing the program near the locations listed above.
  //   Please choose the Technical Support command on the Visual C++
  //    Help menu, or open the Technical Support help file for more information
  // - Note that the next line builds with Visual Studio 2022.
  // for ([[maybe_unused]] const auto &[id, _] : model.keypoints()) {
  for (const auto &[id, ip_unused] : model.keypoints()) {
    (void)ip_unused;
    // Prepare display
    disp_color.display(color_img);
    auto disp_lane { 0 };

    vpDisplay::displayText(color_img, 15 * ++disp_lane, 15, "Select the keypoints " + Model::to_string(id),
                           vpColor::green);
    vpDisplay::displayText(color_img, 15 * ++disp_lane, 15, "Click to select a point", vpColor::green);

    // Display already selected points
    for (const auto &[id, keypoint] : keypoints) {
      vpDisplay::displayCross(color_img, keypoint, 15, vpColor::green);
      vpDisplay::displayText(color_img, keypoint + vpImagePoint(10, 10), Model::to_string(id), vpColor::green);
    }
    disp_color.flush(color_img);

    // Wait for new point
    vpImagePoint ip {};
    vpDisplay::getClick(color_img, ip, true);
    keypoints.try_emplace(id, ip);
  }

  return keypoints;
}
#endif // DOXYGEN_SHOULD_SKIP_THIS
#endif

int main(int, char *argv[])
{
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

#if defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_PUGIXML)

  // Get prior data
  //! [Prior_Data]
  auto [color_img, depth_raw, color_param, depth_param, depth_M_color] =
    readData(vpIoTools::getParent(argv[0]) + "/data/d435_not_align_depth", 0);
  const auto model = Model(vpIoTools::getParent(argv[0]) + "/data/d435_box.model");
  //! [Prior_Data]

  std::cout << "color_param:" << std::endl << color_param << std::endl;
  std::cout << "depth_param:" << std::endl << depth_param << std::endl;
  std::cout << "depth_M_color:" << std::endl << depth_M_color << std::endl;
  std::cout << std::endl << "Model:" << std::endl << model << std::endl;

  // Init display
  Display display_color(color_img, 0, 0, "Color", DispScaleType);
  display_color.display(color_img);
  display_color.flush(color_img);

  vpImage<unsigned char> depth_img {};
  vpImageConvert::createDepthHistogram(depth_raw, depth_img);
  Display display_depth(depth_img, display_color.getWindowXPosition() + display_color.getWidth(), 0, "Depth",
                        DispScaleType);
  display_depth.display(depth_img);
  display_depth.flush(depth_img);

  // Ask roi for plane estimation
  //! [Roi_Plane_Estimation]
  vpPolygon roi_color_img {};
  roi_color_img.build(getRoiFromUser(color_img), true);

  std::vector<vpImagePoint> roi_corners_depth_img {};
  std::transform(
      cbegin(roi_color_img.getCorners()), cend(roi_color_img.getCorners()), std::back_inserter(roi_corners_depth_img),
      std::bind((vpImagePoint(*)(const vpImage<uint16_t> &, double, double, double, const vpCameraParameters &,
                                 const vpCameraParameters &, const vpHomogeneousMatrix &, const vpHomogeneousMatrix &,
                                 const vpImagePoint &)) &
                vpColorDepthConversion::projectColorToDepth,
                depth_raw, DepthScale, 0.1, 0.6, depth_param, color_param, depth_M_color.inverse(), depth_M_color,
                std::placeholders::_1));
  const vpPolygon roi_depth_img { roi_corners_depth_img };
  //! [Roi_Plane_Estimation]

  vpDisplay::displayPolygon(depth_img, roi_depth_img.getCorners(), vpColor::green);
  display_depth.flush(depth_img);

  // Estimate the plane
  vpImage<vpRGBa> heat_map {};
  //! [Plane_Estimation]
  const auto obj_plane_in_depth =
    vpPlaneEstimation::estimatePlane(depth_raw, DepthScale, depth_param, roi_depth_img, 1000, heat_map);
  if (!obj_plane_in_depth) {
    return EXIT_FAILURE;
  }

  // Get the plane in color frame
  auto obj_plane_in_color = *obj_plane_in_depth;
  obj_plane_in_color.changeFrame(depth_M_color.inverse());
  //! [Plane_Estimation]

  Display display_heat_map(heat_map, display_depth.getWindowXPosition(),
                           display_depth.getWindowYPosition() + display_depth.getHeight(), "Plane Estimation Heat map",
                           DispScaleType);
  display_heat_map.display(heat_map);
  display_heat_map.flush(heat_map);

  // Ask user to click on keypoints
  const auto keypoint_color_img = getKeypointsFromUser(color_img, model, vpIoTools::getParent(argv[0]));

  //! [Pose_Estimation]
  const auto cMo = vpPose::computePlanarObjectPoseWithAtLeast3Points(obj_plane_in_color, model.keypoints(),
                                                                     keypoint_color_img, color_param);
  if (!cMo) {
    return EXIT_FAILURE;
  }
  //! [Pose_Estimation]

  // Display the model
  std::vector<vpImagePoint> d435_box_bound {};
  // - The next line produces an internal compiler error with Visual Studio 2017:
  //   tutorial-pose-from-planar-object.cpp(428): fatal error C1001: An internal error has occurred in the compiler.
  //   [C:\projects\visp\build\tutorial\computer-vision\tutorial-pose-from-planar-object.vcxproj] (compiler file
  //   'd:\agent\_work\8\s\src\vctools\compiler\cxxfe\sl\p1\cxx\grammar.y', line 12721)
  //    To work around this problem, try simplifying or changing the program near the locations listed above.
  //   Please choose the Technical Support command on the Visual C++
  //    Help menu, or open the Technical Support help file for more information
  // - Note that the next line builds with Visual Studio 2022.
  //
  // for ([[maybe_unused]] const auto &[_, bound] : model.bounds(*cMo)) {
  for (const auto &[id_unused, bound] : model.bounds(*cMo)) {
    (void)id_unused;
    vpImagePoint ip {};
    vpMeterPixelConversion::convertPoint(color_param, bound.get_x(), bound.get_y(), ip);
    d435_box_bound.push_back(ip);
  }
  vpDisplay::displayPolygon(color_img, d435_box_bound, vpColor::blue);

  for (const auto &[id, keypoint] : model.keypoints(*cMo)) {
    vpImagePoint ip {};
    vpMeterPixelConversion::convertPoint(color_param, keypoint.get_x(), keypoint.get_y(), ip);
    vpDisplay::displayCross(color_img, ip, 15, vpColor::orange);
    vpDisplay::displayText(color_img, ip + vpImagePoint(10, 10), Model::to_string(id), vpColor::orange);
  }
  vpDisplay::flush(color_img);

  // Display the frame
  vpDisplay::displayFrame(color_img, *cMo, color_param, 0.05, vpColor::none, 3);

  // Wait before exiting
  auto disp_lane { 0 };
  vpDisplay::displayText(color_img, 15 * ++disp_lane, 15, "D435 box boundary [from model]", vpColor::blue);
  vpDisplay::displayText(color_img, 15 * ++disp_lane, 15, "Keypoints [from model]", vpColor::orange);
  vpDisplay::displayText(color_img, 15 * ++disp_lane, 15, "Click to quit", vpColor::green);
  vpDisplay::flush(color_img);

  vpDisplay::getClick(color_img);

#else
  (void)argv;
  std::cout << "There is no display and pugixml available to run this tutorial." << std::endl;
#endif // defined(VISP_HAVE_DISPLAY)
#else
  (void)argv;
  std::cout << "c++17 should be enabled to run this tutorial." << std::endl;
#endif

  return EXIT_SUCCESS;
}
