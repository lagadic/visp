//! \example tutorial-hand-eye-calibration.cpp
#include <map>

#include <visp3/core/vpIoTools.h>
#include <visp3/vision/vpHandEyeCalibration.h>

void usage(const char *argv[], int error)
{
  std::cout << "Synopsis" << std::endl
            << "  " << argv[0] << " [--data-path <path>] [--help] [-h]" << std::endl
            << std::endl;
  std::cout << "Description" << std::endl
            << "  --data-path <path>  Path to the folder containing" << std::endl
            << "    pose_fPe_%d.yaml and pose_cPo_%d.yaml data files." << std::endl
            << "    Default: \"./\"" << std::endl
            << std::endl
            << "  --help, -h  Print this helper message." << std::endl
            << std::endl;
  if (error) {
    std::cout << "Error" << std::endl
              << "  "
              << "Unsupported parameter " << argv[error] << std::endl;
  }
}

int main(int argc, const char *argv[])
{
  std::string data_path = "./";
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--data-path" && i + 1 < argc) {
      data_path = std::string(argv[i + 1]);
      i++;
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0);
      return EXIT_SUCCESS;
    } else {
      usage(argv, i);
      return EXIT_FAILURE;
    }
  }

  std::vector<vpHomogeneousMatrix> cMo;
  std::vector<vpHomogeneousMatrix> wMe;
  vpHomogeneousMatrix eMc;

  std::map<long, std::string> map_fPe_files;
  std::map<long, std::string> map_cPo_files;
  std::vector<std::string> files = vpIoTools::getDirFiles(data_path);
  for (unsigned int i = 0; i < files.size(); i++) {
    long index_fPe = vpIoTools::getIndex(files[i], "pose_fPe_%d.yaml");
    long index_cPo = vpIoTools::getIndex(files[i], "pose_cPo_%d.yaml");
    if (index_fPe != -1) {
      map_fPe_files[index_fPe] = files[i];
    }
    if (index_cPo != -1) {
      map_cPo_files[index_cPo] = files[i];
    }
  }

  if (map_fPe_files.size() == 0) {
    std::cout << "No pose_fPe_%d.yaml files found. Use --data-path <path> option to modify data path." << std::endl;
    return EXIT_FAILURE;
  }
  if (map_cPo_files.size() == 0) {
    std::cout << "No pose_cPo_%d.yaml files found. Use --data-path <path> option to modify data path." << std::endl;
    return EXIT_FAILURE;
  }

  for (std::map<long, std::string>::const_iterator it_fPe = map_fPe_files.begin(); it_fPe != map_fPe_files.end();
       ++it_fPe) {
    std::string file_fPe = it_fPe->second;
    std::map<long, std::string>::const_iterator it_cPo = map_cPo_files.find(it_fPe->first);
    if (it_cPo != map_cPo_files.end()) {
      std::string file_cPo = it_cPo->second;
      vpPoseVector wPe;
      if (wPe.loadYAML(file_fPe, wPe) == false) {
        std::cout << "Unable to read data from " << data_path << "/" << file_fPe << ". Skip data" << std::endl;
        continue;
      }

      vpPoseVector cPo;
      if (cPo.loadYAML(file_cPo, cPo) == false) {
        std::cout << "Unable to read data from " << data_path << "/" << file_cPo << ". Skip data" << std::endl;
        continue;
      }
      std::cout << "Use data from " << data_path << "/" << file_fPe << " and from " << data_path << "/" << file_cPo
                << std::endl;
      wMe.push_back(vpHomogeneousMatrix(wPe));
      cMo.push_back(vpHomogeneousMatrix(cPo));
    }
  }

  if (wMe.size() < 3) {
    std::cout << "Not enough data pairs found." << std::endl;
    return EXIT_FAILURE;
  }

  int ret = vpHandEyeCalibration::calibrate(cMo, wMe, eMc);

  if (ret == 0) {
    std::cout << std::endl << "** Hand-eye calibration succeed" << std::endl;
    std::cout << std::endl << "** Hand-eye (eMc) transformation estimated:" << std::endl;
    std::cout << eMc << std::endl;
    std::cout << "** Corresponding pose vector: " << vpPoseVector(eMc).t() << std::endl;

    vpThetaUVector erc(eMc.getRotationMatrix());
    std::cout << std::endl << "** Translation [m]: " << eMc[0][3] << " " << eMc[1][3] << " " << eMc[2][3] << std::endl;
    std::cout << "** Rotation (theta-u representation) [rad]: " << erc.t() << std::endl;
    std::cout << "** Rotation (theta-u representation) [deg]: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1])
              << " " << vpMath::deg(erc[2]) << std::endl;
    vpQuaternionVector quaternion(eMc.getRotationMatrix());
    std::cout << "** Rotation (quaternion representation) [rad]: " << quaternion.t() << std::endl;

    // save eMc
    std::ofstream file_eMc("eMc.txt");
    eMc.save(file_eMc);

    vpPoseVector pose_vec(eMc);
    std::string output_filename("eMc.yaml");
    std::cout << std::endl << "Save transformation matrix eMc as a vpPoseVector in " << output_filename << std::endl;
    pose_vec.saveYAML(output_filename, pose_vec);
  } else {
    std::cout << std::endl << "** Hand-eye calibration failed" << std::endl;
    std::cout << std::endl
              << "Check your input data and ensure they are covering the half sphere over the chessboard." << std::endl;
    std::cout << std::endl
              << "See https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic.html" << std::endl;
  }

  return EXIT_SUCCESS;
}
