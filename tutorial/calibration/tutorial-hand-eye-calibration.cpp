//! \example tutorial-hand-eye-calibration.cpp
#include <visp3/vision/vpHandEyeCalibration.h>

int main(int argc, char *argv[])
{
  unsigned int ndata = 0;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ndata" && i+1 < argc) {
      ndata = atoi(argv[i+1]);
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0] << " [--ndata <number of data to process>] "
                              "[--help] [-h]" << std::endl;
      return EXIT_SUCCESS;
    }
  }
  if (ndata == 0) {
    std::cout << "Number of data to process not specified" << std::endl;
    std::cout << argv[0] << " --help" << std::endl;
    return EXIT_SUCCESS;
  }
  std::vector<vpHomogeneousMatrix> cMo(ndata);
  std::vector<vpHomogeneousMatrix> wMe(ndata);
  vpHomogeneousMatrix eMc;

  for (unsigned int i = 1; i <= ndata; i++) {
    std::ostringstream ss_fPe, ss_cPo;
    ss_fPe << "pose_fPe_" << i << ".yaml";
    ss_cPo << "pose_cPo_" << i << ".yaml";
    std::cout << "Use fPe=" << ss_fPe.str() << ", cPo=" << ss_cPo.str() << std::endl;

    vpPoseVector wPe;
    if (wPe.loadYAML(ss_fPe.str(), wPe) == false) {
      std::cout << "Unable to read data from: " << ss_fPe.str() << std::endl;
      return EXIT_FAILURE;
    }
    wMe[i - 1] = vpHomogeneousMatrix(wPe);

    vpPoseVector cPo;
    if (cPo.loadYAML(ss_cPo.str(), cPo)  == false) {
      std::cout << "Unable to read data from: " << ss_cPo.str() << std::endl;
      return EXIT_FAILURE;
    }
    cMo[i-1] = vpHomogeneousMatrix(cPo);
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
    std::cout << "** Rotation (theta-u representation) [deg]: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;
    vpQuaternionVector quaternion(eMc.getRotationMatrix());
    std::cout << "** Rotation (quaternion representation) [rad]: " << quaternion.t() << std::endl;

    // save eMc
    std::ofstream file_eMc("eMc.txt");
    eMc.save(file_eMc);

    vpPoseVector pose_vec(eMc);
    std::string output_filename("eMc.yaml");
    std::cout << std::endl << "Save transformation matrix eMc as a vpPoseVector in " << output_filename << std::endl;
    pose_vec.saveYAML(output_filename, pose_vec);
  }
  else {
    std::cout << std::endl << "** Hand-eye calibration failed" << std::endl;
    std::cout << std::endl << "Check your input data and ensure they are covering the half sphere over the chessboard." << std::endl;
    std::cout << std::endl << "See https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic.html" << std::endl;
  }

  return EXIT_SUCCESS;
}

