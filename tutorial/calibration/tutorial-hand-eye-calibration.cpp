//! \example tutorial-hand-eye-calibration.cpp
#include <visp3/vision/vpHandEyeCalibration.h>

int main(int argc, char *argv[])
{
  unsigned int ndata = 0;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ndata" && i+1 < argc) {
      ndata = atoi(argv[i+1]);
    } else if (std::string(argv[i]) == "--help") {
      std::cout << argv[0] << " [--ndata <number of data to process>] "
                              "[--help]" << std::endl;
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

  vpHandEyeCalibration::calibrate(cMo, wMe, eMc);

  // save eMc
  std::ofstream file_eMc("eMc.txt");
  eMc.save(file_eMc);

  vpPoseVector pose_vec(eMc);
  std::cout << "\nSave eMc.yaml" << std::endl;
  pose_vec.saveYAML("eMc.yaml", pose_vec);

  std::cout << "\nOutput: Hand-eye calibration result: eMc estimated " << std::endl;
  std::cout << eMc << std::endl;
  vpThetaUVector ePc(eMc);
  std::cout << "theta U (deg): " << vpMath::deg(ePc[0]) << " " << vpMath::deg(ePc[1]) << " " << vpMath::deg(ePc[2]) << std::endl;

  return EXIT_SUCCESS;
}

