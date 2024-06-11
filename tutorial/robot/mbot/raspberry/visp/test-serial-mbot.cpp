#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpSerial.h>
#include <visp3/core/vpTime.h>

int main(int argc, char *argv[])
{
#if !defined(_WIN32)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  double time = 4;
  double v_x = 0;
  double w_z = 0;
  bool rpm_command = false;
  int rpm_l = 0;
  int rpm_r = 0;

  for (int i = 1; i < argc; i++) {
    if ((std::string(argv[i]) == "--t" || std::string(argv[i]) == "-t") && i + 1 < argc) {
      time = (double)atof(argv[i + 1]);
    }
    else if ((std::string(argv[i]) == "--vx" || std::string(argv[i]) == "-vx") && i + 1 < argc) {
      v_x = (double)atof(argv[i + 1]);
    }
    else if ((std::string(argv[i]) == "--wz" || std::string(argv[i]) == "-wz") && i + 1 < argc) {
      w_z = (double)atof(argv[i + 1]);
    }
    else if ((std::string(argv[i]) == "--rpm_l" || std::string(argv[i]) == "-rpm_l") && i + 1 < argc) {
      rpm_command = true;
      rpm_l = (double)atoi(argv[i + 1]);
    }
    else if ((std::string(argv[i]) == "--rpm_r" || std::string(argv[i]) == "-rpm_r") && i + 1 < argc) {
      rpm_command = true;
      rpm_r = (double)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: \n"
        << argv[0]
        << " --vx <linear velocity in m/s> --wz <rotational velocity in deg/s> --rpm_l <motor left RPM> "
        "--rpm_r <motor right RPM> --t <duration of the command in second> --help"
        << std::endl;
      std::cout << "\nExample:\n" << argv[0] << " --vx 0.05 --wz 0 --t 4\n" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  vpSerial serial("/dev/ttyAMA0", 115200);

  {
    std::stringstream ss;
    if (rpm_command) {
      std::cout << "Apply rpm_l=" << rpm_l << " rpm_r=" << rpm_r << " during " << time << " seconds" << std::endl;
      ss << "MOTOR_RPM=" << rpm_l << "," << rpm_r << "\n";
    }
    else {
      vpColVector v(2);
      v[0] = v_x;
      v[1] = vpMath::rad(w_z);
      std::cout << "Apply v_x=" << v_x << " m/s "
        << " w_z=" << w_z << " deg/s during " << time << " seconds" << std::endl;
      double radius = 0.0325;
      double L = 0.0725;
      double motor_left = -(v[0] + L * v[1]) / radius;
      double motor_right = (v[0] - L * v[1]) / radius;
      std::cout << "Motor left vel: " << motor_left << " motor right vel: " << motor_right << " (rad/s)" << std::endl;
      double rpm_left = motor_left * 30. / M_PI;
      double rpm_right = motor_right * 30. / M_PI;

      ss << "MOTOR_RPM=" << (int)rpm_left << "," << (int)rpm_right << "\n";
    }
    std::cout << "Send: " << ss.str() << std::endl;
    double t0 = vpTime::measureTimeSecond();
    while (vpTime::measureTimeSecond() - t0 < time) {
      serial.write(ss.str());
      vpTime::wait(100);
    }
    return EXIT_SUCCESS;
  }
  serial.write("MOTOR_RPM=-100,100\n");
  vpTime::sleepMs(500);
  serial.write("MOTOR_RPM=-50,100\n");
  vpTime::sleepMs(500);
  serial.write("MOTOR_RPM=50,-50\n");
  vpTime::sleepMs(500);
  serial.write("LED_RING=0,0,10,0\n");
  vpTime::sleepMs(500);
  serial.write("LED_RING=0,0,0,10\n");
  vpTime::sleepMs(500);
  serial.write("LED_RING=0,0,0,0\n");
  vpTime::sleepMs(500);
  serial.close();
#else
  (void)argc;
  (void)argv;
  std::cout << "Serial test is only working on unix-like OS." << std::endl;
#endif
  return EXIT_SUCCESS;
}
