/*! \example tutorial-ibvs-4pts.cpp */
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>

int main()
{
  try {
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    vpHomogeneousMatrix cMo(0.15, -0.1, 1.,
                            vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

    vpPoint point[4] ;
    point[0].setWorldCoordinates(-0.1,-0.1, 0);
    point[1].setWorldCoordinates( 0.1,-0.1, 0);
    point[2].setWorldCoordinates( 0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);

    vpServo task ;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);

    vpFeaturePoint p[4], pd[4] ;
    for (unsigned int i = 0 ; i < 4 ; i++) {
      point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
      task.addFeature(p[i], pd[i]);
    }

    vpHomogeneousMatrix wMc, wMo;
    vpSimulatorCamera robot;
    robot.setSamplingTime(0.040);
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    for (unsigned int iter=0; iter < 150; iter ++) {
      robot.getPosition(wMc);
      cMo = wMc.inverse() * wMo;
      for (unsigned int i = 0 ; i < 4 ; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
      }
      vpColVector v = task.computeControlLaw();
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);
    }

    task.kill();
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}

