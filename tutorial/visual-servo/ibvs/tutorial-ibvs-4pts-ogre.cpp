/*! \example tutorial-ibvs-4pts-ogre.cpp */
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_AR
#include <visp3/ar/vpAROgre.h>
#endif
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>

int main()
{
  try {
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

    // Define the target as 4 points
    vpPoint point[4] ;
    point[0].setWorldCoordinates(-0.1,-0.1, 0);
    point[1].setWorldCoordinates( 0.1,-0.1, 0);
    point[2].setWorldCoordinates( 0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);

#if defined(VISP_HAVE_OGRE)
    // Color image used as background texture.
    vpImage<unsigned char> background(480, 640, 255);

    // Parameters of our camera
    vpCameraParameters cam(840, 840, background.getWidth()/2, background.getHeight()/2);

    // Our object
    // A simulator with the camera parameters defined above,
    // and the background image size
    vpAROgre ogre;
    ogre.setShowConfigDialog(false);
    ogre.setCameraParameters(cam);
    ogre.addResource("./"); // Add the path to the Sphere.mesh resource
    ogre.init(background, false, true);

    // Create the scene that contains 4 spheres
    // Sphere.mesh contains a sphere with 1 meter radius
    std::vector<std::string> name(4);
    for (unsigned int i=0; i<4; i++) {
      std::ostringstream s; s << "Sphere" <<  i; name[i] = s.str();
      ogre.load(name[i], "Sphere.mesh");
      ogre.setScale(name[i], 0.02f, 0.02f, 0.02f); // Rescale the sphere to 2 cm radius
      // Set the position of each sphere in the object frame
      ogre.setPosition(name[i], vpTranslationVector(point[i].get_oX(), point[i].get_oY(), point[i].get_oZ()));
    }

    // Add an optional point light source
    Ogre::Light * light = ogre.getSceneManager()->createLight();
    light->setDiffuseColour(1, 1, 1); // scaled RGB values
    light->setSpecularColour(1, 1, 1); // scaled RGB values
    light->setPosition((Ogre::Real)cdMo[0][3], (Ogre::Real)cdMo[1][3], (Ogre::Real)(-cdMo[2][3]));
    light->setType(Ogre::Light::LT_POINT);
#endif

    vpServo task ;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);

    vpFeaturePoint p[4], pd[4] ;
    for (int i = 0 ; i < 4 ; i++) {
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
      for (int i = 0 ; i < 4 ; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
      }
#if defined(VISP_HAVE_OGRE)
      // Update the scene from the new camera position
      ogre.display(background, cMo);
#endif
      vpColVector v = task.computeControlLaw();
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);
      vpTime::wait( robot.getSamplingTime() * 1000);
    }
    task.kill();
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
  catch(...) {
    std::cout << "Catch an exception " << std::endl;
  }
}

