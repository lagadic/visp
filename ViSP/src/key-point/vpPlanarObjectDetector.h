/****************************************************************************
 *
 * $Id: $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Ferns based planar object detector.
 *
 * Authors:
 * Romain Tallonneau
 *
 *****************************************************************************/

#ifndef VPPLANAROBJECTDETECTOR_H_
#define VPPLANAROBJECTDETECTOR_H_


#include <visp/vpConfig.h>

#ifdef VISP_HAVE_OPENCV_VERSION

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/features2d/features2d.hpp>
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020000) // Require opencv >= 2.0.0
#  include <cv.h>
#  include <cvaux.hpp>
#endif

#include <visp/vpConfig.h>
#include <visp/vpList.h>
#include <visp/vpImagePoint.h>
#include <visp/vpImage.h>
#include <visp/vpRect.h>
#include <visp/vpImagePoint.h>
#include <visp/vpPoint.h>
#include <visp/vpList.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomography.h>

/*!
  \class vpPlanarObjectDetector
  \ingroup PlanarSurfaceDetector
  \brief Class used to detect a planar surface based on the Ferns Classifier. 
  
  This class allows to train a Fern classifier for a surface thanks to
  a reference image.
  
  Using this classifier (which can be stored and loaded), it is
  possible to compute the pose of the planar surface in the current
  image (The homography is also available)
   
  For more details about the Ferns Classifier and the planar object detector, see
   - Mustafa Özuysal, Michael Calonder, Vincent Lepetit, Pascal Fua, "Fast 
      KeyPoint Recognition Using Random Ferns", IEEE Transactions on Pattern 
      Analysis and Machine Intelligence, 15 Jan. 2009.

   - Vincent Lepetit, Pascal Fua, “Towards Recognizing Feature Points Using 
      Classification Trees”, Technical Report IC/2004/74, EPFL, 2004.
  
  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpPlanarObjectDetector.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020000 // Planar object detector is only available since OpenCV-2.0.0
int main()
{
  vpImage<unsigned char> Ireference;
  vpImage<unsigned char> Icurrent;
  vpPlanarObjectDetector detector;

  //First grab the reference image Ireference

  //Build the reference plane surface.
  detector.buildReference(Ireference);

  //Then grab another image which represents the current image Icurrent

  //detect between the reference points and the current image.
  detector.matchPoint(Icurrent);

  //Display the detected structure
  detector.display(Icurrent);

  return (0);
}
#else
int main() {}
#endif
  \endcode

  It is also possible to create the reference thanks to only a part of the
  reference image (not the whole image) and find points to match in only a
  part of the current image. The small following example shows how to this

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpPlanarObjectDetector.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020000 // Planar object detector is only available since OpenCV-2.0.0
int main()
{
  vpImage<unsigned char> Ireference;
  vpImage<unsigned char> Icurrent;
  vpPlanarObjectDetector detector;

  //First grab the reference image Ireference

  //Select a part of the image by clicking on two points which define a rectangle
  vpImagePoint corners[2];
  for (int i=0 ; i < 2 ; i++)
  {
    vpDisplay::getClick(Ireference, corners[i]);
  }

  //Build the reference plane structure.
  int nbrRef;
  unsigned int height, width;
  height = (unsigned int)(corners[1].get_i() - corners[0].get_i());
  width = (unsigned int)(corners[1].get_j() - corners[0].get_j());
  nbrRef = detector.buildReference(Ireference, corners[0], height, width);

  //Then grab another image which represents the current image Icurrent

  //Select a part of the image by clicking on two points which define a rectangle
  for (int i=0 ; i < 2 ; i++)
  {
    vpDisplay::getClick(Icurrent, corners[i]);
  }

  //detect between the reference points and the current image.
  int nbrMatched;
  height = (unsigned int)(corners[1].get_i() - corners[0].get_i());
  width = (unsigned int)(corners[1].get_j() - corners[0].get_j());
  nbrMatched = detector.matchPoint(Icurrent, corners[0], height, width);

  //detect between the reference points and the current image.
  detector.display(Icurrent);

  return(0);
}
#else
int main() {}
#endif
  \endcode
*/  
class VISP_EXPORT vpPlanarObjectDetector: public cv::PlanarObjectDetector{
private:  
    /* flag to indicate wether the classifier is trained or not */
  bool hasLearn;
  
    /* parameters for the key-points detector and the classifier */
  int threshold;
  int nbView;
  int dist;
  int nbClassfier;
  int ClassifierSize;
  int nbOctave;
  int patchSize;
  int radius;
  int nbPoints;

    /* parameters for the filter */
  bool blurImage;
  int radiusBlur;
  int sigmaBlur;
  
  unsigned int nbMinPoint;

  std::vector<cv::KeyPoint> objKeypoints;
  std::vector<cv::KeyPoint> imgKeypoints;
  
  vpHomography homography;
  std::vector<cv::Point2f> dst_corners;   /* The corners of the detected planar surface */

    /* The patch generator (OpenCV format) */
  cv::PatchGenerator gen;

    /* the current image in the OpenCV format */
  IplImage* model; 
    /* the ROI in the learned image */
  cv::Rect modelROI_Ref; 
    /* the vector of pair */ 
  std::vector<int> pairs;

public:

    // constructors and destructors
  vpPlanarObjectDetector();
  vpPlanarObjectDetector(const std::string& dataFile, const std::string& objectName);
  
  virtual ~vpPlanarObjectDetector();
  
  
    // set and get
  /*!
    The image is blurred before being processed. This solution can lead to a 
    better recognition rate.
    
    \param _blur : the new option for the blur
  */
  inline void setBlurSetting(const bool _blur){ blurImage = _blur;}
  inline void setBlurSettings(const bool _blur, int _sigma, int _size);
  
  /*!
    return the blur option. The Blur option is used to activate a filter used to blur the input image. 
    The blurring can improve the robustness of the detection.
    
    \return the value of the blur option
  */
  inline bool getBlurSetting(){ return blurImage;}
  
  /*!
    return the blur sigma (for the filter) option
    
    \return the value of the sigma for the blur filter
  */
  inline int getBlurSigma() { return this->sigmaBlur;}
  
  /*!
    return the blur size (for the filter) option
    
    \return the value of the radius for the blur filter
  */
  inline int getBlurSize() { return this->radiusBlur;}
  
  void setDetectorParameters(int _threshold, int _nbView, int _dist, int _nbClassfier, int _ClassifierSize, int _nbOctave, int _patchSize, int _radius, int _nbPoints);
  
  /*!
    return the computed homography between the reference image and the current image
    
    \param H : the computed homography
  */
  inline void getHomography(vpHomography& H) { H = this->homography;}
  
  /*!
    get the minimum number of point to accept the computed homography
    
    \return the minimum of point
  */
  inline unsigned int getNbMinPoint() const { return this->nbMinPoint; }

  /*!
    Set the minimum number of point to validate the computed homography
    
    \param nb : the new minimum number of point
  */
  inline void setNbMinPoint(const unsigned int nb) { this->nbMinPoint = nb;}

    // main functions
      // creation of reference
  int buildReference(const vpImage<unsigned char> &I);
  int buildReference(const vpImage<unsigned char> &I,
		       vpImagePoint &iP,
		       unsigned int height, unsigned int width);
  int buildReference(const vpImage<unsigned char> &I,
		       const vpRect rectangle);
  void train();
    
    // matching
  bool matchPoint(const vpImage<unsigned char> &I);
  bool matchPoint(const vpImage<unsigned char> &I,
		   vpImagePoint &iP, unsigned int height, unsigned int width);
  bool matchPoint(const vpImage<unsigned char> &I, const vpRect rectangle);
    // database management
  void recordDetector(const std::string& objectName, const std::string& dataFile);
  void load(const std::string& dataFilename, const std::string& objName);
    
    
    // display
  void display(vpImage<unsigned char> &I, bool displayKpts = true);
  void display(vpImage<unsigned char> &Iref,
		 vpImage<unsigned char> &Icurrent);

  void setRoi(vpImagePoint tl, vpImagePoint br);
  

  std::vector<vpImagePoint> getDetectedCorners() const;
  
  /*!
    return the number of reference points
    
    \return the number of reference points
  */
  int getNbRefPoints() const { return (int)modelPoints.size();}
  
  vpImagePoint getRefPoint(const int index);
  void getReferencePoint(const int index, vpImagePoint &imP);

  /*!
    return the number of current points
    
    \return the number of currently detected points
  */
  inline int getNbCurPoints() const { return (int)imgKeypoints.size();}
  
  vpImagePoint getCurPoint(const int index);
  void getCurPoint(const int index, vpImagePoint & ip);
  
  int getMatchedPointByRef(const int indexRef);
  
  std::vector<int>* getPairsVector(){return &pairs;}
  
private:
  /*!
    initialise stuff
  */
  void init();
  
  /*!
    set the image (conversion to the openCV IplImage format)
    
    \param I : the image to convert
  */
  void setImage(const vpImage<unsigned char>& I);
  
  void computeRoi(vpImagePoint* ip, const int nbpt);
};

#endif

#endif /* VPPLANAROBJECTDETECTOR_H_ */
