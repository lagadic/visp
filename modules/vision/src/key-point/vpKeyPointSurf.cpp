/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Key point Surf.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#include <visp3/vision/vpKeyPointSurf.h>

#if defined(VISP_HAVE_OPENCV_NONFREE) && (VISP_HAVE_OPENCV_VERSION >= 0x010100) &&                                     \
    (VISP_HAVE_OPENCV_VERSION < 0x030000) // Require opencv >= 1.1.0 < 3.0.0

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>

#include <ctype.h>
#include <list>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

double compareSURFDescriptors(const float *d1, const float *d2, double best, int length);
int naiveNearestNeighbor(const float *vec, int laplacian, const CvSeq *model_keypoints, const CvSeq *model_descriptors);
int naiveNearestNeighbor(const float *vec, const CvSeq *ref_keypoints, const CvSeq *ref_descriptors);
void findPairs(const CvSeq *objectKeypoints, const CvSeq *objectDescriptors, const CvSeq *imageKeypoints,
               const CvSeq *imageDescriptors, std::vector<int> &ptpairs);

// Compare two surf descriptors.
double compareSURFDescriptors(const float *d1, const float *d2, double best, int length)
{
  double total_cost = 0;
  int i;
  assert(length % 4 == 0);
  for (i = 0; i < length; i += 4) {
    double t0 = d1[i] - d2[i];
    double t1 = d1[i + 1] - d2[i + 1];
    double t2 = d1[i + 2] - d2[i + 2];
    double t3 = d1[i + 3] - d2[i + 3];
    total_cost += t0 * t0 + t1 * t1 + t2 * t2 + t3 * t3;
    if (total_cost > best)
      break;
  }
  return total_cost;
}

// Find for a point candidate the most similar point in the reference point
// list.
int naiveNearestNeighbor(const float *vec, int laplacian, const CvSeq *model_keypoints, const CvSeq *model_descriptors)
{
  int length = (int)(model_descriptors->elem_size / (int)sizeof(float));
  int i, neighbor = -1;
  double d, dist1 = 1e6, dist2 = 1e6;
  CvSeqReader reader, kreader;
  cvStartReadSeq(model_keypoints, &kreader, 0);
  cvStartReadSeq(model_descriptors, &reader, 0);

  for (i = 0; i < model_descriptors->total; i++) {
    const CvSURFPoint *kp = (const CvSURFPoint *)kreader.ptr;
    const float *mvec = (const float *)reader.ptr;
    CV_NEXT_SEQ_ELEM(kreader.seq->elem_size, kreader);
    CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);
    if (laplacian != kp->laplacian)
      continue;
    d = compareSURFDescriptors(vec, mvec, dist2, length);
    if (d < dist1) {
      dist2 = dist1;
      dist1 = d;
      neighbor = i;
    } else if (d < dist2)
      dist2 = d;
  }
  if (dist1 < 0.6 * dist2)
    return neighbor;
  return -1;
}

// Find for a point candidate the most similar point in the reference point
// list.
int naiveNearestNeighbor(const float *vec, const CvSeq *ref_keypoints, const CvSeq *ref_descriptors)
{
  int length = (int)(ref_descriptors->elem_size / (int)sizeof(float));
  int i, neighbor = -1;
  double dist1 = 1e6, dist2 = 1e6;
  CvSeqReader reader, kreader;
  cvStartReadSeq(ref_keypoints, &kreader, 0);
  cvStartReadSeq(ref_descriptors, &reader, 0);

  for (i = 0; i < ref_descriptors->total; i++) {
    const float *mvec = (const float *)reader.ptr;
    CV_NEXT_SEQ_ELEM(kreader.seq->elem_size, kreader);
    CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);
    double d = compareSURFDescriptors(vec, mvec, dist2, length);
    if (d < dist1) {
      dist2 = dist1;
      dist1 = d;
      neighbor = i;
    } else if (d < dist2)
      dist2 = d;
  }
  if (dist1 < 0.6 * dist2)
    return neighbor;
  return -1;
}

// Find all the matched points
void findPairs(const CvSeq *objectKeypoints, const CvSeq *objectDescriptors, const CvSeq *imageKeypoints,
               const CvSeq *imageDescriptors, std::vector<int> &ptpairs)
{
  int i;
  CvSeqReader reader, kreader;
  cvStartReadSeq(objectKeypoints, &kreader);
  cvStartReadSeq(objectDescriptors, &reader);
  ptpairs.clear();

  for (i = 0; i < objectDescriptors->total; i++) {
    const CvSURFPoint *kp = (const CvSURFPoint *)kreader.ptr;
    const float *descriptor = (const float *)reader.ptr;
    CV_NEXT_SEQ_ELEM(kreader.seq->elem_size, kreader);
    CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);
    int nearest_neighbor = naiveNearestNeighbor(descriptor, kp->laplacian, imageKeypoints, imageDescriptors);
    if (nearest_neighbor >= 0) {
      ptpairs.push_back(i);
      ptpairs.push_back(nearest_neighbor);
    }
  }
}

/*!

  Basic constructor. It sets by default the hessian threshold to 500
  (a good default value is between 300 and 500) and the descriptor
  type to extended.

*/
vpKeyPointSurf::vpKeyPointSurf()
  : vpBasicKeyPoint(), storage(NULL), params(), storage_cur(NULL), image_keypoints(NULL), image_descriptors(NULL),
    ref_keypoints(NULL), ref_descriptors(NULL), hessianThreshold(500), descriptorType(extendedDescriptor)
{
  init();
}

/*!
  Initialize any OpenCV parameters.
*/
void vpKeyPointSurf::init()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400) // Require opencv >= 2.4.0
  cv::initModule_nonfree();
#endif

  storage = cvCreateMemStorage(0);
  params = cvSURFParams(hessianThreshold, descriptorType);
}

/*!
  Basic Destructor

*/
vpKeyPointSurf::~vpKeyPointSurf()
{
  cvReleaseMemStorage(&storage);
  if (storage_cur != NULL) {
    cvReleaseMemStorage(&storage_cur);
  }
}

/*!

  Build the list of reference points. The computation of the points is
  made all over the image I.

  \param I : The gray scaled image where the reference points are computed.

  \return the number of reference points.
*/
unsigned int vpKeyPointSurf::buildReference(const vpImage<unsigned char> &I)
{
  IplImage *model = NULL;

  if ((I.getWidth() % 8) == 0) {
    int height = (int)I.getHeight();
    int width = (int)I.getWidth();
    CvSize size = cvSize(width, height);
    model = cvCreateImageHeader(size, IPL_DEPTH_8U, 1);
    model->imageData = (char *)I.bitmap;
  } else {
    vpImageConvert::convert(I, model);
  }

  cvExtractSURF(model, 0, &ref_keypoints, &ref_descriptors, storage, params);

  const unsigned int nbPoints = (unsigned int)ref_keypoints->total;

  referenceImagePointsList.resize(nbPoints);

  for (unsigned int i = 0; i < nbPoints; i++) {
    CvSURFPoint *r1 = (CvSURFPoint *)cvGetSeqElem(ref_keypoints, (int)i);

    referenceImagePointsList[i].set_i(r1->pt.y);
    referenceImagePointsList[i].set_j(r1->pt.x);
  }

  if ((I.getWidth() % 8) == 0) {
    model->imageData = NULL;
    cvReleaseImageHeader(&model);
  } else {
    cvReleaseImage(&model);
  }

  _reference_computed = true;
  return nbPoints;
}

/*!

  Build the list of reference points. The computation of the points is
  made only on a part of the image. This part is a rectangle defined
  by its top left corner, its height and its width. The parameters of
  this rectangle must be given in pixel.

  \param I : The gray scaled image where the reference points are computed.

  \param iP : The top left corner of the rectangle.

  \param height : height of the rectangle (in pixel).

  \param width : width of the rectangle (in pixel).

  \return the number of reference points.
*/
unsigned int vpKeyPointSurf::buildReference(const vpImage<unsigned char> &I, const vpImagePoint &iP,
                                            const unsigned int height, const unsigned int width)
{
  if ((iP.get_i() + height) >= I.getHeight() || (iP.get_j() + width) >= I.getWidth()) {
    vpTRACE("Bad size for the subimage");
    throw(vpException(vpImageException::notInTheImage, "Bad size for the subimage"));
  }

  vpImage<unsigned char> subImage;

  vpImageTools::crop(I, (unsigned int)iP.get_i(), (unsigned int)iP.get_j(), height, width, subImage);

  unsigned int nbRefPoint = this->buildReference(subImage);

  for (unsigned int k = 0; k < nbRefPoint; k++) {
    (referenceImagePointsList[k]).set_i((referenceImagePointsList[k]).get_i() + iP.get_i());
    (referenceImagePointsList[k]).set_j((referenceImagePointsList[k]).get_j() + iP.get_j());
  }
  return (nbRefPoint);
}

/*!

  Build the list of reference points. The computation of the points is
  made only on a part of the image. This part is a rectangle. The
  parameters of this rectangle must be given in pixel.

  \param I : The gray scaled image where the reference points are computed.

  \param rectangle : The rectangle which defines the interesting part
  of the image.

  \return the number of reference points.
*/
unsigned int vpKeyPointSurf::buildReference(const vpImage<unsigned char> &I, const vpRect &rectangle)
{
  vpImagePoint iP;
  iP.set_i(rectangle.getTop());
  iP.set_j(rectangle.getLeft());
  return (this->buildReference(I, iP, (unsigned int)rectangle.getHeight(), (unsigned int)rectangle.getWidth()));
}

/*!

  Computes the SURF points in the current image I and try to matched
  them with the points in the reference list. Only the matched points
  are stored.

  \param I : The gray scaled image where the points are computed.

  \return the number of point which have been matched.
*/
unsigned int vpKeyPointSurf::matchPoint(const vpImage<unsigned char> &I)
{
  IplImage *currentImage = NULL;

  if ((I.getWidth() % 8) == 0) {
    int height = (int)I.getHeight();
    int width = (int)I.getWidth();
    CvSize size = cvSize(width, height);
    currentImage = cvCreateImageHeader(size, IPL_DEPTH_8U, 1);
    currentImage->imageData = (char *)I.bitmap;
  } else {
    vpImageConvert::convert(I, currentImage);
  }

  /* we release the memory storage for the current points (it has to be kept
      allocated for the get descriptor points, ...) */
  if (storage_cur != NULL) {
    cvReleaseMemStorage(&storage_cur);
    storage_cur = NULL;
  }
  storage_cur = cvCreateMemStorage(0);

  cvExtractSURF(currentImage, 0, &image_keypoints, &image_descriptors, storage_cur, params);

  CvSeqReader reader, kreader;
  cvStartReadSeq(ref_keypoints, &kreader);
  cvStartReadSeq(ref_descriptors, &reader);

  std::list<int> indexImagePair;
  std::list<int> indexReferencePair;

  unsigned int nbrPair = 0;

  for (int i = 0; i < ref_descriptors->total; i++) {
    const CvSURFPoint *kp = (const CvSURFPoint *)kreader.ptr;
    const float *descriptor = (const float *)reader.ptr;
    CV_NEXT_SEQ_ELEM(kreader.seq->elem_size, kreader);
    CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);
    int nearest_neighbor = naiveNearestNeighbor(descriptor, kp->laplacian, image_keypoints, image_descriptors);
    if (nearest_neighbor >= 0) {
      indexReferencePair.push_back(i);
      indexImagePair.push_back(nearest_neighbor);
      nbrPair++;
    }
  }

  std::list<int>::const_iterator indexImagePairIter = indexImagePair.begin();
  std::list<int>::const_iterator indexReferencePairIter = indexReferencePair.begin();

  matchedReferencePoints.resize(0);

  if (nbrPair == 0)
    return (0);

  currentImagePointsList.resize(nbrPair);
  matchedReferencePoints.resize(nbrPair);

  for (unsigned int i = 0; i < nbrPair; i++) {
    int index = *indexImagePairIter;

    CvSURFPoint *r1 = (CvSURFPoint *)cvGetSeqElem(image_keypoints, index);

    currentImagePointsList[i].set_i(r1->pt.y);
    currentImagePointsList[i].set_j(r1->pt.x);

    matchedReferencePoints[i] = (unsigned int)*indexReferencePairIter;

    ++indexImagePairIter;
    ++indexReferencePairIter;
  }

  if ((I.getWidth() % 8) == 0) {
    currentImage->imageData = NULL;
    cvReleaseImageHeader(&currentImage);
  } else {
    cvReleaseImage(&currentImage);
  }

  return nbrPair;
}

/*!

  Computes the SURF points in only a part of the current image I and
  try to matched them with the points in the reference list. The part
  of the image is a rectangle defined by its top left corner, its
  height and its width. The parameters of this rectangle must be given
  in pixel. Only the matched points are stored.

  \param I : The gray scaled image where the points are computed.

  \param iP : The top left corner of the rectangle.

  \param height : height of the rectangle (in pixel).

  \param width : width of the rectangle (in pixel).

  \return the number of point which have been matched.
*/
unsigned int vpKeyPointSurf::matchPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP,
                                        const unsigned int height, const unsigned int width)
{
  if ((iP.get_i() + height) >= I.getHeight() || (iP.get_j() + width) >= I.getWidth()) {
    vpTRACE("Bad size for the subimage");
    throw(vpException(vpImageException::notInTheImage, "Bad size for the subimage"));
  }

  vpImage<unsigned char> subImage;

  vpImageTools::crop(I, (unsigned int)iP.get_i(), (unsigned int)iP.get_j(), height, width, subImage);

  unsigned int nbMatchedPoint = this->matchPoint(subImage);

  for (unsigned int k = 0; k < nbMatchedPoint; k++) {
    (currentImagePointsList[k]).set_i((currentImagePointsList[k]).get_i() + iP.get_i());
    (currentImagePointsList[k]).set_j((currentImagePointsList[k]).get_j() + iP.get_j());
  }

  return (nbMatchedPoint);
}

/*!

  Computes the SURF points in only a part of the current image I and
  try to matched them with the points in the reference list. The part
  of the image is a rectangle. The parameters of this rectangle must
  be given in pixel. Only the matched points are stored.

  \param I : The gray scaled image where the points are computed.

  \param rectangle : The rectangle which defines the interesting part
  of the image.

  \return the number of point which have been matched.
*/
unsigned int vpKeyPointSurf::matchPoint(const vpImage<unsigned char> &I, const vpRect &rectangle)
{
  vpImagePoint iP;
  iP.set_i(rectangle.getTop());
  iP.set_j(rectangle.getLeft());
  return (this->matchPoint(I, iP, (unsigned int)rectangle.getHeight(), (unsigned int)rectangle.getWidth()));
}

/*!

  This function displays the matched reference points and the matched
  points computed in the current image. The reference points are
  displayed in the image Ireference and the matched points coming from
  the current image are displayed in the image Icurrent. It is
  possible to set Ireference and Icurrent with the same image when
  calling the method.

  \param Ireference : The image where the matched reference points are
  displayed.

  \param Icurrent : The image where the matched points computed in the
  current image are displayed.

  \param size : Size in pixels of the cross that is used to display matched
  points.

*/
void vpKeyPointSurf::display(const vpImage<unsigned char> &Ireference, const vpImage<unsigned char> &Icurrent,
                             unsigned int size)
{
  //  matchedPointsCurrentImageList.front();
  //  matchedPointsReferenceImageList.front();

  //   if (matchedPointsCurrentImageList.nbElements()
  //       != matchedPointsReferenceImageList.nbElements())
  //   {
  //     vpTRACE("Numbers of points mismatch");
  //     throw(vpException(vpException::fatalError,"Numbers of points
  //     mismatch"));
  //   }

  for (unsigned int i = 0; i < matchedReferencePoints.size(); i++) {
    vpDisplay::displayCross(Ireference, referenceImagePointsList[matchedReferencePoints[i]], size, vpColor::red);
    vpDisplay::displayCross(Icurrent, currentImagePointsList[i], size, vpColor::green);
    //       matchedPointsReferenceImageList.next();
    //       matchedPointsCurrentImageList.next();
  }
}

/*!

  This function displays only the matched points computed in the
  current image. They are displayed in the image Icurrent.

  \param Icurrent : The image where the matched points computed in the
  current image are displayed.

  \param size : Size in pixels of the cross that is used to display matched
  points.

  \param color : Color used to display the matched points.
*/
void vpKeyPointSurf::display(const vpImage<unsigned char> &Icurrent, unsigned int size, const vpColor &color)
{
  //   matchedPointsCurrentImageList.front();
  //
  //   vpImagePoint ipCur;
  //
  for (unsigned int i = 0; i < matchedReferencePoints.size(); i++) {
    vpDisplay::displayCross(Icurrent, currentImagePointsList[i], size, color);
  }
}

std::list<int *> *vpKeyPointSurf::matchPoint(std::list<float *> descriptorList, std::list<int> laplacianList)
{
  std::list<int *> *pairPoints = new std::list<int *>;

  if (descriptorList.size() != laplacianList.size()) {
    vpTRACE("Error, the two lists have different number of element");
    return pairPoints;
  }

  CvSeqReader reader;
  cvStartReadSeq(ref_descriptors, &reader);

  std::list<float *>::const_iterator descriptorListIter = descriptorList.begin();
  std::list<int>::const_iterator laplacianListIter = laplacianList.begin();
  descriptorList.front();
  int indexList = 0;
  while (descriptorListIter != descriptorList.end()) {
    float *descriptor = *descriptorListIter;

    int nearest_neighbor = naiveNearestNeighbor(descriptor, *laplacianListIter, ref_keypoints, ref_descriptors);

    if (nearest_neighbor >= 0) {
      int *tab;
      tab = new int[2];
      tab[0] = nearest_neighbor;
      tab[1] = indexList;
      pairPoints->push_back(tab);
    }
    indexList++;
    ++descriptorListIter;
    ++laplacianListIter;
  }

  return pairPoints;
}

/*!
  Get the descriptor of the nth reference point.

 \param index : The index of the desired reference point. The index must be
 between 0 and the number of reference points - 1.
*/
float *vpKeyPointSurf::getDescriptorReferencePoint(const int index)
{
  if (index >= static_cast<int>(referenceImagePointsList.size()) || index < 0) {
    vpTRACE("Index of the reference point out of range");
    throw(vpException(vpException::fatalError, "Index of the reference point out of range"));
  }

  float *descriptor = NULL;

  CvSeqReader reader;
  cvStartReadSeq(ref_descriptors, &reader);

  for (int j = 0; j < ref_descriptors->total; j++) {
    if (j == index) {
      descriptor = (float *)reader.ptr;
      break;
    }
    CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);
  }

  return descriptor;
}

/*!
  Get the laplacian of the nth reference point.

 \param index : The index of the desired reference point. The index must be
 between 0 and the number of reference points - 1.
*/
int vpKeyPointSurf::getLaplacianReferencePoint(const int index)
{
  if (index >= static_cast<int>(referenceImagePointsList.size()) || index < 0) {
    vpTRACE("Index of the reference point out of range");
    throw(vpException(vpException::fatalError, "Index of the reference point out of range"));
  }

  CvSeqReader reader;
  cvStartReadSeq(ref_keypoints, &reader);

  int laplacian = 0; /* normally only -1, 0, +1 are possible */

  for (int j = 0; j < ref_keypoints->total; j++) {
    if (j == index) {
      const CvSURFPoint *kp = (const CvSURFPoint *)reader.ptr;
      laplacian = kp->laplacian;
      break;
    }
    CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);
  }

  return laplacian;
}

/*!
  Get the parameters of the descriptor of the nth reference point.

 \param index : The index of the desired reference point. The index must be
 between 0 and the number of reference points - 1. \param size : The size of
 the point used to compute the descriptor. \param dir : The orientation of the
 descriptor (in degree).
*/
void vpKeyPointSurf::getDescriptorParamReferencePoint(const int index, int &size, float &dir)
{
  if (index >= static_cast<int>(referenceImagePointsList.size()) || index < 0) {
    vpTRACE("Index of the reference point out of range");
    throw(vpException(vpException::fatalError, "Index of the reference point out of range"));
  }

  CvSeqReader reader;
  cvStartReadSeq(ref_keypoints, &reader);

  for (int j = 0; j < ref_keypoints->total; j++) {
    if (j == index) {
      const CvSURFPoint *kp = (const CvSURFPoint *)reader.ptr;
      size = kp->size;
      dir = kp->dir;
      break;
    }
    CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_vision.a(vpKeyPointSurf.cpp.o) has
// no symbols
void dummy_vpKeyPointSurf(){};
#endif
