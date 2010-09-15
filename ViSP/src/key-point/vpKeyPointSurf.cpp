/****************************************************************************
 *
 * $Id$
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
 * Key point Surf.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/


#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <visp/vpKeyPointSurf.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x010100 // Require opencv >= 1.1.0

#include <visp/vpImageConvert.h>
#include <visp/vpImageTools.h>
#include <visp/vpDisplay.h>
#include <vector>


// Compare two surf descriptors.
double compareSURFDescriptors( const float* d1, const float* d2,
			       double best, int length )
{
  double total_cost = 0;
  int i;
  assert( length % 4 == 0 );
  for( i = 0; i < length; i += 4 )
  {
    double t0 = d1[i] - d2[i];
    double t1 = d1[i+1] - d2[i+1];
    double t2 = d1[i+2] - d2[i+2];
    double t3 = d1[i+3] - d2[i+3];
    total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
    if( total_cost > best )
      break;
  }
  return total_cost;
}


//Find for a point candidate the most similar point in the reference point list.
int naiveNearestNeighbor( const float *vec, int laplacian,
			  const CvSeq *model_keypoints,
			  const CvSeq *model_descriptors )
{
  int length = (int)(model_descriptors->elem_size/sizeof(float));
  int i, neighbor = -1;
  double d, dist1 = 1e6, dist2 = 1e6;
  CvSeqReader reader, kreader;
  cvStartReadSeq( model_keypoints, &kreader, 0 );
  cvStartReadSeq( model_descriptors, &reader, 0 );

  for( i = 0; i < model_descriptors->total; i++ )
  {
    const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
    const float* mvec = (const float*)reader.ptr;
    CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    if( laplacian != kp->laplacian )
      continue;
    d = compareSURFDescriptors( vec, mvec, dist2, length );
    if( d < dist1 )
    {
      dist2 = dist1;
      dist1 = d;
      neighbor = i;
    }
    else if ( d < dist2 )
      dist2 = d;
  }
  if ( dist1 < 0.6*dist2 )
    return neighbor;
  return -1;
}


//Find for a point candidate the most similar point in the reference point list.
int naiveNearestNeighbor( const float *vec,
			  const CvSeq *ref_keypoints,
			  const CvSeq *ref_descriptors )
{
  int length = (int)(ref_descriptors->elem_size/sizeof(float));
  int i, neighbor = -1;
  double d, dist1 = 1e6, dist2 = 1e6;
  CvSeqReader reader, kreader;
  cvStartReadSeq( ref_keypoints, &kreader, 0 );
  cvStartReadSeq( ref_descriptors, &reader, 0 );

  for( i = 0; i < ref_descriptors->total; i++ )
  {
    const float* mvec = (const float*)reader.ptr;
    CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    d = compareSURFDescriptors( vec, mvec, dist2, length );
    if( d < dist1 )
    {
      dist2 = dist1;
      dist1 = d;
      neighbor = i;
    }
    else if ( d < dist2 )
      dist2 = d;
  }
  if ( dist1 < 0.6*dist2 )
    return neighbor;
  return -1;
}



//Find all the matched points
void findPairs( const CvSeq* objectKeypoints,
		const CvSeq* objectDescriptors,
		const CvSeq* imageKeypoints,
		const CvSeq* imageDescriptors,
		std::vector<int>& ptpairs )
{
  int i;
  CvSeqReader reader, kreader;
  cvStartReadSeq( objectKeypoints, &kreader );
  cvStartReadSeq( objectDescriptors, &reader );
  ptpairs.clear();

  for( i = 0; i < objectDescriptors->total; i++ )
  {
    const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
    const float* descriptor = (const float*)reader.ptr;
    CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    int nearest_neighbor = naiveNearestNeighbor( descriptor,
						 kp->laplacian,
						 imageKeypoints,
						 imageDescriptors );
    if( nearest_neighbor >= 0 )
    {
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
vpKeyPointSurf::vpKeyPointSurf():vpBasicKeyPoint()
{
  descriptorType = extendedDescriptor;
  hessianThreshold = 500;
  init();

  image_keypoints = NULL;
  image_descriptors = NULL;
  ref_keypoints = NULL;
  ref_descriptors = NULL;

  storage_cur = NULL;
}

/*!
  Initialize any OpenCV parameters.
*/
void vpKeyPointSurf::init()
{
  storage = cvCreateMemStorage(0);
  params = cvSURFParams(hessianThreshold, descriptorType);
}

/*!
  Basic Destructor

*/
vpKeyPointSurf::~vpKeyPointSurf()
{
  cvReleaseMemStorage(&storage);
  if(storage_cur!=NULL){
    cvReleaseMemStorage(&storage_cur); 
  }
}

/*!

  Build the list of reference points. The computation of the points is
  made all over the image I.

  \param I : The gray scaled image where the refrence points are computed.

  \return the number of reference points.
*/
int vpKeyPointSurf::buildReference(const vpImage<unsigned char> &I)
{
  IplImage* model = NULL;

  vpImageConvert::convert(I,model);

  cvExtractSURF( model, 0, &ref_keypoints, &ref_descriptors, storage, params );

  referenceImagePointsList.resize(ref_keypoints->total);

  for(int i = 0; i < ref_keypoints->total; i++ )
  {
    CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem(ref_keypoints, i);

    referenceImagePointsList[i].set_i(r1->pt.y);
    referenceImagePointsList[i].set_j(r1->pt.x);
  }

  cvReleaseImage(&model);

  return ref_keypoints->total;
}


/*!

  Build the list of reference points. The computation of the points is
  made only on a part of the image. This part is a rectangle defined
  by its top left corner, its height and its width. The parameters of
  this rectangle must be given in pixel.

  \param I : The gray scaled image where the refrence points are computed.

  \param iP : The top left corner of the rectangle.

  \param height : height of the rectangle (in pixel).

  \param width : width of the rectangle (in pixel).

  \return the number of reference points.
*/
int  vpKeyPointSurf::buildReference(const vpImage<unsigned char> &I,
				    vpImagePoint &iP,
				    unsigned int height, unsigned int width)
{
  if((iP.get_i()+height) >= I.getHeight()
     || (iP.get_j()+width) >= I.getWidth())
  {
    vpTRACE("Bad size for the subimage");
    throw(vpException(vpImageException::notInTheImage ,
		      "Bad size for the subimage"));
  }

  vpImage<unsigned char> subImage;

  vpImageTools::createSubImage(I,
			       (unsigned int)iP.get_i(),
			       (unsigned int)iP.get_j(),
			       height, width, subImage);

  int nbRefPoint = this->buildReference(subImage);

  for(int k = 0; k < nbRefPoint; k++)
  {
    (referenceImagePointsList[k]).set_i((referenceImagePointsList[k]).get_i()
					+ iP.get_i());
    (referenceImagePointsList[k]).set_j((referenceImagePointsList[k]).get_j()
					+ iP.get_j());
  }
  return(nbRefPoint);
}


/*!

  Build the list of reference points. The computation of the points is
  made only on a part of the image. This part is a rectangle. The
  parameters of this rectangle must be given in pixel.

  \param I : The gray scaled image where the refrence points are computed.

  \param rectangle : The rectangle which defines the interesting part
  of the image.

  \return the number of reference points.
*/
int  vpKeyPointSurf::buildReference(const vpImage<unsigned char> &I,
				    const vpRect rectangle)
{
  vpImagePoint iP;
  iP.set_i(rectangle.getTop());
  iP.set_j(rectangle.getLeft());
  return (this->buildReference(I, iP,
			       (unsigned int)rectangle.getHeight(),
			       (unsigned int)rectangle.getWidth()));
}


/*!

  Computes the SURF points in the current image I and try to matched
  them with the points in the reference list. Only the matched points
  are stored.

  \param I : The gray scaled image where the points are computed.

  \return the number of point which have been matched.
*/
int vpKeyPointSurf::matchPoint(const vpImage<unsigned char> &I)
{
  IplImage* currentImage = NULL;

  vpImageConvert::convert(I,currentImage);
  
  /* we release the memory storage for the current points (it has to be kept 
      allocated for the get descriptor points, ...) */
  if(storage_cur != NULL){
    cvReleaseMemStorage(&storage_cur);
    storage_cur = NULL;
  }
  storage_cur = cvCreateMemStorage(0);

  cvExtractSURF( currentImage, 0, &image_keypoints, &image_descriptors,
		 storage_cur, params );

  cvReleaseImage(&currentImage);

  CvSeqReader reader, kreader;
  cvStartReadSeq( ref_keypoints, &kreader );
  cvStartReadSeq( ref_descriptors, &reader );

//  matchedPointsReferenceImageList.kill();
//  matchedPointsReferenceImageList.front();

  vpList<int> indexImagePair;
  vpList<int> indexReferencePair;
  indexImagePair.front();
  indexReferencePair.front();

  int nbrPair = 0;

  for(int i = 0; i < ref_descriptors->total; i++ )
  {
    const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
    const float* descriptor = (const float*)reader.ptr;
    CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    int nearest_neighbor = naiveNearestNeighbor( descriptor,
						 kp->laplacian,
						 image_keypoints,
						 image_descriptors );
    if( nearest_neighbor >= 0 )
    {
//      matchedPointsReferenceImageList.addRight(referenceImagePointsList+i);
      indexReferencePair.addRight(i);
      indexImagePair.addRight(nearest_neighbor);
      nbrPair++;
    }
  }

  indexReferencePair.front();
  indexImagePair.front();
//   matchedPointsCurrentImageList.kill();
//   matchedPointsCurrentImageList.front();

  matchedReferencePoints.resize(0);

  if (nbrPair == 0)
    return (0);

  currentImagePointsList.resize(nbrPair);
  matchedReferencePoints.resize(nbrPair);

  for (int i = 0; i < nbrPair; i++)
  {
      int index = indexImagePair.value();

      CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem(image_keypoints, index);

      currentImagePointsList[i].set_i(r1->pt.y);
      currentImagePointsList[i].set_j(r1->pt.x);

      matchedReferencePoints[i] = indexReferencePair.value();

//      matchedPointsCurrentImageList.addRight(currentImagePointsList+i);

      indexImagePair.next();
      indexReferencePair.next();
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
int vpKeyPointSurf::matchPoint(const vpImage<unsigned char> &I,
			       vpImagePoint &iP,
			       unsigned int height, unsigned int width)
{
  if((iP.get_i()+height) >= I.getHeight()
     || (iP.get_j()+width) >= I.getWidth())
  {
    vpTRACE("Bad size for the subimage");
    throw(vpException(vpImageException::notInTheImage ,
		      "Bad size for the subimage"));
  }

  vpImage<unsigned char> subImage;

  vpImageTools::createSubImage(I,
			       (unsigned int)iP.get_i(),
			       (unsigned int)iP.get_j(),
			       height, width, subImage);

  int nbMatchedPoint = this->matchPoint(subImage);

  for(int k = 0; k < nbMatchedPoint; k++)
  {
    (currentImagePointsList[k]).set_i((currentImagePointsList[k]).get_i()
				      + iP.get_i());
    (currentImagePointsList[k]).set_j((currentImagePointsList[k]).get_j()
				      + iP.get_j());
  }

  return(nbMatchedPoint);
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
int vpKeyPointSurf::matchPoint(const vpImage<unsigned char> &I,
			       const vpRect rectangle)
{
  vpImagePoint iP;
  iP.set_i(rectangle.getTop());
  iP.set_j(rectangle.getLeft());
  return (this->matchPoint(I, iP,
			   (unsigned int)rectangle.getHeight(),
			   (unsigned int)rectangle.getWidth()));
}


/*!

  This function displays the matched reference points and the matched
  points computed in the current image. The refrence points are
  displayed in the image Ireference and the matched points coming from
  the current image are displayed in the image Icurrent. It is
  possible to set Ireference and Icurrent with the same image when
  calling the method.

  \param Ireference : The image where the matched refrence points are
  displayed.

  \param Icurrent : The image where the matched points computed in the
  current image are displayed.
*/
void vpKeyPointSurf::display(const vpImage<unsigned char> &Ireference,
			     const vpImage<unsigned char> &Icurrent)
{
//  matchedPointsCurrentImageList.front();
//  matchedPointsReferenceImageList.front();

//   if (matchedPointsCurrentImageList.nbElements()
//       != matchedPointsReferenceImageList.nbElements())
//   {
//     vpTRACE("Numbers of points mismatch");
//     throw(vpException(vpException::fatalError,"Numbers of points mismatch"));
//   }

  for (int i = 0; i < matchedReferencePoints.size(); i++)
  {
      vpDisplay::displayCross (Ireference, referenceImagePointsList[matchedReferencePoints[i]], 3, vpColor::red);
      vpDisplay::displayCross (Icurrent, currentImagePointsList[i], 3, vpColor::green);
//       matchedPointsReferenceImageList.next();
//       matchedPointsCurrentImageList.next();
  }
}


/*!

  This function displays only the matched points computed in the
  current image. They are displayed in the image Icurrent.

  \param Icurrent : The image where the matched points computed in the
  current image are displayed.
*/
void vpKeyPointSurf::display(const vpImage<unsigned char> &Icurrent)
{
//   matchedPointsCurrentImageList.front();
//
//   vpImagePoint ipCur;
//
  for (int i = 0; i < matchedReferencePoints.size(); i++)
  {
      vpDisplay::displayCross (Icurrent, currentImagePointsList[i], 3, vpColor::green);
  }
}

/*!

  Computes the SURF points given by their descriptor and laplacian and try to match
  them with the points in the reference list. Only the matched points
  are stored. The two lists must have the same number of element while they correspond
  the same unique list of point.

  \warning The list returned contains allocated data (2 int per element). Must be deleted to avoid memory leak.

  \param descriptorList : The list of the descriptor

  \param laplacianList  : The list of laplacian

  \return the list of the pair, the first element contains the index in the reference sequence and the second element contains the index in the list given in parameter.
*/
vpList<int*>* vpKeyPointSurf::matchPoint(vpList<float*> descriptorList, vpList<int> laplacianList)
{
	vpList<int*>* pairPoints = new vpList<int*>;

	if(descriptorList.nb != laplacianList.nb){
		vpTRACE("Error, the two lists have different number of element");
		return pairPoints;
	}

  CvSeqReader reader;
  cvStartReadSeq( ref_descriptors, &reader );

  descriptorList.front();
  pairPoints->front();
  laplacianList.front();
  int indexList = 0;
  while(!descriptorList.outside()){
  	float* descriptor = descriptorList.value();

  	int nearest_neighbor = naiveNearestNeighbor( descriptor, laplacianList.value(), ref_keypoints, ref_descriptors);

		if(nearest_neighbor >= 0){
			int* tab;
			tab = new int[2];
			tab[0] = nearest_neighbor;
			tab[1] = indexList;
			pairPoints->addRight(tab);
		}
		indexList++;
		descriptorList.next();
		laplacianList.next();
  }

  return pairPoints;
}
 
/*!
  Get the descriptor of the nth reference point.

 \param index : The index of the desired reference point. The index must be between 0 and the number of reference points - 1.
*/
float* vpKeyPointSurf::getDescriptorReferencePoint (const int index)
{
  if (index >= referenceImagePointsList.size() || index < 0){
    vpTRACE("Index of the reference point out of range");
    throw(vpException(vpException::fatalError,"Index of the refrence point out of range"));
  }

	float* descriptor = NULL;

	CvSeqReader reader;
	cvStartReadSeq( ref_descriptors, &reader );

	for(int j = 0; j < ref_descriptors->total; j++ ){
		if(j== index){
			descriptor = (float*)reader.ptr;
			break;
		}
		CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
	}

	return descriptor;
}

/*!
  Get the laplacian of the nth reference point.

 \param index : The index of the desired reference point. The index must be between 0 and the number of reference points - 1.
*/
int vpKeyPointSurf::getLaplacianReferencePoint (const int index)
{
  if (index >= referenceImagePointsList.size() || index < 0){
    vpTRACE("Index of the reference point out of range");
    throw(vpException(vpException::fatalError,"Index of the refrence point out of range"));
  }

	CvSeqReader reader;
  cvStartReadSeq( ref_keypoints, &reader );

	int laplacian = 0;/* normally only -1, 0, +1 are possible */

	for(int j = 0; j < ref_keypoints->total; j++ ){
		if(j== index){
	    const CvSURFPoint* kp = (const CvSURFPoint*)reader.ptr;
			laplacian = kp->laplacian;
			break;
		}
		CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
	}

	return laplacian;
}

/*!
  Get the parameters of the descriptor of the nth reference point.

 \param index : The index of the desired reference point. The index must be between 0 and the number of reference points - 1.
 \param size : The size of the point used to compute the descriptor.
 \param dir : The orientation of the descriptor (in degree).
*/
void vpKeyPointSurf::getDescriptorParamReferencePoint (const int index, int& size, float& dir)
{
  if (index >= referenceImagePointsList.size() || index < 0){
    vpTRACE("Index of the reference point out of range");
    throw(vpException(vpException::fatalError,"Index of the refrence point out of range"));
  }

  CvSeqReader reader;
  cvStartReadSeq( ref_keypoints, &reader );

  for(int j = 0; j < ref_keypoints->total; j++ ){
    if(j== index){
      const CvSURFPoint* kp = (const CvSURFPoint*)reader.ptr;
      size = kp->size;
      dir = kp->dir;
      break;
    }
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
  }
}


#endif


/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */

