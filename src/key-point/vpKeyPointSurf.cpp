/****************************************************************************
 *
 * $Id: vpBasicFeature.h,v 1.10 2008-02-26 10:32:10 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

  nbReferencePoints = 0;
  nbMatchedPoints = 0;
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

  referenceImagePointsList = new vpImagePoint[ref_keypoints->total];

  for(int i = 0; i < ref_keypoints->total; i++ )
  {
    CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem(ref_keypoints, i);

    referenceImagePointsList[i].set_i(r1->pt.y);
    referenceImagePointsList[i].set_j(r1->pt.x);
  }

  cvReleaseImage(&model);

  nbReferencePoints = ref_keypoints->total;

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

  cvExtractSURF( currentImage, 0, &image_keypoints, &image_descriptors, 
		 storage, params );

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

  if (currentImagePointsList != NULL)
  {
    delete [] currentImagePointsList;
    currentImagePointsList = NULL;
  }

  if (matchedReferencePoints != NULL)
  {
    delete [] matchedReferencePoints;
    matchedReferencePoints = NULL;
  }

  if (nbrPair == 0)
    return (0);

  currentImagePointsList = new vpImagePoint[nbrPair];
  matchedReferencePoints = new int[nbrPair];

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

  nbMatchedPoints = nbrPair;

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
void vpKeyPointSurf::display(vpImage<unsigned char> &Ireference, 
			     vpImage<unsigned char> &Icurrent)
{
//  matchedPointsCurrentImageList.front();
//  matchedPointsReferenceImageList.front();

//   if (matchedPointsCurrentImageList.nbElements() 
//       != matchedPointsReferenceImageList.nbElements())
//   {
//     vpTRACE("Numbers of points mismatch");
//     throw(vpException(vpException::fatalError,"Numbers of points mismatch"));
//   }

  for (int i = 0; i < nbMatchedPoints; i++)
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
void vpKeyPointSurf::display(vpImage<unsigned char> &Icurrent)
{
//   matchedPointsCurrentImageList.front();
// 
//   vpImagePoint ipCur;
//   
  for (int i = 0; i < nbMatchedPoints; i++)
  {
      vpDisplay::displayCross (Icurrent, currentImagePointsList[i], 3, vpColor::green);
  }
}

#endif


/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */

