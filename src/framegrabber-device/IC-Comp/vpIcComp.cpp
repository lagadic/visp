/*                                                                -*-c++-*-
#----------------------------------------------------------------------------
#  Copyright (C) 1998  IRISA-INRIA Rennes Vista Project
#  All Rights Reserved.
#
#
#    Contact:
#       Eric Marchand
#       IRISA-INRIA Rennes
#       Campus Universitaire de Beaulieu
#       35042 Rennes Cedex
#       France
#
#    email: marchand@irisa.fr
#    www  : http://www.irisa.fr/vista
#
#----------------------------------------------------------------------------
*/

/*!
  \file vpIcCompGrabber.cpp
  \brief member functions for the ICComp (Imaging Technology) video device class (suitable for new Linux Kernel > 2.4)
  \ingroup libdevice
*/
#include <visp/vpConfig.h>


#ifdef HAVE_FG_ICCOMP


#include <visp/vpIcComp.h>
#include <visp/vpFrameGrabberException.h>
#include <visp/vpImageIo.h>
#define DEBUG_LEVEL1 0

/*!
  \brief constructor

  \param _input : video port
  \param _scale : decimation factor

  By default the framerate is set to 25 fps.

  \sa setFramerate()
*/
vpIcCompGrabber::vpIcCompGrabber( unsigned _input, unsigned _scale)
{
  if (DEBUG_LEVEL1)
    cout << "begin vpIcCompGrabber::vpIcCompGrabber()" << endl ;

  framerate = vpIcCompGrabber::framerate_25fps;
  init = false ;

  framegrabber = new ICcomp2x ;

  setInput(_input);
  setScale(_scale) ;

  if (DEBUG_LEVEL1)
    cout << "end vpIcCompGrabber::vpIcCompGrabber()" << endl ;
}

/*!
  \brief constructor

  \param I : Image data structure (8 bits image)
  \param _input : video port
  \param _scale : decimation factor

  By default the framerate is set to 25 fps.

  \sa setFramerate()
*/
vpIcCompGrabber::vpIcCompGrabber(vpImage<unsigned char> &I, unsigned _input, unsigned _scale )
{
  if (DEBUG_LEVEL1)
    cout << "begin vpIcCompGrabber::vpIcCompGrabber(vpImage<unsigned char> &I, int scal)" << endl ;
  framerate = vpIcCompGrabber::framerate_25fps;

  framegrabber = new ICcomp2x ;

  setInput(_input);
  setScale(_scale) ;

  init = false ;

  open(I) ;

  if (DEBUG_LEVEL1)
    cout << "end vpIcCompGrabber::vpIcCompGrabber(vpImage<unsigned char> &I, int scal)" << endl ;
}

/*!
  \brief constructor

  \param I : Image data structure (32 bits image)
  \param _input : video port
  \param _scale : decimation factor

  By default the framerate is set to 25 fps.

  \sa setFramerate()
*/
vpIcCompGrabber::vpIcCompGrabber(vpImage<vpRGBa> &I, unsigned _input, unsigned _scale )
{
  if (DEBUG_LEVEL1)
    cout << "begin vpIcCompGrabber::vpIcCompGrabber(vpImage<unsigned char> &I, int scal)" << endl ;

  framerate = vpIcCompGrabber::framerate_25fps;
  framegrabber = new ICcomp2x ;

  setInput(_input);
  setScale(_scale) ;

  open(I) ;

  if (DEBUG_LEVEL1)
    cout << "end vpIcCompGrabber::vpIcCompGrabber(vpImage<unsigned char> &I, int scal)" << endl ;
}

/*!
  \brief set the video port
*/
void
vpIcCompGrabber::setInput(unsigned _input)
{
  if (_input >3)
  {
    ERROR_TRACE("Wrong input %d, IC-Comp Frame grabber has only 4 input channels",_input) ;

    throw (vpFrameGrabberException(vpFrameGrabberException::ERRWrongInput,
				   "Wrong input channel") );
  }

  this->input = _input ;
  framegrabber->SetCamera(_input);
}

/*!
  \brief set the scale

  \param scale : decimation factor
*/
void
vpIcCompGrabber::setScale(unsigned scale)
{
  if ((scale <1) || (scale >16))
  {
    ERROR_TRACE("Wrong scale %d, scale shoud be between 1 and 16",scale) ;
    throw (vpFrameGrabberException(vpFrameGrabberException::ERRWrongInput,
				   "Wrong scale") );
  }

  this->scale = scale ;

  framegrabber->SetDecimation(scale);
}

/*!
  \brief Initialize image acquisition

  \param I : Image data structure (8 bits image)
  \param _input : video port
  \param _scale : decimation factor
*/
void
vpIcCompGrabber::open(vpImage<unsigned char> &I)
{
  if (DEBUG_LEVEL1)
    cout << "begin  vpIcCompGrabber::open(vpImage<unsigned char> &I" << endl ;

  if (framegrabber->GetDecimation() ==1)
    framegrabber->SetNBufs(1);
  else
    framegrabber->SetNBufs(2);

  framegrabber->SetDepth(8);

  framegrabber->SetCbCrOrder(ICCOMP_NORMAL_ORDER);

  framegrabber->Init() ;

  ncols = framegrabber->GetWidth() ;
  nrows = framegrabber->GetHeight() ;

  ERROR_TRACE("%d %d", nrows, ncols ) ;

  I.resize(nrows,ncols) ;

  init = true ;

  if (DEBUG_LEVEL1)
    cout << "end  vpIcCompGrabber::open(vpImage<unsigned char> &I" << endl ;
}

/*!
  \brief Initialize image acquisition

  \param I : Image data structure (32 bits image)
  \param _input : video port
  \param _scale : decimation factor
*/
void
vpIcCompGrabber::open(vpImage<vpRGBa> &I)
{
  if (DEBUG_LEVEL1)
    cout << "begin  vpIcCompGrabber::open(vpImage<vpRGBa> &I) " << endl ;

  framegrabber->SetDecimation(1) ;

  framegrabber->SetNBufs(1);
  framegrabber->SetDepth(16);

  framegrabber->SetCbCrOrder(ICCOMP_NORMAL_ORDER);

  framegrabber->Init() ;

  ncols = framegrabber->GetWidth()/scale ;
  nrows = framegrabber->GetHeight()/scale ;

  I.resize(nrows,ncols) ;

  init = true ;

  if (DEBUG_LEVEL1)
    cout << "end  vpIcCompGrabber::open(vpImage<vpRGBa> &I) " << endl ;
}

/*!
  \brief Acquire a color image

  \param I : Image data structure (32 bits image)

  \sa getField()
*/
void
vpIcCompGrabber::acquire(vpImage<vpRGBa> &I)
{

  if (DEBUG_LEVEL1)
    cout << "begin vpIcCompGrabber::acquire(...)" << endl ;

  if (framegrabber==NULL)
  {
    ERROR_TRACE("ICcomp not initialized ") ;

    throw (vpFrameGrabberException(vpFrameGrabberException::ERRNotInitialiazed,
				   "ICcomp not initialized ") );
  }

  int framebuffer = 0 ;

  unsigned  char *bitmap ;
  bitmap = framegrabber->Acquire(field, framebuffer) ;

  if ((I.getCols() != ncols)||(I.getRows() != nrows))
    I.resize(nrows,ncols) ;


  if (scale==1)
    framegrabber->ConvertYCbCrToRGBA(bitmap, (unsigned char *)I.bitmap, nrows, ncols) ;
  else
  {
    unsigned char *bitmaprgba ;
    bitmaprgba = new unsigned char[framegrabber->GetWidth()*framegrabber->GetHeight()*4*sizeof(unsigned char)] ;
    framegrabber->ConvertYCbCrToRGBA(bitmap, bitmaprgba,
				     framegrabber->GetHeight(),
				     framegrabber->GetWidth()) ;

    unsigned char *ptr = bitmaprgba ;
    for (int i=0 ; i < I.getRows() ; i++)
    {
      for (int j=0 ; j < I.getCols() ; j++)
      {
	I[i][j].B = *(ptr ) ;
	I[i][j].G = *(ptr +1 ) ;
	I[i][j].R = *(ptr +2) ;

	ptr += 4*scale ;
      }
      ptr += framegrabber->GetWidth()*4*sizeof(unsigned char)*(scale-1) ;
    }

    delete [] bitmaprgba ;

  }

  if (DEBUG_LEVEL1)
    cout << "end vpIcCompGrabber::acquire(...)" << endl ;

}


/*!
  \brief Acquire a gray level image

  \param I : Image data structure (8 bits image)

  \sa getField()
*/
void
vpIcCompGrabber::acquire(vpImage<unsigned char> &I)
{

  if (DEBUG_LEVEL1)
    cout << "begin vpIcCompGrabber::acquire(..<uchar>.)" << endl ;
  if (framegrabber==NULL)
  {
    ERROR_TRACE("ICcomp not initialized ") ;

    throw (vpFrameGrabberException(vpFrameGrabberException::ERRNotInitialiazed,
				   "ICcomp not initialized ") );
  }

  int framebuffer = 0 ;

  unsigned  char *bitmap = NULL ;

  switch (framerate) {
  case framerate_25fps:
    bitmap = framegrabber->Acquire(field, framebuffer) ;

    if (framegrabber->GetDecimation()!=1) {
      // If subsampling, we get only the even frame
      while (field == 0)
	bitmap = framegrabber->Acquire(field,framebuffer) ;
    }
    break;

  case framerate_50fps:
    bitmap = framegrabber->Acquire(field, framebuffer) ;

    break;
  }

  if ((I.getCols() != ncols)||(I.getRows() != nrows))
    I.resize(nrows,ncols) ;

  memcpy(I.bitmap,bitmap,nrows*ncols)  ;

  //vpImageIo::writePGM(I,"test.pgm") ;

  if (DEBUG_LEVEL1)
    cout << "end vpIcCompGrabber::acquire(...)" << endl ;
}

/*!

  \brief Return the field (odd or even) corresponding to the last acquired
  frame.

  This method is to call after acquire() and has only a sens if the acquisition
  framerate is set to 50 fps.

  \return Field of the acquired frame (0 if odd field, 1 if even field).

  \sa acquire(), setFramerate()

*/
bool
vpIcCompGrabber::getField()
{
  return field;
}
/*!

  \brief Set the framerate of the acquisition.

  \param framerate The framerate for the acquisition.

  \sa getFramerate()

*/
void
vpIcCompGrabber::setFramerate(vpIcCompGrabber::framerateEnum framerate)
{
   this->framerate = framerate;
}
/*!

  \brief Return the framerate of the acquisition.

  \return The actual framerate of the framegrabber.

  \sa setFramerate()
*/


vpIcCompGrabber::framerateEnum
vpIcCompGrabber::getFramerate()
{
  return framerate;
}


/*!
  \brief Close the video port
*/
void
vpIcCompGrabber::close()
{

  if (framegrabber!=NULL)
  {
    TRACE("appel destructeur framegrabber->~ICcomp() ") ;
    framegrabber->~ICcomp2x() ;
    framegrabber = NULL ;
  }
}


/*!
  \brief Destructor
  \sa close() ;
*/
vpIcCompGrabber::~vpIcCompGrabber()
{
  if (DEBUG_LEVEL1)
    cout << "begin vpIcCompGrabber::~vpIcCompGrabber()" << endl ;

  close() ;

  if (DEBUG_LEVEL1)
    cout << "end vpIcCompGrabber::~vpIcCompGrabber()" << endl ;
}

#undef DEBUG_LEVEL1

#endif
