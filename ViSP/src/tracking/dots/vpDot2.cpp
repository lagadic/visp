/****************************************************************************
 *
 * $Id: vpDot2.cpp,v 1.21 2007-05-23 11:58:55 fspindle Exp $
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
 * Track a white dot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpDot2.cpp
  \brief Track a white dot
*/

/*!
  \class vpDot2


  \brief This vpTracker is meant to track some white zones on a vpImage.

  The center of gravity of a vpDot2 white zone have to be of the right color
  level. track() and searchDotsInArea() are the most important features
  of this class.

  - track() estimate the current position of the dot using it's previous
    position, then try to compute the new parameters of the dot. It everything
    went ok, tracking succeded, otherwise we search this dot in an window
    around the last position of the dot.

  - searchDotsInArea() enable to find dots similar to this dot in a window. It
    is used when there was a problem performing basic tracking of the dot, but
    can also be used to find a certain type of dots in the full image.
*/

#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpList.h>
#include <visp/vpColor.h>

// exception handling
#include <visp/vpTrackingException.h>

#include <visp/vpDot2.h>


/******************************************************************************
 *
 *      CONSTRUCTORS AND DESTRUCTORS
 *
 ******************************************************************************/

/*!
  Default contructor .... just do basic default initialisation.
*/
vpDot2::vpDot2() : vpTracker()
{
  cog_ufloat = 0 ;
  cog_vfloat = 0 ;

  width = 0;
  height = 0;
  surface = 0;
  gray_level_min = 128;
  gray_level_max = 255;
  accuracy = 0.65;

  m00 = m11 = m02 = m20 = m10 = m01 = 0 ;

  bbox_u_min = bbox_u_max = bbox_v_min = bbox_v_max = 0;

  firstBorder_u = 0;
  firstBorder_v = 0;

  compute_moment = false ;
  graphics = false;
}

/*!

  Constructor initialise the coordinates of the gravity center of the dot to
  (u,v).  Rest is the same as the default constructor.

  \param u : The horizontal coordinate of the dot's center of gravity in the
  image.

  \param v : The vertical coordinate of the dot's center of gravity in the
  image.

*/
vpDot2::vpDot2(const unsigned int u, const unsigned int v ) : vpTracker()
{
  cog_ufloat = u ;
  cog_vfloat = v ;

  width = 0;
  height = 0;
  surface = 0;
  gray_level_min = 128;
  gray_level_max = 255;
  accuracy = 0.65;

  m00 = m11 = m02 = m20 = m10 = m01 = 0 ;

  bbox_u_min = bbox_u_max = bbox_v_min = bbox_v_max = 0;

  firstBorder_u = 0;
  firstBorder_v = 0;

  compute_moment = false ;
  graphics = false;
}

/*!

  Constructor initialise the coordinates of the gravity center of the dot to
  (u,v).  Rest is the same as the default constructor.

  \param u : The horizontal coordinate of the dot's center of gravity in the
  image.

  \param v : The vertical coordinate of the dot's center of gravity in the
  image.

*/
vpDot2::vpDot2(const double u, const double v ) : vpTracker()
{
  cog_ufloat = u ;
  cog_vfloat = v ;

  width = 0;
  height = 0;
  surface = 0;
  gray_level_min = 128;
  gray_level_max = 255;
  accuracy = 0.65;

  m00 = m11 = m02 = m20 = m10 = m01 = 0 ;

  bbox_u_min = bbox_u_max = bbox_v_min = bbox_v_max = 0;

  firstBorder_u = 0;
  firstBorder_v = 0;

  compute_moment = false ;
  graphics = false;
}


/*!
  Copy contructor.
*/
vpDot2::vpDot2( const vpDot2& twinDot ) : vpTracker()
{
  *this = twinDot;
}

/*!
  = operator.
*/
void vpDot2::operator=( const vpDot2& twinDot )
{
  cog_ufloat = twinDot.cog_ufloat;
  cog_vfloat = twinDot.cog_vfloat;

  width    = twinDot.width;
  height   = twinDot.height;
  surface  = twinDot.surface;
  gray_level_min = twinDot.gray_level_min;
  gray_level_max = twinDot.gray_level_max;
  accuracy = twinDot.accuracy;

  area = twinDot.area;

  m00 = twinDot.m00;
  m01 = twinDot.m01;
  m11 = twinDot.m11;
  m10 = twinDot.m10;
  m02 = twinDot.m02;
  m20 = twinDot.m20;

  bbox_u_min = twinDot.bbox_u_min;
  bbox_u_max = twinDot.bbox_u_max;
  bbox_v_min = twinDot.bbox_v_min;
  bbox_v_max = twinDot.bbox_v_max;

  firstBorder_u = twinDot.firstBorder_u;
  firstBorder_v = twinDot.firstBorder_v;

  compute_moment = twinDot.compute_moment;
  graphics = twinDot.graphics;
}

/*!
  Destructor... do nothing for the moment.
*/
vpDot2::~vpDot2(){}


/******************************************************************************
 *
 *      PUBLIC METHODS
 *****************************************************************************/




/*!

  Initialize the tracking with a mouse click on the image and update the dot
  characteristics (center of gravity, moments) by a call to track().

  Wait a user click in a white area in the image. The clicked pixel
  will be the starting point from which the dot will be tracked.

  To get center of gravity of the dot, see get_u() and get_v(). To compute the
  moments see setComputeMoments().

  \param I : Image.

  \sa track()

*/
void vpDot2::initTracking(vpImage<unsigned char>& I)
{
  unsigned u = 0;
  unsigned v = 0;

  while ( vpDisplay::getClick(I, v, u) != true) ;

  cog_ufloat = (double) u ;
  cog_vfloat = (double) v ;

  gray_level_min = (unsigned int) (I[v][u] * accuracy);
  if (gray_level_min > 255)
    gray_level_min = 255;

  gray_level_max = (unsigned int) (I[v][u] * (2 - accuracy));
  if (gray_level_max > 255)
    gray_level_max = 255;

  setWidth(0);

  try {
    track( I );
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}

/*!

  Initialize the tracking for a dot supposed to be located at (u,v) and update
  the dot characteristics (center of gravity, moments) by a call to track().

  \param I : Image.
  \param u : Dot location (column).
  \param v : Dot location (row).

  To get center of gravity of the dot, see get_u() and get_v(). To compute the
  moments see setComputeMoments().

*/
void vpDot2::initTracking(vpImage<unsigned char>& I,
			  unsigned int u, unsigned int v)
{
  cog_ufloat = (double) u ;
  cog_vfloat = (double) v ;

  gray_level_min = (unsigned int) (I[v][u] * accuracy);
  if (gray_level_min > 255)
    gray_level_min = 255;

  gray_level_max = (unsigned int) (I[v][u] * (2 - accuracy));
  if (gray_level_max > 255)
    gray_level_max = 255;

  setWidth(0);

  try {
    track( I );
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}

/*!

  Initialize the tracking for a dot supposed to be located at (u,v) and update
  the dot characteristics (center of gravity, moments) by a call to track().

  The sub pixel coordinates of the dot are updated. To get the center
  of gravity coordinates of the dot, use get_u() and get_v(). To
  compute the moments use setComputeMoments(true) before a call to
  initTracking().

  \param I : Image to process.

  \param u : Dot location or starting point (column pixel coordinate)
  from which the dot will be tracked in the image.

  \param v : Dot location or starting point (row pixel coordinate)
  from which the dot will be tracked in the image.

  \param gray_level_min : Minimum gray level threshold used to segment the dot;
  value comprised between 0 and 255.

  \param gray_level_max : Maximum gray level threshold used to segment the
  dot; value comprised between 0 and 255. \e gray_level_max should be
  greater than \e gray_level_min.

  \sa track(), get_u(), get_v()

*/
void vpDot2::initTracking(vpImage<unsigned char>& I,
			  unsigned int u, unsigned int v,
			  unsigned int gray_level_min,
			  unsigned int gray_level_max)
{
  cog_ufloat = (double) u ;
  cog_vfloat = (double) v ;

  this->gray_level_min = gray_level_min;
  this->gray_level_max = gray_level_max;

  setWidth(0);

  try {
    track( I );
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}



/*!

  Try to locate the dot in the image:

  - First estimate the new position of the dot, using it's previous position.
  - Then compute the center of gravity (surface, width height) of the
    tracked entity from the Freeman chain elements.
  - If the dot is lost (estimated point too dark, too much surface change,...),
    search the dot in a window around the previous position.
  - If no valid dot was found in the window, return an exception.

  \param I : Image.

  \exception vpTrackingException::featureLostError : If the dot tracking
  failed.

  To get the center of gravity of the dot, call get_u() and get_v(). To get the
  with or hight of the dot, call getWidth() and getHeight(). The surface of the
  dot is given by getSurface().

  To compute all the inertia moments associated to the dot see
  setComputeMoments().

  To get the pixels coordinates on the dot boundary, see getList_u() and
  getList_v().


*/
void vpDot2::track(vpImage<unsigned char> &I)
{
  m00 = m11 = m02 = m20 = m10 = m01 = 0 ;

  // First, we will estimate the potition of the tracked point

  double estimated_u = get_u();
  double estimated_v = get_v();

  // Set the search area to the entire image
  setArea(I);

//   vpDEBUG_TRACE(0, "Previous dot: ");
//   vpDEBUG_TRACE(0, "u: %f v: %f", get_u(), get_v());
//   vpDEBUG_TRACE(0, "w: %f h: %f", getWidth(), getHeight());
  if (computeParameters(I, estimated_u, estimated_v) == false)
  {
//     vpDEBUG_TRACE(0, "Search the dot in a bigest window around the last position");
//     vpDEBUG_TRACE(0, "Bad computed dot: ");
//     vpDEBUG_TRACE(0, "u: %f v: %f", get_u(), get_v());
//     vpDEBUG_TRACE(0, "w: %f h: %f", getWidth(), getHeight());

    // if estimation was wrong (get an error tracking), look for the dot
    // closest from the estimation,
    // i.e. search for dots in an area arround the this dot and get the first
    // element in the area.

    // first get the size of the search window from the dot size
    double searchWindowWidth, searchWindowHeight;
    if( getWidth() == 0 || getHeight() == 0 )
    {
      searchWindowWidth = 80.;
      searchWindowHeight = 80.;
    }
    else
    {
      searchWindowWidth  = getWidth() * 5;
      searchWindowHeight = getHeight() * 5;
    }

    vpList<vpDot2>* candidates =
      searchDotsInArea( I,
			(int)(this->get_u()-searchWindowWidth /2.0),
			(int)(this->get_v()-searchWindowHeight/2.0),
			(int)searchWindowWidth,
			(int)searchWindowHeight);

    // if the vector is empty, that mean we didn't find any candidate
    // in the area, return an error tracking.
    if( candidates->nbElement() == 0 )
    {
      // desallocation
      candidates->kill();
      delete candidates;
      vpERROR_TRACE("No dot was found") ;
      throw(vpTrackingException(vpTrackingException::featureLostError,
				"No dot was found")) ;
    }

    // otherwise we've got our dot, update this dot's parameters
    vpDot2 movingDot = candidates->firstValue();

    set_u( movingDot.get_u() );
    set_v( movingDot.get_v() );
    setSurface( movingDot.getSurface() );
    setWidth( movingDot.getWidth() );
    setHeight( movingDot.getHeight() );
    // Update the moments
    m00 = movingDot.m00;
    m01 = movingDot.m01;
    m10 = movingDot.m10;
    m11 = movingDot.m11;
    m20 = movingDot.m20;
    m02 = movingDot.m02;

    // desallocation
    candidates->kill();
    delete candidates;
  }

  // if this dot is partially out of the image, return an error tracking.
  if( !isInImage( I ) )
  {
    vpERROR_TRACE("The center of gravity of the dot is not in the image") ;
    throw(vpTrackingException(vpTrackingException::featureLostError,
			      "No dot was found")) ;
  }

  // Get dots center of gravity
  unsigned int u = (unsigned int) this->get_u();
  unsigned int v = (unsigned int) this->get_v();

  // Updates the min and max gray levels for the next iteration
  gray_level_min = (unsigned int) (I[v][u] * accuracy);
  if (gray_level_min > 255)
    gray_level_min = 255;

  gray_level_max = (unsigned int) (I[v][u] * (2 - accuracy));
  if (gray_level_max > 255)
    gray_level_max = 255;

  if (graphics) {
    // display a red cross at the center of gravity's location in the image.

    vpDisplay::displayCross_uv(I, u,v, 15, vpColor::red);
    //vpDisplay::flush(I);
  }
}

/*!
  Track and get the new dot coordinates.

  \param I : Image
  \param u : Dot location (column)
  \param v : Dot location (row)

  This method is similar to call:
  \code
  track(I);
  u = get_u();
  v = get_v();
  \endcode

  \sa track()
*/
void
vpDot2::track(vpImage<unsigned char> &I, double &u, double &v)
{
  track(I);
  u = get_u();
  v = get_v();
}

///// GET METHODS /////////////////////////////////////////////////////////////

/*!

  Return \f$u_0\f$ the coordinate of the center of the dot along the u axis
  (horizontal).

  This value comes directly from the moments computation: \f$ u_0 =
  m_{10}/m_{00}\f$.

  \sa m00, m10
*/
double vpDot2::get_u() const
{
  return cog_ufloat;
}

/*!

  Return \f$v_0\f$ the coordinate of the center of the dot along the v axis
  (vertical).

  This value comes directly from the moments computation: \f$ v_0 =
  m_{01}/m_{00}\f$.

  \sa m00, m01

*/
double vpDot2::get_v() const
{
  return cog_vfloat;
}
/*!
  Return the width of the dot.

  \sa getHeight()
*/
double vpDot2::getWidth() const
{
  return width;
}

/*!
  Return the height of the dot.

  \sa getWidth()
*/
double vpDot2::getHeight() const
{
  return height;
}

/*!
  Return the surface of the dot.
*/
double vpDot2::getSurface() const
{
  return surface;
}

/*!

  Return the level of accuracy of informations about the dot. It is an double
  precision float witch value is in ]0,1]. 1 means full precision, wereas
  values close to 0 show a very bad accuracy.
*/
double vpDot2::getAccuracy() const
{
  return accuracy;
}


/*!
  Return the distance between the two center of dots.
*/
double vpDot2::getDistance( const vpDot2& distantDot ) const
{
  double diff_u = get_u() - distantDot.get_u();
  double diff_v = get_v() - distantDot.get_v();
  return sqrt( diff_u*diff_u + diff_v*diff_v );
}


///// SET METHODS ////////////////////////////////////////////////////////////


/*!

  Set the u (horizontal) coordinate of the dot's center of gravity in the
  image.

  \param u : Center of gravity of a dot along the horizontal axis.

  \sa set_v()
*/
void vpDot2::set_u( const double & u )
{
  cog_ufloat = u;
}

/*!

  Set the v (vertical) coordinate of the dot's center of gravity in the image.

  \param v : Center of gravity of a dot along the vertical axis.

  \sa set_u()

*/
void vpDot2::set_v( const double & v )
{
  cog_vfloat = v;
}


/*!

  Set the width of the dot. This is meant to be used to search a dot in an
  area.

  \param width : Width of a dot to search in an area.

  \sa setHeight(), setSurface(), setInLevel(), setOutLevel(), setAccuracy()
*/
void vpDot2::setWidth( const double & width )
{
  this->width = width;
}

/*!

  Set the height of the dot. This is meant to be used to search a dot in an
  area.

  \param height : Height of a dot to search in an area.

  \sa setWidth(), setSurface(), setInLevel(), setOutLevel(), setAccuracy()

*/
void vpDot2::setHeight( const double & height )
{
  this->height = height;
}

/*!

  Set the surface of the dot. This is meant to be used to search a dot in an
  area.

  \param surface : Surface of a dot to search in an area.

  \sa setWidth(), setHeight(), setInLevel(), setOutLevel(), setAccuracy()

*/
void vpDot2::setSurface( const double & surface )
{
  this->surface = surface;
}

/*!

  Set the level of accuracy of informations about the dot.

  \param accuracy : It is an double precision float which value is in ]0,1]:
  - 1 means full precision, wereas values close to 0 show a very bad accuracy.
  - Values lower or equal to 0 are brought back to an epsion>0
  - Values higher than  1 are brought back to 1

  \sa setWidth(), setHeight(), setSurface(), setInLevel(), setOutLevel()
*/
void vpDot2::setAccuracy( const double & accuracy )
{
  double epsilon = 0.05;
  if( accuracy<epsilon )
  {
    this->accuracy = epsilon;
  }
  else if( accuracy>1 )
  {
    this->accuracy = 1.0;
  }
  else
  {
    this->accuracy = accuracy;
  }
}

/*!

  Set the parameters of the area in which a dot is search to the image
  dimension.

  \param I : Image.

*/
void
vpDot2::setArea(vpImage<unsigned char> &I)
{

  setArea(I, 0, 0, I.getWidth(), I.getHeight());
}

/*!

  Set the parameters of an area by setting the upper-left corner coordinates
  (u, v), width and height.

  \param I : The image we are working with.
  \param u : Area horizontal left coordinate.
  \param v : Area vertical top coordinate.
  \param w : Area width.
  \param h : Area height.

*/
void
vpDot2::setArea(vpImage<unsigned char> &I,
		int u, int v,
		int w, int h)
{
  int image_w = I.getWidth();
  int image_h = I.getHeight();

  // Bounds the area to the image
  if (u < 0) u = 0;
  else if (u >= image_w) u = image_w - 1;
  if (v < 0) v = 0;
  else if (v >= image_h) v = image_h - 1;

  if ((u + w) > image_w) w = image_w - u - 1;
  if ((v + h) > image_h) h = image_h - v - 1;

  area.setRect(u, v, w, h);
}

/*!

  Set the parameters of the area.

  \param a : Area.

*/
void
vpDot2::setArea(const vpRect & a)
{
  area = a;
}

///// CLASS FUNCTIONALITY ////////////////////////////////////////////////////


/*!

  Look for a list of dot matching this dot parameters within the entire
  image.

  \warning Allocates memory for the list of dots returned by this method.
  Desallocation has to be done by yourself.

  \param I : Image.

  Before calling this method, dot characteristics to found have to be set like:

  \code
  vpDot2 d;

  // Set dot characteristics for the auto detection
  d.setWidth(15.0);
  d.setHeight(12.0);
  d.setSurface(124);
  d.setInLevel(164);
  d.setOutLevel(164);
  d.setAccuracy(0.65);
  \endcode

  To search dots in the whole image:
  \code
  vpList<vpDot2> * list_d;
  list_d = d.searchDotsInArea(I, 0, 0, I.getWidth(), I.getHeight()) ;
  \endcode

  The number of dots found in the area is given by:
  \code
  std::cout << list_d->nbElement();
  \endcode

  To parse all the dots:
  \code
  list_d->front();
  while (! list_d->outside()) {
    vpDot2 tmp_d;
    tmp_d = list_d->value() ; // A matching dot found in the image
    list_d->next() ;
  }
  \endcode

  To free memory associated to the list of dots:
  \code
  list_d->kill();
  delete list_d;
  \endcode

*/
vpList<vpDot2>* vpDot2::searchDotsInArea( vpImage<unsigned char>& I)
{
  vpList<vpDot2>* niceDotsVector = new vpList<vpDot2>();

  niceDotsVector = searchDotsInArea( I, 0, 0,
				     I.getWidth()-1, I.getHeight()-1 );

  return niceDotsVector;

}

/*!

  Look for a list of dot matching this dot parameters within a rectangle
  search area in the image. The rectangle upper-left coordinates are given by
  (\e area_u, \e area_v). The size of the rectangle is given by \e area_w and
  \e area_h.

  \param I : Image to process.
  \param area_u : Coordinate (column) of the upper-left area corner.
  \param area_v : Coordinate (row) of the upper-left area corner.

  \param area_w : Width or the area in which a dot is searched.
  \param area_h : Height or the area in which a dot is searched.

  \warning Allocates memory for the list of vpDot2 returned by this method.
  Desallocation has to be done by yourself, see searchDotsInArea()

  \sa searchDotsInArea(vpImage<unsigned char>& I)
*/
vpList<vpDot2>* vpDot2::searchDotsInArea( vpImage<unsigned char>& I,
					  int area_u,
					  int area_v,
					  int area_w,
					  int area_h)
{
  setArea(I, area_u, area_v, area_w, area_h);

  // compute the size of the search grid
  unsigned int gridWidth = 1;
  unsigned int gridHeight = 1;
  getGridSize( gridWidth, gridHeight );

  if (graphics) {
    // Display the area were the dot is search
    vpDisplay::displayRectangle(I, area, vpColor::blue);
    //vpDisplay::flush(I);
  }

  // start the search loop; for all points of the search grid,
  // test if the pixel belongs to a valid dot.
  // if it is so eventually add it to the vector of valid dots.
  vpList<vpDot2>* niceDotsVector = new vpList<vpDot2>();
  vpList<vpDot2>* badDotsVector  = new vpList<vpDot2>();

  vpDot2* dotToTest = NULL;

  unsigned int area_u_min = (unsigned int) area.getLeft();
  unsigned int area_u_max = (unsigned int) area.getRight();
  unsigned int area_v_min = (unsigned int) area.getTop();
  unsigned int area_v_max = (unsigned int) area.getBottom();

  unsigned int u, v;
  for( v=area_v_min ; v<area_v_max ; v=v+gridHeight )
  {
    for( u=area_u_min ; u<area_u_max ; u=u+gridWidth )
    {
      // if the pixel we're in doesn't have the right color (not white enough),
      // no need to check futher, just get to the next grid intersection.
      if( !hasGoodLevel(I, u, v) ) continue;

      // Test if an other germ is inside the bounding box of a dot previoulsy
      // detected
      bool good_germ = true;
      niceDotsVector->front();
      while( !niceDotsVector->outside() ) {
	vpDot2 tmpDot = niceDotsVector->value();
	double u0 = tmpDot.get_u();
	double v0 = tmpDot.get_v();
	double half_w = tmpDot.getWidth()  / 2.;
	double half_h = tmpDot.getHeight() / 2.;

	if ( u >= (u0-half_w) && u <= (u0+half_w) &&
	     v >= (v0-half_h) && v <= (v0+half_h) ) {
	  // Germ is in a previously detected dot
	  good_germ = false;
	}
	niceDotsVector->next();
      }

      if (! good_germ)
	continue;

      badDotsVector->front();
      while( !badDotsVector->outside() ) {
	vpDot2 tmpDot = badDotsVector->value();
	double u0 = tmpDot.get_u();
	double v0 = tmpDot.get_v();
	double half_w = tmpDot.getWidth()  / 2.;
	double half_h = tmpDot.getHeight() / 2.;

	if ( u >= (u0-half_w) && u <= (u0+half_w) &&
	     v >= (v0-half_h) && v <= (v0+half_h) ) {
	  // Germ is in a previously detected dot
	  good_germ = false;
	}
	badDotsVector->next();
      }

      if (! good_germ)
	continue;


      // otherwise estimate the width, height and surface of the dot we
      // created, and test it.
      if( dotToTest != NULL ) delete dotToTest;
      dotToTest = getInstance();
      dotToTest->set_u(u);
      dotToTest->set_v(v);
      dotToTest->setGrayLevelMin ( getGrayLevelMin()  );
      dotToTest->setGrayLevelMax ( getGrayLevelMax()  );
      dotToTest->setGraphics( graphics );
      dotToTest->setComputeMoments( true );
      dotToTest->setArea( area );

      // first compute the parameters of the dot.
      // if for some reasons this caused an error tracking
      // (dot partially out of the image...), check the next intersection
      if( dotToTest->computeParameters( I ) == false ) {
	// Jump all the pixels between v,u and v, dotToTest->getFirstBorder_u()
	u = dotToTest->getFirstBorder_u();
	v = dotToTest->getFirstBorder_v();
	continue;
      }
      // if the dot to test is valid,
      if( dotToTest->isValid( I, *this ) )
      {
	// compute the distance to the center
	double area_center_u, area_center_v;
	area.getCenter(area_center_u, area_center_v);
	double thisDiff_u = dotToTest->get_u() - area_center_u;
	double thisDiff_v = dotToTest->get_v() - area_center_v;
	double thisDist = sqrt( thisDiff_u*thisDiff_u + thisDiff_v*thisDiff_v);

	bool stopLoop = false;
	niceDotsVector->front();
	while( !niceDotsVector->outside() &&  stopLoop == false )
	{
	  vpDot2 tmpDot = niceDotsVector->value();

	  //double epsilon = 0.001; // detecte +sieurs points
	  double epsilon = 3.0;
	  // if the center of the dot is the same than the current
	  // don't add it, test the next point of the grid
	  if( fabs( tmpDot.get_u() - dotToTest->get_u() ) < epsilon &&
	      fabs( tmpDot.get_v() - dotToTest->get_v() ) < epsilon )
	  {
	    stopLoop = true;
	    // Jump all the pixels between v,u and v, tmpDot->getFirstBorder_u()
	    u = tmpDot.getFirstBorder_u();
	    v = tmpDot.getFirstBorder_v();
	    continue;
	  }

	  double otherDiff_u = tmpDot.get_u() - area_center_u;
	  double otherDiff_v = tmpDot.get_v() - area_center_v;
	  double otherDist = sqrt( otherDiff_u*otherDiff_u +
				   otherDiff_v*otherDiff_v );


	  // if the distance of the curent vector element to the center
	  // is greater than the distance of this dot to the center,
	  // then add this dot before the current vector element.
	  if( otherDist > thisDist )
	  {
	    niceDotsVector->addLeft( *dotToTest );
	    niceDotsVector->next();
	    stopLoop = true;
	    // Jump all the pixels between v,u and v, tmpDot->getFirstBorder_u()
	    u = tmpDot.getFirstBorder_u();
	    v = tmpDot.getFirstBorder_v();
	    continue;
	  }
	  niceDotsVector->next();
	}

	// if we reached the end of the vector without finding the dot
	// or inserting it, insert it now.
	if( niceDotsVector->outside() && stopLoop == false )
	{
	  niceDotsVector->end();
	  niceDotsVector->addRight( *dotToTest );
	}
      }
      else {
	// Store bad dots
	badDotsVector->front();
	badDotsVector->addRight( *dotToTest );
      }

    }
  }
  if( dotToTest != NULL ) delete dotToTest;

  delete badDotsVector;

  return niceDotsVector;
}


/*!

  Check if the this dot is "like" the wanted dot passed in.

  \return If it is so, return true, otherwise return false.

  \warning Parameters of the wanted dot (width, height, surface, in level, out
  level, accuracy) must already be set before; see
  searchDotsInArea(vpImage<unsigned char>& I)

  \param I : Image.
  \param wantedDot : Wanted dot passed in.

*/
bool vpDot2::isValid( vpImage<unsigned char>& I, const vpDot2& wantedDot )
{
  double accuracy = wantedDot.getAccuracy();
  double epsilon = 0.001;

  //
  // First, check the width, height and surface of the dot. Those parameters
  // must be the same */accuracy.
  //
  if( ( wantedDot.getWidth()*accuracy-epsilon < getWidth() ) == false )
  {
    return false;
  }

  if( ( getWidth() < wantedDot.getWidth()/accuracy+epsilon ) == false )
  {
    return false;
  }

  if( ( wantedDot.getHeight()*accuracy-epsilon < getHeight() ) == false )
  {
    return false;
  }

  if( ( getHeight() < wantedDot.getHeight()/accuracy+epsilon ) == false )
  {
    return false;
  }

  if( ( wantedDot.getSurface()*(accuracy*accuracy)-epsilon < getSurface() ) == false )
  {
    return false;
  }

  if( ( getSurface() < wantedDot.getSurface()/(accuracy*accuracy)+epsilon ) == false )
  {
    return false;
  }


  //
  // Now we can proceed to more advanced (and costy) checks.
  // First check ther is a white (>level) elipse within dot
  // Then check the dot is surrounded by a black elipse.
  //

  double innerCoef = 0.4;
  int u, v;

  for( double alpha=0. ; alpha<2*M_PI ; alpha+= 0.4 )
  {
    u = (int) ( (this->get_u() + sin( alpha )*innerCoef*getWidth()/2) );
    v = (int) ( (this->get_v() + cos( alpha )*innerCoef*getHeight()/2) );
    if (graphics) {
      vpDisplay::displayCross( I, v, u, 1, vpColor::green ) ;
      //vpDisplay::flush(I);
    }
    if( !wantedDot.hasGoodLevel( I, u, v ) )
    {
      return false;
    }

  }

  double outCoef = 1.6;
  for( double alpha=0. ; alpha<2*M_PI ; alpha+= 0.3 )
  {
    u = (int) ( (this->get_u() + sin( alpha )*outCoef*getWidth()/2) );
    v = (int) ( (this->get_v() + cos( alpha )*outCoef*getHeight()/2) );
    if (graphics) {
      vpDisplay::displayCross( I, v, u, 1, vpColor::green ) ;
      //vpDisplay::flush(I);
    }
    // If outside the area, continue
    if (u < area.getLeft() || u > area.getRight()
	|| v < area.getTop() || v > area.getBottom())
      continue;
    if( !wantedDot.hasReverseLevel( I, u, v ) )
    {
      return false;
    }
  }

  return true;
}


/*!

  Check if a the pixel of coordinates (u, v) is in the image and has
  a good level to belong to this dot.

  \param I : Image.
  \param u : Pixel to test.
  \param v : Pixel to test.

  \return true : If the pixel of coordinates (u, v) is in the area and
  has a value greater than the in level fixed by setInLevel().

  \return false : Otherwise

  \sa setInLevel()

*/
bool vpDot2::hasGoodLevel(const vpImage<unsigned char>& I,
			  const unsigned int &u,
			  const unsigned int &v) const
{
  if( !isInArea( u, v ) )
    return false;

  if( I[v][u] >= gray_level_min &&  I[v][u] <= gray_level_max)
  {
    return true;
  }
  else
  {
    return false;
  }
}


/*!

  Check if a the pixel of coordinates (u, v) in the image has a good level to
  be a dark zone arround the dot.

  \param I : Image.
  \param u : Pixel to test.
  \param v : Pixel to test.

  \return true if it is so, and false otherwise.

*/
bool vpDot2::hasReverseLevel(vpImage<unsigned char>& I,
			     const unsigned int &u,
			     const unsigned int &v) const
{

  if( I[v][u] < gray_level_min ||  I[v][u] > gray_level_max)
  {
    return true;
  }
  else
  {
    return false;
  }
}


/*!
  Return a new instance of vpDot2.
  Should be used by child classed to return their own instance of vpDot2.

  \return An instance of vpDot2.

*/
vpDot2* vpDot2::getInstance()
{
  return new vpDot2();
}



/******************************************************************************
 *
 *      PROTECTED METHODS
 *
 ******************************************************************************/



/*!

  Returns the list of Freeman elements used to turn around the dot
  counterclockwise.

  \return List of Freeman element list.

*/
vpList<int> vpDot2::getListFreemanElement()
{
  return direction_list;
}

/*!
  Returns the list of u coordinates of the pixels on the dot border.

  \return List of u coodinates of all the pixels on the dot boundary.

*/
vpList<unsigned int> vpDot2::getList_u()
{
  return u_list;
}

/*!
  Returns the list v coordinates of the pixels on the dot border.

  \return List of v coodinates of all the pixels on the dot boundary.
*/
vpList<unsigned int> vpDot2::getList_v()
{
  return v_list;
}


/******************************************************************************
 *
 *      PRIVATE METHODS
 *
 ******************************************************************************/



/*!

  Compute all the parameters of the dot (center, width, height, surface,
  inertia moments...).

  This is done the followin way:

  - First, we check the point (_u, _v) passed in has the right level in the
    image

  - Then we cross the tracked entity from left to right until we reach it's
    border.

  - We follow this border until we come back to the first point or we get to
    border of the image. Each time we update variables used to compute the
    dot parameters

  \param I : The image we are working with.

  \param _u : A pixel coordinate inside the dot.

  \param _v : A pixel coordinate inside the dot.

  \return false : If a dot can't be found around pixel coordinates given as
  parameter

  \return true : If a dot was found.

  \sa getFirstBorder_u(), getFirstBorder_v()

*/
bool vpDot2::computeParameters( vpImage<unsigned char> &I,
				const double &_u,
				const double &_v)
{
  direction_list.kill();
  u_list.kill();
  v_list.kill();

  double est_u = _u; // estimated
  double est_v = _v;

  // if u has default value, set it to the actual center value
  if( est_u == -1.0 )
  {
    est_u = this->get_u();
  }

  // if v has default value, set it to the actual center value
  if( est_v == -1.0 )
  {
    est_v = this->get_v();
  }

  // if the estimated position of the dot is out of the image, not need to continue,
  // return an error tracking
  if( !isInArea( (int) est_u, (int) est_v ) )
  {
    vpDEBUG_TRACE(3, "Initial pixel coordinates (%d, %d) for dot tracking are not in the area",
		(int) est_u, (int) est_v) ;
    return false;
  }

  bbox_u_min = I.getWidth();
  bbox_u_max = 0;
  bbox_v_min = I.getHeight();
  bbox_v_max = 0;

  // if the first point doesn't have the right level then there's no point to
  // continue.
  if( !hasGoodLevel( I, (unsigned int) est_u, (unsigned int) est_v ) )
  {
    vpDEBUG_TRACE(3, "Can't find a dot from pixel (%d, %d) coordinates",
		(int) est_u, (int) est_v) ;
    return false;
  }

  // find the border

  // NOTE:
  // from here we use int and not double. This is because we don't have
  // rounding problems and it's actually more a trouble than smth else to
  // work with double when navigating around the dot.
  this->firstBorder_u = (unsigned int) est_u;
  this->firstBorder_v = (unsigned int) est_v;
  while( hasGoodLevel( I, this->firstBorder_u+1, this->firstBorder_v ) &&
	 firstBorder_u < area.getRight()/*I.getWidth()*/ )
  {
    // if the width of this dot was initialised and we already crossed the dot
    // on more than the max possible width, no need to continue, return an
    // error tracking
    if( getWidth() > 0 && fabs( est_u - this->firstBorder_u )> getWidth()/getAccuracy() )
    {
      vpDEBUG_TRACE(3, "The found dot has a greater with than the required one") ;
      return false;
    }

    this->firstBorder_u++;
  }

  unsigned int dir = 6;

  // Determine the first element of the Freeman chain
  computeFreemanChainElement(I, this->firstBorder_u, this->firstBorder_v, dir);
  unsigned int firstDir = dir;

  // if we are now out of the image, return an error tracking
  if( !isInArea( this->firstBorder_u, this->firstBorder_v ) )
  {
    vpDEBUG_TRACE(3, "Border pixel coordinates (%d, %d) of the dot are not in the area",
		  this->firstBorder_u, this->firstBorder_v);
    return false;
  }

  // store the new direction and dot border coordinates.
  direction_list.addRight( dir );
  u_list.addRight( this->firstBorder_u );
  v_list.addRight( this->firstBorder_v );

  int border_u = this->firstBorder_u;
  int border_v = this->firstBorder_v;

//   vpTRACE("-----------------------------------------");
//   vpTRACE("first border_u: %d border_v: %d dir: %d",
// 	this->firstBorder_u, this->firstBorder_v,firstDir);
  int du, dv;
  float dS, dMu, dMv, dMuv, dMu2, dMv2;
  m00 = 0.0;
  m10 = 0.0;
  m01 = 0.0;
  m11 = 0.0;
  m20 = 0.0;
  m02 = 0.0;
  // while we didn't come back to the first point, follow the border
  do {
    // if it was asked, show the border
    if (graphics) {
      vpDisplay::displayPoint_uv(I, border_u, border_v, vpColor::red) ;
      //vpDisplay::flush(I);
    }
    // Determine the increments for the parameters
    computeFreemanParameters(border_u, border_v, dir, du, dv,
			     dS, // surface
			     dMu, dMv, // first order moments
			     dMuv, dMu2, dMv2); // second order moment

    // Update the parameters
    border_u += du; // Next position on the border
    border_v += dv;
    m00 += dS; // enclosed area
    m10 += dMu; // First order moment along v axis
    m01 += dMv; // First order moment along u axis
    if (compute_moment) {
      m11 += dMuv; // Second order moment
      m20 += dMu2; // Second order moment along v axis
      m02 += dMv2; // Second order moment along u axis
    }
    // if we are now out of the image, return an error tracking
    if( !isInArea( border_u, border_v ) )  {

      // Can Occur on a single pixel dot located on the top border
      return false;
    }

    // store the new direction and dot border coordinates.

    direction_list.addRight( dir );
    u_list.addRight( border_u );
    v_list.addRight( border_v );

    // vpDisplay::getClick(I);

    // update the extreme point of the dot.
    if( border_v < bbox_v_min ) bbox_v_min = border_v;
    if( border_v > bbox_v_max ) bbox_v_max = border_v;
    if( border_u < bbox_u_min ) bbox_u_min = border_u;
    if( border_u > bbox_u_max ) bbox_u_max = border_u;

    // move around the tracked entity by following the border.
    if (computeFreemanChainElement(I, border_u, border_v, dir) == false)
      return false;

//     vpTRACE("border_u: %d border_v: %d dir: %d", border_u, border_v, dir);

  }
  while( (this->firstBorder_u != border_u
	  || this->firstBorder_v != border_v
	  || firstDir != dir) &&
	 isInArea( border_u, border_v ) );


  // if the surface is one or zero , the center of gravity wasn't properly
  // detected. Return an error tracking.
  if( m00 == 0 || m00 == 1 )
  {
    vpDEBUG_TRACE(3, "The center of gravity of the dot wasn't properly detected");
    return false;
  }
  else // compute the center
  {
    // this magic formula gives the coordinates of the center of gravity
    double tmpCenter_u = m10 / m00;
    double tmpCenter_v = m01 / m00;

    // check the center is in the image... never know...
    if( !hasGoodLevel( I, (unsigned int)tmpCenter_u,
		       (unsigned int)tmpCenter_v ) )
    {
      vpDEBUG_TRACE(3, "The center of gravity of the dot has not a good in level");
      return false;
    }

    cog_ufloat = tmpCenter_u;
    cog_vfloat = tmpCenter_v;
  }

  width   = bbox_u_max - bbox_u_min + 1;
  height  = bbox_v_max - bbox_v_min + 1;
  surface = m00;

  return true;
}

/*!

  Considering a pixel (u, v) compute the next element of the Freeman chain
  code.

  According to the gray level of pixel (u, v) and his eight neighbors determine
  the next element of the chain in order to turn around the dot
  counterclockwise.

  \param I : The image we are working with.
  \param v : The row coordinate of a pixel on a border.
  \param u : The column coordinate of the pixel on a border.
  \param element : The next freeman element chain code (0, 1, 2, 3, 4, 5, 6, 7)
  with 0 for right moving, 2 for down, 4 for left and 6 for up moving.

  \return false if an element cannot be found. Occurs for example with an area
  constituted by a single pixel. Return true if success.
*/
bool
vpDot2::computeFreemanChainElement(const vpImage<unsigned char> &I,
				   const unsigned int &u,
				   const unsigned int &v,
				   unsigned int &element)
{

  if (hasGoodLevel( I, u, v )) {
    unsigned int _u = u;
    unsigned int _v = v;
    // get the point on the right of the point passed in
    updateFreemanPosition( _u, _v, (element + 2) %8 );
    if (hasGoodLevel( I, _u, _v )) {
      element = (element + 2) % 8;      // turn right
    }
    else {
      unsigned int _u = u;
      unsigned int _v = v;
      updateFreemanPosition( _u, _v, (element + 1) %8 );

      if ( hasGoodLevel( I, _u, _v )) {
	element = (element + 1) % 8;      // turn diag right
      }
      else {
	unsigned int _u = u;
	unsigned int _v = v;
	updateFreemanPosition( _u, _v, element ); // same direction

	if ( hasGoodLevel( I, _u, _v )) {
	  element = element;      // keep same dir
	}
	else {
	  unsigned int _u = u;
	  unsigned int _v = v;
	  updateFreemanPosition( _u, _v, (element + 7) %8 ); // diag left

	  if ( hasGoodLevel( I, _u, _v )) {
	    element = (element + 7) %8;      // turn diag left
	  }
	  else {
	    unsigned int _u = u;
	    unsigned int _v = v;
	    updateFreemanPosition( _u, _v, (element + 6) %8 ); // left

	    if ( hasGoodLevel( I, _u, _v )) {
	      element = (element + 6) %8 ;      // turn left
	    }
	    else {
	      unsigned int _u = u;
	      unsigned int _v = v;
	      updateFreemanPosition( _u, _v, (element + 5) %8 ); // left

	      if ( hasGoodLevel( I, _u, _v )) {
		element = (element + 5) %8 ;      // turn diag down
	      }
	      else {
		unsigned int _u = u;
		unsigned int _v = v;
		updateFreemanPosition( _u, _v, (element + 4) %8 ); // left

		if ( hasGoodLevel( I, _u, _v )) {
		  element = (element + 4) %8 ;      // turn down
		}
		else {
		  unsigned int _u = u;
		  unsigned int _v = v;
		  updateFreemanPosition( _u, _v, (element + 3) %8 ); // diag

		  if ( hasGoodLevel( I, _u, _v )) {
		    element = (element + 3) %8 ;      // turn diag right down
		  }
		  else {
		    // No neighbor with a good level
		    //
		    return false;
		  }
		}
	      }
	    }
	  }
	}
      }
    }
  }

  else {
    return false;
  }

  return true;

}

/*!

  Given the previous position of a pixel (u_p, v_p) on the dot border and the
  direction to reach the next pixel on the border, compute Freeman parameters.

  \param u_p : Previous value of the row coordinate of a pixel on a border.
  \param v_p : Previous value of the column coordinate of a pixel on a border.
  \param du : Increment to go from previous to next pixel on the dot border.
  \param dv : Increment to go from previous to next pixel on the dot border.

  \param dS : Enclosed area increases. Cumulated values of dS gives m00.

  \param dMu : First order moment along v axis increases. Cumulated values of
  dMu gives m10.

  \param dMv : First order moment along u axis increases. Cumulated values of
  dMv gives m01.

  \param dMuv : Moment increases. Cumulated values of dMuv gives m11.

  \param dMu2 : Second order moment along v axis increases. Cumulated values of
  dMu2 gives m20.

  \param dMv2 : Second order moment along u axis increases. Cumulated values of
  dMv2 gives m02.

  Considering the previous coordinates (u_p, v_p) of a pixel on a border, the
  next coordinates (u, v) are given by: u = u_p + du and v = v_p + dv


*/
void
vpDot2::computeFreemanParameters(const int &u_p,
				 const int &v_p,
				 unsigned int &element,
				 int &du, int &dv,
				 float &dS,
				 float &dMu, float &dMv,
				 float &dMuv,
				 float &dMu2, float &dMv2)
{
  du = 0;
  dv = 0;
  dMuv = 0;
  dMu2 = 0;
  dMv2 = 0;

  switch(element) {
  case 0:
    du = 1;
    dS = (float) v_p;
    dMu = 0.0;
    dMv = (float)(0.5 * v_p * v_p);
    if (compute_moment) {
      dMuv = (float)(0.25 * v_p * v_p * (2 * u_p + 1));
      dMu2 = 0;
      dMv2 = (float)(1.0/ 3. * v_p * v_p * v_p);
    }
    break;

  case 1:
    du = 1;
    dv = 1;
    dS = (float)(v_p + 0.5);
    dMu = - (float)(0.5 * u_p * ( u_p + 1 ) + 1.0 / 6.0);
    dMv = (float)(0.5 * v_p * ( v_p + 1 ) + 1.0 / 6.0);
    if (compute_moment) {
      float half_u_p = (float)(0.5*u_p);
      dMuv = (float)(v_p*v_p*(0.25+half_u_p) + v_p*(1./3.+half_u_p) + 1./6.*u_p +0.125);
      dMu2 = (float)(-1./3. * u_p * (u_p * u_p + 1.5 * u_p  + 1.) - 1./12.0);
      dMv2 = (float)( 1./3. * v_p * (v_p * v_p + 1.5 * v_p  + 1.) + 1./12.0);
    }
    break;

  case 2:
    dv = 1;
    dS = 0.0;
    dMu = (float)(- 0.5 * u_p *  u_p);
    dMv = 0.0;
    if (compute_moment) {
      dMuv = 0;
      dMu2 = (float)(-1.0/ 3. * u_p * u_p * u_p);
      dMv2 = 0;
    }
    break;

  case 3:
    du = -1;
    dv = 1;
    dS = (float)(- v_p - 0.5);
    dMu = - (float)(0.5 * u_p * ( u_p - 1 ) + 1.0 / 6.0);
    dMv = - (float)(0.5 * v_p * ( v_p + 1 ) + 1.0 / 6.0);
    if (compute_moment) {
      float half_u_p = (float)(0.5*u_p);
      dMuv = (float)(v_p*v_p*(0.25-half_u_p) + v_p*(1./3.-half_u_p) - 1./6.*u_p +0.125);
      dMu2 = (float)(-1./3. * u_p * (u_p * u_p - 1.5 * u_p  + 1.) - 1./12.0);
      dMv2 = (float)(-1./3. * v_p * (v_p * v_p + 1.5 * v_p  + 1.) - 1./12.0);
    }
    break;

  case 4:
    du = -1;
    dS = (float)(- v_p);
    dMv = (float)(- 0.5 * v_p * v_p);
    dMu = 0.0;
    if (compute_moment) {
      dMuv = (float)(-0.25 * v_p * v_p * (2 * u_p - 1));
      dMu2 = 0;
      dMv2 = (float)(-1.0/ 3. * v_p * v_p * v_p);
    }
    break;

  case 5:
    du = -1;
    dv = -1;
    dS = (float)(- v_p + 0.5);
    dMu = (float)(   0.5 * u_p * ( u_p - 1 ) + 1.0 / 6.0);
    dMv = (float)(- (0.5 * v_p * ( v_p - 1 ) + 1.0 / 6.0));
    if (compute_moment) {
      float half_u_p = (float)(0.5*u_p);
      dMuv = (float)(v_p*v_p*(0.25-half_u_p) - v_p*(1./3.-half_u_p) - 1./6.*u_p +0.125);
      dMu2 = (float)( 1./3. * u_p * (u_p * u_p - 1.5 * u_p  + 1.) - 1./12.0);
      dMv2 = (float)(-1./3. * v_p * (v_p * v_p - 1.5 * v_p  + 1.) - 1./12.0);
    }
    break;

  case 6:
    dv = -1;
    dS = 0.0;
    dMu = (float)(0.5 * u_p *  u_p);
    dMv = 0.0;
    if (compute_moment) {
      dMuv = 0;
      dMu2 = (float)(1.0/ 3. * u_p * u_p * u_p);
      dMv2 = 0;
    }
    break;

  case 7:
    du = 1;
    dv = -1;
    dS = (float)(v_p - 0.5);
    dMu = (float)(0.5 * u_p * ( u_p + 1 ) + 1.0 / 6.0);
    dMv = (float)(0.5 * v_p * ( v_p - 1 ) + 1.0 / 6.0);
    if (compute_moment) {
      float half_u_p = (float)(0.5*u_p);
      dMuv = (float)(v_p*v_p*(0.25+half_u_p) - v_p*(1./3.+half_u_p) + 1./6.*u_p +0.125);
      dMu2 = (float)(1./3. * u_p * (u_p * u_p + 1.5 * u_p  + 1.) + 1./12.0);
      dMv2 = (float)(1./3. * v_p * (v_p * v_p - 1.5 * v_p  + 1.) - 1./12.0);
    }
    break;
  }
}


/*!

  From a pixel coordinate and a direction, update the pixel coordinates after
  moving forward.

  \param v : The row coordinate of the pixel, updated by this method.

  \param u : The column coordinate of the pixel, updated by this method.

  \param dir : The direction in the image, 0=right, 1, 2=down, 3, 4=left, 5,
  6=up and 7.

*/
void vpDot2::updateFreemanPosition( unsigned int& u, unsigned int& v,
				    const unsigned int &dir )
{
  switch(dir) {
  case 0: u += 1;         break;
  case 1: u += 1; v += 1; break;
  case 2: v += 1;         break;
  case 3: u -= 1; v += 1; break;
  case 4: u -= 1;         break;
  case 5: u -= 1; v -= 1; break;
  case 6: v -= 1;         break;
  case 7: u += 1; v -= 1; break;
  }
}

/*!

  Test if a pixel is in the image. Points of the border are not considered to
  be in the image.  Call the isInImage( vpImage<unsigned char> &I, int u, int
  v) method.

  \param I : The image.


  \return true if the pixel of coordinates (posI, posJ) is in the image and
  false otherwise.
*/
bool vpDot2::isInImage( vpImage<unsigned char> &I) const
{
  return isInImage( I, (int)this->get_u(), (int)this->get_v());
}



/*!

  Test if a pixel is in the image. Points of the border are not considered to
  be in the image.

  \param I : The image.
  \param u : The column coordinate of the pixel.
  \param v : The row coordinate of the pixel .

  \return true if the pixel of coordinates (u, v) is in the image and false
  otherwise.
*/
bool vpDot2::isInImage( vpImage<unsigned char> &I,
			const int &u, const int &v) const
{
  int height = I.getHeight();
  int width = I.getWidth();
  if( u < 0 || u >= width ) return false;
  if( v < 0 || v >= height ) return false;
  return true;
}

/*!

  Test if a pixel is in an area. Points of the border are not considered to
  be in the area.

  \param u : The column coordinate of the pixel.
  \param v : The row coordinate of the pixel .

  \return true if the pixel of coordinates (u, v) is in the image and false
  otherwise.
*/
bool vpDot2::isInArea( const unsigned int &u, const unsigned int &v) const
{
  unsigned int area_u_min = (unsigned int) area.getLeft();
  unsigned int area_u_max = (unsigned int) area.getRight();
  unsigned int area_v_min = (unsigned int) area.getTop();
  unsigned int area_v_max = (unsigned int) area.getBottom();

  if( u < area_u_min || u > area_u_max ) return false;
  if( v < area_v_min || v > area_v_max ) return false;
  return true;
}


/*!

  Get the search grid size used to found a dot in an area. This grid is used to
  parse only some pixels of the search area.

  \param gridWidth : Number of pixels between to vertical lines of the grid

  \param gridHeight : Number of pixels between to horizontal lines of the grid


*/
void vpDot2::getGridSize( unsigned int &gridWidth, unsigned int &gridHeight )
{
  // first get the research grid width and height Note that
  // 1/sqrt(2)=cos(pi/4). The grid squares should be small enough to be
  // contained in the dot. We gent this here if the dot is a perfect disc.
  // More accurate criterium to define the grid should be implemented if
  // necessary
  gridWidth = (unsigned int) (getWidth() * getAccuracy() / sqrt(2.));
  gridHeight = (unsigned int) (getHeight() * getAccuracy() / sqrt(2.0));

  if( gridWidth == 0 ) gridWidth = 1;
  if( gridHeight == 0 ) gridHeight = 1;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
