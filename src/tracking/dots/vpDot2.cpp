/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpDot.cpp
 * Project:   ViSP2
 * Author:    Fabien Spindler
 *
 * Version control
 * ===============
 *
 *  $Id: vpDot2.cpp,v 1.7 2005-12-07 15:54:28 fspindle Exp $
 *
 * Description
 * ============
 *   Track a white dot
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*
  \file vpDot2.cpp
  \brief Track a white dot
*/

/*!
  \class vpDot2


  \brief This vpTracker is meant to track some white zones on a vpImage.

  The center of gravity of a vpDot2 white zone have to be of the right color
  level. track() and searchDotsInArea() are the most important features
  of this class.

  - track() estimate the current position of the dot using it's previous speed
    and acceleratio, then try to compute the new parameters of the dot. It
    everything went ok, tracking succeded, otherwise we search this dot in an
    window.

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

/* __DEBUG_LEVEL_ fixed by configure */
#define __DEBUG_LEVEL_ 0 // 3


//! minumum level for the dot, pixel with lower level
//! don't belong to this dot.
const int vpDot2::MIN_IN_LEVEL = 70;

//! Default level for the pixels inside the dot
const int vpDot2::DEFAULT_IN_LEVEL = 220;

//! Default level for the pixels surrounding the dot
const int vpDot2::DEFAULT_OUT_LEVEL = 140;

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
  inLevel = 210;
  outLevel = 150;
  accuracy = 0.65;

  m00 = m11 = m02 = m20 = m10 = m01 = 0 ;

  compute_moment = false ;
  graphics = false;
}

/*!
  Constructor initialise the coordinates of the gravity center of the dot to (i,j).
  Rest is the same as the default constructor.

  \param u The horizontal coordinate of the dot's center of gravity in the image.
  \param v The vertical coordinate of the dot's center of gravity in the image.

*/
vpDot2::vpDot2( int u, int v ) : vpTracker()
{
  cog_ufloat = u ;
  cog_vfloat = v ;

  width = 0;
  height = 0;
  surface = 0;
  inLevel = 210;
  outLevel = 150;
  accuracy = 0.65;

  m00 = m11 = m02 = m20 = m10 = m01 = 0 ;

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

  width = twinDot.width;
  height = twinDot.height;
  surface = twinDot.surface;
  inLevel = twinDot.inLevel;
  outLevel = twinDot.outLevel;
  accuracy = twinDot.accuracy;

  m00 = twinDot.m00;
  m01 = twinDot.m01;
  m10 = twinDot.m10;
  m02 = twinDot.m02;
  m20 = twinDot.m20;

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
 *
 ******************************************************************************/




/*!

  Initialize the tracking with a mouse click.

  Wait for the user to click in a white zone in the image. This point will be a
  point of the dot.

*/
void vpDot2::initTracking(vpImage<unsigned char>& I)
{
  int u = 0;
  int v = 0;

  while ( vpDisplay::getClick(I, v, u) != true) ;

  cog_ufloat = (double) u ;
  cog_vfloat = (double) v ;

  inLevel  = (int) (I[v][u] * accuracy);
  outLevel = (int) (I[v][u] * accuracy);

  if ( inLevel < MIN_IN_LEVEL ) inLevel = MIN_IN_LEVEL;

  setWidth(0);

  try {
    track( I );
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!

  Initialize the tracking for a dot supposed to be located at (u,v).

  \param I Image
  \param u Dot location (column)
  \param v Dot location (row)

*/
void vpDot2::initTracking(vpImage<unsigned char>& I, int u, int v)
{
  cog_ufloat = (double) u ;
  cog_vfloat = (double) v ;

  inLevel  = (int) (I[v][u] * accuracy);
  outLevel = (int) (I[v][u] * accuracy);

  if ( inLevel < MIN_IN_LEVEL ) inLevel = MIN_IN_LEVEL;

  setWidth(0);

  try {
    track( I );
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}



/*!

  Try to locate the dot in the image:

  - First estimate the new position of the Dot, using position, speed and
    acceleration at the last time.

  - Then compute the center of gravity (and the surface, width height) of the
    tracked entity, using the estimated position.

  - If the dot is lost ( estimated point too dark, too much speed of the center
    of gravity, too much volume change,...), search the dot in a window.

  - If no valid dot was found in the window, return an exception.

*/
void vpDot2::track(vpImage<unsigned char> &I)
{
  if (__DEBUG_LEVEL_ & 1)
    cout << "Start vpDot2::Track()" << endl;

  if (compute_moment)
    m00 = m11 = m02 = m20 = m10 = m01 = 0 ;

  // First, we will estimate the potition of the tracked point

  double estimated_u = get_u();
  double estimated_v = get_v();

//   if ( ! isInImage( I, (int)estimated_u, (int)estimated_v ) ) {
//     ERROR_TRACE("Dot (%d, %d) to track is not in the image",
// 		(int)estimatedI, (int)estimatedJ) ;
//     throw(vpTrackingException(vpTrackingException::featureLostError,
// 			      "Dot has been lost")) ;
//   }

  if (computeParameters(I, estimated_u, estimated_v) == false)
  {
    DEBUG_TRACE(0, "Search the dot in a bigest window around the last position");

    // if estimation was wrong (get an error tracking), look for the dot
    // closest from the estimation,
    // i.e. search for dots in an area arround the this dot and get the first
    // element in the area.

    // first get the size of the search window from the dot size
    int searchWindowWidth, searchWindowHeight;
    if( getWidth() == 0 || getHeight() == 0 )
    {
      searchWindowWidth = 80;
      searchWindowHeight = 80;
    }
    else
    {
      searchWindowWidth = (int)getWidth() * 5;
      searchWindowHeight = (int)getHeight() * 5;
    }

    vpList<vpDot2>* candidates =
      searchDotsInArea( I,
			(int)this->get_u()-searchWindowWidth/2,
			(int)this->get_v()-searchWindowHeight/2,
			searchWindowWidth,
			searchWindowHeight);

    // if the vector is empty, that mean we didn't find any candidate
    // in the area, return an error tracking.
    if( candidates->nbElement() == 0 )
    {
      // desallocation
      candidates->kill();
      delete candidates;
      ERROR_TRACE("No dot was found") ;
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

    // desallocation
    candidates->kill();
    delete candidates;
  }

  // if this dot is partially out of the image, return an error tracking.
  if( !isInImage( I ) )
  {
    ERROR_TRACE("The center of gravity of the dot is not in the image") ;
    throw(vpTrackingException(vpTrackingException::featureLostError,
			      "No dot was found")) ;
  }

  // Updates the in and out levels for the next iteration
  setInLevel ( (int) (I[(int)get_v()][(int)get_u()] * accuracy) );
  //setOutLevel( (int) (I[(int)this->I()][(int)this->J()] / accuracy) );
  setOutLevel( (int) (I[(int)get_v()][(int)get_u()] * accuracy) );

  if (graphics) {
    // display a red cross at the center of gravity's location in the image.

    vpDisplay::displayCross_uv(I, (int)get_u(), (int) get_v(), 15,
			       vpColor::red) ;
    vpDisplay::flush(I);
  }


  if (__DEBUG_LEVEL_ & 1)
    cout << "End vpDot2::Track()" << endl;
}

/*!
  Track and get the new dot coordinates

  \param I Image
  \param u Dot location (column)
  \param v Dot location (row)
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

  Return the "u" (column) coordinate of the center of the dot within the image
  it comes from.
*/
double vpDot2::get_u() const
{
  return cog_ufloat;
}

/*!

  Return the "v" (row) coordinate of the center of the dot within the image it
  comes from.
*/
double vpDot2::get_v() const
{
  return cog_vfloat;
}
/*!
  Return the width of the dot.
*/
double vpDot2::getWidth() const
{
  return width;
}

/*!
  Return the height of the dot.
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
  Return the level of points inside the dot.
*/
int vpDot2::getInLevel() const
{
  return inLevel;
}

/*!
  Return the level of pixels outside the dot.
*/
int vpDot2::getOutLevel() const
{
  return outLevel;
}

/*!

  Return the level of accuracy of informations about the dot". It is an double
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
  Set the u (horizontal) coordinate of the dot's center of gravity in the image.
*/
void vpDot2::set_u( const double & u )
{
  cog_ufloat = u;
}

/*!
  Set the v (vertical) coordinate of the dot's center of gravity in the image.
*/
void vpDot2::set_v( const double & v )
{
  cog_vfloat = v;
}


/*!
  Set the width of the dot. This is meant t be used to search a dot in an area.
*/
void vpDot2::setWidth( const double & width )
{
  this->width = width;
}

/*!
  Set the height of the dot. This is meant t be used to search a dot in an area.
*/
void vpDot2::setHeight( const double & height )
{
  this->height = height;
}

/*!
  Set the surface of the dot. This is meant t be used to search a dot in an area.
*/
void vpDot2::setSurface( const double & surface )
{
  this->surface = surface;
}

/*!

  Set the color level of the dot. This level will be used to know if a pixel in
  the image belongs to the dot or not. Only pixels with higher level can belong
  to the dot.  If the level is lower than the minimum level for a dot, set the
  level to MIN_IN_LEVEL.
*/
void vpDot2::setInLevel( const int & inLevel )
{
  if (__DEBUG_LEVEL_ & 1)
    cout << "Start vpDot2::setInLevel(" << inLevel << ")" << endl;

  if (inLevel > MIN_IN_LEVEL) {
    this->inLevel = inLevel;
  }
  else {
    this->inLevel = MIN_IN_LEVEL;
  }
  if (__DEBUG_LEVEL_ & 1)
    cout << "inLevel fixed to: " << this->inLevel << endl;

  if (__DEBUG_LEVEL_ & 1)
    cout << "End vpDot2::setInLevel()" << endl;
}


/*!
  set the color level ok pixel surrounding the dot.
*/
void vpDot2::setOutLevel( const int & outLevel )
{
  this->outLevel = outLevel;
}


/*!
  set the level of accuracy of informations about the dot". It is an double precision float witch value is in ]0,1]. 1 means full precision, wereas values close to 0 show a very bad accuracy.
  Values lower or equal to 0 are brought back to an epsion>0
  Values higher than 1 are brought back to 1
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


///// CLASS FUNCTIONALITY ////////////////////////////////////////////////////


/*!

  Look for a list of dot matching this dot parameters within the entire
  image.

  \warning Allocates memory for the list of vpDot2 returned by this method.
  Desallocation has to be done by yourself.


*/
vpList<vpDot2>* vpDot2::searchDotsInArea( vpImage<unsigned char>& I)
{
  vpList<vpDot2>* niceDotsVector = new vpList<vpDot2>();

  niceDotsVector = searchDotsInArea( I, 0, 0,
				     I.getCols()-1, I.getRows()-1 );

  return niceDotsVector;

}

/*!

  Look for a list of dot matching this dot parameters within a rectangle
  search area in the image. The rectangle upper-left coordinates are given by
  (\e corner_u, \e corner_v). The size of the rectangle is given by \e with and
  \e height.

  \param area_u Coordinate (column) of the upper-left area corner.
  \param area_v Coordinate (row) of the upper-left area corner.

  \param area_width Width or the area in which a dot is searched.
  \param area_height Height or the area in which a dot is searched.

  \warning Allocates memory for the list of vpDot2 returned by this method.
  Desallocation has to be done by yourself.
*/
vpList<vpDot2>* vpDot2::searchDotsInArea( vpImage<unsigned char>& I,
					  int area_u, int area_v,
					  int area_width, int area_height)
{
  if (__DEBUG_LEVEL_ & 1)
    cout << "Start vpDot2::searchDotsInArea()" << endl;

  if( area_u < 0 ) area_u = 0;
  if( area_v < 0 ) area_v = 0;
  if( area_u + area_width  >= I.getCols() ) area_width  = I.getCols()-area_u-1;
  if( area_v + area_height >= I.getRows() ) area_height = I.getRows()-area_v-1;


  // compute the size of the search grid
  int gridWidth = 1;
  int gridHeight = 1;
  getGridSize( gridWidth, gridHeight );

  if (graphics) {
    // Display the area were the dot is search
    vpDisplay::displayRectangle_uv(I, area_u, area_v,
				   area_width, area_height, vpColor::blue);
    vpDisplay::flush(I);
  }
  // compute the area center, we need it later in the loop
  double areaCenter_u = area_u + area_width/2.;
  double areaCenter_v = area_v + area_height/2.;

  // start the search loop; for all points of the search grid,
  // test if the pixel belongs to a valid dot.
  // if it is so eventually add it to the vector of valid dots.
  vpList<vpDot2>* niceDotsVector = new vpList<vpDot2>();

  if (__DEBUG_LEVEL_ & 1) {
    cout << "Actual inlevel: " << getInLevel()
	 << " outlevel: " << getOutLevel() << endl;
  }
  vpDot2* dotToTest = NULL;
  for( int v=area_v ; v<area_v+area_height ; v=v+gridHeight )
  {
    for( int u=area_u ; u<area_u+area_width ; u=u+gridWidth )
    {
      // if the pixel we're in doesn't have the right color (not white enough), no
      // need to check futher, just get to the next grid intersection.
      if( !hasGoodLevel(I, u, v) ) continue;

      // otherwise estimate the width, height and surface of the dot we created,
      // and test it.
      if( dotToTest != NULL ) delete dotToTest;
      dotToTest = getInstance();
      dotToTest->set_u(u);
      dotToTest->set_v(v);
      dotToTest->setInLevel ( (int) getInLevel()  );
      dotToTest->setOutLevel( (int) getOutLevel() );
      dotToTest->setGraphics( graphics );


      // first comput the parameters of the dot.
      // if for some reasons this caused an error tracking
      // (dot partially out of the image...), check the next intersection
      if( dotToTest->computeParameters( I ) == false ) {
	continue;
      }
      // if the dot to test is valid,
      if( dotToTest->isValid( I, *this ) )
      {
	// compute the distance to the center
	double thisDiff_u = dotToTest->get_u() - areaCenter_u;
	double thisDiff_v = dotToTest->get_v() - areaCenter_v;
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
	    continue;
	  }

	  double otherDiff_u = tmpDot.get_u() - areaCenter_u;
	  double otherDiff_v = tmpDot.get_v() - areaCenter_v;
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

    }
  }
  if( dotToTest != NULL ) delete dotToTest;



  if (__DEBUG_LEVEL_ & 1)
    cout << "end vpDot2::searchDotsInArea()" << endl;

  return niceDotsVector;
}


/*!
  Check if the this dot is "like" the wanted dot passed in.
  If it is so, return true, otherwise return false.
  Note: Parameters the dot ( center, width, height, etc...) must already be known, eventually with the computeParameters(...) option.
*/
bool vpDot2::isValid( vpImage<unsigned char>& I, const vpDot2& wantedDotParam )
{
  if (__DEBUG_LEVEL_ & 1) cout << "Start vpDot2::isValid()" << endl;

  double accuracy = wantedDotParam.getAccuracy();
  double epsilon = 0.001;

  //
  // First, check the width, height and surface of the dot. Those parameters
  // must be the same */accuracy.
  //
  if (__DEBUG_LEVEL_ & 2) {
    cout << "Carateristique du dot precedent:" << endl;
    cout << " width= " << getWidth()
	 << " height=" << getHeight()
	 << " surface= " << getSurface() << endl;
    cout << "Carateristique du dot a tester:" << endl;
    cout << " width= " << wantedDotParam.getWidth()
	 << " height=" << wantedDotParam.getHeight()
	 << " surface= " << wantedDotParam.getSurface() << endl;
  }

  if( ( wantedDotParam.getWidth()*accuracy-epsilon < getWidth() ) == false )
  {
    return false;
  }

  if( ( getWidth() < wantedDotParam.getWidth()/accuracy+epsilon ) == false )
  {
    return false;
  }

  if( ( wantedDotParam.getHeight()*accuracy-epsilon < getHeight() ) == false )
  {
    return false;
  }

  if( ( getHeight() < wantedDotParam.getHeight()/accuracy+epsilon ) == false )
  {
    return false;
  }

  if( ( wantedDotParam.getSurface()*(accuracy*accuracy)-epsilon < getSurface() ) == false )
  {
    return false;
  }

  if( ( getSurface() < wantedDotParam.getSurface()/(accuracy*accuracy)+epsilon ) == false )
  {
    return false;
  }


  //
  // Now we can procede to more advanced (and costy) checks.
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
      vpDisplay::flush(I);
    }
    if( !wantedDotParam.hasGoodLevel( I, u, v ) )
    {
      return false;
    }

  }

  double outCoef = 1.4;
  for( double alpha=0. ; alpha<2*M_PI ; alpha+= 0.3 )
  {
    u = (int) ( (this->get_u() + sin( alpha )*outCoef*getWidth()/2) );
    v = (int) ( (this->get_v() + cos( alpha )*outCoef*getHeight()/2) );
    if (graphics) {
      vpDisplay::displayCross( I, v, u, 1, vpColor::green ) ;
      vpDisplay::flush(I);
    }
    if( !wantedDotParam.hasReverseLevel( I, u, v ) )
    {
      return false;
    }
  }

  if (__DEBUG_LEVEL_ & 1) cout << "End vpDot2::isValid()" << endl;

  return true;
}


/*!

  Check if a the pixel of coordinates (u, v) is in the image and has
  a good level to belong to this dot. Return true if it is so, and false
  otherwise.

  \return true If the pixel coordinates (u, v) are in the image and
  has a value greater than the in level fixed by setInLevel().

  \return false Otherwise

  \sa isInImage(), setInLevel()

*/
bool vpDot2::hasGoodLevel(vpImage<unsigned char>& I,
			  const int &u, const int &v) const
{
  if( !isInImage( I, u, v ) )
  {
    return false;
  }

  if( I[v][u] > inLevel )
  {
    return true;
  }
  else
  {
    return false;
  }
}


/*!

  Check if a the pixel of coordinates (u, v) in the image has good level to be
  a dark zone arround the dot. Return true if it is so, and false otherwise.
*/
bool vpDot2::hasReverseLevel(vpImage<unsigned char>& I,
			     const int &u, const int &v) const
{

  if( I[v][u] < outLevel )
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
  Should be used by clild classed to return their own instance of vpDot2.
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
  Returns the list of directions used to turn around the dot.
*/
vpList<int> vpDot2::getList_directions()
{
  return direction_list;
}

/*!
  Returns the list u coordinates of the dot border
*/
vpList<int> vpDot2::getList_u()
{
  return u_list;
}

/*!
  Returns the list v coordinates of the dot border
*/
vpList<int> vpDot2::getList_v()
{
  return v_list;
}


/******************************************************************************
 *
 *      PRIVATE METHODS
 *
 ******************************************************************************/



/*!

  Compute the center of the dot.

  This is done the followin way:
  - First, we check the point passed in have the right level in the image

  - Then we cross the tracked entity from left to right until we reach it's
    border

  - We follow this border until we come back to the first point or we get to
    border of the image. Each time we update variables used to compute the
    center of gravity

  \param I The image we are working with.

  \param u The estimated (column) coordinated of the center of
  gravity of the dot in the image.

  \param v The estimated (row) coordinated of the center of gravity
  of the dot in the image.

  \return false If a dot can't be found around pixel coordinates given as
  parameter

  \retunn true If a dot was found.

*/
bool vpDot2::computeParameters( vpImage<unsigned char> &I,
				const double &_u,
				const double &_v)
{
  //#define NEW_CALCUL

  if (__DEBUG_LEVEL_ & 1)
    cout << "Begin vpDot2::computeParameters()" << endl;

  direction_list.kill();
  u_list.kill();
  v_list.kill();

  double newSurface = 0.;
  int path_u = 0;
  int path_v = 0;
  double sum_u = 0.;
  double sum_v = 0.;
  double est_u = _u; // estimated
  double est_v = _v;
#ifdef NEW_CALCUL
  double sumP_u = 0.;
  double sumP_v = 0.;
#endif

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
  if( !isInImage( I, (int) est_u, (int) est_v ) )
  {
    DEBUG_TRACE(3, "Initial pixel coordinates (%d, %d) for dot tracking are not in the image",
		(int) est_u, (int) est_v) ;
    return false;
  }

  int leftestPix = I.getCols();
  int rightestPix = 0;
  int rightPixHeight = 0;
  int highestPix = I.getRows();
  int lowestPix = 0;

  // if the first point doesn't have the right level then there's no point to
  // continue.
  if( !hasGoodLevel( I, (int) est_u, (int) est_v ) )
  {
    DEBUG_TRACE(3, "Can't find a dot from pixel (%d, %d) coordinates",
		(int) est_u, (int) est_v) ;
    return false;
  }

  // find the border

  // NOTE:
  // from here we use int and not double. This is because we don't have
  // rounding problems and it's actually more a trouble than smth else to
  // work with double when navigating around the dot.
  int firstBorder_u = (int) est_u;
  int firstBorder_v = (int) est_v;
  while( hasGoodLevel( I, firstBorder_u+1, firstBorder_v ) &&
	 firstBorder_u < I.getCols() )
  {
    // if the width of this dot was initialised and we already crossed the dot
    // on more than the max possible width, no need to continue, return an
    // error tracking
    if( getWidth() > 0 && fabs( est_u - firstBorder_u )> getWidth()/getAccuracy() )
    {
      DEBUG_TRACE(3, "The found dot has a greater with than the required one") ;
      return false;
    }

    firstBorder_u++;
  }

  int firstDir = 6;
  int dir = firstDir;
  // procede to the first move, to get on track....
  move( I, firstBorder_u, firstBorder_v, firstDir);

  // if we are now out of the image, return an error tracking
  if( !isInImage( I, firstBorder_u, firstBorder_v ) )
  {
    DEBUG_TRACE(3, "Border pixel coordinates (%d, %d) of the dot are not in the image",
		firstBorder_u, firstBorder_v);
    return false;
  }

  // store the new direction and dot border coordinates.

  direction_list.addRight( dir );
  u_list.addRight( firstBorder_u );
  v_list.addRight( firstBorder_v );


  int border_u = firstBorder_u;
  int border_v = firstBorder_v;
  path_u = firstBorder_u;
  path_v = firstBorder_v;

//   TRACE("Debut loop");
//   cout << "pathI: " << pathI << " pathJ: " << pathJ << endl;

  // while we didn't come back to the first point, follow the border
  do {
    // if it was asked, show the border
    if (graphics) {
      vpDisplay::displayPoint_uv(I, path_u, path_v, vpColor::red) ;
      vpDisplay::flush(I);
    }

    // move around the tracked entity by following the border.
    move( I, border_u, border_v, dir);

    // if we are now out of the image, return an error tracking
    if( !isInImage( I, border_u, border_v ) )
    {
      // Modif FS 12/05/2005: If we rich a border, turn right
      // turnRight( borderI, borderJ, dir );
      move( I, border_u, border_v, dir);

      // Did not work if the dot is close to a border
      // return ERR_TRACKING;
    }

    // store the new direction and dot border coordinates.

    direction_list.addRight( dir );
    u_list.addRight( border_u );
    v_list.addRight( border_v );


    // update the extreme point of the dot.
    if( border_v < highestPix ) highestPix = border_v;
    if( border_v > lowestPix  ) lowestPix  = border_v;
    if( border_u < leftestPix ) leftestPix = border_u;
    if( border_u > rightestPix) {
      rightestPix    = border_u;
      rightPixHeight = border_v;
    }

    //    TRACE("dir: %d", dir);

    // according to the new direction, update the informations used
    // to compute the center.
#ifdef NEW_CALCUL
    switch(dir) {
    case 0: // we are going right
      {
	path_u ++;
	sumP_v += path_v;
	sum_v  += ((path_v+1)*path_v)/2.;
	break;
      }
    case 2: // we are going up
      {
	path_v --;
	sumP_u -= path_u;
	sum_u  -= ((path_u+1)*path_u)/2.;
	break;
      }
    case 4: // we are going left
      {
	path_u --;
	sumP_v -= path_v;
	sum_v  -= ((path_v+1)*path_v)/2.;
	break;
      }
    case 6: // we are going down
      {
	path_v ++;
	sumP_u += path_u;
	sum_u  += ((path_u+1)*path_u)/2.;
	break;
      }
    }
//     TRACE("");
//     cout << "path_v: " << path_v << " path_u: " << path_u << endl;
//     cout << "sum_v: " << sum_v << " sum_u: " << sum_u
// 	 << "sumP_v: " << sumP_v << " sumP_u: " << sumP_u << endl;
#else
    switch(dir) {
    case 0: // we are going right
      {
	path_u += 1;

	newSurface = newSurface - path_v; // +
	sum_v = sum_v - ((path_v+1)*path_v)/2.; // +

	break;
      }
    case 2: // we are going up
      {
	path_v -= 1;

	sum_u = sum_u - ((path_u+1)*path_u)/2.;
	break;
      }
    case 4: // we are going left
      {
	path_u -= 1;

	newSurface = newSurface + path_v;
	sum_v = sum_v + ((path_v+1)*path_v)/2.;
	break;
      }
    case 6: // we are going down
      {
	path_v += 1;

	sum_u = sum_u + ((path_u+1)*path_u)/2.;
	break;
      }
    }
//     TRACE("");
//     cout << "path_v: " << path_v << " path_u: " << path_u << endl;
//     cout << "sum_v: " << sum_v << " sum_u: " << sum_u
// 	 << " newSurface: " << newSurface << endl;
#endif

//     vpDisplay::getClick(I);
  }
  while( (firstBorder_u != border_u
	  || firstBorder_v != border_v
	  || firstDir != dir) &&
	 isInImage( I, border_u, border_v ) );

  // if the the surface is negative, we turned around a black dot.
  // it may be a dark pixel in the white dot...
  // get over it and compute the parameters again.
  // that way we are sure to reach the border of the dot... or the border of the
  // image.
#ifdef NEW_CALCUL
  if( sumP_v < 0  || sumP_u < 0)
#else
  if( newSurface < 0 )
#endif
  {
    return computeParameters( I, rightPixHeight, rightestPix+1 );
  }



  // if the surface is one or zero , the center of gravity wasn't properly
  // detected. Return an error tracking.
#ifdef NEW_CALCUL
  if( sumP_v == 0.  || sumP_u == 0.)
#else
  if( newSurface == 0 || newSurface == 1 )
#endif
  {
    DEBUG_TRACE(3, "The center of gravity of the dot wasn't properly detected");
    return false;
  }
  else // compute the center
  {
    // this magic formula gives the coordinates of the center of gravity
#ifdef NEW_CALCUL
    double tmpCenter_v = fabs( sum_v/sumP_v );
    double tmpCenter_u = fabs( sum_u/sumP_u );
#else
    double tmpCenter_v = fabs( sum_v/newSurface );
    double tmpCenter_u = fabs( sum_u/newSurface );
#endif

    // check the center is in the image... never know...
    if( !hasGoodLevel( I, (int)tmpCenter_u, (int)tmpCenter_v ) )
    {
      DEBUG_TRACE(3, "The center of gravity of the dot has not a good in level");
      return false;
    }

    cog_ufloat = tmpCenter_u;
    cog_vfloat = tmpCenter_v;
  }

  width   = rightestPix - leftestPix;
  height  = lowestPix   - highestPix;
  surface = newSurface;

  // Computes the moments
  if (compute_moment) {
    m00 = surface;
    m01 = sum_v;
    m10 = sum_u;
  }

  if (__DEBUG_LEVEL_ & 1)
    cout << "End vpDot2::computeParameters()" << endl;
  return true;
}


/*!
  Move around the tracked dot clockwise.

  We use the direction to get the point left to the point passed in. Then,
  according to the gray level of the two pixels, we move around the dot
  clockwise.  If the new point is out of the image, return an error tracking.
*/
void vpDot2::move( vpImage<unsigned char> &I,
		   int& u, int& v, int& dir)
{
  // get the point on the left of the point passed in
  int leftPoint_u = u;
  int leftPoint_v = v;
  getLeftPixelPosition( leftPoint_u, leftPoint_v, dir );

  // compare the value of the image at both points, and move
  if( !hasGoodLevel( I, u, v ) &&
      !hasGoodLevel( I, leftPoint_u, leftPoint_v ) )
  {
    turnRight( u, v, dir );
    dir = (dir+6)%8;
  }
  else if( hasGoodLevel( I, u, v ) &&
	   !hasGoodLevel( I, leftPoint_u, leftPoint_v ) )
  {
    getTop( u, v, dir );
  }
  else if( hasGoodLevel( I, u, v ) &&
	   hasGoodLevel( I, leftPoint_u, leftPoint_v ) )
  {
    turnLeft( u, v, dir );
    dir = (dir+2)%8;
  }
  else
  {
    turnRight( u, v, dir );
    dir = (dir+6)%8;
  }
}


/*!

  From a pixel coordinate and a direction, gives the coordinates of the left
  pixel.

  \param v The row coordinate of the pixel, updated by this method.
  \param u The column coordinate of the pixel, updated by this method.
  \param dir The direction in the image, 0=right, 2=up, 4=left, 6=down.

*/
void vpDot2::getLeftPixelPosition( int& u, int& v, const int &dir )
{
  switch(dir)
  {
  case 0:
    {
      v -= 1;
      break;
    }
  case 2:
    {
      u -= 1;
      break;
    }
  case 4:
    {
      v += 1;
      break;
    }
  case 6:
    {
      u += 1;
      break;
    }
  default:{}
  }
}


/*!

  From a pixel coordinates and a direction, get coordinates of the pixel after
  turning left.

  \param v The row coordinate of the pixel, updated by this method.
  \param u The column coordinate of the pixel, updated by this method.
  \param dir The direction in the image, 0=right, 2=up, 4=left, 6=down.
*/
void vpDot2::turnLeft( int& u, int& v, const int &dir )
{
  switch(dir)
  {
  case 0:
    {
      v -= 2;
      break;
    }
  case 2:
    {
      u -= 2;
      break;
    }
  case 4:
    {
      v += 2;
      break;
    }
  case 6:
    {
      u += 2;
      break;
    }
  default:{}
  }
}

/*!

  From a pixel coordinate and a direction, get coordinates of the pixel after
  turning right.

  \param v The row coordinate of the pixel, updated by this method.
  \param u The column coordinate of the pixel, updated by this method.
  \param dir The direction in the image, 0=right, 2=up, 4=left, 6=down.
*/
void vpDot2::turnRight( int& u, int& v, const int &dir )
{
  switch(dir)
  {
  case 0:
    {
      v += 1;
      u -= 1;
      break;
    }
  case 2:
    {
      v += 1;
      u += 1;
      break;
    }
  case 4:
    {
      v -= 1;
      u += 1;
      break;
    }
  case 6:
    {
      v -= 1;
      u -= 1;
      break;
    }
  default:
    {
    }
  }
}

/*!
  From a pixel coordinate and a direction, get the pixel after moving forward.

  \param v The row coordinate of the pixel, updated by this method.
  \param u The column coordinate of the pixel, updated by this method.
  \param dir The direction in the image, 0=right, 2=up, 4=left, 6=down.
*/
void vpDot2::getTop( int& u, int& v, const int &dir )
{
  switch(dir)
  {
  case 0:
    {
      u += 1;
      break;
    }
  case 2:
    {
      v -= 1;
      break;
    }
  case 4:
    {
      u -= 1;
      break;
    }
  case 6:
    {
      v += 1;
      break;
    }
  default:
    {
    }
  }
}

/*!
  Test if a pixel is in the image. Points of the border are not considered to be in the image.
  Call the isInImage( vpImage<unsigned char> &I, int i, int j, int border ) method.

  \param I The image.
  \param border The size of the border, points of the border are considered out of the image.

  \return true if the pixel of coordinates (posI, posJ) is in the image and false otherwise.
*/
bool vpDot2::isInImage( vpImage<unsigned char> &I, const int &border ) const
{
  return isInImage( I, (int)this->get_u(), (int)this->get_v(), border );
}



/*!
  Test if a pixel is in the image. Points of the border are not considered to be in the image.

  \param I The image.
  \param u The column coordinate of the pixel.
  \param v The row coordinate of the pixel .
  \param border The size of the border, points of the border are considered out of the image.

  \return true if the pixel of coordinates (i, j) is in the image and false otherwise.
*/
bool vpDot2::isInImage( vpImage<unsigned char> &I,
			const int &u, const int &v, const int &border ) const
{
  int nbRows = I.getRows();
  int nbCols = I.getCols();
  if( u <= border || u >= nbCols-border ) return false;
  if( v <= border || v >= nbRows-border ) return false;
  return true;
}



/*!
  Compute the size of size a search grid have to be to be sure to find this
  dot in an area.
*/
void vpDot2::getGridSize( int &gridWidth, int &gridHeight )
{
  // first get the research grid width and height
  // Note that 1/sqrt(2)=cos(pi/4). The grid squares should be small enough
  // to be contained in the dot. We gent this here if the dot is a perfect disc.
  // More accurate criterium to define the grid should be implemented if necessary
  gridWidth = (int) (getWidth() * getAccuracy() / sqrt(2.));
  gridHeight = (int) (getHeight() * getAccuracy() / sqrt(2.0));

  if( gridWidth == 0 ) gridWidth = 1;
  if( gridHeight == 0 ) gridHeight = 1;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
