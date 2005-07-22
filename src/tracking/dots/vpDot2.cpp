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
 *  $Id: vpDot2.cpp,v 1.3 2005-07-22 10:36:56 fspindle Exp $
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

//! max dot I speed, in pixels per milliseconds
const double vpDot2::MAX_DOT_I_SPEED = 1.;
//! max dot J speed, in pixels per milliseconds
const double vpDot2::MAX_DOT_J_SPEED = 1.;
//! max change of volume rate for the dot per milliseconds
const double vpDot2::MAX_VOLUME_CHANGE = 0.005;

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
  this->i = 0;
  this->j = 0;

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

  \param i The vertical coordinate of the dot's center of gravity in the image.
  \param j The horizontal coordinate of the dot's center of gravity in the image.
*/
vpDot2::vpDot2( int i, int j ) : vpTracker()
{
  this->i = i;
  this->j = j;

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
  this->i = twinDot.i;
  this->j = twinDot.j;

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



///// GET METHODS /////////////////////////////////////////////////////////////



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

  double estimatedI = this->I();
  double estimatedJ = this->J();

//   if ( ! isInImage( I, (int)estimatedI, (int)estimatedJ ) ) {
//     ERROR_TRACE("Dot (%d, %d) to track is not in the image",
// 		(int)estimatedI, (int)estimatedJ) ;
//     throw(vpTrackingException(vpTrackingException::featureLostError,
// 			      "Dot has been lost")) ;
//   }

  if (computeParameters(I, estimatedI, estimatedJ) == false)
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
			(int)this->I()-searchWindowHeight/2,
			(int)this->J()-searchWindowWidth/2,
			searchWindowHeight,
			searchWindowWidth);

    // if the vector is empty, that mean we didn't find any candidate
    // in the area, return an error tracking.
    if( candidates->nbElement() == 0 )
    {
      ERROR_TRACE("No dot was found") ;
      throw(vpTrackingException(vpTrackingException::featureLostError,
				"No dot was found")) ;
    }

    // otherwise we've got our dot, update this dot's parameters
    vpDot2 movingDot = candidates->firstValue();

    setI( movingDot.I() );
    setJ( movingDot.J() );
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
  setInLevel ( (int) (I[(int)this->I()][(int)this->J()] * accuracy) );
  //setOutLevel( (int) (I[(int)this->I()][(int)this->J()] / accuracy) );
  setOutLevel( (int) (I[(int)this->I()][(int)this->J()] * accuracy) );
  // display a red cross at the center of gravity's location in the image.
  // vpDisplay::displayCross(I, (int)this->I(), (int)this->J(), 15, vpColor::red) ;
  //vpDisplay::flush(I);

  if (__DEBUG_LEVEL_ & 1)
    cout << "End vpDot2::Track()" << endl;
}


/*!
  Return the "i" (vertical) coordinate of the center of the dot within the image it comes from.
*/
double vpDot2::I() const
{
  return i;
}

/*!
  Return the "j" (horizontal) coordinate of the center of the dot within the image it comes from.
*/
double vpDot2::J() const
{
  return j;
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
  Return the level of accuracy of informations about the dot". It is an double precision float witch value is in ]0,1]. 1 means full precision, wereas values close to 0 show a very bad accuracy.
*/
double vpDot2::getAccuracy() const
{
  return accuracy;
}


/*!
  Reimplemented from vpTracker.
*/
int vpDot2::getNumberOfInformation()
{
  return 2;
}

/*!
  Reimplemented from vpTracker.
*/
vpColVector vpDot2::getState()
{
  vpColVector state(2);
  state[0] = I();
  state[1] = J();
  return state;
}

/*!
  Return the distance between the two center of dots.
*/
double vpDot2::getDistance( const vpDot2& distantDot ) const
{
  double diffI = I() - distantDot.I();
  double diffJ = J() - distantDot.J();
  return sqrt( diffI*diffI + diffJ*diffJ );
}


///// SET METHODS ////////////////////////////////////////////////////////////


/*!
  Set the i (vertical) coordinate of the dot's center of gravity in the image.
*/
void vpDot2::setI( double iCoord )
{
  this->i = iCoord;
}

/*!
  Set the j (horizontal) coordinate of the dot's center of gravity in the image.
*/
void vpDot2::setJ( double jCoord )
{
  this->j = jCoord;
}

/*!
  Set the width of the dot. This is meant t be used to search a dot in an area.
*/
void vpDot2::setWidth( double width )
{
  this->width = width;
}

/*!
  Set the height of the dot. This is meant t be used to search a dot in an area.
*/
void vpDot2::setHeight( double height )
{
  this->height = height;
}

/*!
  Set the surface of the dot. This is meant t be used to search a dot in an area.
*/
void vpDot2::setSurface( double surface )
{
  this->surface = surface;
}

/*!
  Set the color level of the dot. This level will be used to know if a pixel in the image belongs to the dot or not. Only pixels with higher level can belong to the dot.
  If the level is lower than the minimum level for a dot, set the level to MIN_IN_LEVEL.
*/
void vpDot2::setInLevel( int inLevel )
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
void vpDot2::setOutLevel( int outLevel )
{
  this->outLevel = outLevel;
}


/*!
  set the level of accuracy of informations about the dot". It is an double precision float witch value is in ]0,1]. 1 means full precision, wereas values close to 0 show a very bad accuracy.
  Values lower or equal to 0 are brought back to an epsion>0
  Values higher than 1 are brought back to 1
*/
void vpDot2::setAccuracy( double accuracy )
{
  double epsilon = 0.05;
  if( accuracy<epsilon )
  {
    accuracy = epsilon;
  }
  else if( accuracy>1 )
  {
    accuracy = 1.0;
  }
  else
  {
    this->accuracy = accuracy;
  }
}


///// CLASS FUNCTIONALITY ////////////////////////////////////////////////////


/*!

  Initialize the tracking.

  Wait for the user to click in a white zone in the image. This point will be a
  point of the dot.

*/
void vpDot2::initTracking(vpImage<unsigned char>& I)
{
  int intI = 0;
  int intJ = 0;

  while ( vpDisplay::getClick(I, intI,intJ) != true) ;

  i = intI;
  j = intJ;

  inLevel  = (int) (I[(int)i][(int)j] * accuracy);
  outLevel = (int) (I[(int)i][(int)j] * accuracy);

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

  Look for a list of dot matching this dot parameters within the entire
  image.

  \warning Allocates memory for the list of vpDot2 returned by this method.
  Desallocation has to be done by yourself.


*/
vpList<vpDot2>* vpDot2::searchDotsInArea( vpImage<unsigned char>& I)
{
  vpList<vpDot2>* niceDotsVector = new vpList<vpDot2>();

  niceDotsVector = searchDotsInArea( I, 0, 0,
				     I.getRows()-1, I.getCols()-1);

  return niceDotsVector;

}

/*!

  Look for a list of dot matching this dot parameters within a rectangle
  search area in the image. The rectangle upper-left coordinates are given by
  (\e areaI, \e areaJ). The size of the rectangle is given by \e areaHeight and
  \e areaWidth.

  \warning Allocates memory for the list of vpDot2 returned by this method.
  Desallocation has to be done by yourself.
*/
vpList<vpDot2>* vpDot2::searchDotsInArea( vpImage<unsigned char>& I,
					  int areaI, int areaJ,
					  int areaHeight, int areaWidth)
{
  if (__DEBUG_LEVEL_ & 1)
    cout << "Start vpDot2::searchDotsInArea()" << endl;

  if( areaI < 0 ) areaI = 0;
  if( areaJ < 0 ) areaJ = 0;
  if( areaI + areaHeight >= I.getRows() ) areaHeight = I.getRows()-areaI-1;
  if( areaJ + areaWidth >= I.getCols() ) areaWidth = I.getCols()-areaJ-1;


  // compute the size of the search grid
  int gridWidth = 1;
  int gridHeight = 1;
  getGridSize( gridWidth, gridHeight );


  if (graphics) {
    // Display the area were the dot is search
    vpDisplay::displayRectangle(I, areaI, areaJ,
				areaWidth, areaHeight, vpColor::blue);
    vpDisplay::flush(I);
  }
  // compute the area center, we need it later in the loop
  double areaCenterI = areaI + areaHeight/2.;
  double areaCenterJ = areaJ + areaWidth/2.;

  // start the search loop; for all points of the search grid,
  // test if the pixel belongs to a valid dot.
  // if it is so eventually add it to the vector of valid dots.
  vpList<vpDot2>* niceDotsVector = new vpList<vpDot2>();

  if (__DEBUG_LEVEL_ & 1) {
    cout << "Actual inlevel: " << getInLevel()
	 << " outlevel: " << getOutLevel() << endl;
  }
  vpDot2* dotToTest = NULL;
  for( int i=areaI ; i<areaI+areaHeight ; i=i+gridHeight )
  {
    for( int j=areaJ ; j<areaJ+areaWidth ; j=j+gridWidth )
    {
      // if the pixel we're in doesn't have the right color (not white enough), no
      // need to check futher, just get to the next grid intersection.
      if( !hasGoodLevel(I,i,j) ) continue;

      // otherwise estimate the width, height and surface of the dot we created,
      // and test it.
      if( dotToTest != NULL ) delete dotToTest;
      dotToTest = getInstance();
      dotToTest->setI(i);
      dotToTest->setJ(j);
      dotToTest->setInLevel ( (int) getInLevel()  );
      dotToTest->setOutLevel( (int) getOutLevel() );


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
	double thisDiffI = dotToTest->I() - areaCenterI;
	double thisDiffJ = dotToTest->J() - areaCenterJ;
	double thisDist = sqrt( thisDiffI*thisDiffI + thisDiffJ*thisDiffJ );

	bool stopLoop = false;
	niceDotsVector->front();
	while( !niceDotsVector->outside() &&  stopLoop == false )
	{
	  vpDot2 tmpDot = niceDotsVector->value();

	  //double epsilon = 0.001; // detecte +sieurs points
	  double epsilon = 3.0;
	  // if the center of the dot is the same than the current
	  // don't add it, test the next point of the grid
	  if( fabs( tmpDot.I() - dotToTest->I() ) < epsilon &&
	      fabs( tmpDot.J() - dotToTest->J() ) < epsilon )
	  {
	    stopLoop = true;
	    continue;
	  }

	  double otherDiffI = tmpDot.I() - areaCenterI;
	  double otherDiffJ = tmpDot.J() - areaCenterJ;
	  double otherDist = sqrt( otherDiffI*otherDiffI +
				   otherDiffJ*otherDiffJ );


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
  int iCoord, jCoord;
  for( double alpha=0. ; alpha<2*M_PI ; alpha+= 0.4 )
  {
    iCoord = (int) ( (this->I() + cos( alpha )*innerCoef*getHeight()/2) );
    jCoord = (int) ( (this->J() + sin( alpha )*innerCoef*getWidth()/2) );
    if (graphics) {
      vpDisplay::displayCross( I, iCoord, jCoord, 1, vpColor::green ) ;
      vpDisplay::flush(I);
    }
    if( !wantedDotParam.hasGoodLevel( I, iCoord, jCoord ) )
    {
      return false;
    }

  }

  double outCoef = 1.4;
  for( double alpha=0. ; alpha<2*M_PI ; alpha+= 0.3 )
  {
    iCoord = (int) ( (this->I() + cos( alpha )*outCoef*getHeight()/2) );
    jCoord = (int) ( (this->J() + sin( alpha )*outCoef*getWidth()/2) );
    if (graphics) {
      vpDisplay::displayCross( I, iCoord, jCoord, 1, vpColor::green ) ;
      vpDisplay::flush(I);
    }
    if( !wantedDotParam.hasReverseLevel( I, iCoord, jCoord ) )
    {
      return false;
    }
  }

  if (__DEBUG_LEVEL_ & 1) cout << "End vpDot2::isValid()" << endl;

  return true;
}


/*!

  Check if a the pixel of coordinates (iCoord, jCoord) is in the image and has
  a good level to belong to this dot. Return true if it is so, and false
  otherwise.

  \return true If the pixel coordinates (iCoord, jCoord) are in the image and
  has a value greater than the in level fixed by setInLevel().

  \return false Otherwise

  \sa isInImage(), setInLevel()

*/
bool vpDot2::hasGoodLevel( vpImage<unsigned char>& I,
			   int iCoord,
			   int jCoord ) const
{
  if( !isInImage( I, iCoord, jCoord ) )
  {
    return false;
  }

  if( I[iCoord][jCoord] > inLevel )
  {
    return true;
  }
  else
  {
    return false;
  }
}


/*!
  Check if a  the pixel of coordinates (iCoord, jCoord) in the image has good level to be a dark zone arround the dot. Return true if it is so, and false otherwise.
*/
bool vpDot2::hasReverseLevel( vpImage<unsigned char>& I,
                                  int iCoord,
                                  int jCoord ) const
{

  if( I[iCoord][jCoord] < outLevel )
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
vpList<int> vpDot2::getDirections()
{
  return directions;
}

/*!
  Returns the list i Coordinates of the dot border
*/
vpList<int> vpDot2::getICoords()
{
  return iCoords;
}

/*!
  Returns the list j Coordinates of the dot border
*/
vpList<int> vpDot2::getJCoords()
{
  return jCoords;
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

  \param iEstimated The estimated I coordinated of the center of gravity of the
  dot int the image.

  \param jEstimated The estimated J coordinated of the center of gravity of the
  dot in the image.

  \return false If a dot can't be found around pixel coordinates given as
  parameter

  \retunn true If a dot was found.

*/
bool vpDot2::computeParameters( vpImage<unsigned char> &I,
				double iEstimated,
				double jEstimated)
{
  //#define NEW_CALCUL

  if (__DEBUG_LEVEL_ & 1)
    cout << "Begin vpDot2::computeParameters()" << endl;

  directions.kill();
  iCoords.kill();
  jCoords.kill();

  double newSurface = 0.;
  int pathI = 0;
  int pathJ = 0;
  double sumI = 0.;
  double sumJ = 0.;
#ifdef NEW_CALCUL
  double sumPI = 0.;
  double sumPJ = 0.;
#endif

  // if iEstimated has default value, set it to the actual center value
  if( iEstimated == -1 )
  {
    iEstimated = this->I();
  }

  // if jEstimated has default value, set it to the actual center value
  if( jEstimated == -1 )
  {
    jEstimated = this->J();
  }

  // if the estimated position of the dot is out of the image, not need to continue,
  // return an error tracking
  if( !isInImage( I, (int)iEstimated, (int)jEstimated ) )
  {
    DEBUG_TRACE(3, "Initial pixel coordinates (%d, %d) for dot tracking are not in the image",
		(int)iEstimated, (int)jEstimated) ;
    return false;
  }

  int leftestPix = I.getCols();
  int rightestPix = 0;
  int rightPixHeight = 0;
  int highestPix = I.getRows();
  int lowestPix = 0;

  // if the first point doesn't have the right level then there's no point to
  // continue.
  if( !hasGoodLevel( I, (int)iEstimated, (int)jEstimated ) )
  {
    DEBUG_TRACE(3, "Can't find a dot from pixel (%d, %d) coordinates",
		(int)iEstimated, (int)jEstimated) ;
    return false;
  }

  // find the border

  // NOTE:
  // from here we use int and not double. This is because we don't have
  // rounding problems and it's actually more a trouble than smth else to
  // work with double when navigating around the dot.
  int firstBorderI = (int)iEstimated;
  int firstBorderJ = (int)jEstimated;
  while( hasGoodLevel( I, firstBorderI, firstBorderJ+1 ) &&
	 firstBorderJ < I.getCols() )
  {
    // if the width of this dot was initialised and we already crossed the dot
    // on more than the max possible width, no need to continue, return an
    // error tracking
    if( getWidth() > 0 && fabs( jEstimated - firstBorderJ )> getWidth()/getAccuracy() )
    {
      DEBUG_TRACE(3, "The found dot has a greater with than the required one") ;
      return false;
    }

    firstBorderJ++;
  }

  int firstDir = 6;
  int dir = firstDir;
  // procede to the first move, to get on track....
  move( I, firstBorderI, firstBorderJ, firstDir);

  // if we are now out of the image, return an error tracking
  if( !isInImage( I, firstBorderI, firstBorderJ ) )
  {
    DEBUG_TRACE(3, "Border pixel coordinates (%d, %d) of the dot are not in the image",
		firstBorderI, firstBorderJ);
    return false;
  }

  // store the new direction and dot border coordinates.

  directions.addRight( dir );
  iCoords.addRight( firstBorderI );
  jCoords.addRight( firstBorderJ );


  int borderI = firstBorderI;
  int borderJ = firstBorderJ;
  pathI = firstBorderI;
  pathJ = firstBorderJ;

//   TRACE("Debut loop");
//   cout << "pathI: " << pathI << " pathJ: " << pathJ << endl;

  // while we didn't come back to the first point, follow the border
  do {
    // if it was asked, show the border
    if (graphics) {
      vpDisplay::displayPoint(I, pathI, pathJ, vpColor::red) ;
      vpDisplay::flush(I);
    }

    // move around the tracked entity by following the border.
    move( I, borderI, borderJ, dir);

    // if we are now out of the image, return an error tracking
    if( !isInImage( I, borderI, borderJ ) )
    {
      // Modif FS 12/05/2005: If we rich a border, turn right
      // turnRight( borderI, borderJ, dir );
      move( I, borderI, borderJ, dir);

      // Did not work if the dot is close to a border
      // return ERR_TRACKING;
    }

    // store the new direction and dot border coordinates.

    directions.addRight( dir );
    iCoords.addRight( borderI );
    jCoords.addRight( borderJ );


    // update the extreme point of the dot.
    if( borderI < highestPix ) highestPix = borderI;
    if( borderI > lowestPix ) lowestPix = borderI;
    if( borderJ < leftestPix ) leftestPix = borderJ;
    if( borderJ > rightestPix ) {
      rightestPix = borderJ;
      rightPixHeight = borderI;
    }

    //    TRACE("dir: %d", dir);

    // according to the new direction, update the informations used
    // to compute the center.
#ifdef NEW_CALCUL
    switch(dir) {
    case 0: // we are going right
      {
	pathJ ++;
	sumPI += pathI;
	sumI  += ((pathI+1)*pathI)/2.;
	break;
      }
    case 2: // we are going up
      {
	pathI --;
	sumPJ -= pathJ;
	sumJ  -= ((pathJ+1)*pathJ)/2.;
	break;
      }
    case 4: // we are going left
      {
	pathJ --;
	sumPI -= pathI;
	sumI  -= ((pathI+1)*pathI)/2.;
	break;
      }
    case 6: // we are going down
      {
	pathI ++;
	sumPJ += pathJ;
	sumJ  += ((pathJ+1)*pathJ)/2.;
	break;
      }
    }
//     TRACE("");
//     cout << "pathI: " << pathI << " pathJ: " << pathJ << endl;
//     cout << "sumI: " << sumI << " sumJ: " << sumJ
// 	 << "sumPI: " << sumPI << " sumPJ: " << sumPJ << endl;
#else
    switch(dir) {
    case 0: // we are going right
      {
	pathJ += 1;

	newSurface = newSurface - pathI; // +
	sumI = sumI - ((pathI+1)*pathI)/2.; // +

	break;
      }
    case 2: // we are going up
      {
	pathI -= 1;

	sumJ = sumJ - ((pathJ+1)*pathJ)/2.;
	break;
      }
    case 4: // we are going left
      {
	pathJ -= 1;

	newSurface = newSurface + pathI;
	sumI = sumI + ((pathI+1)*pathI)/2.;
	break;
      }
    case 6: // we are going down
      {
	pathI += 1;

	sumJ = sumJ + ((pathJ+1)*pathJ)/2.;
	break;
      }
    }
//     TRACE("");
//     cout << "pathI: " << pathI << " pathJ: " << pathJ << endl;
//     cout << "sumI: " << sumI << " sumJ: " << sumJ
// 	 << " newSurface: " << newSurface << endl;
#endif

//     vpDisplay::getClick(I);
  }
  while( (firstBorderI!=borderI || firstBorderJ!=borderJ || firstDir!=dir) &&
	 isInImage( I, borderI, borderJ ) );

  // if the the surface is negative, we turned around a black dot.
  // it may be a dark pixel in the white dot...
  // get over it and compute the parameters again.
  // that way we are sure to reach the border of the dot... or the border of the
  // image.
#ifdef NEW_CALCUL
  if( sumPI < 0  || sumPJ < 0)
#else
  if( newSurface < 0 )
#endif
  {
    return computeParameters( I, rightPixHeight, rightestPix+1 );
  }



  // if the surface is one or zero , the center of gravity wasn't properly
  // detected. Return an error tracking.
#ifdef NEW_CALCUL
  if( sumPI == 0.  || sumPJ == 0.)
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
    double tmpCenterI = fabs( sumI/sumPI );
    double tmpCenterJ = fabs( sumJ/sumPJ );
#else
    double tmpCenterI = fabs( sumI/newSurface );
    double tmpCenterJ = fabs( sumJ/newSurface );
#endif

    // check the center is in the image... never know...
    if( !hasGoodLevel( I, (int)tmpCenterI, (int)tmpCenterJ ) )
    {
      DEBUG_TRACE(3, "The center of gravity of the dot has not a good in level");
      return false;
    }

    this->i = tmpCenterI;
    this->j = tmpCenterJ;
  }

  width = rightestPix - leftestPix;
  height = lowestPix - highestPix;
  surface = newSurface;

  // Computes the moments
  if (compute_moment) {
    m00 = surface;
    m01 = sumI;
    m10 = sumJ;
  }

  if (__DEBUG_LEVEL_ & 1)
    cout << "End vpDot2::computeParameters()" << endl;
  return true;
}


/*!
  Move around the tracked dot clockwise
  We use the direction to get the point left to the point passed in. Then, according to the gray level of the two pixels, we move around the dot clockwise.
  If the new point is out of the image, return an error tracking
*/
void vpDot2::move( vpImage<unsigned char> &I,
		   int& posI, int& posJ, int& dir)
{
  // get the point on the left of the point passed in
  int leftPointI = posI;
  int leftPointJ = posJ;
  getLeftPixelPosition( leftPointI, leftPointJ, dir );

  // compare the value of the image at both points, and move
  if( !hasGoodLevel( I, posI, posJ ) &&
      !hasGoodLevel( I, leftPointI, leftPointJ ) )
  {
    turnRight( posI, posJ, dir );
    dir = (dir+6)%8;
  }
  else if( hasGoodLevel( I, posI, posJ ) &&
	   !hasGoodLevel( I, leftPointI, leftPointJ ) )
  {
    getTop( posI, posJ, dir );
  }
  else if( hasGoodLevel( I, posI, posJ ) &&
	   hasGoodLevel( I, leftPointI, leftPointJ ) )
  {
    turnLeft( posI, posJ, dir );
    dir = (dir+2)%8;
  }
  else
  {
    turnRight( posI, posJ, dir );
    dir = (dir+6)%8;
  }
}


/*!
  From a pixel coordinate and a direction, gives the coordinates of the left pixel.
  \param posI The I coordinate of the pixel, updated by this method.
  \param posJ The J coordinate of the pixel, updated by this method.
  \param dir The direction in the image, 0=right, 2=up, 4=left, 6=down.
*/
void vpDot2::getLeftPixelPosition( int& posI, int& posJ, int dir )
{
  switch(dir)
  {
  case 0:
    {
      posI-=1;
      break;
    }
  case 2:
    {
      posJ-=1;
      break;
    }
  case 4:
    {
      posI+=1;
      break;
    }
  case 6:
    {
      posJ+=1;
      break;
    }
  default:{}
  }
}


/*!
  From a pixel coordinates and a direction, get coordinates of the pixel after turning left.
  \param posI The I coordinate of the pixel, updated by this method.
  \param posJ The J coordinate of the pixel, updated by this method.
  \param dir The direction in the image, 0=right, 2=up, 4=left, 6=down.
*/
void vpDot2::turnLeft( int& posI, int& posJ, int dir )
{
  switch(dir)
  {
  case 0:
    {
      posI-=2;
      break;
    }
  case 2:
    {
      posJ-=2;
      break;
    }
  case 4:
    {
      posI+=2;
      break;
    }
  case 6:
    {
      posJ+=2;
      break;
    }
  default:{}
  }
}

/*!
  From a pixel coordinate and a direction,  get coordinates of the pixel after turning right.
  \param posI The I coordinate of the pixel, updated by this method.
  \param posJ The J coordinate of the pixel, updated by this method.
  \param dir The direction in the image, 0=right, 2=up, 4=left, 6=down.
*/
void vpDot2::turnRight( int& posI, int& posJ, int dir )
{
  switch(dir)
  {
  case 0:
    {
      posI+=1;
      posJ-=1;
      break;
    }
  case 2:
    {
      posI+=1;
      posJ+=1;
      break;
    }
  case 4:
    {
      posI-=1;
      posJ+=1;
      break;
    }
  case 6:
    {
      posI-=1;
      posJ-=1;
      break;
    }
  default:
    {
    }
  }
}

/*!
  From a pixel coordinate and a direction, get the pixel after moving forward.
  \param posI The I coordinate of the pixel, updated by this method.
  \param posJ The J coordinate of the pixel, updated by this method.
  \param dir The direction in the image, 0=right, 2=up, 4=left, 6=down.
*/
void vpDot2::getTop( int& posI, int& posJ, int dir )
{
  switch(dir)
  {
  case 0:
    {
      posJ+=1;
      break;
    }
  case 2:
    {
      posI-=1;
      break;
    }
  case 4:
    {
      posJ-=1;
      break;
    }
  case 6:
    {
      posI+=1;
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
bool vpDot2::isInImage( vpImage<unsigned char> &I, int border ) const
{
  return isInImage( I, (int)this->I(), (int)this->J(), border );
}



/*!
  Test if a pixel is in the image. Points of the border are not considered to be in the image.

  \param I The image.
  \param i The I coordinate of the pixel.
  \param j The J coordinate of the pixel.
  \param border The size of the border, points of the border are considered out of the image.

  \return true if the pixel of coordinates (i, j) is in the image and false otherwise.
*/
bool vpDot2::isInImage( vpImage<unsigned char> &I, int i, int j, int border ) const
{
  int nbRows = I.getRows();
  int nbCols = I.getCols();
  if( i <= border || i >= nbRows-border ) return false;
  if( j <= border || j >= nbCols-border ) return false;
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
