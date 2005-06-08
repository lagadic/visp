/*!
  \file vpMeTracker.h
  \name Distance
*/
// ===================================================================
/*!
 *\class vpMeTracker
 *\brief 2D state = list of points, 3D state = feature
 *\n		 Contains abstract elements for a Distance to Feature type feature.
 *\author Andrew Comport
 *\date 4/7/03
 */
// ===================================================================


#ifndef vpMeTracker_HH
#define vpMeTracker_HH


#include <math.h>
#include <iostream>

#include <visp/vpColVector.h>
#include <visp/vpMeSite.h>
#include <visp/vpMe.h>
#include <visp/vpList.h>
#include <visp/vpTracker.h>


class vpMeTracker : public vpTracker
{

public:

  //! Tracking dependent variables/functions =====================
  //! \todo create list of points and move tracking dependent
  //! functionality into a seperate class i.e DistanceEcm

  //! List of tracked points
  vpList<vpMeSite> list ;
  //! Ecm initialisation parameters
  vpMe *me ;
  //! Used for backwards compatibility...could be removed
  int nGoodElement;
  int query_range;
  double seuil;
  bool display_point;// if 1 (TRUE) displays the line that is being tracked

  //! Distance variables/functions ==========================================

  //! Constructor/Destructor
  vpMeTracker() ;
  virtual ~vpMeTracker() ;
  void init() ;

  vpMeTracker& operator =(vpMeTracker& f);

  //! Displays the number of elements in the list
  void displayNumberOfElements() ;
  void setMe(vpMe *me1) { me = me1 ; }
  int outOfImage( int i , int j , int half , int rows , int cols) ;

  int numberOfSignal() ;
  int totalNumberOfSignal() ;

  //! Virtual functions for vpMeTracker
  //! Feature dependent functions

  //!display contour
  virtual void display(vpImage<unsigned char> &I,  int col)=0;
  //!Sample pixels at a given interval
  virtual void sample(vpImage<unsigned char> &image)=0;

  void initTracking(vpImage<unsigned char>& I);
  //!Track sampled pixels
  void track(vpImage<unsigned char>& I);
  //!Displays the status of me site
  void display(vpImage<unsigned char>& I);
  //!Displays the status of me sites
  void display(vpImage<unsigned char>& I,vpColVector &w,int &index_w);

protected:
  int selectDisplay ;
public:
  enum displayEnum
    {
      NONE,
      RANGE,
      RESULT,
      RANGE_RESULT
    } ;
  void setDisplay(int select)  { selectDisplay = select ; }

};


#endif


