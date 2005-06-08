
/*
#----------------------------------------------------------------------------
#  Copyright (C) 1998  IRISA-INRIA Rennes Vista Project
#  All Rights Reserved.
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


#ifndef vpList_h
#define vpList_h

/*!
  \file vpList.h
  \brief Definition of the list managment class
*/

#include <stdio.h>

#include <visp/vpDebug.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
  \class vpListElement
  \brief Each element of a list
*/
template <class type>
class vpListElement
{
 public:
  vpListElement<type> *prev; //<! pointer to the previous element in the list
  vpListElement<type> *next; //<! pointer to the next element in the list
  type val;             //<! value of the element
} ;

#endif /* DOXYGEN_SHOULD_SKIP_THIS */


/*!
  \class vpList

  \brief Provide simple list managment

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes

  \verbatim
  Data structure:
    each object vpListElement contains its own value and
    two pointers to the next element of the list and to the previous one

    A list is mainly a pointer to three of these elements
      - one (cur) points the current element
      - one (first) points to a virtual element located at the
        beginning of the list
      - one (last) points  to a virtual element located at the
        end of the list

      first, last and cur are used internally and are not to be considered
      by "basic" user.

  Notation:
    In this documentation we note such a list as
          [*, a, b, c, *]
           f     ^     l
    where
     - the * are the two virtual elements (first (f) and last (l))
     - the ^ denotes the position of the current element

  \endverbatim
*/

template <class type>
class vpList
{
 private:
  void init() ;
 public:

  int nb;                    //<! number of items in the List
  /*!
    \brief the first virtual item in the list
    \verbatim
          [*, a, b, c, *]
           f
    \endverbatim
  */
  vpListElement<type> *first;
  /*!
    \brief the last virtualitem in the list
    \verbatim
          [*, a, b, c, *]
                       l
    \endverbatim
  */
  vpListElement<type> *last;
  /*!
    \brief the current item in the list
    \verbatim
          [*, a, b, c, *]
                 ^
    \endverbatim
  */
  vpListElement<type> *cur;       // the current element
 public:
  vpList() ;                  // constr.
  vpList(vpList& Liste);       // cloning
  ~vpList();                  // destr.
  void next(void);           // current element's successor ( cur = cur->next )
  void previous(void);       // current element's predecessor ( cur = cur->pred )
  void front(void);          // go to the front of the List (cur = first)
  void end(void);            // go back to the end of the List ( cur = last )
  bool outside(void);         // test whether we are outside the List

  bool empty(void);       // tests whether the List is empty

  type& value(void);         // returns the current element value

  void suppress(void);       // deletes the current item
  void kill();              // empties the List

  void display() ;          // displays the content of the list
  void print() {display() ;}           // displays the content of the list


  void addRight(const type& el);   // inserts an element on the right
  void addLeft(const type& el);    // inserts an element on the left
  void modify(const type& el);     // modifies thevalue field of the curr. el.
  int nbElement(void);       // returns the number of items currently in the list
  int nbElements(void);       // returns the number of items currently in the list

  void operator=(vpList<type>& Liste);
  void operator+=(vpList<type>& Liste);
  void operator+=(const type& l);

  // Other non fundamental member (may be somehow usefull)
  bool nextOutside(void);     // test whether we are outside the List
  bool previousOutside(void) ;// test whether we are outside the List


  type& previousValue(void); // returns the previous element value
  type& nextValue(void);     // returns the next element value
  type& firstValue(void) ;
  type& lastValue(void) ;


};


/*!
  \brief initialization, Create an empty list
  \verbatim
  init() --> [*, *]
  \endverbatim */
template<class type>
void vpList<type>::init()
{
  vpListElement<type> *x=new  vpListElement<type>;
  vpListElement<type> *y=new  vpListElement<type> ;

  first = x ;
  last = y ;

  x->prev = NULL ;
  x->next = y ;
  y->prev = x ;
  y->next =NULL ;

  cur = x ;
  nb = 0 ;
}

/*!
  \brief Basic constructor, initialization, Create an empty list
  \verbatim
  init() --> [*, *]
  \endverbatim
  \sa init()
 */
template<class type>
vpList<type>::vpList()
{
  init() ;
}



/*!
  \brief vpList destructor
  \sa kill()
 */
template<class type>
vpList<type>::~vpList()
{

  kill() ;

  if (first != NULL) delete first ;
  if (last != NULL) delete last ;


}

/*!
  \brief return the number of element in the list
 */
template<class type>
int vpList<type>::nbElement(void)
{
  return(nb) ;
}

/*!
  \brief return the number of element in the list
 */
template<class type>
int vpList<type>::nbElements(void)
{
  return(nb) ;
}


/*!
  \brief position the current element on the next one
  \verbatim
  [*, a, b, c, d, *] --> next() -->   [*, a, b, c, d, *]
         ^                                      ^
  \endverbatim
*/
template<class type>
void vpList<type>::next(void)
{
  cur = cur->next ;
}


/*!
  \brief position the current element on the previous one
  \verbatim
  [*, a, b, c, d, *] --> previous() -->   [*, a, b, c, d, *]
         ^                                ^
  \endverbatim
*/
template<class type>
void vpList<type>::previous(void)
{
  cur = cur->prev ;
}

/*!
  \brief return the value of the current element

  \verbatim
  [*, a, b, c, *]  --> value() return b
         ^
  \endverbatim
 */
template<class type>
type& vpList<type>::value(void)
{
  return(cur->val) ;
}

/*!
  \brief return the value of the previous element

  \verbatim
  [*, a, b, c, *]  --> previousValue() return a
         ^
  \endverbatim
*/
template<class type>
type& vpList<type>::previousValue(void)
{
  return(cur->prev->val) ;
}

/*!
  \brief return the value of the next element
  \verbatim
  [*, a, b, c, d, *]  --> nextValue() return c
         ^
  \endverbatim
*/
template<class type>
type& vpList<type>::nextValue(void)
{
  return(cur->next->val) ;
}



/*!
  \brief return the first element of the list
   \verbatim
   [*, a, b, c, d, *]  --> firstValue() return a
  \endverbatim
 */
template<class type>
type& vpList<type>::firstValue(void)
{
  return(first->next->val) ;
}



/*!\brief return the last element of the list
   \verbatim
   [*, a, b, c, d, *]  --> lastValue() return d
  \endverbatim
 */
template<class type>
type& vpList<type>::lastValue(void)
{
  return(last->prev->val) ;
}


/*!
  \brief Position the current element on the first element of the list

  \verbatim
  [*, a, b, c, d, *]  --> front() --> [*, a, b, c, d, *]
         ^                                ^
  \endverbatim
 */
template<class type>
void vpList<type>::front(void)
{
    cur = first->next ;
}

/*!
  \brief Position the current element on the last element of the list

  \verbatim
  [*, a, b, c, d, *]  --> end() --> [*, a, b, c, d, *]
         ^                                         ^
  \endverbatim
 */
template<class type>
void vpList<type>::end(void)
{
    cur = last->prev ;
}

/*!
  \brief Test if the list is empty

  \verbatim
  [*, a, b, c, d, *]  --> empty return false
  [*, *]              --> empty return true
  \endverbatim
 */
template<class type>
bool vpList<type>::empty(void)
{
  return((first->next == last) &&( first == last->prev)) ;
}

/*!
  \brief Test if the current element is outside the list
  (on the virtual element)

  \verbatim
  [*, a, b, c, d, *]  --> outside return false
         ^
  [*, a, b, c, d, *]  --> outside return true
   ^      or      ^
  \endverbatim
 */
template<class type>
bool vpList<type>::outside(void)
{

  return((cur==first)||(cur==last)) ;
}

/*!
  \brief Test if the next element is outside the list
  (ie if the current element is the last one)

  \verbatim
  [*, a, b, c, d, *]  --> nextOutside return true
               ^
  \endverbatim
 */
template<class type>
bool vpList<type>::nextOutside(void)
{
  return((cur->next==first)||(cur->next==last)) ;
}


/*!
  \brief Test if the previous element is outside the list
  (ie if the current element is the firts one)

  \verbatim
  [*, a, b, c, d, *]  --> nextOutside return true
      ^
  \endverbatim
 */
template<class type>
bool vpList<type>::previousOutside(void)
{
  return((cur->prev==first)||(cur->prev==last)) ;
}


/*!
  \brief add a new element in the list, at the right of the current one

  \warning the new element becomes the current one

  \verbatim
  [*, a, b, c, *]  --> addRight(i) -->   [*, a, b, i, c, *]
         ^                                         ^
  \endverbatim
 */
template<class type>
void vpList<type>::addRight(const type& v)
{
  vpListElement<type> *x=new  vpListElement<type>;

  x->val = v ;
  if (empty())
  {
    cur = first ;
  }
  else
  {
    if (outside()) cout << "vpList: outside with addRight " << endl ;
  }
  cur->next->prev = x ;
  x->next = cur->next ;
  x->prev = cur ;
  cur->next = x ;
  cur = x ;
  nb++ ;

}


/*!
  \brief add a new element in the list, at the left of the current one

  \warning the new element becomes the current one

  \verbatim
  [*, a, b, c, *]  --> addLeft(i) -->   [*, a, i, b, c, *]
         ^                                     ^
  \endverbatim
 */
template<class type>
void vpList<type>::addLeft(const type& v)
{
  vpListElement<type> *x=new  vpListElement<type>;

  x->val = v ;

  if (empty())
  {
    cur = last ;
  }
  else
  {
    if (outside()) cout << "vpList: outside with addLeft " << endl ;
  }
  x->next = cur ;
  x->prev = cur->prev ;
  cur->prev->next = x ;
  cur->prev = x ;
  cur = x ;
  nb++ ;

}

/*!
  \brief Modify the value of the current element

  \verbatim
  [*, a, b, c, *]  --> modify(i) -->   [*, a, i, c, *]
         ^                                    ^
  \endverbatim
 */
template<class type>
void vpList<type>::modify(const type& v)
{
  cur->val = v ;
}

/*!
  \brief Destroy the list

  \verbatim
  [*, a, b, c, *]  --> kill -->   [*, *]
         ^                            ^
  \endverbatim
 */
template<class type>
void vpList<type>::kill()
{

  front() ;
  while (!empty())
  {
    suppress() ;
  }

}



/*!
  \brief suppress the current element

  \warning new current element is on the next one

  \verbatim
  [*, a, b, c, d, *] --> suppress -->  [*, a, c, d, *]
         ^                                    ^
  \endverbatim
 */
template<class type>
void vpList<type>::suppress(void)
{
  vpListElement<type> *x ;

  cur->prev->next = cur->next ;
  cur->next->prev = cur->prev ;
  x = cur ;
  cur = cur->next ;

  if (x!=NULL) delete x ;

  nb-- ;


}



/*!
  \brief Copy constructor

  \remarks Cannot define this function as usual, ie, :
   <tt>template<class type>
   vpList<type>::vpList(const vpList<type>& liste)</tt>
  since the liste is indeed modified (not the element but the position
  of the current element.
 */

template<class type>
void vpList<type>::operator=(vpList<type>& l)
{
  type x ;
  vpListElement<type> *ecm ;

  kill() ;
  ecm = l.cur ;
  l.front() ;
  front() ;
  while (!l.outside())
  {
    x = l.value() ;
    addRight(x) ;
    l.next() ;
  }

  nb = l.nb ;
  cur = first->next ;
  l.cur = ecm ;
}

/*!
  \brief Append two lists

  \verbatim
  [*, a, b, *] += [*, c, d, *] --> [*, a, b, c, d, *]
                                                ^
  \endverbatim
 */
template<class type>
void vpList<type>::operator+=(vpList<type>& l)
{
  type x ;

  l.front() ;
  end() ;
  while (!l.outside())
  {
    x = l.value() ;
    addRight(x) ;
    l.next() ;
  }
}

/*!
  \brief  Append an element to a list

  \verbatim
  [*, a, b, *] += c --> [*, a, b, c, *]
                                  ^
  \endverbatim
 */
template<class type>
void vpList<type>::operator += (const type& l)
{
  end() ;
  addRight(l) ;
}


/*!
  \brief Copy constructor

  \remarks Cannot define this function as usual, ie, :
   <tt>template<class type>
   vpList<type>::vpList(const vpList<type>& liste)</tt>
  since the liste is indeed modified (not the element but the position
  of the current element.

  \sa operator=(vpList<type>& liste)
 */
template<class type>
vpList<type>::vpList(vpList<type>& liste)
{
  init() ;
  *this = liste ;
}

/*!
  \brief Print (cout) all the element of the list
 */
template<class type>
void vpList<type>::display()
{
  int k = 1 ;
  front() ;
  while(!outside()) {
    cout<<k<<" ---> "<<value()<<endl ;
    next() ;
    k++ ;
  }
  cout<< endl << endl ;
}



#endif

