/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $Source: /udd/fspindle/poub/cvs2svn/ViSP/cvsroot/visp/ViSP/src/data-structure/vpList.h,v $
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * Modif:     Separation in vpList.h - vpList.t.cpp (N.Mansard 4-NOV-5).
 *            Toward const-specification respect (N.Mansard 4-NOV-05).
 *
 * Version control
 * ===============
 *
 *  $Id: vpList.h,v 1.2 2005-11-04 14:40:54 nmansard Exp $
 *
 * Description
 * ============
 *  
 *  Definition of template class vpList<>.
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef VP_LIST_H
#define VP_LIST_H


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
  void next(void) ;           // current element's successor ( cur = cur->next )
  void previous(void) ;       // current element's predecessor ( cur = cur->pred )
  void front(void) ;          // go to the front of the List (cur = first)
  void end(void) ;            // go back to the end of the List ( cur = last )
  bool outside(void) const;         // test whether we are outside the List

  bool empty(void) const;       // tests whether the List is empty

  type& value(void);         // returns the current element value
  const type& value(void) const;         // returns the current element value

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
  bool nextOutside(void) const;     // test whether we are outside the List
  bool previousOutside(void) const;// test whether we are outside the List


  type& previousValue(void); // returns the previous element value
  type& nextValue(void);     // returns the next element value
  type& firstValue(void) ;
  type& lastValue(void) ;


};


#include <visp/vpList.t.cpp>       /* Implementation of template functions.   */

#endif  /* #ifndef VP_LIST_H */


/*!
  \file $Source: /udd/fspindle/poub/cvs2svn/ViSP/cvsroot/visp/ViSP/src/data-structure/vpList.h,v $
  \brief Definition of the list managment class
*/

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
