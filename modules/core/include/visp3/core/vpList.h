/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * List data structure.
 *
 * Authors:
 * Eric Marchand
 * Nicolas Mansard : Toward const-specification respect
 *
 *****************************************************************************/

#ifndef VP_LIST_H
#define VP_LIST_H

/*!
  \file vpList.h
  \brief Definition of the list managment class
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>

#include <stdio.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
  \class vpListElement
  \brief Each element of a list
*/
template <class type> class vpListElement
{
  // private:
  //  vpListElement(const vpListElement &)
  //    : prev(NULL), next(NULL), val()
  //  {
  //    throw vpException(vpException::functionNotImplementedError,"Not
  //    implemented!");
  //  }
  //  vpListElement &operator=(const vpListElement &){
  //    throw vpException(vpException::functionNotImplementedError,"Not
  //    implemented!"); return *this;
  //  }

public:
  vpListElement() : prev(NULL), next(NULL), val(){};
  vpListElement<type> *prev; ///! pointer to the previous element in the list
  vpListElement<type> *next; ///! pointer to the next element in the list
  type val;                  ///! value of the element
};

#endif /* DOXYGEN_SHOULD_SKIP_THIS */

/*!
  \class vpList
  \brief Provide simple list management

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

template <class type> class vpList
{
private:
  void init();

public:
  unsigned int nb; ///! number of items in the List
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
  vpListElement<type> *cur; // the current element
public:
  vpList();                // constr.
  vpList(const vpList &l); // cloning
  virtual ~vpList();       // destr.

  inline void next(void);          // current element's successor ( cur = cur->next )
  inline void previous(void);      // current element's predecessor ( cur = cur->pred )
  inline void front(void);         // go to the front of the List (cur = first)
  inline void end(void);           // go back to the end of the List ( cur = last )
  inline bool outside(void) const; // test whether we are outside the List

  bool empty(void) const; // tests whether the List is empty

  inline type &value(void);             // returns the current element value
  inline const type &value(void) const; // returns the current element value

  void suppress(void); // deletes the current item
  void kill();         // empties the List

  void display();             // displays the content of the list
  void print() { display(); } // displays the content of the list

  inline void addRight(const type &el); // inserts an element on the right
  inline void addLeft(const type &el);  // inserts an element on the left
  inline void modify(const type &el);   // modifies thevalue field of the curr. el.
  inline void addRight(type &el);       // inserts an element on the right
  inline void addLeft(type &el);        // inserts an element on the left
  inline void swapLeft();               // Switch the current element with the element on the left
  inline void swapRight();              // Switch the current element with the element on the right
  inline unsigned int nbElement(void);  // returns the number of items currently in the list
  inline unsigned int nbElements(void); // returns the number of items currently in the list

  vpList<type> &operator=(const vpList<type> &l);
  inline void operator+=(vpList<type> &l);
  inline void operator+=(const type &l);

  // Other non fundamental member (may be somehow useful)
  bool nextOutside(void) const;     // test whether we are outside the List
  bool previousOutside(void) const; // test whether we are outside the List

  type &previousValue(void); // returns the previous element value
  type &nextValue(void);     // returns the next element value
  type &firstValue(void);
  type &lastValue(void);
};

/*!
  \brief initialization, Create an empty list
  \verbatim
  init() --> [*, *]
  \endverbatim */
template <class type> void vpList<type>::init()
{
  vpListElement<type> *x = new vpListElement<type>;
  vpListElement<type> *y = new vpListElement<type>;

  first = x;
  last = y;

  x->prev = NULL;
  x->next = y;
  y->prev = x;
  y->next = NULL;

  cur = x;
  nb = 0;
}

/*!
  \brief Basic constructor, initialization, Create an empty list
  \verbatim
  init() --> [*, *]
  \endverbatim
  \sa init()
 */
template <class type> vpList<type>::vpList() : nb(0), first(NULL), last(NULL), cur(NULL) { init(); }

/*!
  \brief vpList destructor
  \sa kill()
 */
template <class type> vpList<type>::~vpList()
{
  kill();

  /*if (first != NULL) */ delete first;
  /*if (last != NULL)  */ delete last;
}

/*!
  \brief return the number of element in the list
 */
template <class type> unsigned int vpList<type>::nbElement(void) { return (nb); }

/*!
  \brief return the number of element in the list
 */
template <class type> unsigned int vpList<type>::nbElements(void) { return (nb); }

/*!
  \brief position the current element on the next one
  \verbatim
  [*, a, b, c, d, *] --> next() -->   [*, a, b, c, d, *]
         ^                                      ^
  \endverbatim
*/
template <class type> void vpList<type>::next(void) { cur = cur->next; }

/*!
  \brief position the current element on the previous one
  \verbatim
  [*, a, b, c, d, *] --> previous() -->   [*, a, b, c, d, *]
         ^                                ^
  \endverbatim
*/
template <class type> void vpList<type>::previous(void) { cur = cur->prev; }

/*!
  \brief return the value of the current element

  \verbatim
  [*, a, b, c, *]  --> value() return b
         ^
  \endverbatim
 */
template <class type> type &vpList<type>::value(void) { return (cur->val); }

/*!
  \brief return the value of the current element using a const ref.

  \verbatim
  [*, a, b, c, *]  --> value() return b
         ^
  \endverbatim
 */
template <class type> const type &vpList<type>::value(void) const { return (cur->val); }

/*!
  \brief return the value of the previous element

  \verbatim
  [*, a, b, c, *]  --> previousValue() return a
         ^
  \endverbatim
*/
template <class type> type &vpList<type>::previousValue(void) { return (cur->prev->val); }

/*!
  \brief return the value of the next element
  \verbatim
  [*, a, b, c, d, *]  --> nextValue() return c
         ^
  \endverbatim
*/
template <class type> type &vpList<type>::nextValue(void) { return (cur->next->val); }

/*!
  \brief return the first element of the list
   \verbatim
   [*, a, b, c, d, *]  --> firstValue() return a
  \endverbatim
 */
template <class type> type &vpList<type>::firstValue(void) { return (first->next->val); }

/*!\brief return the last element of the list
   \verbatim
   [*, a, b, c, d, *]  --> lastValue() return d
  \endverbatim
 */
template <class type> type &vpList<type>::lastValue(void) { return (last->prev->val); }

/*!
  \brief Position the current element on the first element of the list

  \verbatim
  [*, a, b, c, d, *]  --> front() --> [*, a, b, c, d, *]
         ^                                ^
  \endverbatim
 */
template <class type> void vpList<type>::front(void) { cur = first->next; }

/*!
  \brief Position the current element on the last element of the list

  \verbatim
  [*, a, b, c, d, *]  --> end() --> [*, a, b, c, d, *]
         ^                                         ^
  \endverbatim
 */
template <class type> void vpList<type>::end(void) { cur = last->prev; }

/*!
  \brief Test if the list is empty

  \verbatim
  [*, a, b, c, d, *]  --> empty return false
  [*, *]              --> empty return true
  \endverbatim
 */
template <class type> bool vpList<type>::empty(void) const { return ((first->next == last) && (first == last->prev)); }

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
template <class type> bool vpList<type>::outside(void) const { return ((cur == first) || (cur == last)); }

/*!
  \brief Test if the next element is outside the list
  (ie if the current element is the last one)

  \verbatim
  [*, a, b, c, d, *]  --> nextOutside return true
               ^
  \endverbatim
 */
template <class type> bool vpList<type>::nextOutside(void) const
{
  return ((cur->next == first) || (cur->next == last));
}

/*!
  \brief Test if the previous element is outside the list
  (ie if the current element is the firts one)

  \verbatim
  [*, a, b, c, d, *]  --> nextOutside return true
      ^
  \endverbatim
 */
template <class type> bool vpList<type>::previousOutside(void) const
{
  return ((cur->prev == first) || (cur->prev == last));
}

/*!
  \brief add a new element in the list, at the right of the current one

  \warning the new element becomes the current one

  \verbatim
  [*, a, b, c, *]  --> addRight(i) -->   [*, a, b, i, c, *]
         ^                                         ^
  \endverbatim
 */
template <class type> void vpList<type>::addRight(const type &v)
{
  vpListElement<type> *x = new vpListElement<type>;

  x->val = v;
  if (empty()) {
    cur = first;
  } else {
    if (outside())
      std::cout << "vpList: outside with addRight " << std::endl;
  }
  cur->next->prev = x;
  x->next = cur->next;
  x->prev = cur;
  cur->next = x;
  cur = x;
  nb++;
}

/*!
  \brief add a new element in the list, at the left of the current one

  \warning the new element becomes the current one

  \verbatim
  [*, a, b, c, *]  --> addLeft(i) -->   [*, a, i, b, c, *]
         ^                                     ^
  \endverbatim
 */
template <class type> void vpList<type>::addLeft(const type &v)
{
  vpListElement<type> *x = new vpListElement<type>;

  x->val = v;

  if (empty()) {
    cur = last;
  } else {
    if (outside())
      std::cout << "vpList: outside with addLeft " << std::endl;
  }
  x->next = cur;
  x->prev = cur->prev;
  cur->prev->next = x;
  cur->prev = x;
  cur = x;
  nb++;
}

/*!
  \brief add a new element in the list, at the right of the current one

  \warning the new element becomes the current one

  \verbatim
  [*, a, b, c, *]  --> addRight(i) -->   [*, a, b, i, c, *]
         ^                                         ^
  \endverbatim
 */
template <class type> void vpList<type>::addRight(type &v)
{
  vpListElement<type> *x = new vpListElement<type>;

  x->val = v;
  if (empty()) {
    cur = first;
  } else {
    if (outside())
      std::cout << "vpList: outside with addRight " << std::endl;
  }
  cur->next->prev = x;
  x->next = cur->next;
  x->prev = cur;
  cur->next = x;
  cur = x;
  nb++;
}

/*!
  \brief add a new element in the list, at the left of the current one

  \warning the new element becomes the current one

  \verbatim
  [*, a, b, c, *]  --> addLeft(i) -->   [*, a, i, b, c, *]
         ^                                     ^
  \endverbatim
 */
template <class type> void vpList<type>::addLeft(type &v)
{
  vpListElement<type> *x = new vpListElement<type>;

  x->val = v;

  if (empty()) {
    cur = last;
  } else {
    if (outside())
      std::cout << "vpList: outside with addLeft " << std::endl;
  }
  x->next = cur;
  x->prev = cur->prev;
  cur->prev->next = x;
  cur->prev = x;
  cur = x;
  nb++;
}

/*!
  \brief Modify the value of the current element

  \verbatim
  [*, a, b, c, *]  --> modify(i) -->   [*, a, i, c, *]
         ^                                    ^
  \endverbatim
 */
template <class type> void vpList<type>::modify(const type &v) { cur->val = v; }

/*!
  \brief Switch the current element with the element on the left

  \verbatim
  [*, a, b, c, *]  --> swapLeft -->   [*, b, a, c, *]
         ^                                  ^
  \endverbatim
 */
template <class type> void vpList<type>::swapLeft()
{
  if (cur->prev != first) {
    cur->prev->prev->next = cur;
    cur->next->prev = cur->prev;

    vpListElement<type> *nextTmp;
    vpListElement<type> *prevTmp;

    nextTmp = cur->next;
    prevTmp = cur->prev;

    cur->next = cur->prev;
    cur->prev = cur->prev->prev;

    prevTmp->prev = cur;
    prevTmp->next = nextTmp;
  } else {
    std::cout << "vpList: previous element is outside (swapLeft) " << std::endl;
  }
}

/*!
  \brief Switch the current element with the element on the right

  \verbatim
  [*, a, b, c, *]  --> swapRight -->   [*, a, c, b, *]
         ^                                         ^
  \endverbatim
 */
template <class type> void vpList<type>::swapRight()
{
  if (cur->next != last) {
    cur->prev->next = cur->next;
    cur->next->next->prev = cur;

    vpListElement<type> *nextTmp;
    vpListElement<type> *prevTmp;

    nextTmp = cur->next;
    prevTmp = cur->prev;

    cur->next = nextTmp->next;
    cur->prev = nextTmp;

    nextTmp->prev = prevTmp;
    nextTmp->next = cur;
  } else {
    std::cout << "vpList: next element is outside (swapRight) " << std::endl;
  }
}

/*!
  \brief Destroy the list

  \verbatim
  [*, a, b, c, *]  --> kill -->   [*, *]
         ^                            ^
  \endverbatim
 */
template <class type> void vpList<type>::kill()
{

  front();
  while (!empty()) {
    suppress();
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
template <class type> void vpList<type>::suppress(void)
{
  vpListElement<type> *x;

  cur->prev->next = cur->next;
  cur->next->prev = cur->prev;
  x = cur;
  cur = cur->next;

  if (x != NULL)
    delete x;

  nb--;
}

/*!
  \brief Copy constructor const

  \param l : the list to copy
 */

template <class type> vpList<type> &vpList<type>::operator=(const vpList<type> &l)
{
  type x;
  vpListElement<type> *e;

  kill();
  e = l.first->next;
  front();
  while (e != l.last) {
    x = e->val;
    addRight(x);
    e = e->next;
  }

  nb = l.nb;
  cur = first->next;

  return *this;
}

/*!
  \brief Append two lists

  \verbatim
  [*, a, b, *] += [*, c, d, *] --> [*, a, b, c, d, *]
                                                ^
  \endverbatim
 */
template <class type> void vpList<type>::operator+=(vpList<type> &l)
{
  type x;

  l.front();
  end();
  while (!l.outside()) {
    x = l.value();
    addRight(x);
    l.next();
  }
}

/*!
  \brief  Append an element to a list

  \verbatim
  [*, a, b, *] += c --> [*, a, b, c, *]
                                  ^
  \endverbatim
 */
template <class type> void vpList<type>::operator+=(const type &l)
{
  end();
  addRight(l);
}

/*!
  \brief copy constructor

  \param l : the list to copy
*/
template <class type> vpList<type>::vpList(const vpList<type> &l) : nb(0), first(NULL), last(NULL), cur(NULL)
{
  init();
  *this = l;
}

/*!
  \brief Print (std::cout) all the element of the list
 */
template <class type> void vpList<type>::display()
{
  unsigned int k = 1;
  front();
  while (!outside()) {
    std::cout << k << " ---> " << value() << std::endl;
    next();
    k++;
  }
  std::cout << std::endl << std::endl;
}

#endif /* #ifndef VP_LIST_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
