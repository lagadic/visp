/****************************************************************************
 *
 * $Id: vpList.t.cpp,v 1.4 2007-02-26 17:44:21 fspindle Exp $
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
 * This file is part of the ViSP toolkit
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
 * List data structure.
 *
 * Authors:
 * Eric Marchand
 * Nicolas Mansard : Toward const-specification respect
 *
 *****************************************************************************/

#ifndef VP_LIST_T_CPP
#define VP_LIST_T_CPP

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
  \brief return the value of the current element using a const ref.

  \verbatim
  [*, a, b, c, *]  --> value() return b
         ^
  \endverbatim
 */
template<class type>
const type& vpList<type>::value(void) const
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
bool vpList<type>::empty(void) const
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
bool vpList<type>::outside(void) const
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
bool vpList<type>::nextOutside(void) const
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
bool vpList<type>::previousOutside(void) const
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
  vpListElement<type> *e ;

  kill() ;
  e = l.cur ;
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
  l.cur = e ;
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


#endif /* #ifndef VP_LIST_T_CPP */

/*!
  \file vpList.t.cpp
  \brief Implementation of the list managment class
*/

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
