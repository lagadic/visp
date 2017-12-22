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
 * Network Request.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#ifndef vpRequest_H
#define vpRequest_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImageException.h>

#include <string.h>
#include <vector>

/*!
  \class vpRequest

  \ingroup group_core_network

  \brief This the request that will transit on the network

  Exemple request decoding an image on a specific form.
  First parameter : Height of the image.
  Second parameter : Width of the image.
  Thirs parameter : Bitmap of the image (not compress).

  Here is the header of the vpRequestImage class.

  \code
#ifndef vpRequestImage_H
#define vpRequestImage_H

#include <visp3/core/vpImage.h>
#include <visp3/core/vpRequest.h>

class vpRequestImage : public vpRequest
{
private:
  vpImage<unsigned char> *I;

public:
  vpRequestImage();
  vpRequestImage(vpImage<unsigned char> *);
  ~vpRequestImage();

  virtual void encode();
  virtual void decode();
};

#endif
  \endcode

  Here is the definition of the vpRequestImage class.

  \code
#include <vpRequestImage.h>

vpRequestImage::vpRequestImage(){
  request_id = "image";
}

vpRequestImage::vpRequestImage(vpImage<unsigned char> *Im){
  request_id = "image";
  I = Im;
}

vpRequestImage::~vpRequestImage(){}

void vpRequestImage::encode(){
  clear();

  unsigned int h = I->getHeight();
  unsigned int w = I->getWidth();

  addParameterObject(&h);
  addParameterObject(&w);
  addParameterObject(I->bitmap,h*w*sizeof(unsigned char));
}

void vpRequestImage::decode(){
  if(listOfParams.size() == 3){
    unsigned int w, h;
    memcpy((void*)&h, (void*)listOfParams[0].c_str(), sizeof(unsigned int));
    memcpy((void*)&w, (void*)listOfParams[1].c_str(), sizeof(unsigned int));

    I->resize(h,w);
    memcpy((void*)I->bitmap,(void*)listOfParams[2].c_str(),w*h*sizeof(unsigned char));
  }
}
  \endcode

  \sa vpClient
  \sa vpServer
  \sa vpNetwork
*/
class VISP_EXPORT vpRequest
{
protected:
  std::string request_id;
  std::vector<std::string> listOfParams;

public:
  vpRequest();
  virtual ~vpRequest();

  void addParameter(char *params);
  void addParameter(std::string &params);
  void addParameter(std::vector<std::string> &listOfparams);
  template <typename T> void addParameterObject(T *params, const int &sizeOfObject = sizeof(T));

  /*!
    Decode the parameters of the request (Funtion that has to be redifined).

    \sa vpRequest::encode()
  */
  virtual void decode() = 0;

  /*!
    Clear the parameters of the request.
  */
  void clear() { listOfParams.clear(); }

  /*!
    Encode the parameters of the request (Funtion that has to be redifined).

    \sa vpRequest::decode()
  */
  virtual void encode() = 0;

  /*!
    Accessor on the parameters.

    \return Parameter at the index i.
  */
  inline std::string &operator[](const unsigned int &i) { return listOfParams[i]; }

  /*!
    Accessor on the parameters (const).

    \return Parameter at the index i (const).
  */
  inline const std::string &operator[](const unsigned int &i) const { return listOfParams[i]; }

  /*!
    Get the ID of the request.

    \sa vpRequest::setId()

    \return ID of the request.
  */
  std::string getId() { return request_id; }

  /*!
    Change the ID of the request.

    \sa vpRequest::getId()

    \param id : new ID.
  */
  void setId(const char *id) { request_id = id; }

  /*!
    Get the number of parameters.

    \return Number of parameters.
  */
  unsigned int size() { return (unsigned int)listOfParams.size(); }
};

//######## Definition of Template Functions ########
//#                                                #
//##################################################

/*!
  Add an object as parameter of the request.

  \warning Only simple object can be sent unless you know its size.
  Sending object containing pointers, virtual methods, etc, won't probably
  work. Unless the size is well defined...

  \sa vpRequest::addParameter()

  \param params : Object to add.
  \param sizeOfObject : Size of the object.
*/
template <typename T> void vpRequest::addParameterObject(T *params, const int &sizeOfObject)
{
  if (sizeOfObject != 0) {
    char *tempS = new char[sizeOfObject];
    memcpy((void *)tempS, (void *)params, sizeOfObject);
    std::string returnVal(tempS, (size_t)sizeOfObject);

    listOfParams.push_back(returnVal);

    delete[] tempS;
  }
}

#endif
