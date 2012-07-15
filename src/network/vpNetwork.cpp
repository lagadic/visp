/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * TCP Network
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#include <visp/vpNetwork.h>

vpNetwork::vpNetwork()
{
  separator = "[*@*]";
  beginning = "[*start*]";
  end = "[*end*]";
  param_sep = "[*|*]";
  max_size_message = 999999;
  
  tv_sec = 0;
  tv_usec = 10;
  
  verboseMode = false;

#ifdef WIN32
  //Enable the sockets to be used
  //Note that: if we were using "winsock.h" instead of "winsock2.h" we would had to use:
  //WSAStartup(MAKEWORD(1,0), &WSAData);
  WSADATA WSAData;
  WSAStartup(MAKEWORD(2,0), &WSAData);
#endif
}

vpNetwork::~vpNetwork()
{
#ifdef WIN32
  WSACleanup();
#endif
}

/*!
  Add a decoding request to the emitter. This request will be used to decode the received messages.
  Each request must have a different id.
  
  \warning vpRequest is a virtual pure class. It has to be implemented according to the way how you want
  to decode the message received.
  
  \sa vpNetwork::removeDecodingRequest()
  
  \param req : Request to add.
*/
void vpNetwork::addDecodingRequest(vpRequest *req)
{
  bool alreadyHas = false;
  
  for(unsigned int i = 0 ; i < request_list.size() ; i++)
    if(request_list[i]->getId() == req->getId()){
      alreadyHas = true;
      break;
    }
  
  if(alreadyHas)
    std::cout << "Server already has one request with the similar ID. Request hasn't been added." << std::endl;
  else
    request_list.push_back(req);
}

/*!
  Delete a decoding request from the emitter. 
  
  \sa vpNetwork::addDecodingRequest()
  
  \param id : Id of the request to delete.
*/
void vpNetwork::removeDecodingRequest(const char *id)
{
  for(unsigned int i = 0 ; i < request_list.size() ; i++)
  {
    if(request_list[i]->getId() == id)
    {
      request_list.erase(request_list.begin()+i);
      break;
    }
  }
}

/*!
  Print the receptors. 
  
  \param id : Message to display before the receptor's index.
*/
void vpNetwork::print(const char *id)
{
  for(unsigned int i = 0 ; i < receptor_list.size() ; i++)
  {
    std::cout << id << i << " : " << inet_ntoa(receptor_list[i].receptorAddress.sin_addr) << std::endl;
  }
}

/*!
  Get the receptor index from its name. The name can be either the IP, or its name on the network. 
  
  \param name : Name of the receptor.
  
  \return Index of the receptor.
*/
int vpNetwork::getReceptorIndex(const char *name)
{
  struct hostent *server = gethostbyname(name);
  
  if ( server == NULL )
  {
    std::string noSuchHostMessage( "ERROR, " );
    noSuchHostMessage.append( name );
    noSuchHostMessage.append( ": no such host\n" );
    vpERROR_TRACE( noSuchHostMessage.c_str(), "vpClient::getReceptorIndex()" );
  }
  
  std::string ip = inet_ntoa(*(in_addr *)server->h_addr);
  
  for(unsigned int i = 0 ; i < receptor_list.size() ; i++)
  {
    if(receptor_list[i].receptorIP == ip)
      return i;
  }
  
  return -1;
}

/*!
  Send a request to the first receptor in the list.
  
  \sa vpNetwork::sendRequestTo()
  \sa vpNetwork::sendAndEncodeRequest()
  \sa vpNetwork::sendAndEncodeRequestTo()
  \sa vpNetwork::send()
  \sa vpNetwork::sendTo()
  
  \param req : Request to send.
  
  \return The number of bytes that have been sent, -1 if an error occured.
*/
int vpNetwork::sendRequest(vpRequest &req)
{
  return sendRequestTo(req,0);
}

/*!
  Send a request to a specific receptor.
  
  \sa vpNetwork::sendRequest()
  \sa vpNetwork::sendAndEncodeRequest()
  \sa vpNetwork::sendAndEncodeRequestTo()
  \sa vpNetwork::send()
  \sa vpNetwork::sendTo()
  
  \param req : Request to send.
  \param dest : Index of the receptor receiving the request.
  
  \return The number of bytes that have been sent, -1 if an error occured.
*/
int vpNetwork::sendRequestTo(vpRequest &req, const int &dest)
{
  if(receptor_list.size() == 0 || dest > (int)receptor_list.size()-1)
  {
    if(verboseMode)
      vpTRACE( "Cannot Send Request! Bad Index" );
    return 0;
  }
  
  std::string message = beginning + req.getId() + separator;
  
  if(req.size() != 0){
    message += req[0];
    
    for(unsigned int i = 1 ; i < req.size() ; i++){
        message += param_sep + req[i];
    }
  }
  
  message += end;
  
  int flags = 0;
#if ! defined(APPLE) && ! defined(WIN32)
  flags = MSG_NOSIGNAL; // Only for Linux
#endif

  int value = sendto(receptor_list[dest].socketFileDescriptorReceptor, message.c_str(), message.size(), flags,
                     (sockaddr*) &receptor_list[dest].receptorAddress,receptor_list[dest].receptorAddressSize);
  
  return value;
}

/*!
  Send and encode a request to the first receptor in the list.
  
  \sa vpNetwork::sendRequestTo()
  \sa vpNetwork::sendAndEncodeRequest()
  \sa vpNetwork::sendAndEncodeRequestTo()
  \sa vpNetwork::send()
  \sa vpNetwork::sendTo()
  
  \param req : Request to send.
  
  \return The number of bytes that have been sent, -1 if an error occured.
*/
int vpNetwork::sendAndEncodeRequest(vpRequest &req)
{
  req.encode();
  return sendRequest(req);
}

/*!
  Send and encode a request to a specific receptor.
  
  \sa vpNetwork::sendRequest()
  \sa vpNetwork::sendAndEncodeRequest()
  \sa vpNetwork::sendAndEncodeRequestTo()
  \sa vpNetwork::send()
  \sa vpNetwork::sendTo()
  
  \param req : Request to send.
  \param dest : Index of the receptor receiving the request.
  
  \return The number of bytes that have been sent, -1 if an error occured.
*/
int vpNetwork::sendAndEncodeRequestTo(vpRequest &req, const int &dest)
{
  req.encode();
  return sendRequestTo(req,dest);
}

/*!
  Receive requests untils there is requests to receive.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequestFrom()
  \sa vpNetwork::receiveRequestOnce()
  \sa vpNetwork::receiveRequestOnceFrom()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
*/
std::vector<int> vpNetwork::receiveRequest()
{
  _receiveRequest();
  return _handleRequests();
}

/*!
  Receives requests, from a specific emitter, untils there is request to receive.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequest()
  \sa vpNetwork::receiveRequestOnce()
  \sa vpNetwork::receiveRequestOnceFrom()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
  
  \param receptorEmitting : Index of the receptor emitting the message
*/
std::vector<int> vpNetwork::receiveRequestFrom(const int &receptorEmitting)
{
  _receiveRequestFrom(receptorEmitting);
  return _handleRequests();
}

/*!
  Receives a message once (in the limit of the Maximum message size value).
  This message can represent an entire request or not. Several calls to this function
  might be necessary to get the entire request.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequestFrom()
  \sa vpNetwork::receiveRequest()
  \sa vpNetwork::receiveRequestOnceFrom()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
  
  \return The number of bytes received, -1 if an error occured.
*/
int vpNetwork::receiveRequestOnce()
{
  _receiveRequestOnce();
  return _handleFirstRequest();
}

/*!
  Receives a message once (in the limit of the Maximum message size value), from a specific emitter.
  This message can represent an entire request or not. Several calls to this function
  might be necessary to get the entire request.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequestFrom()
  \sa vpNetwork::receiveRequest()
  \sa vpNetwork::receiveRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
  
  \param receptorEmitting : Index of the receptor emitting the message.
  
  \return The number of bytes received, -1 if an error occured.
*/
int vpNetwork::receiveRequestOnceFrom(const int &receptorEmitting)
{
  _receiveRequestOnceFrom(receptorEmitting);
  return _handleFirstRequest();
}

/*!
  Receives and decode requests untils there is requests to receive.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequestFrom()
  \sa vpNetwork::receiveRequestOnce()
  \sa vpNetwork::receiveRequestOnceFrom()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
*/
std::vector<int> vpNetwork::receiveAndDecodeRequest()
{
  std::vector<int> res = receiveRequest();
  for(unsigned int i = 0 ; i < res.size() ; i++)
    if(res[i] != -1)
      request_list[res[i]]->decode();
    
  return res;
}

/*!
  Receives and decode requests, from a specific emitter, untils there is request to receive.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequest()
  \sa vpNetwork::receiveRequestOnce()
  \sa vpNetwork::receiveRequestOnceFrom()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
  
  \param receptorEmitting : Index of the receptor emitting the message
*/
std::vector<int> vpNetwork::receiveAndDecodeRequestFrom(const int &receptorEmitting)
{
  std::vector<int> res = receiveRequestFrom(receptorEmitting);
  for(unsigned int i = 0 ; i < res.size() ; i++)
    if(res[i] != -1)
      request_list[res[i]]->decode();
  
    return res;
}

/*!
  Receives a message once (in the limit of the Maximum message size value).
  This message can represent an entire request or not. Several calls to this function
  might be necessary to get the entire request.
  If it represents an entire request, it decodes the request.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequestFrom()
  \sa vpNetwork::receiveRequest()
  \sa vpNetwork::receiveRequestOnceFrom()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
  
  \return The number of bytes received, -1 if an error occured.
*/
int vpNetwork::receiveAndDecodeRequestOnce()
{
  int res = receiveRequestOnce();
  if(res != -1)
    request_list[res]->decode();
  
  return res;
}

/*!
  Receives a message once (in the limit of the Maximum message size value), from a specific emitter.
  This message can represent an entire request or not. Several calls to this function
  might be necessary to get the entire request.
  If it represents an entire request, it decodes the request.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequestFrom()
  \sa vpNetwork::receiveRequest()
  \sa vpNetwork::receiveRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
  
  \param receptorEmitting : Index of the receptor emitting the message.
  
  \return The number of bytes received, -1 if an error occured.
*/
int vpNetwork::receiveAndDecodeRequestOnceFrom(const int &receptorEmitting)
{
  int res = receiveRequestOnceFrom(receptorEmitting);
  if(res != -1)
    request_list[res]->decode();
  
  return res;
}
  

//######## Definition of Template Functions ########
//#                                                #
//##################################################


/*!
  Handle requests until there are requests to handle.
  
  \warning : This function doesn't decode the requests. If it does handle a request that hasn't been ran yet,
  The request's parameters will be replace. 
  
  \sa vpNetwork::handleFirstRequest()
  
  \return : The list of index corresponding to the requests that have been handled.
*/
std::vector<int> vpNetwork::_handleRequests()
{
  std::vector<int> resIndex;
  int index = _handleFirstRequest();
  
  while(index != -1)
  {
    resIndex.push_back(index);
    index = _handleFirstRequest();
  }
  
  return resIndex;
}

/*!
  Handle the first request in the queue.
  
  \warning : This function doesn't run the request. If it does handle a request that hasn't been ran yet,
  The request's parameters will be replace. 
  
  \sa vpNetwork::handleRequests()
  
  \return : The index of the request that has been handled.
*/
int vpNetwork::_handleFirstRequest()
{
  int indStart = currentMessageReceived.find(beginning);
  int indSep = currentMessageReceived.find(separator);
  int indEnd = currentMessageReceived.find(end);
  
  if (indStart == -1 && indSep == -1 && indEnd == -1)
  {
    if(currentMessageReceived.size() != 0)
      currentMessageReceived.clear();
    
    if(verboseMode)
      vpTRACE("Incorrect message");
    
    return -1;
  }
  
  if(indStart == -1 || indSep == -1 || indEnd == -1)
    return -1;
  
  if(indEnd < indStart)
  {
    if(verboseMode)
      vpTRACE("Incorrect message");
    currentMessageReceived.erase(indStart,indEnd+end.size());
    return -1;
  }
  
  int indStart2 = currentMessageReceived.find(beginning,indStart+1);
  if(indStart2 != -1 && indStart2 < indEnd)
  {
    if(verboseMode)
      vpTRACE("Incorrect message");
    currentMessageReceived.erase(indStart,indStart2);
    return -1;
  }
  
  int deb = indStart + beginning.size();
  std::string id = currentMessageReceived.substr(deb, indSep - deb);
  
  deb = indSep+separator.size();
  std::string params = currentMessageReceived.substr(deb, indEnd - deb);
  
//   std::cout << "Handling : " << currentMessageReceived.substr(indStart, indEnd+end.size() - indStart) << std::endl;
   
  int indRequest;
  bool hasBeenFound = false;
  for(unsigned int i = 0 ; i < request_list.size() ; i++)
  {
    if(id == request_list[i]->getId()){
        hasBeenFound = true;
        request_list[i]->clear();
        indRequest = i;
        break;
    }
  }
  
  if(!hasBeenFound){
    //currentMessageReceived.erase(indStart,indEnd+end.size());
    if(verboseMode)
      vpTRACE("No request corresponds to the received message");
    return -1;
  }
  
  int indDebParam = indSep + separator.size();
  int indEndParam = currentMessageReceived.find(param_sep,indDebParam);
  
  std::string param;
  while(indEndParam != -1)
  {
    param = currentMessageReceived.substr(indDebParam, indEndParam - indDebParam);
    request_list[indRequest]->addParameter(param);
    indDebParam = indEndParam+param_sep.size();
    indEndParam = currentMessageReceived.find(param_sep,indDebParam);
  }
  
  param = currentMessageReceived.substr(indDebParam, indEnd - indDebParam);
  request_list[indRequest]->addParameter(param);
  currentMessageReceived.erase(indStart,indEnd+end.size());
  
  return indRequest;
}

/*!
  Receive requests untils there is requests to receive.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequestFrom()
  \sa vpNetwork::receiveRequestOnce()
  \sa vpNetwork::receiveRequestOnceFrom()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
*/
void vpNetwork::_receiveRequest()
{
  while(_receiveRequestOnce() > 0) {};
}

/*!
  Receives requests, from a specific emitter, untils there is request to receive.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequest()
  \sa vpNetwork::receiveRequestOnce()
  \sa vpNetwork::receiveRequestOnceFrom()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
  
  \param receptorEmitting : Index of the receptor emitting the message
*/
void vpNetwork::_receiveRequestFrom(const int &receptorEmitting)
{
  while(_receiveRequestOnceFrom(receptorEmitting) > 0) {};
}

/*!
  Receives a message once (in the limit of the Maximum message size value).
  This message can represent an entire request or not. Several calls to this function
  might be necessary to get the entire request.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequestFrom()
  \sa vpNetwork::receiveRequest()
  \sa vpNetwork::receiveRequestOnceFrom()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
  
  \return The number of bytes received, -1 if an error occured.
*/
int vpNetwork::_receiveRequestOnce()
{
  if(receptor_list.size() == 0)
  {
    if(verboseMode)
      vpTRACE( "No Receptor!" );
    return -1;
  }
  
  tv.tv_sec = tv_sec;
  tv.tv_usec = tv_usec;
  
  FD_ZERO(&readFileDescriptor);        
  
  for(unsigned int i=0; i<receptor_list.size(); i++){ 
    if(i == 0)
      socketMax = receptor_list[i].socketFileDescriptorReceptor;
    
    FD_SET(receptor_list[i].socketFileDescriptorReceptor,&readFileDescriptor); 
    if(socketMax < receptor_list[i].socketFileDescriptorReceptor) socketMax = receptor_list[i].socketFileDescriptorReceptor; 
  }

  int value = select(socketMax+1,&readFileDescriptor,NULL,NULL,&tv);
  int numbytes = 0;
  
  if(value == -1){
    if(verboseMode)
      vpERROR_TRACE( "Select error" );
    return -1;
  }
  else if(value == 0){
    //Timeout
    return 0;
  }
  else{
    for(unsigned int i=0; i<receptor_list.size(); i++){
      if(FD_ISSET(receptor_list[i].socketFileDescriptorReceptor,&readFileDescriptor)){
        char *buf = new char [max_size_message];
        numbytes=recv(receptor_list[i].socketFileDescriptorReceptor, buf, max_size_message, 0);
        
        if(numbytes <= 0)
        {
          std::cout << "Disconnected : " << inet_ntoa(receptor_list[i].receptorAddress.sin_addr) << std::endl;
          receptor_list.erase(receptor_list.begin()+i);
          return numbytes;
        }
        else if(numbytes > 0){
          std::string returnVal(buf, numbytes);              
          currentMessageReceived.append(returnVal);
        }
        delete [] buf;
        break;
      }
    }
  }
  
  return numbytes;
}

/*!
  Receives a message once (in the limit of the Maximum message size value), from a specific emitter.
  This message can represent an entire request or not. Several calls to this function
  might be necessary to get the entire request.
  
  \warning Requests will be received but not decoded.
  
  \sa vpNetwork::receive()
  \sa vpNetwork::receiveRequestFrom()
  \sa vpNetwork::receiveRequest()
  \sa vpNetwork::receiveRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()
  
  \param receptorEmitting : Index of the receptor emitting the message.
  
  \return The number of bytes received, -1 if an error occured.
*/
int vpNetwork::_receiveRequestOnceFrom(const int &receptorEmitting)
{
  if(receptor_list.size() == 0 || receptorEmitting > (int)receptor_list.size()-1 )
  {
    if(verboseMode)
      vpTRACE( "No receptor at the specified index!" );
    return -1;
  }
  
  tv.tv_sec = tv_sec;
  tv.tv_usec = tv_usec;
  
  FD_ZERO(&readFileDescriptor);        
  
  socketMax = receptor_list[receptorEmitting].socketFileDescriptorReceptor;
  FD_SET(receptor_list[receptorEmitting].socketFileDescriptorReceptor,&readFileDescriptor);

  int value = select(socketMax+1,&readFileDescriptor,NULL,NULL,&tv);
  int numbytes = 0;
  if(value == -1){
    if(verboseMode)
      vpERROR_TRACE( "Select error" );
    return -1;
  }
  else if(value == 0){
    //Timeout
    return 0;
  }
  else{
    if(FD_ISSET(receptor_list[receptorEmitting].socketFileDescriptorReceptor,&readFileDescriptor)){
      char *buf = new char [max_size_message];
      numbytes=recv(receptor_list[receptorEmitting].socketFileDescriptorReceptor, buf, max_size_message, 0);
      
      if(numbytes <= 0)
      {
        std::cout << "Disconnected : " << inet_ntoa(receptor_list[receptorEmitting].receptorAddress.sin_addr) << std::endl;
        receptor_list.erase(receptor_list.begin()+receptorEmitting);
        return numbytes;
      }
      else if(numbytes > 0){
        std::string returnVal(buf, numbytes);              
        currentMessageReceived.append(returnVal);
      }
      delete [] buf;
    }
  }
  
  return numbytes;
}







