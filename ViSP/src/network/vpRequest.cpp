#include <visp/vpRequest.h>

vpRequest::vpRequest()
{}

vpRequest::~vpRequest()
{}
  
/*!
  Add a message as parameter of the request.
  
  \sa vpNetwork::addParameterObject()
  
  \param params : Array of characters representing the message to add.
*/
void vpRequest::addParameter(char *params)
{  
  std::string val = params;
  listOfParams.push_back(val);
}

/*!
  Add a message as parameter of the request.
  
  \sa vpNetwork::addParameterObject()
  
  \param params : std::string representing the message to add.
*/
void vpRequest::addParameter(std::string &params)
{
  listOfParams.push_back(params);
}

/*!
  Add messages as parameters of the request.
  Each message correponds to one parameter.
  
  \sa vpNetwork::addParameterObject()
  
  \param listOfparams : Array of std::string representing the messages to add.
*/
void vpRequest::addParameter(std::vector< std::string > &listOfparams)
{  
  for(unsigned int i = 0; i < listOfparams.size() ; i++)
    listOfparams.push_back(listOfparams[i]);
}