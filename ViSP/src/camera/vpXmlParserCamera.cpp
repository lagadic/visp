/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
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
 * XML parser to load and save camera intrinsic parameters.
 *
 * Authors:
 * Anthony Saunier
 *
 *****************************************************************************/


/*!
  \file vpXmlParserCamera.cpp
  \brief Definition of the vpXmlParserCamera class member functions.
  Class vpXmlParserCamera allowed to load and save intrinsic camera parameters

*/
#include <visp/vpXmlParserCamera.h>
#ifdef VISP_HAVE_XML2

#include <stdlib.h>
#include <string.h>

#include <visp/vpDebug.h>
/* -------------------------------------------------------------------------- */
/* --- LABEL XML ------------------------------------------------------------ */
/* -------------------------------------------------------------------------- */

#define LABEL_XML_ROOT                               "root"
#define LABEL_XML_CAMERA                             "camera"
#define LABEL_XML_CAMERA_NAME                        "name"
#define LABEL_XML_WIDTH                              "image_width"
#define LABEL_XML_HEIGHT                             "image_height"
#define LABEL_XML_SUBSAMPLING_WIDTH                  "subsampling_width"
#define LABEL_XML_SUBSAMPLING_HEIGHT                 "subsampling_height"
#define LABEL_XML_FULL_WIDTH                         "full_width"
#define LABEL_XML_FULL_HEIGHT                        "full_height"
#define LABEL_XML_MODEL                              "model"
#define LABEL_XML_MODEL_TYPE                         "type"
#define LABEL_XML_U0                                 "u0"
#define LABEL_XML_V0                                 "v0"
#define LABEL_XML_PX                                 "px"
#define LABEL_XML_PY                                 "py"
#define LABEL_XML_KUD                                "kud"
#define LABEL_XML_KDU                                "kdu"

#define LABEL_XML_MODEL_WITHOUT_DISTORTION    "perspectiveProjWithoutDistortion"
#define LABEL_XML_MODEL_WITH_DISTORTION       "perspectiveProjWithDistortion"
/*!
  Default constructor
*/
vpXmlParserCamera::vpXmlParserCamera(){
  camera_name = "";
  image_width = 0;
  image_height = 0;
  subsampling_width = 0;
  subsampling_height = 0;
  full_width = 0;
  full_height = 0;
}
/*!
  Copy constructor
  \param twinParser : parser object to copy
*/
vpXmlParserCamera::vpXmlParserCamera(vpXmlParserCamera& twinParser){
  this->camera = twinParser.camera;
  this->camera_name = twinParser.camera_name;
  this->image_width = twinParser.image_width;
  this->image_height = twinParser.image_height;
  this->subsampling_width = twinParser.subsampling_width;
  this->subsampling_height = twinParser.subsampling_height;
  this->full_width = twinParser.full_width;
  this->full_height = twinParser.full_height;
}

/*!
  Copy operator
  \param twinParser : parser object to copy
  \return a copy of the input.
*/
vpXmlParserCamera&
vpXmlParserCamera::operator =(const vpXmlParserCamera& twinParser) {
  this->camera = twinParser.camera;
  this->camera_name = twinParser.camera_name;
  this->image_width = twinParser.image_width;
  this->image_height = twinParser.image_height;
  this->subsampling_width = twinParser.subsampling_width;
  this->subsampling_height = twinParser.subsampling_height;
  this->full_width = twinParser.full_width;
  this->full_height = twinParser.full_height;
  return *this ;
}

/*!
  Parse an xml file to load camera parameters.
  \param cam : camera parameters to fill.
  \param filename : name of the xml file to parse
  \param camera_name : name of the camera : usefull if the xml file has multiple
  camera parameters. Set as "" if the camera name is not ambiguous.
  \param projModel : camera projection model needed.
  \param image_width : image width on which camera calibration was performed.
  Set as 0 if not ambiguous.
  \param image_height : image height on which camera calibration was performed.
  Set as 0 if not ambiguous.

  \return error code.
*/
int
vpXmlParserCamera::parse(vpCameraParameters &cam, const char * filename,
			 const std::string& camera_name,
       const vpCameraParameters::vpCameraParametersProjType &projModel,
			 const unsigned int image_width,
			 const unsigned int image_height)
{
  xmlDocPtr doc;
  xmlNodePtr node;

  doc = xmlParseFile(filename);
  if (doc == NULL)
  {
    return SEQUENCE_ERROR;
  }

  node = xmlDocGetRootElement(doc);
  if (node == NULL)
  {
    xmlFreeDoc(doc);
    return SEQUENCE_ERROR;
  }

  this ->read (doc, node, camera_name, projModel, image_width, image_height);

  cam = camera ;

  xmlFreeDoc(doc);

  return SEQUENCE_OK;
}

/*!
  Save camera parameters in an xml file.
  \param cam : camera parameters to save.
  \param filename : name of the xml file to fill.
  \param camera_name : name of the camera : usefull if the xml file has multiple
    camera parameters. Set as "" if the camera name is not ambiguous.
  \param image_width : width of image  on which camera calibration was performed.
    Set as 0 if not ambiguous.
  \param image_height : height of the image  on which camera calibration was performed.
    Set as 0 if not ambiguous.

  \return error code.
*/
int
vpXmlParserCamera::save(const vpCameraParameters &cam, const char * filename,
			const std::string& camera_name,
			const unsigned int image_width,
			const unsigned int image_height)
{
  xmlDocPtr doc;
  xmlNodePtr node;
  xmlNodePtr nodeCamera = NULL;

  doc = xmlReadFile(filename,NULL,XML_PARSE_NOWARNING + XML_PARSE_NOERROR
          + XML_PARSE_NOBLANKS);
  if (doc == NULL){
    doc = xmlNewDoc ((xmlChar*)"1.0");
    node = xmlNewNode(NULL,(xmlChar*)LABEL_XML_ROOT);
    xmlDocSetRootElement(doc,node);
    xmlNodePtr node_tmp = xmlNewComment((xmlChar*)
      "This file stores intrinsic camera parameters used\n"
      "   in the vpCameraParameters Class of ViSP available\n"
      "   at http://www.irisa.fr/lagadic/visp/visp.html .\n"
      "   It can be read with the parse method of\n"
      "   the vpXmlParserCamera class.");
    xmlAddChild(node,node_tmp);
  }

  node = xmlDocGetRootElement(doc);
  if (node == NULL)
  {
    xmlFreeDoc(doc);
    return SEQUENCE_ERROR;
  }

  int nbCamera = count(doc, node, camera_name,cam.get_projModel(),
                       image_width, image_height);
  if( nbCamera > 0){
    vpCERROR << nbCamera
            << " set(s) of camera parameters is(are) already "<< std::endl
            << "available in the file with your specifications : "<< std::endl
            << "precise the grabber parameters or delete manually"<< std::endl
            << "the previous one."<<std::endl;
    xmlFreeDoc(doc);
    return SEQUENCE_ERROR;
  }
  this->camera = cam;

  nodeCamera = find_camera(doc, node, camera_name, image_width, image_height);
  if(nodeCamera == NULL){
    write(node, camera_name, image_width, image_height);
  }
  else{
    write_camera(nodeCamera);
  }
  xmlSaveFormatFile(filename,doc,1);
  xmlFreeDoc(doc);

  return SEQUENCE_OK;
}

/*!
  Read a place of the XML file in char*

 \param doc : XML file.
 \param node : XML tree, pointing on a marker equipement.
 \param res : variable where to place the result.
*/
void
vpXmlParserCamera::myXmlReadCharChild (xmlDocPtr doc,
				       xmlNodePtr node,
				       char **res)
{
  xmlNodePtr cur;

  cur = node ->xmlChildrenNode;

  *res = (char *) xmlNodeListGetString(doc, cur, 1);

  return ;
} /* myXmlReadCharChild () */


/*!
  Read a place of the XML file in integer

 \param doc : XML file.
 \param node : XML tree, pointing on a marker equipement.
 \param res : variable where to place the result.
 \param code_error: place a SEQUENCE_ERROR if error.
*/
void
vpXmlParserCamera::myXmlReadIntChild (xmlDocPtr doc,
				      xmlNodePtr node,
				      int &res,
				      vpXmlCodeSequenceType &code_error)
{
  char * val_char;
  char * control_convert;
  int val_int;
  xmlNodePtr cur;

  cur = node ->xmlChildrenNode;

  val_char = (char *) xmlNodeListGetString(doc, cur, 1);
  val_int = strtol ((char *)val_char, &control_convert, 10);

  if (val_char == control_convert)
  {
    //  vpERROR_TRACE ("String %s can't be read "
    //   "as an integer.", val_char);
    //  vpERROR_TRACE ("Error line %d of the file %s.",
    //     XML_GET_LINE (node), doc ->name);
    val_int = 0;
    code_error = SEQUENCE_ERROR;
  }

  //  vpDEBUG_TRACE (15, "Prop: \"%s\" -> %d.", val_char, val_int);
  //  free (val_char);

  res = val_int;
  //  vpDEBUG_TRACE (25, "# OUTPUT: res = %d, err = %d.", res, code_erreur);
  return ;
} /* myXmlReadIntChild () */

/*!
  Read a place of the XML file in double

 \param doc : XML file.
 \param node : XML tree, pointing on a marker equipement.
 \param res : variable where to place the result.
 \param code_error: place a SEQUENCE_ERROR if error.
*/
void
vpXmlParserCamera::myXmlReadDoubleChild (xmlDocPtr doc,
					 xmlNodePtr node,
					 double &res,
					 vpXmlCodeSequenceType &code_error)
{
  char * val_char;
  char * control_convert;
  double val_double;
  xmlNodePtr cur;

  //  vpDEBUG_TRACE (25, "# INPUT.");

  cur = node ->xmlChildrenNode;

  val_char = (char *) xmlNodeListGetString(doc, cur, 1);
  val_double = strtod ((char *)val_char, &control_convert);

  if (val_char == control_convert)
  {
    //   vpERROR_TRACE ("String %s can't be read "
    //     "as a double.", val_char);
    //   vpERROR_TRACE ("Error line %d of the file %s.",
    //     XML_GET_LINE (node), doc ->name);
    val_double = 0;
    code_error = SEQUENCE_ERROR;
  }

  //  vpDEBUG_TRACE (15, "Prop: \"%s\" -> %f.", val_char, val_double);
  //  free (val_char);

  res = val_double;
  //  vpDEBUG_TRACE (25, "# Output: res = %f, err = %d.", res, code_error);
  return ;
} /* myXmlReaddoubleChild () */


/*!
  Read camera parameters from a XML file.

  \param doc : XML file.
  \param node : XML tree, pointing on a marker equipement.
  \param camera_name : name of the camera : usefull if the xml file has multiple
  camera parameters. Set as "" if the camera name is not ambiguous.
  \param image_width : width of image  on which camera calibration was performed.
    Set as 0 if not ambiguous.
  \param image_height : height of the image  on which camera calibration
    was performed. Set as 0 if not ambiguous.
  \param subsampling_width : subsampling of the image width sent by the camera.
    Set as 0 if not ambiguous.
  \param subsampling_height : subsampling of the image height sent by the camera.
    Set as 0 if not ambiguous.

  \return error code.
 */
int
vpXmlParserCamera::read (xmlDocPtr doc, xmlNodePtr node,
			 const std::string& camera_name,
       const vpCameraParameters::vpCameraParametersProjType &projModel,
       const unsigned int image_width,
			 const unsigned int image_height,
			 const unsigned int subsampling_width,
			 const unsigned int subsampling_height)
{
  //    char * val_char;
  vpXmlCodeType prop;

  vpXmlCodeSequenceType back = SEQUENCE_OK;
  int nbCamera = 0;

  for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
  {
    if (node->type != XML_ELEMENT_NODE) continue;
    if (SEQUENCE_OK != str2xmlcode ((char*)(node ->name), prop))
    {
      prop = CODE_XML_OTHER;
      back = SEQUENCE_ERROR;
    }
    switch (prop)
    {
    case CODE_XML_CAMERA:
      if (SEQUENCE_OK == this->read_camera (doc, node, camera_name, projModel,
          image_width, image_height, subsampling_width, subsampling_height)){
        nbCamera++;
      }
      break;
    default:
      back = SEQUENCE_ERROR;
      break;
    }
  }

  if (nbCamera == 0){
    back = SEQUENCE_ERROR;
    vpCERROR << "No camera parameters is available" << std::endl
                  << "with your specifications" << std::endl;
  }
  else if(nbCamera > 1){
    back = SEQUENCE_ERROR;
    vpCERROR << nbCamera << " sets of camera parameters are available"  << std::endl
                  << "with your specifications : "              << std::endl
                  << "precise your choice..."                   << std::endl;
  }

  return back;
}
/*!
  Read camera parameters from a XML file and count the number of available
  sets of camera parameters corresponding with inputs.

  \param doc : XML file.
  \param node : XML tree, pointing on a marker equipement.
  \param camera_name : name of the camera : usefull if the xml file has multiple
  camera parameters. Set as "" if the camera name is not ambiguous.
  \param image_width : width of image  on which camera calibration was performed.
    Set as 0 if not ambiguous.
  \param image_height : height of the image  on which camera calibration
    was performed. Set as 0 if not ambiguous.
  \param subsampling_width : subsampling of the image width sent by the camera.
    Set as 0 if not ambiguous.
  \param subsampling_height : subsampling of the image height sent by the camera.
    Set as 0 if not ambiguous.

  \return number of available camera parameters corresponding with inputs.
 */
int
vpXmlParserCamera::count (xmlDocPtr doc, xmlNodePtr node,
			  const std::string& camera_name,
        const vpCameraParameters::vpCameraParametersProjType &projModel,
  		  const unsigned int image_width,
			  const unsigned int image_height,
			  const unsigned int subsampling_width,
			  const unsigned int subsampling_height)
{
  //    char * val_char;
  vpXmlCodeType prop;
  int nbCamera = 0;

  for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
  {
    if (node->type != XML_ELEMENT_NODE) continue;
    if (SEQUENCE_OK != str2xmlcode ((char*)(node ->name), prop))
    {
      prop = CODE_XML_OTHER;
    }
    switch (prop)
    {
    case CODE_XML_CAMERA:
      if (SEQUENCE_OK == this->read_camera (doc, node, camera_name, projModel,
          image_width, image_height,
          subsampling_width, subsampling_height)){
        nbCamera++;
      }
      break;
    default:
      break;
    }
  }

  return nbCamera;
}
/*!
  Read camera headers from a XML file and return the last available
  node pointeur in the xml tree corresponding with inputs.

  \param doc : XML file.
  \param node : XML tree, pointing on a marker equipement.
  \param camera_name : name of the camera : usefull if the xml file has multiple
  camera parameters. Set as "" if the camera name is not ambiguous.
  \param image_width : width of image  on which camera calibration was performed.
    Set as 0 if not ambiguous.
  \param image_height : height of the image  on which camera calibration
    was performed. Set as 0 if not ambiguous.
  \param subsampling_width : subsampling of the image width sent by the camera.
    Set as 0 if not ambiguous.
  \param subsampling_height : subsampling of the image height sent by the camera.
    Set as 0 if not ambiguous.

  \return number of available camera parameters corresponding with inputs.
 */
xmlNodePtr
vpXmlParserCamera::find_camera (xmlDocPtr doc, xmlNodePtr node,
                                const std::string& camera_name,
                                const unsigned int image_width,
                                const unsigned int image_height,
                                const unsigned int subsampling_width,
                                const unsigned int subsampling_height)
{
  //    char * val_char;
  vpXmlCodeType prop;

  for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
  {
    if (node->type != XML_ELEMENT_NODE) continue;
    if (SEQUENCE_OK != str2xmlcode ((char*)(node ->name), prop))
    {
      prop = CODE_XML_OTHER;
    }
    switch (prop)
    {
      case CODE_XML_CAMERA:
        if (SEQUENCE_OK == this->read_camera_header(doc, node, camera_name,
            image_width, image_height,
            subsampling_width, subsampling_height)){
              return node;
        }
        break;
      default:
        break;
    }
  }
  return NULL;
}

/*!
  Read camera fields from a XML file.

  \param doc : XML file.
  \param node : XML tree, pointing on a marker equipement.
  \param camera_name : name of the camera : usefull if the xml file has multiple
  camera parameters. Set as "" if the camera name is not ambiguous.
  \param image_width : width of image  on which camera calibration was performed.
    Set as 0 if not ambiguous.
  \param image_height : height of the image  on which camera calibration
    was performed. Set as 0 if not ambiguous.
  \param subsampling_width : scale of the image width sent by the camera.
    Set as 0 if not ambiguous.
  \param subsampling_height : scale of the image height sent by the camera.
    Set as 0 if not ambiguous.

  \return error code.

 */
int
vpXmlParserCamera::read_camera (xmlDocPtr doc, xmlNodePtr node,
				const std::string& camera_name,
        const vpCameraParameters::vpCameraParametersProjType &projModel,
  			const unsigned int image_width,
				const unsigned int image_height,
				const unsigned int subsampling_width,
				const unsigned int subsampling_height)
{
  char * val_char;
  vpXmlCodeType prop;
  /* read value in the XML file. */
  int val;
  std::string camera_name_tmp = "";
  unsigned int image_height_tmp = 0 ;
  unsigned int image_width_tmp = 0 ;
  unsigned int subsampling_width_tmp = 0;
  unsigned int subsampling_height_tmp = 0;
  unsigned int full_width_tmp = 0;
  unsigned int full_height_tmp = 0;
  vpCameraParameters cam_tmp;
  vpCameraParameters cam_tmp_model;
  bool projModelFound = false;
  vpXmlCodeSequenceType back = SEQUENCE_OK;

  for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
  {
    // vpDEBUG_TRACE (15, "Carac : %s.", node ->name);
    if (node->type != XML_ELEMENT_NODE) continue;
    if (SEQUENCE_OK != str2xmlcode ((char*)(node ->name), prop))
    {
      prop = CODE_XML_OTHER;
      back = SEQUENCE_ERROR;
    }


    switch (prop)
    {
    case CODE_XML_CAMERA_NAME:
      myXmlReadCharChild (doc, node, &val_char);
      camera_name_tmp = val_char;
     break;

    case CODE_XML_WIDTH:
      myXmlReadIntChild (doc, node, val, back);
      image_width_tmp=val;
     break;

    case CODE_XML_HEIGHT:
      myXmlReadIntChild (doc, node, val, back);
      image_height_tmp = val;
    break;
    case CODE_XML_SUBSAMPLING_WIDTH:
      myXmlReadIntChild (doc, node, val, back);
      subsampling_width_tmp = val;
     break;
    case CODE_XML_SUBSAMPLING_HEIGHT:
      myXmlReadIntChild (doc, node, val, back);
      subsampling_height_tmp = val;
    break;
    case CODE_XML_FULL_WIDTH:
      myXmlReadIntChild (doc, node, val, back);
      full_width_tmp = val;
     break;

    case CODE_XML_FULL_HEIGHT:
      myXmlReadIntChild (doc, node, val, back);
      full_height_tmp = val;
    break;

    case CODE_XML_MODEL:
      back = read_camera_model(doc, node, cam_tmp_model);
      if(cam_tmp_model.get_projModel() == projModel){
          cam_tmp = cam_tmp_model;
          projModelFound = true;
      }
      break;
    default:
      back = SEQUENCE_ERROR;
      break;
    }
  }
  if( !((projModelFound == true) && (camera_name == camera_name_tmp) &&
        (image_width == image_width_tmp || image_width == 0) &&
        (image_height == image_height_tmp || image_height == 0) &&
        (subsampling_width == subsampling_width_tmp ||
          subsampling_width == 0)&&
        (subsampling_height == subsampling_height_tmp ||
          subsampling_height == 0))){
    back = SEQUENCE_ERROR;
  }
  else{
    this->camera = cam_tmp;
    this->camera_name = camera_name_tmp;
    this->image_width = image_width_tmp;
    this->image_height = image_height_tmp;
    this->subsampling_width = subsampling_width_tmp;
    this->subsampling_height = subsampling_height_tmp;
    this->full_width = subsampling_width_tmp * image_width_tmp;
    this->full_height = subsampling_height_tmp * image_height_tmp;
  }
  return back;
}
/*!
  Read camera header fields from a XML file.

  \param doc : XML file.
  \param node : XML tree, pointing on a marker equipement.
  \param camera_name : name of the camera : usefull if the xml file has multiple
  camera parameters. Set as "" if the camera name is not ambiguous.
  \param image_width : width of image  on which camera calibration was performed.
    Set as 0 if not ambiguous.
  \param image_height : height of the image  on which camera calibration
    was performed. Set as 0 if not ambiguous.
  \param subsampling_width : scale of the image width sent by the camera.
    Set as 0 if not ambiguous.
  \param subsampling_height : scale of the image height sent by the camera.
    Set as 0 if not ambiguous.

  \return error code.

 */
int
vpXmlParserCamera::
read_camera_header (xmlDocPtr doc, xmlNodePtr node,
              const std::string& camera_name,
              const unsigned int image_width,
              const unsigned int image_height,
              const unsigned int subsampling_width,
              const unsigned int subsampling_height)
{
  char * val_char;
  vpXmlCodeType prop;
  /* read value in the XML file. */
  int val;
  std::string camera_name_tmp = "";
  unsigned int image_height_tmp = 0 ;
  unsigned int image_width_tmp = 0 ;
  unsigned int subsampling_width_tmp = 0;
  unsigned int subsampling_height_tmp = 0;
  unsigned int full_width_tmp = 0;
  unsigned int full_height_tmp = 0;
  vpXmlCodeSequenceType back = SEQUENCE_OK;

  for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
  {
    // vpDEBUG_TRACE (15, "Carac : %s.", node ->name);
    if (node->type != XML_ELEMENT_NODE) continue;
    if (SEQUENCE_OK != str2xmlcode ((char*)(node ->name), prop))
    {
      prop = CODE_XML_OTHER;
      back = SEQUENCE_ERROR;
    }


    switch (prop)
    {
      case CODE_XML_CAMERA_NAME:
        myXmlReadCharChild (doc, node, &val_char);
        camera_name_tmp = val_char;
        break;

      case CODE_XML_WIDTH:
        myXmlReadIntChild (doc, node, val, back);
        image_width_tmp=val;
        break;

      case CODE_XML_HEIGHT:
        myXmlReadIntChild (doc, node, val, back);
        image_height_tmp = val;
        break;
      case CODE_XML_SUBSAMPLING_WIDTH:
        myXmlReadIntChild (doc, node, val, back);
        subsampling_width_tmp = val;
        break;
      case CODE_XML_SUBSAMPLING_HEIGHT:
        myXmlReadIntChild (doc, node, val, back);
        subsampling_height_tmp = val;
        break;
      case CODE_XML_FULL_WIDTH:
        myXmlReadIntChild (doc, node, val, back);
        full_width_tmp = val;
        break;

      case CODE_XML_FULL_HEIGHT:
        myXmlReadIntChild (doc, node, val, back);
        full_height_tmp = val;
        break;

      case CODE_XML_MODEL:
        break;
      default:
        back = SEQUENCE_ERROR;
        break;
    }
  }
  if( !((camera_name == camera_name_tmp) &&
         (image_width == image_width_tmp || image_width == 0) &&
         (image_height == image_height_tmp || image_height == 0) &&
         (subsampling_width == subsampling_width_tmp ||
         subsampling_width == 0)&&
         (subsampling_height == subsampling_height_tmp ||
         subsampling_height == 0))){
    back = SEQUENCE_ERROR;
   }
   return back;
}

/*!
  Read camera model fields from a XML file.

  \param doc : XML file.
  \param node : XML tree, pointing on a marker equipement.
  \param cam_tmp : camera parameters to fill with read data (output).

  \return error code.

 */
vpXmlParserCamera::vpXmlCodeSequenceType
vpXmlParserCamera::read_camera_model (xmlDocPtr doc, xmlNodePtr node,
				      vpCameraParameters &cam_tmp)
{
  // counter of the number of read parameters
  int nb = 0;
  vpXmlCodeType prop;
  /* read value in the XML file. */
  double vald;
  char* val_char=NULL;

  char* model_type = NULL;
  double u0 = cam_tmp.get_u0();
  double v0 = cam_tmp.get_v0();
  double px = cam_tmp.get_px();
  double py = cam_tmp.get_py();
  double kud = cam_tmp.get_kud();
  double kdu = cam_tmp.get_kdu();
  vpXmlCodeSequenceType back = SEQUENCE_OK;
  int validation = 0;

  for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
  {
    // vpDEBUG_TRACE (15, "Carac : %s.", node ->name);
    if (node->type != XML_ELEMENT_NODE) continue;
    if (SEQUENCE_OK != str2xmlcode ((char*)(node ->name), prop))
    {
      prop = CODE_XML_OTHER;
      back = SEQUENCE_ERROR;
    }

    switch (prop)
    {
    case CODE_XML_MODEL_TYPE:
      myXmlReadCharChild (doc, node, &val_char);
      model_type = val_char;
      nb++;
      validation = validation | 0x01;
      break;
    case CODE_XML_U0:
      myXmlReadDoubleChild (doc, node, vald, back);
      u0=vald;
      nb++;
      validation = validation | 0x02;
      break;
    case CODE_XML_V0:
      myXmlReadDoubleChild (doc, node, vald, back);
      v0 = vald;
      nb++;
      validation = validation | 0x04;
      break;
    case CODE_XML_PX:
      myXmlReadDoubleChild (doc, node, vald, back);
      px = vald;
      nb++;
      validation = validation | 0x08;
      break;
    case CODE_XML_PY:
      myXmlReadDoubleChild (doc, node, vald, back);
      py = vald;
      nb++;
      validation = validation | 0x10;
     break;
    case CODE_XML_KUD:
      myXmlReadDoubleChild (doc, node, vald, back);
      kud = vald;
      nb++;
      validation = validation | 0x20;
      break;
    case CODE_XML_KDU:
      myXmlReadDoubleChild (doc, node, vald, back);
      kdu = vald;
      nb++;
      validation = validation | 0x40;
      break;
    default:
      back = SEQUENCE_ERROR;
      break;
    }
  }

  if( !strcmp(model_type,LABEL_XML_MODEL_WITHOUT_DISTORTION)){
    if (nb != 5 || validation != 0x1F)
    {
      vpCERROR <<"ERROR in 'model' field:\n";
      vpCERROR << "it must contain 5 parameters\n";
      return SEQUENCE_ERROR;
    }
    cam_tmp.initPersProjWithoutDistortion(px,py,u0,v0) ;
  }
  else if( !strcmp(model_type,LABEL_XML_MODEL_WITH_DISTORTION)){
    if (nb != 7 || validation != 0x7F)
    {
      vpCERROR <<"ERROR in 'model' field:\n";
      vpCERROR << "it must contain 7 parameters\n";
      return SEQUENCE_ERROR;
    }
    cam_tmp.initPersProjWithDistortion(px,py,u0,v0,kud,kdu);
  }
  else{
    vpERROR_TRACE("projection model type doesn't match with any known model !");
    return SEQUENCE_ERROR;
  }
  return back;
}

/*!
  Write camera parameters in an XML Tree.

  \param node : XML tree, pointing on a marker equipement.
  \param camera_name : name of the camera : usefull if the xml file has multiple
  camera parameters. Set as "" if the camera name is not ambiguous.
  \param image_width : width of image  on which camera calibration was performed.
    Set as 0 if not ambiguous.
  \param image_height : height of the image  on which camera calibration
    was performed. Set as 0 if not ambiguous.
  \param subsampling_width : subsampling of the image width sent by the camera.
    Set as 0 if not ambiguous.
  \param subsampling_height : subsampling of the image height sent by the camera.
    Set as 0 if not ambiguous.

  \return error code.
 */
int vpXmlParserCamera::
write (xmlNodePtr node, const std::string& camera_name,
      const unsigned int image_width, const unsigned int image_height,
      const unsigned int subsampling_width,
      const unsigned int subsampling_height)
{
  int back = SEQUENCE_OK;

  xmlNodePtr node_tmp;
  xmlNodePtr node_camera;

  // <camera>
  node_camera = xmlNewNode(NULL,(xmlChar*)LABEL_XML_CAMERA);
  xmlAddChild(node,node_camera);
  {
    //<name>

    if(!camera_name.empty()){
      node_tmp = xmlNewComment((xmlChar*)"Name of the camera");
      xmlAddChild(node_camera,node_tmp);
      xmlNewTextChild(node_camera,NULL,(xmlChar*)LABEL_XML_CAMERA_NAME,
        (xmlChar*)camera_name.c_str());
    }

    if(image_width != 0 || image_height != 0){
      char str[11];
      //<image_width>
      node_tmp = xmlNewComment((xmlChar*)"Size of the image on which camera calibration was performed");
      xmlAddChild(node_camera,node_tmp);

      sprintf(str,"%u",image_width);
      xmlNewTextChild(node_camera,NULL,(xmlChar*)LABEL_XML_WIDTH,(xmlChar*)str);
      //<image_height>

      sprintf(str,"%u",image_height);
      xmlNewTextChild(node_camera,NULL,(xmlChar*)LABEL_XML_HEIGHT,(xmlChar*)str);
      if(subsampling_width != 0 || subsampling_height != 0){
        node_tmp = xmlNewComment((xmlChar*)"Subsampling used to obtain the current size of the image.");
        xmlAddChild(node_camera,node_tmp);

        //<subsampling_width>
        sprintf(str,"%u",subsampling_width);
        xmlNewTextChild(node_camera,NULL,(xmlChar*)LABEL_XML_SUBSAMPLING_WIDTH,
                        (xmlChar*)str);
        //<subsampling_height>
        sprintf(str,"%u",subsampling_height);
        xmlNewTextChild(node_camera,NULL,(xmlChar*)LABEL_XML_SUBSAMPLING_HEIGHT,
                        (xmlChar*)str);
        node_tmp = xmlNewComment((xmlChar*)"The full size is the sensor size actually used to grab the image. full_width = subsampling_width * image_width");
        xmlAddChild(node_camera,node_tmp);

        //<full_width>
        sprintf(str,"%u",image_width*subsampling_width);
        xmlNewTextChild(node_camera,NULL,(xmlChar*)LABEL_XML_FULL_WIDTH,
                        (xmlChar*)str);
        //<full_height>
        sprintf(str,"%u",image_height*subsampling_height);
        xmlNewTextChild(node_camera,NULL,(xmlChar*)LABEL_XML_FULL_HEIGHT,
                        (xmlChar*)str);
      }
    }

    node_tmp = xmlNewComment((xmlChar*)"Intrinsic camera parameters computed for each projection model");

    xmlAddChild(node_camera,node_tmp);

    back = write_camera(node_camera);
  }
  return back;
}
 /*!
  Write camera parameters in an XML Tree.

  \param node_camera : XML pointer node, pointing on a camera node.

  \return error code.
  */
int vpXmlParserCamera::
write_camera(xmlNodePtr node_camera){
  xmlNodePtr node_model;
  xmlNodePtr node_tmp;

  int back = SEQUENCE_OK;
  switch(camera.get_projModel()){
  case vpCameraParameters::perspectiveProjWithoutDistortion :
    //<model>
    node_model = xmlNewNode(NULL,(xmlChar*)LABEL_XML_MODEL);
    xmlAddChild(node_camera,node_model);
    {
      char str[21];
      node_tmp = xmlNewComment((xmlChar*)"Projection model type");
      xmlAddChild(node_model,node_tmp);

      //<type>without_distortion</type>
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_MODEL_TYPE,
                      (xmlChar*)LABEL_XML_MODEL_WITHOUT_DISTORTION);

      node_tmp = xmlNewComment((xmlChar*)"Pixel ratio");
      xmlAddChild(node_model,node_tmp);
      //<px>
      sprintf(str,"%.10f",camera.get_px());
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_PX,(xmlChar*)str);
      //<py>
      sprintf(str,"%.10f",camera.get_py());
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_PY,(xmlChar*)str);

      node_tmp = xmlNewComment((xmlChar*)"Principal point");
      xmlAddChild(node_model,node_tmp);

      //<u0>
      sprintf(str,"%.10f",camera.get_u0());
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_U0,(xmlChar*)str);
      //<v0>
      sprintf(str,"%.10f",camera.get_v0());
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_V0,(xmlChar*)str);
    }
    break;
  case vpCameraParameters::perspectiveProjWithDistortion :
    //<model>
    node_model = xmlNewNode(NULL,(xmlChar*)LABEL_XML_MODEL);
    xmlAddChild(node_camera,node_model);
    {
      char str[21];
      node_tmp = xmlNewComment((xmlChar*)"Projection model type");
      xmlAddChild(node_model,node_tmp);
      //<type>with_distortion</type>
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_MODEL_TYPE,
                      (xmlChar*)LABEL_XML_MODEL_WITH_DISTORTION);

      node_tmp = xmlNewComment((xmlChar*)"Pixel ratio");
      xmlAddChild(node_model,node_tmp);
      //<px>
      sprintf(str,"%.10f",camera.get_px());
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_PX,(xmlChar*)str);
      //<py>
      sprintf(str,"%.10f",camera.get_py());
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_PY,(xmlChar*)str);

      node_tmp = xmlNewComment((xmlChar*)"Principal point");
      xmlAddChild(node_model,node_tmp);
      //<u0>
      sprintf(str,"%.10f",camera.get_u0());
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_U0,(xmlChar*)str);
      //<v0>
      sprintf(str,"%.10f",camera.get_v0());
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_V0,(xmlChar*)str);

      //<kud>
      node_tmp = xmlNewComment((xmlChar*)"Undistorted to distorted distortion parameter");
      xmlAddChild(node_model,node_tmp);
      sprintf(str,"%.10f",camera.get_kud());
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_KUD,(xmlChar*)str);

      //<kud>
      node_tmp = xmlNewComment((xmlChar*)"Distorted to undistorted distortion parameter");
      xmlAddChild(node_model,node_tmp);
      sprintf(str,"%.10f",camera.get_kdu());
      xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_KDU,(xmlChar*)str);
    }
    break;
  }
  return back;
}

/*!
  Translate a string (label) to a xml code.
  \param str : string to translate.
  \param res : resulting code.

  \return error code.
*/

vpXmlParserCamera::vpXmlCodeSequenceType
vpXmlParserCamera::str2xmlcode (char * str, vpXmlCodeType & res)
{
  vpXmlCodeType val_int = CODE_XML_BAD;
  vpXmlCodeSequenceType back = vpXmlParserCamera::SEQUENCE_OK;

  // DEBUG_TRACE (9, "# Entree :str=%s.", str);

  if (! strcmp (str,  LABEL_XML_CAMERA))
  {
    val_int = CODE_XML_CAMERA;
  }
  else if (! strcmp (str,  LABEL_XML_CAMERA_NAME))
  {
    val_int = CODE_XML_CAMERA_NAME;
  }
  else if (! strcmp (str,  LABEL_XML_MODEL))
  {
    val_int = CODE_XML_MODEL;
  }
  else if (! strcmp (str,  LABEL_XML_MODEL_TYPE))
  {
    val_int = CODE_XML_MODEL_TYPE;
  }
  else if (! strcmp (str,  LABEL_XML_WIDTH))
  {
    val_int = CODE_XML_WIDTH;
  }
  else if (! strcmp (str,  LABEL_XML_HEIGHT))
  {
    val_int = CODE_XML_HEIGHT;
  }
  else if (! strcmp (str,  LABEL_XML_SUBSAMPLING_WIDTH))
  {
    val_int = CODE_XML_SUBSAMPLING_WIDTH;
  }
  else if (! strcmp (str,  LABEL_XML_SUBSAMPLING_HEIGHT))
  {
    val_int = CODE_XML_SUBSAMPLING_HEIGHT;
  }
  else if (! strcmp (str,  LABEL_XML_FULL_WIDTH))
  {
    val_int = CODE_XML_FULL_WIDTH;
  }
  else if (! strcmp (str,  LABEL_XML_FULL_HEIGHT))
  {
    val_int = CODE_XML_FULL_HEIGHT;
  }
  else if (! strcmp (str,  LABEL_XML_U0))
  {
    val_int = CODE_XML_U0;
  }
  else if (! strcmp (str,  LABEL_XML_V0))
  {
    val_int = CODE_XML_V0;
  }
  else if (! strcmp (str,  LABEL_XML_PX))
  {
    val_int = CODE_XML_PX;
  }
  else if (! strcmp (str,  LABEL_XML_PY))
  {
    val_int = CODE_XML_PY;
  }
  else if (! strcmp (str,  LABEL_XML_KUD))
  {
    val_int = CODE_XML_KUD;
  }
  else if (! strcmp (str,  LABEL_XML_KDU))
  {
    val_int = CODE_XML_KDU;
  }
  else
  {
    val_int = CODE_XML_OTHER;
  }
  res = val_int;

  return back;
}
 #endif //VISP_HAVE_XML2

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
