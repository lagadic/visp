/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * XML parser to load and save camera intrinsic parameters.
 *
*****************************************************************************/

/*!
  \file vpXmlParserCamera.cpp
  \brief Definition of the vpXmlParserCamera class member functions.
  Class vpXmlParserCamera allowed to load and save intrinsic camera parameters
*/

#include <visp3/core/vpXmlParserCamera.h>

#if defined(VISP_HAVE_PUGIXML)
#include <pugixml.hpp>

/* --------------------------------------------------------------------------
 */
 /* --- LABEL XML ------------------------------------------------------------
  */
  /* --------------------------------------------------------------------------
   */

#define LABEL_XML_ROOT "root"
#define LABEL_XML_CAMERA "camera"
#define LABEL_XML_CAMERA_NAME "name"
#define LABEL_XML_WIDTH "image_width"
#define LABEL_XML_HEIGHT "image_height"
#define LABEL_XML_SUBSAMPLING_WIDTH "subsampling_width"
#define LABEL_XML_SUBSAMPLING_HEIGHT "subsampling_height"
#define LABEL_XML_FULL_WIDTH "full_width"
#define LABEL_XML_FULL_HEIGHT "full_height"
#define LABEL_XML_MODEL "model"
#define LABEL_XML_MODEL_TYPE "type"
#define LABEL_XML_U0 "u0"
#define LABEL_XML_V0 "v0"
#define LABEL_XML_PX "px"
#define LABEL_XML_PY "py"
#define LABEL_XML_KUD "kud"
#define LABEL_XML_KDU "kdu"
#define LABEL_XML_K1 "k1"
#define LABEL_XML_K2 "k2"
#define LABEL_XML_K3 "k3"
#define LABEL_XML_K4 "k4"
#define LABEL_XML_K5 "k5"

#define LABEL_XML_MODEL_WITHOUT_DISTORTION "perspectiveProjWithoutDistortion"
#define LABEL_XML_MODEL_WITH_DISTORTION "perspectiveProjWithDistortion"
#define LABEL_XML_MODEL_WITH_KANNALA_BRANDT_DISTORTION "ProjWithKannalaBrandtDistortion"

#define LABEL_XML_ADDITIONAL_INFO "additional_information"

BEGIN_VISP_NAMESPACE
#ifndef DOXYGEN_SHOULD_SKIP_THIS
class vpXmlParserCamera::Impl
{
public:  /* --- XML Code------------------------------------------------------------
   */
  enum vpXmlCodeType
  {
    CODE_XML_BAD = -1,
    CODE_XML_OTHER,
    CODE_XML_CAMERA,
    CODE_XML_CAMERA_NAME,
    CODE_XML_HEIGHT,
    CODE_XML_WIDTH,
    CODE_XML_SUBSAMPLING_WIDTH,
    CODE_XML_SUBSAMPLING_HEIGHT,
    CODE_XML_FULL_HEIGHT,
    CODE_XML_FULL_WIDTH,
    CODE_XML_MODEL,
    CODE_XML_MODEL_TYPE,
    CODE_XML_U0,
    CODE_XML_V0,
    CODE_XML_PX,
    CODE_XML_PY,
    CODE_XML_KUD,
    CODE_XML_KDU,
    CODE_XML_K1,
    CODE_XML_K2,
    CODE_XML_K3,
    CODE_XML_K4,
    CODE_XML_K5,
    CODE_XML_ADDITIONAL_INFO
  };

  Impl()
    : camera(), camera_name(), image_width(0), image_height(0), subsampling_width(0), subsampling_height(0),
    full_width(0), full_height(0)
  { }

  int parse(vpCameraParameters &cam, const std::string &filename, const std::string &cam_name,
            const vpCameraParameters::vpCameraParametersProjType &projModel, unsigned int im_width,
            unsigned int im_height, bool verbose)
  {
    pugi::xml_document doc;
    if (!doc.load_file(filename.c_str())) {
      return SEQUENCE_ERROR;
    }

    pugi::xml_node node = doc.document_element();
    if (!node) {
      return SEQUENCE_ERROR;
    }

    int ret = read(node, cam_name, projModel, im_width, im_height, verbose);

    cam = camera;

    return ret;
  }

  /*!
    Read camera parameters from a XML file.
    \param node_ : XML tree, pointing on a marker equipment.
    \param cam_name : Name of the camera. Useful if the xml file has multiple
    camera parameters. Set to an empty string "" if the camera name is not ambiguous.
    \param projModel : Projection model type.
    \param im_width : width of image  on which camera calibration was performed.
    Set to 0 if not ambiguous.
    \param im_height : height of the image  on which camera calibration
    was performed. Set as 0 if not ambiguous.
    \param subsampl_width : subsampling of the image width sent by the camera.
    Set to 0 if not ambiguous.
    \param subsampl_height : subsampling of the image height sent by the camera.
    Set to 0 if not ambiguous.
    \param verbose true to enable verbose mode, false otherwise.
    \return vpXmlParserCamera::SEQUENCE_OK if success and vpXmlParserCamera::SEQUENCE_ERROR otherwise.
   */
  int read(const pugi::xml_node &node_, const std::string &cam_name,
           const vpCameraParameters::vpCameraParametersProjType &projModel, unsigned int im_width,
           unsigned int im_height, bool verbose, unsigned int subsampl_width = 0, unsigned int subsampl_height = 0)
  {
    vpXmlCodeType prop;
    vpXmlCodeSequenceType back = SEQUENCE_OK;
    unsigned int nbCamera = 0;

    for (pugi::xml_node node = node_.first_child(); node; node = node.next_sibling()) {
      if (node.type() == pugi::node_element) {
        if (SEQUENCE_OK != str2xmlcode(node.name(), prop)) {
          prop = CODE_XML_OTHER;
          back = SEQUENCE_ERROR;
        }
        if (prop == CODE_XML_CAMERA) {
          if (SEQUENCE_OK == read_camera(node, cam_name, projModel, im_width, im_height, subsampl_width, subsampl_height, verbose)) {
            ++nbCamera;
          }
        }
        else {
          back = SEQUENCE_ERROR;
        }
      }
    }

    if (nbCamera == 0) {
      back = SEQUENCE_ERROR;
      std::cout << "Warning: No camera parameters is available" << std::endl << "with your specifications" << std::endl;
    }
    else if (nbCamera > 1) {
      back = SEQUENCE_ERROR;
      std::cout << "Warning: " << nbCamera << " sets of camera parameters are available" << std::endl
        << "with your specifications : " << std::endl
        << "precise your choice..." << std::endl;
    }

    return back;
  }

  /*!
    Read camera fields from a XML file.
    \param node_ : XML tree, pointing on a marker equipment.
    \param cam_name : Name of the camera. Useful if the xml file has multiple
    camera parameters. Set to an empty string "" if the camera name is not ambiguous.
    \param projModel : Projection model type.
    \param im_width : width of image  on which camera calibration was performed.
    Set to 0 if not ambiguous.
    \param im_height : height of the image  on which camera calibration
    was performed. Set as 0 if not ambiguous.
    \param subsampl_width : scale of the image width sent by the camera.
    Set to 0 if not ambiguous.
    \param subsampl_height : scale of the image height sent by the camera.
    Set to 0 if not ambiguous.
    \param verbose true to enable verbose mode, false otherwise.
    \return Error code: SEQUENCE_OK when the required camera is found, SEQUENCE_ERROR otherwise.
   */
  int read_camera(const pugi::xml_node &node_, const std::string &cam_name,
                  const vpCameraParameters::vpCameraParametersProjType &projModel, unsigned int im_width,
                  unsigned int im_height, unsigned int subsampl_width, unsigned int subsampl_height, bool verbose)
  {
    vpXmlCodeType prop;
    /* read value in the XML file. */
    std::string camera_name_tmp = "";
    unsigned int image_height_tmp = 0;
    unsigned int image_width_tmp = 0;
    unsigned int subsampling_width_tmp = 0;
    unsigned int subsampling_height_tmp = 0;
    vpCameraParameters cam_tmp;
    vpCameraParameters cam_tmp_model;
    bool same_proj_model = false;
    vpXmlCodeSequenceType back = SEQUENCE_OK;

    for (pugi::xml_node node = node_.first_child(); node; node = node.next_sibling()) {
      if (node.type() == pugi::node_element) {
        if (SEQUENCE_OK != str2xmlcode(node.name(), prop)) {
          prop = CODE_XML_OTHER;
          back = SEQUENCE_ERROR;
        }

        switch (prop) {
        case CODE_XML_CAMERA_NAME: {
          camera_name_tmp = node.text().as_string();
          if (verbose) {
            std::cout << "Found camera with name: \"" << camera_name_tmp << "\"" << std::endl;
          }
          break;
        }
        case CODE_XML_WIDTH:
          image_width_tmp = node.text().as_uint();
          break;

        case CODE_XML_HEIGHT:
          image_height_tmp = node.text().as_uint();
          break;
        case CODE_XML_SUBSAMPLING_WIDTH:
          subsampling_width_tmp = node.text().as_uint();
          break;
        case CODE_XML_SUBSAMPLING_HEIGHT:
          subsampling_height_tmp = node.text().as_uint();
          break;

        case CODE_XML_MODEL:
          back = read_camera_model(node, cam_tmp_model);
          if (cam_tmp_model.get_projModel() == projModel) {
            cam_tmp = cam_tmp_model;
            same_proj_model = true; // Same projection model
          }
          break;

        case CODE_XML_ADDITIONAL_INFO:
          break;

        case CODE_XML_BAD:
        case CODE_XML_OTHER:
        case CODE_XML_CAMERA:
        case CODE_XML_FULL_HEIGHT:
        case CODE_XML_FULL_WIDTH:
        case CODE_XML_MODEL_TYPE:
        case CODE_XML_U0:
        case CODE_XML_V0:
        case CODE_XML_PX:
        case CODE_XML_PY:
        case CODE_XML_KUD:
        case CODE_XML_KDU:
        case CODE_XML_K1:
        case CODE_XML_K2:
        case CODE_XML_K3:
        case CODE_XML_K4:
        case CODE_XML_K5:
        default:
          back = SEQUENCE_ERROR;

          break;
        }
      }
    }
    // Create a specific test for subsampling_width and subsampling_height to
    // ensure that division by zero is not possible in the next test
    bool test_subsampling_width = true;
    bool test_subsampling_height = true;

    if (subsampling_width) {
      test_subsampling_width = (abs(static_cast<int>(subsampl_width) - static_cast<int>(subsampling_width_tmp)) <
                                (allowedPixelDiffOnImageSize * static_cast<int>(subsampling_width_tmp / subsampling_width)));
    }
    if (subsampling_height) {
      test_subsampling_height = (abs(static_cast<int>(subsampl_height) - static_cast<int>(subsampling_height_tmp)) <
                                 (allowedPixelDiffOnImageSize * static_cast<int>(subsampling_height_tmp / subsampling_height)));
    }
    // if same name && same projection model && same image size camera already exists, we return SEQUENCE_OK
    // otherwise it is a new camera that need to be updated and we return SEQUENCE_OK
    bool same_name = (cam_name.empty() || (cam_name == camera_name_tmp));
    bool same_img_size = (abs(static_cast<int>(im_width) - static_cast<int>(image_width_tmp)) < allowedPixelDiffOnImageSize || im_width == 0) &&
      (abs(static_cast<int>(im_height) - static_cast<int>(image_height_tmp)) < allowedPixelDiffOnImageSize || im_height == 0) &&
      (test_subsampling_width) && (test_subsampling_height);
    if (same_name && same_img_size && same_proj_model) {
      back = SEQUENCE_OK; // Camera exists
      camera = cam_tmp;
      camera_name = camera_name_tmp;
      image_width = image_width_tmp;
      image_height = image_height_tmp;
      subsampling_width = subsampling_width_tmp;
      subsampling_height = subsampling_height_tmp;
      full_width = subsampling_width_tmp * image_width_tmp;
      full_height = subsampling_height_tmp * image_height_tmp;
    }
    else {

      back = SEQUENCE_ERROR; // Camera doesn't exist yet in the file
    }
#if 0
    if (!((projModelFound == true) &&
          (abs(static_cast<int>(im_width) - static_cast<int>(image_width_tmp)) < allowedPixelDiffOnImageSize || im_width == 0) &&
          (abs(static_cast<int>(im_height) - static_cast<int>(image_height_tmp)) < allowedPixelDiffOnImageSize || im_height == 0) &&
          (test_subsampling_width) && (test_subsampling_height))) {
      // Same images size, we need to check if the camera have the same name
      if (!cam_name.empty() && (cam_name != camera_name_tmp)) {
        back = SEQUENCE_ERROR; // Camera doesn't exist yet in the file
      }
      else {
        back = SEQUENCE_OK; // Camera already found
      }
    }
    else {
      camera = cam_tmp;
      camera_name = camera_name_tmp;
      image_width = image_width_tmp;
      image_height = image_height_tmp;
      subsampling_width = subsampling_width_tmp;
      subsampling_height = subsampling_height_tmp;
      full_width = subsampling_width_tmp * image_width_tmp;
      full_height = subsampling_height_tmp * image_height_tmp;
      back = SEQUENCE_ERROR; // Camera doesn't exist yet in the file
    }
#endif
    return back;
  }

  /*!
    Read camera model fields from a XML file.
    \param node_ : XML tree, pointing on a marker equipment.
    \param cam_tmp : Camera parameters to fill with read data (output).
    \return Error code.
   */
  vpXmlCodeSequenceType read_camera_model(const pugi::xml_node &node_, vpCameraParameters &cam_tmp)
  {
    // counter of the number of read parameters
    int nb = 0;
    vpXmlCodeType prop;
    /* read value in the XML file. */

    std::string model_type = "";
    double u0 = cam_tmp.get_u0();
    double v0 = cam_tmp.get_v0();
    double px = cam_tmp.get_px();
    double py = cam_tmp.get_py();
    double kud = cam_tmp.get_kud();
    double kdu = cam_tmp.get_kdu();
    std::vector<double> distortion_coeffs;
    vpXmlCodeSequenceType back = SEQUENCE_OK;
    unsigned int validation = 0;

    for (pugi::xml_node node = node_.first_child(); node; node = node.next_sibling()) {
      if (node.type() == pugi::node_element) {
        if (SEQUENCE_OK != str2xmlcode(node.name(), prop)) {
          prop = CODE_XML_OTHER;
          back = SEQUENCE_ERROR;
        }

        switch (prop) {
        case CODE_XML_MODEL_TYPE: {
          model_type = node.text().as_string();
          ++nb;
          validation = validation | 0x01;
        } break;
        case CODE_XML_U0:
          u0 = node.text().as_double();
          ++nb;
          validation = validation | 0x02;
          break;
        case CODE_XML_V0:
          v0 = node.text().as_double();
          ++nb;
          validation = validation | 0x04;
          break;
        case CODE_XML_PX:
          px = node.text().as_double();
          ++nb;
          validation = validation | 0x08;
          break;
        case CODE_XML_PY:
          py = node.text().as_double();
          ++nb;
          validation = validation | 0x10;
          break;
        case CODE_XML_KUD:
          kud = node.text().as_double();
          ++nb;
          validation = validation | 0x20;
          break;
        case CODE_XML_KDU:
          kdu = node.text().as_double();
          ++nb;
          validation = validation | 0x40;
          break;
        case CODE_XML_K1:
          distortion_coeffs.push_back(node.text().as_double());
          ++nb;
          validation = validation | 0x20;
          break;
        case CODE_XML_K2:
          distortion_coeffs.push_back(node.text().as_double());
          ++nb;
          validation = validation | 0x40;
          break;
        case CODE_XML_K3:
          distortion_coeffs.push_back(node.text().as_double());
          ++nb;
          validation = validation | 0x80;
          break;
        case CODE_XML_K4:
          distortion_coeffs.push_back(node.text().as_double());
          ++nb;
          validation = validation | 0x100;
          break;
        case CODE_XML_K5:
          distortion_coeffs.push_back(node.text().as_double());
          ++nb;
          validation = validation | 0x200;
          break;
        case CODE_XML_BAD:
        case CODE_XML_OTHER:
        case CODE_XML_CAMERA:
        case CODE_XML_CAMERA_NAME:
        case CODE_XML_HEIGHT:
        case CODE_XML_WIDTH:
        case CODE_XML_SUBSAMPLING_WIDTH:
        case CODE_XML_SUBSAMPLING_HEIGHT:
        case CODE_XML_FULL_HEIGHT:
        case CODE_XML_FULL_WIDTH:
        case CODE_XML_MODEL:
        case CODE_XML_ADDITIONAL_INFO:
        default:
          back = SEQUENCE_ERROR;
          break;
        }
      }
    }

    if (model_type.empty()) {
      std::cout << "Warning: projection model type doesn't match with any known model !" << std::endl;
      return SEQUENCE_ERROR;
    }

    if (!strcmp(model_type.c_str(), LABEL_XML_MODEL_WITHOUT_DISTORTION)) {
      if (nb != 5 || validation != 0x001F) {
        std::cout << "ERROR in 'model' field:\n";
        std::cout << "it must contain 5 parameters\n";

        return SEQUENCE_ERROR;
      }
      cam_tmp.initPersProjWithoutDistortion(px, py, u0, v0);
    }
    else if (!strcmp(model_type.c_str(), LABEL_XML_MODEL_WITH_DISTORTION)) {
      if (nb != 7 || validation != 0x7F) {
        std::cout << "ERROR in 'model' field:\n";
        std::cout << "it must contain 7 parameters\n";

        return SEQUENCE_ERROR;
      }
      cam_tmp.initPersProjWithDistortion(px, py, u0, v0, kud, kdu);
    }
    else if (!strcmp(model_type.c_str(), LABEL_XML_MODEL_WITH_KANNALA_BRANDT_DISTORTION)) {
      if (nb != 10 || validation != 0x3FF) { // at least one coefficient is missing. We should know which one
        std::cout << "ERROR in 'model' field:\n";
        std::cout << "it must contain 10 parameters\n";

        std::vector<double> fixed_distortion_coeffs;

        // In case distortion coefficients are missing, we should complete them with 0 values
        // Since 0x3FF is 0011|1111|1111 and we are interested in the most significant 1s shown below
        //                  -- ---
        // If we divide by 32 (>> 2^5 : 5 remaining least significant bits), we will have to check 5 bits only
        const int dividerForBitCheck = 32;
        int check = validation / dividerForBitCheck;
        int j = 0;

        const int nbRemainingBits = 5;
        const int moduloForOddity = 2;
        const int dividerForRightShift = 2;
        for (int i = 0; i < nbRemainingBits; ++i) {
          int bit = check % moduloForOddity; // if bit == 1 => the corresponding distortion coefficient is present.
          if (!bit) {
            fixed_distortion_coeffs.push_back(0.);
          }
          else {
            fixed_distortion_coeffs.push_back(distortion_coeffs[j++]);
          }
          check /= dividerForRightShift;
        }

        cam_tmp.initProjWithKannalaBrandtDistortion(px, py, u0, v0, fixed_distortion_coeffs);
        return SEQUENCE_ERROR;
      }
      cam_tmp.initProjWithKannalaBrandtDistortion(px, py, u0, v0, distortion_coeffs);
    }
    else {
      std::cout << "Warning: projection model type doesn't match with any known model !" << std::endl;

      return SEQUENCE_ERROR;
    }
    return back;
  }

  int save(const vpCameraParameters &cam, const std::string &filename, const std::string &cam_name,
           unsigned int im_width, unsigned int im_height, const std::string &additionalInfo, bool verbose)
  {
    pugi::xml_document doc;
    pugi::xml_node node;

    if (!doc.load_file(filename.c_str(), pugi::parse_default | pugi::parse_comments)) {
      node = doc.append_child(pugi::node_declaration);
      node.append_attribute("version") = "1.0";
      node = doc.append_child(LABEL_XML_ROOT);
      pugi::xml_node nodeComment = node.append_child(pugi::node_comment);
      nodeComment.set_value("This file stores intrinsic camera parameters used\n"
                            "   in the vpCameraParameters Class of ViSP available\n"
                            "   at https://visp.inria.fr/download/ .\n"
                            "   It can be read with the parse method of\n"
                            "   the vpXmlParserCamera class.");
    }

    node = doc.document_element();
    if (!node) {
      return SEQUENCE_ERROR;
    }

    camera = cam;

    int nbCamera = count(node, cam_name, cam.get_projModel(), im_width, im_height, verbose);
    if (nbCamera) {
      return SEQUENCE_ERROR;
    }

    pugi::xml_node nodeCamera = find_camera(node, cam_name, im_width, im_height);
    if (!nodeCamera) {
      write(node, cam_name, im_width, im_height);
    }
    else {
      write_camera(nodeCamera);
    }

    if (!additionalInfo.empty()) {
      // Get camera node pointer
      nodeCamera = find_camera(node, cam_name, im_width, im_height);

      // Additional information provided by the user
      pugi::xml_node nodeAdditionalInfo = find_additional_info(nodeCamera);

      if (!nodeAdditionalInfo) {
        // Create the additional information node
        pugi::xml_node node_comment = nodeCamera.append_child(pugi::node_comment);
        node_comment.set_value("Additional information");

        nodeAdditionalInfo = nodeCamera.append_child(LABEL_XML_ADDITIONAL_INFO);
      }

      if (nodeAdditionalInfo) {
        // Add the information in this specific node
        pugi::xml_document tmpDoc;
        if (tmpDoc.load_string(additionalInfo.c_str())) {
          for (node = tmpDoc.first_child(); node; node = node.next_sibling()) {
            nodeAdditionalInfo.append_copy(node);
          }
        }
      }
    }

    doc.save_file(filename.c_str(), PUGIXML_TEXT("  "));

    return SEQUENCE_OK;
  }

  /*!
    Read camera parameters from a XML file and count the number of available
    sets of camera parameters corresponding with inputs.
    \param node_ : XML tree, pointing on a marker equipment.
    \param cam_name : name of the camera : useful if the xml file has multiple
    camera parameters. Set as "" if the camera name is not ambiguous.
    \param projModel : Projection model type.
    \param im_width : width of image  on which camera calibration was performed.
      Set as 0 if not ambiguous.
    \param im_height : height of the image  on which camera calibration
      was performed. Set as 0 if not ambiguous.
    \param subsampl_width : subsampling of the image width sent by the camera.
      Set as 0 if not ambiguous.
    \param subsampl_height : subsampling of the image height sent by the camera.
      Set as 0 if not ambiguous.
    \param verbose true to enable verbose mode, false otherwise.
    \return number of available camera parameters corresponding with inputs.
   */
  int count(const pugi::xml_node &node_, const std::string &cam_name,
            const vpCameraParameters::vpCameraParametersProjType &projModel, unsigned int im_width,
            unsigned int im_height, bool verbose, unsigned int subsampl_width = 0, unsigned int subsampl_height = 0)
  {
    vpXmlCodeType prop;
    int nbCamera = 0;

    for (pugi::xml_node node = node_.first_child(); node; node = node.next_sibling()) {
      if (node.type() == pugi::node_element) {
        if (SEQUENCE_OK != str2xmlcode(node.name(), prop)) {
          prop = CODE_XML_OTHER;
        }

        if (prop == CODE_XML_CAMERA) {
          if (SEQUENCE_OK == read_camera(node, cam_name, projModel, im_width, im_height, subsampl_width, subsampl_height, verbose)) {
            ++nbCamera;
          }
        }
      }
    }

    return nbCamera;
  }

  /*!
    Read camera headers from a XML file and return the last available
    node pointeur in the xml tree corresponding with inputs.
    \param node_ : XML tree, pointing on a marker equipment.
    \param cam_name : name of the camera : useful if the xml file has multiple
    camera parameters. Set as "" if the camera name is not ambiguous.
    \param im_width : width of image  on which camera calibration was performed.
      Set as 0 if not ambiguous.
    \param im_height : height of the image  on which camera calibration
      was performed. Set as 0 if not ambiguous.
    \param subsampl_width : subsampling of the image width sent by the camera.
      Set as 0 if not ambiguous.
    \param subsampl_height : subsampling of the image height sent by the camera.
      Set as 0 if not ambiguous.
    \return number of available camera parameters corresponding with inputs.
   */
  pugi::xml_node find_camera(const pugi::xml_node &node_, const std::string &cam_name, unsigned int im_width,
                             unsigned int im_height, unsigned int subsampl_width = 0, unsigned int subsampl_height = 0)
  {
    vpXmlCodeType prop;
    pugi::xml_node resNode = pugi::xml_node();

    pugi::xml_node node = node_.first_child();
    bool hasNotFoundCam = true;
    while (node && hasNotFoundCam) {
      if (node.type() == pugi::node_element) {
        if (SEQUENCE_OK != str2xmlcode(node.name(), prop)) {
          prop = CODE_XML_OTHER;
        }
        if (prop == CODE_XML_CAMERA) {
          if (SEQUENCE_OK == read_camera_header(node, cam_name, im_width, im_height, subsampl_width, subsampl_height)) {
            resNode = node;
            hasNotFoundCam = false;
          }
        }
      }
      node = node.next_sibling();
    }
    return resNode;
  }

  /*!
    Read camera header fields from a XML file.
    \param node_ : XML tree, pointing on a marker equipment.
    \param cam_name : name of the camera : useful if the xml file has multiple
    camera parameters. Set as "" if the camera name is not ambiguous.
    \param im_width : width of image  on which camera calibration was performed.
      Set as 0 if not ambiguous.
    \param im_height : height of the image  on which camera calibration
      was performed. Set as 0 if not ambiguous.
    \param subsampl_width : scale of the image width sent by the camera.
      Set as 0 if not ambiguous.
    \param subsampl_height : scale of the image height sent by the camera.
      Set as 0 if not ambiguous.
    \return error code.
   */
  int read_camera_header(const pugi::xml_node &node_, const std::string &cam_name, unsigned int im_width,
                         unsigned int im_height, unsigned int subsampl_width = 0, unsigned int subsampl_height = 0)
  {
    vpXmlCodeType prop;
    /* read value in the XML file. */
    std::string camera_name_tmp = "";
    unsigned int image_height_tmp = 0;
    unsigned int image_width_tmp = 0;
    unsigned int subsampling_width_tmp = 0;
    unsigned int subsampling_height_tmp = 0;
    vpXmlCodeSequenceType back = SEQUENCE_OK;

    for (pugi::xml_node node = node_.first_child(); node; node = node.next_sibling()) {
      if (node.type() == pugi::node_element) {
        if (SEQUENCE_OK != str2xmlcode(node.name(), prop)) {
          prop = CODE_XML_OTHER;
          back = SEQUENCE_ERROR;
        }

        switch (prop) {
        case CODE_XML_CAMERA_NAME:
          camera_name_tmp = node.text().as_string();
          break;

        case CODE_XML_WIDTH:
          image_width_tmp = node.text().as_uint();
          break;

        case CODE_XML_HEIGHT:
          image_height_tmp = node.text().as_uint();
          break;

        case CODE_XML_SUBSAMPLING_WIDTH:
          subsampling_width_tmp = node.text().as_uint();
          break;

        case CODE_XML_SUBSAMPLING_HEIGHT:
          subsampling_height_tmp = node.text().as_uint();
          break;

        case CODE_XML_MODEL:
          break;

        case CODE_XML_ADDITIONAL_INFO:
          break;

        case CODE_XML_BAD:
        case CODE_XML_OTHER:
        case CODE_XML_CAMERA:
        case CODE_XML_FULL_HEIGHT:
        case CODE_XML_FULL_WIDTH:
        case CODE_XML_MODEL_TYPE:
        case CODE_XML_U0:
        case CODE_XML_V0:
        case CODE_XML_PX:
        case CODE_XML_PY:
        case CODE_XML_KUD:
        case CODE_XML_KDU:
        default:
          back = SEQUENCE_ERROR;
          break;
        }
      }
    }
    if (!((cam_name == camera_name_tmp) && (im_width == image_width_tmp || im_width == 0) &&
          (im_height == image_height_tmp || im_height == 0) &&
          (subsampl_width == subsampling_width_tmp || subsampl_width == 0) &&
          (subsampl_height == subsampling_height_tmp || subsampl_height == 0))) {
      back = SEQUENCE_ERROR;
    }
    return back;
  }

  /*!
    Write camera parameters in an XML Tree.
    \param node : XML tree, pointing on a marker equipment.
    \param cam_name : name of the camera : useful if the xml file has multiple
    camera parameters. Set as "" if the camera name is not ambiguous.
    \param im_width : width of image  on which camera calibration was performed.
      Set as 0 if not ambiguous.
    \param im_height : height of the image  on which camera calibration
      was performed. Set as 0 if not ambiguous.
    \param subsampl_width : subsampling of the image width sent by the camera.
      Set as 0 if not ambiguous.
    \param subsampl_height : subsampling of the image height sent by the camera.
      Set as 0 if not ambiguous.
    \return error code.
   */
  int write(pugi::xml_node &node, const std::string &cam_name, unsigned int im_width, unsigned int im_height,
            unsigned int subsampl_width = 0, unsigned int subsampl_height = 0)
  {
    int back = SEQUENCE_OK;

    // <camera>
    pugi::xml_node node_camera = node.append_child(LABEL_XML_CAMERA);

    pugi::xml_node node_tmp;
    {
      //<name>
      if (!cam_name.empty()) {
        node_tmp = node_camera.append_child(pugi::node_comment);
        node_tmp.set_value("Name of the camera");
        node_tmp = node_camera.append_child(LABEL_XML_CAMERA_NAME);
        node_tmp.append_child(pugi::node_pcdata).set_value(cam_name.c_str());
      }

      if (im_width != 0 || im_height != 0) {
        node_tmp = node_camera.append_child(pugi::node_comment);
        node_tmp.set_value("Size of the image on which camera "
                           "calibration was performed");

        //<image_width>
        node_tmp = node_camera.append_child(LABEL_XML_WIDTH);
        node_tmp.append_child(pugi::node_pcdata).text() = im_width;

        //<image_height>
        node_tmp = node_camera.append_child(LABEL_XML_HEIGHT);
        node_tmp.append_child(pugi::node_pcdata).text() = im_height;
        if (subsampling_width != 0 || subsampling_height != 0) {
          node_tmp = node_camera.append_child(pugi::node_comment);
          node_tmp.set_value("Subsampling used to obtain the "
                             "current size of the image.");

          //<subsampling_width>
          node_tmp = node_camera.append_child(LABEL_XML_SUBSAMPLING_WIDTH);
          node_tmp.append_child(pugi::node_pcdata).text() = subsampl_width;
          //<subsampling_height>
          node_tmp = node_camera.append_child(LABEL_XML_SUBSAMPLING_HEIGHT);
          node_tmp.append_child(pugi::node_pcdata).text() = subsampl_height;
          node_tmp = node_camera.append_child(pugi::node_comment);
          node_tmp.set_value("The full size is the sensor size actually used to "
                             "grab the image. full_width = subsampling_width * "
                             "image_width");

          //<full_width>
          node_tmp = node_camera.append_child(LABEL_XML_FULL_WIDTH);
          node_tmp.append_child(pugi::node_pcdata).text() = im_width * subsampl_width;
          //<full_height>
          node_tmp = node_camera.append_child(LABEL_XML_FULL_HEIGHT);
          node_tmp.append_child(pugi::node_pcdata).text() = im_height * subsampl_height;
        }
      }

      node_tmp = node_camera.append_child(pugi::node_comment);
      node_tmp.set_value("Intrinsic camera parameters "
                         "computed for each projection model");

      back = write_camera(node_camera);
    }
    return back;
  }

  /*!
    Write camera parameters in an XML Tree.
    \param node_camera : XML pointer node, pointing on a camera node.
    \return error code.
    */
  int write_camera(pugi::xml_node &node_camera)
  {
    pugi::xml_node node_model;
    pugi::xml_node node_tmp;

    int back = SEQUENCE_OK;

    switch (camera.get_projModel()) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      writeCameraWithoutDistortion(node_camera);
      break;

    case vpCameraParameters::perspectiveProjWithDistortion:
      writeCameraWithDistortion(node_camera);
      break;

    case vpCameraParameters::ProjWithKannalaBrandtDistortion:
      writeCameraWithKannalaBrandt(node_camera);
      break;
    default:
      break;
    }
    return back;
  }

  /*!
    Read camera headers from a XML file and return the last available
    node pointer in the xml tree corresponding with inputs.
    \param node_ : XML tree, pointing on a marker equipment.
    \return additional information node.
   */
  pugi::xml_node find_additional_info(const pugi::xml_node &node_)
  {
    vpXmlCodeType prop;
    pugi::xml_node resNode = pugi::xml_node();

    pugi::xml_node node = node_.first_child();
    bool hasNotFoundInfo = true;
    while (node && hasNotFoundInfo) {
      if (node.type() == pugi::node_element) {
        if (SEQUENCE_OK != str2xmlcode(node.name(), prop)) {
          prop = CODE_XML_OTHER;
        }

        if (prop == CODE_XML_ADDITIONAL_INFO) {
          // We found the node
          resNode = node;
          hasNotFoundInfo = false;
        }
      }

      node = node.next_sibling();
    }

    return resNode;
  }

  /*!
    Translate a string (label) to a xml code.
    \param str : string to translate.
    \param res : resulting code.
    \return error code.
  */
  vpXmlCodeSequenceType str2xmlcode(const char *str, vpXmlCodeType &res)
  {
    vpXmlCodeType val_int = CODE_XML_BAD;
    vpXmlCodeSequenceType back = vpXmlParserCamera::SEQUENCE_OK;

    if (!strcmp(str, LABEL_XML_CAMERA)) {
      val_int = CODE_XML_CAMERA;
    }
    else if (!strcmp(str, LABEL_XML_CAMERA_NAME)) {
      val_int = CODE_XML_CAMERA_NAME;
    }
    else if (!strcmp(str, LABEL_XML_MODEL)) {
      val_int = CODE_XML_MODEL;
    }
    else if (!strcmp(str, LABEL_XML_MODEL_TYPE)) {
      val_int = CODE_XML_MODEL_TYPE;
    }
    else if (!strcmp(str, LABEL_XML_WIDTH)) {
      val_int = CODE_XML_WIDTH;
    }
    else if (!strcmp(str, LABEL_XML_HEIGHT)) {
      val_int = CODE_XML_HEIGHT;
    }
    else if (!strcmp(str, LABEL_XML_SUBSAMPLING_WIDTH)) {
      val_int = CODE_XML_SUBSAMPLING_WIDTH;
    }
    else if (!strcmp(str, LABEL_XML_SUBSAMPLING_HEIGHT)) {
      val_int = CODE_XML_SUBSAMPLING_HEIGHT;
    }
    else if (!strcmp(str, LABEL_XML_FULL_WIDTH)) {
      val_int = CODE_XML_FULL_WIDTH;
    }
    else if (!strcmp(str, LABEL_XML_FULL_HEIGHT)) {
      val_int = CODE_XML_FULL_HEIGHT;
    }
    else if (!strcmp(str, LABEL_XML_U0)) {
      val_int = CODE_XML_U0;
    }
    else if (!strcmp(str, LABEL_XML_V0)) {
      val_int = CODE_XML_V0;
    }
    else if (!strcmp(str, LABEL_XML_PX)) {
      val_int = CODE_XML_PX;
    }
    else if (!strcmp(str, LABEL_XML_PY)) {
      val_int = CODE_XML_PY;
    }
    else if (!strcmp(str, LABEL_XML_KUD)) {
      val_int = CODE_XML_KUD;
    }
    else if (!strcmp(str, LABEL_XML_KDU)) {
      val_int = CODE_XML_KDU;
    }
    else if (!strcmp(str, LABEL_XML_K1)) {
      val_int = CODE_XML_K1;
    }
    else if (!strcmp(str, LABEL_XML_K2)) {
      val_int = CODE_XML_K2;
    }
    else if (!strcmp(str, LABEL_XML_K3)) {
      val_int = CODE_XML_K3;
    }
    else if (!strcmp(str, LABEL_XML_K4)) {
      val_int = CODE_XML_K4;
    }
    else if (!strcmp(str, LABEL_XML_K5)) {
      val_int = CODE_XML_K5;
    }
    else if (!strcmp(str, LABEL_XML_ADDITIONAL_INFO)) {
      val_int = CODE_XML_ADDITIONAL_INFO;
    }
    else {
      val_int = CODE_XML_OTHER;
    }
    res = val_int;

    return back;
  }

  std::string getCameraName() const { return camera_name; }
  vpCameraParameters getCameraParameters() const { return camera; }
  unsigned int getHeight() const { return image_height; }
  unsigned int getSubsampling_width() const { return subsampling_width; }
  unsigned int getSubsampling_height() const { return subsampling_height; }
  unsigned int getWidth() const { return image_width; }

  void setCameraName(const std::string &name) { camera_name = name; }
  void setHeight(unsigned int height) { image_height = height; }
  void setSubsampling_width(unsigned int subsampling) { subsampling_width = subsampling; }
  void setSubsampling_height(unsigned int subsampling) { subsampling_height = subsampling; }
  void setWidth(unsigned int width) { image_width = width; }

private:
   /*!
    Write perspective projection camera parameters without distortion in an XML Tree.
    \param node_camera : XML pointer node, pointing on a camera node.
    */
  void writeCameraWithoutDistortion(pugi::xml_node &node_camera)
  {
    pugi::xml_node node_model;
    pugi::xml_node node_tmp;
    //<model>
    node_model = node_camera.append_child(LABEL_XML_MODEL);
    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Projection model type");

    //<type>without_distortion</type>
    node_tmp = node_model.append_child(LABEL_XML_MODEL_TYPE);
    node_tmp.append_child(pugi::node_pcdata).set_value(LABEL_XML_MODEL_WITHOUT_DISTORTION);

    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Pixel ratio");
    //<px>
    node_tmp = node_model.append_child(LABEL_XML_PX);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_px();
    //<py>
    node_tmp = node_model.append_child(LABEL_XML_PY);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_py();

    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Principal point");

    //<u0>
    node_tmp = node_model.append_child(LABEL_XML_U0);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_u0();
    //<v0>
    node_tmp = node_model.append_child(LABEL_XML_V0);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_v0();
  }

  /*!
    Write perspective projection camera parameters with distortion in an XML Tree.
    \param node_camera : XML pointer node, pointing on a camera node.
    */
  void writeCameraWithDistortion(pugi::xml_node &node_camera)
  {
    pugi::xml_node node_model;
    pugi::xml_node node_tmp;
    //<model>
    node_model = node_camera.append_child(LABEL_XML_MODEL);
    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Projection model type");
    //<type>with_distortion</type>
    node_tmp = node_model.append_child(LABEL_XML_MODEL_TYPE);
    node_tmp.append_child(pugi::node_pcdata).set_value(LABEL_XML_MODEL_WITH_DISTORTION);

    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Pixel ratio");
    //<px>
    node_tmp = node_model.append_child(LABEL_XML_PX);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_px();
    //<py>
    node_tmp = node_model.append_child(LABEL_XML_PY);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_py();

    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Principal point");
    //<u0>
    node_tmp = node_model.append_child(LABEL_XML_U0);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_u0();
    //<v0>
    node_tmp = node_model.append_child(LABEL_XML_V0);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_v0();

    //<kud>
    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Undistorted to distorted distortion parameter");
    node_tmp = node_model.append_child(LABEL_XML_KUD);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_kud();

    //<kud>
    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Distorted to undistorted distortion parameter");
    node_tmp = node_model.append_child(LABEL_XML_KDU);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_kdu();
  }

  /*!
    Write perspective projection camera parameters with Kannala-Brandt distortion in an XML Tree.
    \param node_camera : XML pointer node, pointing on a camera node.
    */
  void writeCameraWithKannalaBrandt(pugi::xml_node &node_camera)
  {
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    const unsigned int index_3 = 3;
    const unsigned int index_4 = 4;
    const unsigned int requiredNbCoeff = 5;
    pugi::xml_node node_model;
    pugi::xml_node node_tmp;
    //<model>
    node_model = node_camera.append_child(LABEL_XML_MODEL);
    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Projection model type");
    //<type>with_KannalaBrandt_distortion</type>
    node_tmp = node_model.append_child(LABEL_XML_MODEL_TYPE);
    node_tmp.append_child(pugi::node_pcdata).set_value(LABEL_XML_MODEL_WITH_KANNALA_BRANDT_DISTORTION);

    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Pixel ratio");
    //<px>
    node_tmp = node_model.append_child(LABEL_XML_PX);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_px();
    //<py>
    node_tmp = node_model.append_child(LABEL_XML_PY);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_py();

    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Principal point");
    //<u0>
    node_tmp = node_model.append_child(LABEL_XML_U0);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_u0();
    //<v0>
    node_tmp = node_model.append_child(LABEL_XML_V0);
    node_tmp.append_child(pugi::node_pcdata).text() = camera.get_v0();

    //<k1>, <k2>, <k3>, <k4>, <k5>
    std::vector<double> distortion_coefs = camera.getKannalaBrandtDistortionCoefficients();

    if (distortion_coefs.size() != requiredNbCoeff) {
      std::cout << "Make sure to have 5 distortion coefficients for Kannala-Brandt distortions." << std::endl;
    }

    node_tmp = node_model.append_child(pugi::node_comment);
    node_tmp.set_value("Distortion coefficients");
    node_tmp = node_model.append_child(LABEL_XML_K1);
    distortion_coefs.size() == index_0 ? (node_tmp.append_child(pugi::node_pcdata).text() = 0)
      : (node_tmp.append_child(pugi::node_pcdata).text() = distortion_coefs[index_0]);
    node_tmp = node_model.append_child(LABEL_XML_K2);
    distortion_coefs.size() <= index_1 ? (node_tmp.append_child(pugi::node_pcdata).text() = 0)
      : (node_tmp.append_child(pugi::node_pcdata).text() = distortion_coefs[index_1]);
    node_tmp = node_model.append_child(LABEL_XML_K3);
    distortion_coefs.size() <= index_2 ? (node_tmp.append_child(pugi::node_pcdata).text() = 0)
      : (node_tmp.append_child(pugi::node_pcdata).text() = distortion_coefs[index_2]);
    node_tmp = node_model.append_child(LABEL_XML_K4);
    distortion_coefs.size() <= index_3 ? (node_tmp.append_child(pugi::node_pcdata).text() = 0)
      : (node_tmp.append_child(pugi::node_pcdata).text() = distortion_coefs[index_3]);
    node_tmp = node_model.append_child(LABEL_XML_K5);
    distortion_coefs.size() <= index_4 ? (node_tmp.append_child(pugi::node_pcdata).text() = 0)
      : (node_tmp.append_child(pugi::node_pcdata).text() = distortion_coefs[index_4]);
  }

  vpCameraParameters camera;
  std::string camera_name;
  unsigned int image_width;
  unsigned int image_height;
  unsigned int subsampling_width;
  unsigned int subsampling_height;
  unsigned int full_width;
  unsigned int full_height;

  //! Allowed size difference between input image and data from the xml parser
  //! to handle minor differences (ex. FORMAT7 can creates 648*488 images).
  static const int allowedPixelDiffOnImageSize = 15;
};
#endif // DOXYGEN_SHOULD_SKIP_THIS

vpXmlParserCamera::vpXmlParserCamera() : m_impl(new Impl()) { }

vpXmlParserCamera::~vpXmlParserCamera() { delete m_impl; }

/*!
  Parse an xml file to load camera parameters.
  \param cam : Camera parameters to fill.
  \param filename : Name of the xml file to parse.
  \param cam_name : Name of the camera. Useful if the xml file has multiple
  camera parameters. Set to an empty string "" if the camera name is not ambiguous.
  \param projModel : Camera projection model needed.
  \param im_width : Image width on which camera calibration was performed.
  Set to 0 if not ambiguous.
  \param im_height : Image height on which camera calibration was performed.
  Set to 0 if not ambiguous.
  \param verbose true to enable verbose mode, false otherwise.

  \return vpXmlParserCamera::SEQUENCE_OK if success and vpXmlParserCamera::SEQUENCE_ERROR otherwise.
*/
int vpXmlParserCamera::parse(vpCameraParameters &cam, const std::string &filename, const std::string &cam_name,
                             const vpCameraParameters::vpCameraParametersProjType &projModel, unsigned int im_width,
                             unsigned int im_height, bool verbose)
{
  return m_impl->parse(cam, filename, cam_name, projModel, im_width, im_height, verbose);
}

/*!
  Save camera parameters in an xml file.
  \param cam : Camera parameters to save.
  \param filename : Name of the xml file to fill.
  \param cam_name : Name of the camera: useful if the xml file has multiple
  camera parameters. Set to "" if the camera name is not ambiguous.
  \param im_width : Width of image on which camera calibration was performed.
  Set to 0 if not ambiguous.
  \param im_height : Height of the image on which camera calibration was
  performed. Set to 0 if not ambiguous.
  \param additionalInfo : Additional information added in the saved xml file.
  The content of this string should be in xml format.
  \param verbose true to enable verbose mode, false otherwise.

  \return error code.

  A typical usage would be the following:
  \code
  #include <visp3/core/vpTime.h>
  #include <visp3/core/vpXmlParserCamera.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpCameraParameters cam;
    std::stringstream ss_additional_info;
    ss_additional_info << "<date>" << vpTime::getDateTime() << "</date>";

    vpXmlParserCamera p;
    if (p.save(cam, "camera.xml", "myCamera", 320, 240, ss_additional_info.str()) != vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Cannot save camera parameters" << std::endl;
    }
  }
  \endcode
  In \c camera.xml file, you will see:
  \code
  <camera>
    ...
    <!--Additional information-->
    <additional_information>
      <date>2016/06/10 09:15:56</date>
    </additional_information>
  </camera>
  \endcode
*/
int vpXmlParserCamera::save(const vpCameraParameters &cam, const std::string &filename, const std::string &cam_name,
                            unsigned int im_width, unsigned int im_height, const std::string &additionalInfo, bool verbose)
{
  return m_impl->save(cam, filename, cam_name, im_width, im_height, additionalInfo, verbose);
}

std::string vpXmlParserCamera::getCameraName() const { return m_impl->getCameraName(); }

vpCameraParameters vpXmlParserCamera::getCameraParameters() const { return m_impl->getCameraParameters(); }

unsigned int vpXmlParserCamera::getHeight() const { return m_impl->getHeight(); }

unsigned int vpXmlParserCamera::getSubsampling_width() const { return m_impl->getSubsampling_width(); }

unsigned int vpXmlParserCamera::getSubsampling_height() const { return m_impl->getSubsampling_height(); }

unsigned int vpXmlParserCamera::getWidth() const { return m_impl->getWidth(); }

void vpXmlParserCamera::setCameraName(const std::string &name) { m_impl->setCameraName(name); }

void vpXmlParserCamera::setHeight(unsigned int height) { m_impl->setHeight(height); }

void vpXmlParserCamera::setSubsampling_width(unsigned int subsampling) { m_impl->setSubsampling_width(subsampling); }

void vpXmlParserCamera::setSubsampling_height(unsigned int subsampling) { m_impl->setSubsampling_height(subsampling); }

void vpXmlParserCamera::setWidth(unsigned int width) { m_impl->setWidth(width); }
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_core.a(vpXmlParserCamera.cpp.o) has no symbols
void dummy_vpXmlParserCamera() { };

#endif
