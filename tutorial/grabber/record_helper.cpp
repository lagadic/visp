#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/io/vpImageIo.h>

#include "record_helper.h"

/*!
 * Manage image recording using user mouse click
 * \param opt_seqname : Name of the images to write.
 * \param opt_record_mode : Record mode: 0 for continuous recording of a sequence of images,
 * 1 for single shot image acquisition.
 * \param I : Last image acquired.
 * \return true if quit asked, false otherwise.
 */
bool record_helper(const std::string &opt_seqname, int opt_record_mode,
                  const vpImage<unsigned char> &I)
{
  static bool start_record = false;
  static unsigned int counter = 1;
  static std::string text_record_mode = std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

  if (! opt_seqname.empty()) {
    if (! opt_record_mode) { // continuous
      if (start_record) {
        vpDisplay::displayText(I, 20, 10, "Left  click: stop recording", vpColor::red);
      }
      else {
        vpDisplay::displayText(I, 20, 10, "Left  click: start recording", vpColor::red);
      }
    }
    else {
      vpDisplay::displayText(I, 20, 10, "Left  click: record image", vpColor::red);
    }
    vpDisplay::displayText(I, 40, 10, "Right click: quit", vpColor::red);
  }
  else {
    vpDisplay::displayText(I, 20, 10, "Click to quit", vpColor::red);
  }

  if (! opt_seqname.empty()) {
    vpDisplay::displayText(I, 60, 10, text_record_mode, vpColor::red);
  }
  vpMouseButton::vpMouseButtonType button;
  if (vpDisplay::getClick(I, button, false)) {
    if (! opt_seqname.empty()) { // Recording requested
      if (button == vpMouseButton::button1) { // enable/disable recording
        start_record = !start_record;
      }
      else if (button == vpMouseButton::button3) { // quit
        return true;
      }
    }
    else { // any button to quit
      return true;
    }
  }
  if (start_record) {
    char filename[FILENAME_MAX];
    sprintf(filename, opt_seqname.c_str(), counter);
    {
      // check if parent folder exists. Create otherwise
      static bool parent_exists = false;
      if (! parent_exists) {
        std::string parent = vpIoTools::getParent(filename);
        if (! parent.empty()) {
          if (! vpIoTools::checkDirectory(parent)) {
            vpIoTools::makeDirectory(parent);
          }
        }
        parent_exists = true;
      }
    }

    counter ++;
    std::string text = std::string("Save: ") + std::string(filename);
    vpDisplay::displayText(I, 80, 10, text, vpColor::red);
    std::cout << text << std::endl;
    vpImageIo::write(I, filename);
    if (opt_record_mode == 1) { // single shot mode
      start_record = false;
    }
  }

  return false;
}
