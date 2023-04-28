#include <visp3/gui/vpColorBlindFriendlyPalette.h>
#include <visp3/core/vpIoTools.h>

std::vector<std::string> vpColorBlindFriendlyPalette::g_paletteNames =
{
  "black"     ,
  "orange"    ,
  "sky-blue"   ,
  "green"     ,
  "yellow"    ,
  "blue"      ,
  "vermillon" ,
  "purple"    ,
  "unknown"
};

std::vector<vpColor> vpColorBlindFriendlyPalette::g_palette = {
  vpColor(0,0,0),       // Black = 0,
  vpColor(230,159,0),   // Orange = 1,
  vpColor(86,180,233),  // SkyBlue = 2,
  vpColor(0,158,115),   // Green = 3,
  vpColor(240,228,66),  // Yellow = 4,
  vpColor(0,114,178),   // Blue = 5,
  vpColor(213,94,0),    // Vermillon = 6,
  vpColor(204,121,167), // Purple = 7,
  vpColor(255,255,255)  // COUNT = 8
};

vpColorBlindFriendlyPalette::vpColorBlindFriendlyPalette()
  : _colorID(Palette::COUNT)
{

}

vpColorBlindFriendlyPalette::vpColorBlindFriendlyPalette(vpColorBlindFriendlyPalette::Palette colorID)
  : _colorID(colorID)
{

}

vpColorBlindFriendlyPalette::vpColorBlindFriendlyPalette(std::string nameColor)
  : _colorID(Palette::COUNT)
{
  set_fromString(nameColor);
}

vpColor vpColorBlindFriendlyPalette::to_vpColor()
{
  return g_palette[to_uint(_colorID)];
}

std::vector<unsigned char> vpColorBlindFriendlyPalette::to_RGB() const
{
  vpColor color = g_palette[to_uint(_colorID)];
  std::vector<unsigned char> v_rgb;
  v_rgb.push_back(color.R);
  v_rgb.push_back(color.G);
  v_rgb.push_back(color.B);
  return v_rgb;
}

std::vector<double> vpColorBlindFriendlyPalette::to_colorRatio() const
{
  vpColor color = g_palette[to_uint(_colorID)];
  std::vector<double> v_rgb;
  v_rgb.push_back((double)color.R / 255.0);
  v_rgb.push_back((double)color.G / 255.0);
  v_rgb.push_back((double)color.B / 255.0);
  return v_rgb;
}

bool vpColorBlindFriendlyPalette::set_fromString(const std::string &nameColor)
{
  _colorID = Palette::COUNT;
  std::string nameLowerCase = nameColor; // vpIoTools::toLowerCase(nameColor);
  bool wasFound(false);
  for(unsigned int i= 0 ; i < to_uint(Palette::COUNT) && !wasFound; i++){
    vpColorBlindFriendlyPalette::Palette candidate = (Palette) i;
    if(to_string(candidate) == nameLowerCase){
      _colorID = candidate;
      wasFound = true;
    }
  }
  return wasFound;
}

std::string vpColorBlindFriendlyPalette::to_string() const
{
  std::string nameColor = to_string(_colorID);
  return nameColor;
}

std::string vpColorBlindFriendlyPalette::getAvailableColorsNames(const std::string &prefix, const std::string &separator, const std::string &suffix)
{
  std::string list(prefix);
  const unsigned int nbAvailableColors = (unsigned int) Palette::COUNT ;
  for(unsigned int i = 0; i < nbAvailableColors - 1; i++)
  {
    std::string nameCandidateID = g_paletteNames[i];
    list += nameCandidateID + separator;
  }
  list += g_paletteNames[nbAvailableColors - 1] + suffix;
  return list;
}

unsigned int vpColorBlindFriendlyPalette::to_uint(Palette colorID)
{
  const unsigned int nbAvailableColors = (unsigned int) Palette::COUNT ;
  unsigned int ID = nbAvailableColors;
  std::string nameSearchedColor = to_string(colorID);
  bool wasFound = false;
  for(unsigned int i=0; i < nbAvailableColors && !wasFound; i++){
    Palette candidate = (Palette) i;
    if(to_string(candidate) == nameSearchedColor){
      ID = i;
      wasFound = true;
    }
  }
  return ID;
}

std::string vpColorBlindFriendlyPalette::to_string(vpColorBlindFriendlyPalette::Palette colorID)
{
  std::string nameColor;
  switch(colorID){
  case Palette::Black:
    nameColor = g_paletteNames[0];
    break;
  case Palette::Orange:
    nameColor = g_paletteNames[1];
    break;
  case Palette::SkyBlue:
    nameColor = g_paletteNames[2];
    break;
  case Palette::Green:
    nameColor = g_paletteNames[3];
    break;
  case Palette::Yellow:
    nameColor = g_paletteNames[4];
    break;
  case Palette::Blue:
    nameColor = g_paletteNames[5];
    break;
  case Palette::Vermillon:
    nameColor = g_paletteNames[6];
    break;
  case Palette::Purple:
    nameColor = g_paletteNames[7];
    break;
  default:
    nameColor = g_paletteNames[8];
  }
  return nameColor;
}

std::ostream &operator<<(std::ostream &os, const vpColorBlindFriendlyPalette &color)
{
  os << color.to_string();
  return os;
}

std::istream &operator>>(std::istream &is, vpColorBlindFriendlyPalette &color)
{
  std::string nameColor;
  is >> nameColor;
  color.set_fromString(nameColor);
  return is;
}
