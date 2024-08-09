# TODO:
# visp> In file included from /nix/store/g6lpg2p890jn3hkv63jjkk2f7k66y6hk-ogre-14.2.5/include/OGRE/Ogre.h:52,
# visp>                  from /build/source/modules/ar/include/visp3/ar/vpAROgre.h:66:
# visp> /nix/store/g6lpg2p890jn3hkv63jjkk2f7k66y6hk-ogre-14.2.5/include/OGRE/OgreConfigFile.h:94:41: note: declared here
# visp>    94 |         OGRE_DEPRECATED SectionIterator getSectionIterator(void);
# visp>       |                                         ^~~~~~~~~~~~~~~~~~
# visp> /build/source/modules/ar/src/ogre-simulator/vpAROgre.cpp:315:33: error: no matching function for call to 'Ogre::Root::showConfigDialog()'
# visp>   315 |     if (!mRoot->showConfigDialog()) {
# visp>       |          ~~~~~~~~~~~~~~~~~~~~~~~^~
{
  cmake,
  coin3d,
  darwin,
  doxygen,
  eigen,
  #fetchFromGitHub,
  lapack,
  lib,
  libdc1394,
  libdmtx,
  libglvnd,
  libjpeg, # this is libjpeg-turbo
  libpng,
  librealsense,
  libxml2,
  libX11,
  nix-gitignore,
  nlohmann_json,
  #ogre,
  openblas,
  opencv,
  pkg-config,
  python3Packages,
  stdenv,
  texliveSmall,
  v4l-utils,
  xorg,
  zbar,
  zlib,
}:

stdenv.mkDerivation (finalAttrs: {
  pname = "visp";
  version = "3.6.0";

  src = nix-gitignore.gitignoreSource [ ./.nixignore ] ./.;

  nativeBuildInputs = [
    cmake
    doxygen
    pkg-config
    texliveSmall
  ];

  buildInputs =
    [
      eigen
      lapack
      libdc1394
      libdmtx
      libglvnd
      libjpeg
      libpng
      librealsense
      libX11
      libxml2
      nlohmann_json
      #ogre
      openblas
      opencv
      python3Packages.numpy
      xorg.libpthreadstubs
      zbar
      zlib
    ]
    ++ lib.optionals stdenv.isLinux [
      coin3d
      v4l-utils
    ]
    ++ lib.optionals stdenv.isDarwin [ darwin.IOKit ];

  doCheck = true;

  meta = {
    description = "Open Source Visual Servoing Platform";
    homepage = "https://visp.inria.fr";
    changelog = "https://github.com/lagadic/visp/blob/v${finalAttrs.version}/ChangeLog.txt";
    license = lib.licenses.gpl2Plus;
    maintainers = with lib.maintainers; [ nim65s ];
  };
})
