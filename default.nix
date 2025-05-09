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
  ogre,
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
      ogre
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
