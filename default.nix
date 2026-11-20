{
  cmake,
  coin3d,
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
  libx11,
  llvmPackages,
  nix-gitignore,
  nlohmann_json,
  #ogre,
  openblas,
  opencv,
  perl,
  pkg-config,
  python3,
  stdenv,
  texliveSmall,
  libv4l,
  libpthread-stubs,
  zbar,
  zlib,

  withPython ? true,
}:

let
  # only required for building bindings, stubs & docs
  python = python3.withPackages (
    p: with p; [
      cxxheaderparser
      doxmlparser
      lxml
      mypy
      numpy
      pcpp
      pip
      pybind11
      setuptools
      sphinx
      sphinx-design
      sphinxcontrib-jsmath
      tqdm
    ]
  );
in

stdenv.mkDerivation (finalAttrs: {
  pname = "visp";
  version = "3.7.0";

  src = nix-gitignore.gitignoreSource [ ./.nixignore ] ./.;

  nativeBuildInputs = [
    cmake
    doxygen
    perl
    pkg-config
    texliveSmall
  ]
  ++ lib.optional withPython python;

  buildInputs = [
    eigen
    lapack
    libdc1394
    libdmtx
    libglvnd
    libjpeg
    libpng
    libx11
    libxml2
    nlohmann_json
    #ogre
    openblas
    opencv
    libpthread-stubs
    zbar
    zlib
  ]
  ++ lib.optionals stdenv.hostPlatform.isLinux [
    coin3d
    librealsense
    libv4l
  ]
  ++ lib.optionals stdenv.cc.isClang [
    llvmPackages.openmp
  ];

  cmakeFlags = [
    (lib.cmakeBool "ALLOW_SYSTEM_PYTHON" withPython)
    (lib.cmakeBool "BUILD_SHARED_LIBS" (!stdenv.hostPlatform.isStatic))
    (lib.cmakeBool "BUILD_TESTS" finalAttrs.finalPackage.doCheck)
    (lib.cmakeBool "INSTALL_PYTHON_BINDINGS" withPython)
  ];

  doCheck = true;

  nativeCheckInputs = lib.optionals withPython [
    python3.pkgs.pythonImportsCheckHook
    python3.pkgs.pytestCheckHook
  ];

  pythonImportsCheck = [ "visp" ];

  enabledTestPaths = [
    "../modules/python/test"
    "../modules/python/doc"
  ];

  meta = {
    description = "Open Source Visual Servoing Platform";
    homepage = "https://visp.inria.fr";
    changelog = "https://github.com/lagadic/visp/blob/v${finalAttrs.version}/ChangeLog.txt";
    license = lib.licenses.gpl2Plus;
    maintainers = with lib.maintainers; [ nim65s ];
    platforms = lib.platforms.unix ++ lib.platforms.windows;
  };
})
