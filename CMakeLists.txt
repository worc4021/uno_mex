cmake_minimum_required(VERSION 3.25)
project(UnoMex)

include(GNUInstallDirs)
set(MKL_LINK static)
set(MKL_THREADING sequential)
set(MKL_INTERFACE lp64)
find_package(MKL REQUIRED)

include($ENV{CMAKE_INSTALL_PREFIX}/cmake/GKlibTargets.cmake)
include($ENV{CMAKE_INSTALL_PREFIX}/cmake/metisTargets.cmake)
include($ENV{CMAKE_INSTALL_PREFIX}/cmake/bqpdTargets.cmake)
include($ENV{CMAKE_INSTALL_PREFIX}/cmake/mumpsTargets.cmake)
include($ENV{CMAKE_INSTALL_PREFIX}/cmake/unoTargets.cmake)
include($ENV{CMAKE_INSTALL_PREFIX}/cmake/MexUtilities.cmake)

include(FetchContent)

FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt
  GIT_TAG        11.0.2
  EXCLUDE_FROM_ALL
  )
FetchContent_MakeAvailable(fmt)

FetchContent_Declare(
      Eigen
      GIT_REPOSITORY    https://gitlab.com/libeigen/eigen.git
      GIT_TAG           3.4.0
      EXCLUDE_FROM_ALL
    )
FetchContent_MakeAvailable(Eigen)


find_package(Matlab REQUIRED)

matlab_add_mex(
    NAME uno_mex
    SRC src/main.cpp
    LINK_TO MexUtilities::MexUtilities uno::uno
    R2018a
)
target_include_directories(uno_mex PRIVATE include)

matlab_add_mex(
    NAME uno_options
    SRC src/get_options.cpp
    LINK_TO MexUtilities::MexUtilities uno::uno
    R2018a
)
target_include_directories(uno_mex PRIVATE include)