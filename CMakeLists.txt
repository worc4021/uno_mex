cmake_minimum_required(VERSION 3.25)
project(UnoMex LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find MATLAB before restricting find_library to .a only (MinGW superbuild below).
# That suffix list prevents FindMatlab from locating libmex.lib / libmx.lib.
set(MATLAB_ADDITIONAL_VERSION
   "R2024b=24.2"
   "R2025a=25.1"
   "R2025b=25.2"
)
find_package(Matlab REQUIRED)

if(MINGW)
   set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
endif()

include(UnoDeps)

enable_language(Fortran)
find_package(OpenMP REQUIRED)

include(UnoSources)

find_package(MexUtilities CONFIG REQUIRED)

# bqpd install exports a bogus INSTALL_INTERFACE path "/bqpd" (missing prefix).
foreach(_bqpd_tgt IN ITEMS bqpd::bqpd_sparse bqpd::bqpd_dense)
   if(TARGET ${_bqpd_tgt})
      get_target_property(_bqpd_inc ${_bqpd_tgt} INTERFACE_INCLUDE_DIRECTORIES)
      if(_bqpd_inc)
         list(FILTER _bqpd_inc EXCLUDE REGEX "^/bqpd$")
         set_target_properties(${_bqpd_tgt} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_bqpd_inc}")
      endif()
   endif()
endforeach()

if(NOT TARGET uno::uno_static)
   message(FATAL_ERROR "uno::uno_static is not available after UnoSources")
endif()

set(_uno_c_include "${UNO_MEX_UNO_SOURCE_DIR}/interfaces/C")
set(_uno_cpp_include "${UNO_MEX_UNO_SOURCE_DIR}/uno")
if(NOT EXISTS "${_uno_cpp_include}/ingredients/subproblem_solvers/LPSolverFactory.hpp")
   message(FATAL_ERROR
      "Uno C++ sources not found for uno_mex (expected under ${_uno_cpp_include})")
endif()

if(UNO_MEX_USE_INSTALLED_UNO)
   set(_uno_public_include "${UNO_UNO_INSTALL}/include/uno")
else()
   set(_uno_public_include "${_uno_c_include}")
endif()

add_executable(hs015 src/hs015.cpp)
target_include_directories(hs015 PRIVATE "${_uno_public_include}")
target_link_libraries(hs015 PRIVATE uno::uno_static)
if(MINGW)
   target_compile_options(hs015 PRIVATE -finput-charset=UTF-8 -fexec-charset=UTF-8)
endif()

matlab_add_mex(
   NAME uno_mex
   SRC src/main.cpp
   LINK_TO MexUtilities::MexUtilities uno::uno_static
   R2018a
)
target_include_directories(uno_mex PRIVATE "${_uno_public_include}" "${_uno_cpp_include}")
if(MINGW)
   target_compile_options(uno_mex PRIVATE -finput-charset=UTF-8 -fexec-charset=UTF-8)
endif()

matlab_add_mex(
   NAME uno_options
   SRC src/get_options.cpp
   LINK_TO MexUtilities::MexUtilities uno::uno_static
   R2018a
)
target_include_directories(uno_options PRIVATE "${_uno_public_include}")
if(MINGW)
   target_compile_options(uno_options PRIVATE -finput-charset=UTF-8 -fexec-charset=UTF-8)
endif()

if(UNO_MEX_BUILD_UNO_TESTS)
   include(CTest)
   enable_testing()
endif()
