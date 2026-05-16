# Upstream install discovery for uno_mex (mirrors Uno/CMakeLists.txt).
#
# Each upstream library lives in a sibling source tree and is installed to
#   <root>/<lib>/out/install/<UNO_DEPS_PRESET>
# Prepend each existing install prefix to CMAKE_PREFIX_PATH so find_package()
# picks up the packaged CMake config files.

get_filename_component(_uno_mex_default_deps_root "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(UNO_DEPS_ROOT "${_uno_mex_default_deps_root}" CACHE STRING
   "Semicolon-separated list of parent dirs containing sibling Uno upstreams")

get_filename_component(_uno_mex_inferred_preset "${CMAKE_BINARY_DIR}" NAME)
set(UNO_DEPS_PRESET "${_uno_mex_inferred_preset}" CACHE STRING
   "Subdirectory name under each sibling's out/install/ tree to consume")

foreach(_uno_dep IN ITEMS Uno HiGHS METIS bqpd_lib coinhsl MUMPS_cmake GKlib OpenBLAS)
   string(TOUPPER ${_uno_dep} _uno_dep_upper)

   set(_uno_dep_default "")
   foreach(_uno_root IN LISTS UNO_DEPS_ROOT)
      set(_candidate "${_uno_root}/${_uno_dep}/out/install/${UNO_DEPS_PRESET}")
      if(EXISTS "${_candidate}")
         set(_uno_dep_default "${_candidate}")
         break()
      endif()
   endforeach()

   set(UNO_${_uno_dep_upper}_INSTALL "${_uno_dep_default}" CACHE PATH
      "Install prefix for upstream ${_uno_dep} (overrides UNO_DEPS_ROOT search)")

   if(UNO_${_uno_dep_upper}_INSTALL AND EXISTS "${UNO_${_uno_dep_upper}_INSTALL}")
      list(APPEND CMAKE_PREFIX_PATH "${UNO_${_uno_dep_upper}_INSTALL}")
      message(STATUS "uno_mex upstream: ${_uno_dep} -> ${UNO_${_uno_dep_upper}_INSTALL}")
   endif()
endforeach()

set(UNO_MEXUTILITIES_INSTALL "" CACHE PATH
   "Install prefix for MexUtilities (outside the Uno superbuild tree)")

if(UNO_MEXUTILITIES_INSTALL AND EXISTS "${UNO_MEXUTILITIES_INSTALL}")
   list(APPEND CMAKE_PREFIX_PATH "${UNO_MEXUTILITIES_INSTALL}")
   message(STATUS "uno_mex upstream: MexUtilities -> ${UNO_MEXUTILITIES_INSTALL}")
endif()
