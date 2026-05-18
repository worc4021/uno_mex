# Fetch official Uno sources (populate only) or use a pre-installed Uno package.

option(UNO_MEX_USE_INSTALLED_UNO
   "Use find_package(Uno) from UNO_UNO_INSTALL instead of building sources in-tree"
   OFF)

option(UNO_MEX_BUILD_UNO_TESTS
   "Build run_unotest from fetched Uno sources (requires ENABLE_TESTS in uno_build)"
   OFF)

set(UNO_MEX_UNO_GIT_REPOSITORY "https://github.com/cvanaret/Uno"
   CACHE STRING "Git repository for official Uno sources")
set(UNO_MEX_UNO_GIT_TAG "v2.7.2"
   CACHE STRING "Git tag or commit for official Uno sources")

if(UNO_MEX_UNO_SOURCE_DIR AND NOT UNO_MEX_USE_INSTALLED_UNO)
   get_filename_component(UNO_MEX_UNO_SOURCE_DIR "${UNO_MEX_UNO_SOURCE_DIR}" ABSOLUTE)
   if(NOT EXISTS "${UNO_MEX_UNO_SOURCE_DIR}/uno/Uno.cpp")
      message(FATAL_ERROR
         "UNO_MEX_UNO_SOURCE_DIR is set but does not contain uno/Uno.cpp: ${UNO_MEX_UNO_SOURCE_DIR}")
   endif()
   message(STATUS "uno_mex: using local Uno sources at ${UNO_MEX_UNO_SOURCE_DIR}")
elseif(NOT UNO_MEX_USE_INSTALLED_UNO)
   include(FetchContent)
   if(DEFINED FETCHCONTENT_SOURCE_DIR_UNO_SOURCES AND FETCHCONTENT_SOURCE_DIR_UNO_SOURCES)
      set(_uno_fetch_src "${FETCHCONTENT_SOURCE_DIR_UNO_SOURCES}")
   else()
      FetchContent_Declare(
         uno_sources
         GIT_REPOSITORY "${UNO_MEX_UNO_GIT_REPOSITORY}"
         GIT_TAG "${UNO_MEX_UNO_GIT_TAG}"
      )
      FetchContent_GetProperties(uno_sources)
      if(NOT uno_sources_POPULATED)
         FetchContent_Populate(uno_sources)
      endif()
      set(_uno_fetch_src "${uno_sources_SOURCE_DIR}")
   endif()
   get_filename_component(UNO_MEX_UNO_SOURCE_DIR "${_uno_fetch_src}" ABSOLUTE)
   if(NOT EXISTS "${UNO_MEX_UNO_SOURCE_DIR}/uno/Uno.cpp")
      message(FATAL_ERROR
         "Fetched Uno sources do not contain uno/Uno.cpp under ${UNO_MEX_UNO_SOURCE_DIR}")
   endif()
   message(STATUS "uno_mex: Uno sources at ${UNO_MEX_UNO_SOURCE_DIR}")
endif()

if(UNO_MEX_USE_INSTALLED_UNO)
   if(NOT UNO_UNO_INSTALL OR NOT EXISTS "${UNO_UNO_INSTALL}")
      foreach(_uno_root IN LISTS UNO_DEPS_ROOT)
         set(_candidate "${_uno_root}/Uno/out/install/${UNO_DEPS_PRESET}")
         if(EXISTS "${_candidate}")
            set(UNO_UNO_INSTALL "${_candidate}" CACHE PATH "Installed Uno prefix" FORCE)
            break()
         endif()
      endforeach()
   endif()
   if(NOT UNO_UNO_INSTALL OR NOT EXISTS "${UNO_UNO_INSTALL}")
      message(FATAL_ERROR
         "UNO_MEX_USE_INSTALLED_UNO=ON requires UNO_UNO_INSTALL (build/install Uno or set cache)")
   endif()
   list(APPEND CMAKE_PREFIX_PATH "${UNO_UNO_INSTALL}")
   find_package(Uno CONFIG REQUIRED)
   if(NOT TARGET uno::uno_static)
      message(FATAL_ERROR "find_package(Uno) did not provide uno::uno_static")
   endif()
   foreach(_uno_root IN LISTS UNO_DEPS_ROOT)
      set(_candidate "${_uno_root}/Uno/uno")
      if(EXISTS "${_candidate}/ingredients/subproblem_solvers/LPSolverFactory.hpp")
         set(UNO_MEX_UNO_SOURCE_DIR "${_uno_root}/Uno")
         break()
      endif()
   endforeach()
   if(NOT UNO_MEX_UNO_SOURCE_DIR)
      get_filename_component(UNO_MEX_UNO_SOURCE_DIR "${UNO_UNO_INSTALL}/../../../" ABSOLUTE)
   endif()
   message(STATUS "uno_mex: installed Uno at ${UNO_UNO_INSTALL}")
else()
   set(UNO_SOURCE_DIR "${UNO_MEX_UNO_SOURCE_DIR}" CACHE PATH "Official Uno source tree" FORCE)
   set(BUILD_STATIC_LIBS ON CACHE BOOL "" FORCE)
   set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
   set(ENABLE_TESTS ${UNO_MEX_BUILD_UNO_TESTS} CACHE BOOL "" FORCE)
   add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/uno_build" "${CMAKE_BINARY_DIR}/_deps/uno_build")
endif()
