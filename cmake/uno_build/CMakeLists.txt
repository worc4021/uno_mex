# Owned Uno build for uno_mex: compiles official Uno sources (UNO_SOURCE_DIR) with
# CONFIG-package dependencies discovered by the parent via UnoDeps.cmake.
# Derived from the monorepo Uno/CMakeLists.txt (no install/export, no AMPL/unopy).

if(NOT UNO_SOURCE_DIR)
   message(FATAL_ERROR "UNO_SOURCE_DIR must point at the official Uno source tree")
endif()
get_filename_component(UNO_SOURCE_DIR "${UNO_SOURCE_DIR}" ABSOLUTE)

if(NOT EXISTS "${UNO_SOURCE_DIR}/uno/Uno.cpp")
   message(FATAL_ERROR "UNO_SOURCE_DIR does not look like an Uno tree: ${UNO_SOURCE_DIR}")
endif()

set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED BUILD_STATIC_LIBS)
   set(BUILD_STATIC_LIBS ON)
endif()
if(NOT DEFINED BUILD_SHARED_LIBS)
   set(BUILD_SHARED_LIBS OFF)
endif()
if(NOT DEFINED ENABLE_TESTS)
   set(ENABLE_TESTS OFF)
endif()

if(NOT BUILD_STATIC_LIBS)
   message(FATAL_ERROR "uno_mex uno_build requires BUILD_STATIC_LIBS=ON")
endif()

include(FortranCInterface)
FortranCInterface_HEADER(${CMAKE_CURRENT_BINARY_DIR}/include/fortran_interface.h
        MACRO_NAMESPACE "FC_"
        SYMBOL_NAMESPACE "FC_")

if(MSVC)
   set(_uno_warning_flags "$<$<COMPILE_LANGUAGE:C>:/utf-8>" "$<$<COMPILE_LANGUAGE:CXX>:/utf-8>")
else()
   set(_uno_warning_flags -Wall -Wextra -Wnon-virtual-dtor -pedantic -Wunused-value -Wconversion)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
   list(APPEND _uno_warning_flags -Wmaybe-uninitialized)
   if(MINGW)
      list(APPEND _uno_warning_flags -finput-charset=UTF-8 -fexec-charset=UTF-8)
   endif()
endif()

list(APPEND CMAKE_MODULE_PATH "${UNO_SOURCE_DIR}/cmake")
if(EXISTS "${UNO_SOURCE_DIR}/cmake-library/finders")
   list(APPEND CMAKE_MODULE_PATH "${UNO_SOURCE_DIR}/cmake-library/finders")
endif()

add_library(compiled_uno_files OBJECT)
set_property(TARGET compiled_uno_files PROPERTY POSITION_INDEPENDENT_CODE ON)
target_compile_options(compiled_uno_files PRIVATE ${_uno_warning_flags})
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
   target_compile_definitions(compiled_uno_files PUBLIC _GLIBCXX_USE_CXX11_ABI=1)
endif()
if(NOT MSVC)
   target_compile_options(compiled_uno_files PRIVATE $<$<CONFIG:Release>:-O3>)
   target_compile_definitions(compiled_uno_files PRIVATE $<$<CONFIG:Release>:NDEBUG>)
endif()

add_library(uno_dependencies INTERFACE)
target_include_directories(uno_dependencies INTERFACE
   $<BUILD_INTERFACE:${UNO_SOURCE_DIR}/uno>
   $<BUILD_INTERFACE:${UNO_SOURCE_DIR}/interfaces/C>
   $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

if(MINGW)
   target_link_options(uno_dependencies INTERFACE -static)
   foreach(_lang C CXX Fortran)
      if(CMAKE_${_lang}_IMPLICIT_LINK_LIBRARIES)
         list(REMOVE_ITEM CMAKE_${_lang}_IMPLICIT_LINK_LIBRARIES gcc_s)
      endif()
   endforeach()
endif()

if(ENABLE_TESTS)
   add_executable(run_unotest)
   target_compile_options(run_unotest PRIVATE ${_uno_warning_flags})
endif()

file(GLOB UNO_SOURCE_FILES CONFIGURE_DEPENDS
   "${UNO_SOURCE_DIR}/interfaces/C/Uno_C_API.cpp"
   "${UNO_SOURCE_DIR}/uno/Uno.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/constraint_relaxation_strategies/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/constraint_relaxation_strategies/relaxed_problems/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/globalization_mechanisms/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/globalization_strategies/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/globalization_strategies/switching_methods/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/globalization_strategies/switching_methods/filter_methods/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/globalization_strategies/switching_methods/filter_methods/filters/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/globalization_strategies/switching_methods/funnel_methods/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/hessian_models/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/hessian_models/quasi_newton/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/hessian_models/quasi_newton/direct/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/hessian_models/quasi_newton/inverse/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/inequality_handling_methods/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/inequality_handling_methods/interior_point_methods/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/inequality_handling_methods/interior_point_methods/barrier_problems/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/inertia_correction_strategies/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/subproblem/*.cpp"
   "${UNO_SOURCE_DIR}/uno/ingredients/subproblem_solvers/*.cpp"
   "${UNO_SOURCE_DIR}/uno/model/*.cpp"
   "${UNO_SOURCE_DIR}/uno/optimization/*.cpp"
   "${UNO_SOURCE_DIR}/uno/options/*.cpp"
   "${UNO_SOURCE_DIR}/uno/tools/*.cpp"
)
target_sources(compiled_uno_files PRIVATE ${UNO_SOURCE_FILES})

file(GLOB TESTS CONFIGURE_DEPENDS
   "${UNO_SOURCE_DIR}/unotest/unotest.cpp"
   "${UNO_SOURCE_DIR}/unotest/functional_tests/BLASTests.cpp"
   "${UNO_SOURCE_DIR}/unotest/functional_tests/LAPACKTests.cpp"
   "${UNO_SOURCE_DIR}/unotest/unit_tests/*.cpp"
)
if(ENABLE_TESTS)
   target_sources(run_unotest PRIVATE ${TESTS})
endif()

function(link_to_uno library_name library_path)
   target_link_libraries(uno_dependencies INTERFACE ${library_path})
   string(TOUPPER ${library_name} library_name_upper)
   target_compile_definitions(uno_dependencies INTERFACE HAS_${library_name_upper})
   get_filename_component(directory ${library_path} DIRECTORY)
   target_include_directories(uno_dependencies SYSTEM INTERFACE ${directory})
   message(STATUS "uno_build: found ${library_name}")
endfunction()

function(link_uno_target define_name)
   foreach(_tgt IN LISTS ARGN)
      target_link_libraries(uno_dependencies INTERFACE ${_tgt})
   endforeach()
   string(TOUPPER ${define_name} _define_upper)
   target_compile_definitions(uno_dependencies INTERFACE HAS_${_define_upper})
   message(STATUS "uno_build: found ${define_name} (CMake config package)")
endfunction()

find_package(OpenMP)
if(OpenMP_FOUND AND MINGW)
   foreach(_lang C CXX Fortran)
      if(TARGET OpenMP::OpenMP_${_lang})
         get_target_property(_omp_libs OpenMP::OpenMP_${_lang} INTERFACE_LINK_LIBRARIES)
         if(_omp_libs)
            list(FILTER _omp_libs EXCLUDE REGEX "libgcc_s\\.a$")
            set_target_properties(OpenMP::OpenMP_${_lang} PROPERTIES
               INTERFACE_LINK_LIBRARIES "${_omp_libs}")
         endif()
      endif()
   endforeach()
endif()

find_package(coinhsl CONFIG QUIET)
if(coinhsl_FOUND)
   target_link_libraries(uno_dependencies INTERFACE coinhsl::coinhsl)
   set(_has_libhsl_isfunctional FALSE)
   get_target_property(_coinhsl_loc coinhsl::coinhsl IMPORTED_LOCATION_RELEASE)
   if(NOT _coinhsl_loc)
      get_target_property(_coinhsl_loc coinhsl::coinhsl IMPORTED_LOCATION)
   endif()
   if(_coinhsl_loc AND EXISTS "${_coinhsl_loc}")
      find_program(_uno_nm NAMES nm)
      if(_uno_nm)
         execute_process(
            COMMAND "${_uno_nm}" --defined-only "${_coinhsl_loc}"
            OUTPUT_VARIABLE _coinhsl_syms
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE)
         if(_coinhsl_syms MATCHES "LIBHSL_isfunctional")
            set(_has_libhsl_isfunctional TRUE)
         endif()
      endif()
   endif()
   if(_has_libhsl_isfunctional)
      target_compile_definitions(uno_dependencies INTERFACE HAS_HSL)
      set(HSL TRUE)
      message(STATUS "uno_build: coinhsl with LIBHSL_isfunctional -> HAS_HSL")
   else()
      target_compile_definitions(uno_dependencies INTERFACE HAS_MA57 HAS_MA27)
      set(MA57 TRUE)
      set(MA27 TRUE)
      message(STATUS "uno_build: coinhsl without LIBHSL_isfunctional -> HAS_MA57 + HAS_MA27")
   endif()
else()
   find_library(HSL hsl)
   if(HSL)
      link_to_uno(hsl ${HSL})
   else()
      find_library(MA57 ma57)
      if(MA57)
         link_to_uno(ma57 ${MA57})
      endif()
      find_library(MA27 ma27)
      if(MA27)
         link_to_uno(ma27 ${MA27})
      endif()
   endif()
endif()
if(HSL OR MA57)
   target_sources(compiled_uno_files PRIVATE
      "${UNO_SOURCE_DIR}/uno/ingredients/subproblem_solvers/MA57/MA57Solver.cpp")
   if(ENABLE_TESTS)
      target_sources(run_unotest PRIVATE
         "${UNO_SOURCE_DIR}/unotest/functional_tests/MA57SolverTests.cpp")
   endif()
endif()
if(HSL OR MA27)
   target_sources(compiled_uno_files PRIVATE
      "${UNO_SOURCE_DIR}/uno/ingredients/subproblem_solvers/MA27/MA27Solver.cpp")
   if(ENABLE_TESTS)
      target_sources(run_unotest PRIVATE
         "${UNO_SOURCE_DIR}/unotest/functional_tests/MA27SolverTests.cpp")
   endif()
endif()

find_package(GKlib CONFIG QUIET)

find_package(METIS CONFIG QUIET)
if(TARGET METIS::metis)
   set(_metis_link_targets METIS::metis)
   if(TARGET GKlib::GKlib)
      list(APPEND _metis_link_targets GKlib::GKlib)
   endif()
   link_uno_target(metis ${_metis_link_targets})
   set(METIS TRUE)
   unset(_metis_link_targets)
else()
   find_library(METIS metis)
   if(METIS)
      link_to_uno(metis ${METIS})
   endif()
endif()

find_package(bqpd CONFIG QUIET)
if(TARGET bqpd::bqpd_sparse)
   link_uno_target(bqpd bqpd::bqpd_sparse)
   set(BQPD TRUE)
else()
   find_library(BQPD bqpd)
   if(BQPD)
      link_to_uno(bqpd ${BQPD})
   endif()
endif()
if(BQPD)
   target_sources(compiled_uno_files PRIVATE
      "${UNO_SOURCE_DIR}/uno/ingredients/subproblem_solvers/BQPD/BQPDSolver.cpp"
      "${UNO_SOURCE_DIR}/uno/ingredients/subproblem_solvers/BQPD/BQPDWorkspace.cpp")
   if(ENABLE_TESTS)
      target_sources(run_unotest PRIVATE
         "${UNO_SOURCE_DIR}/unotest/functional_tests/BQPDSolverTests.cpp")
   endif()
endif()

find_package(highs CONFIG QUIET)
if(TARGET highs::highs)
   link_uno_target(highs highs::highs)
   set(HIGHS TRUE)
else()
   find_library(HIGHS highs)
   if(HIGHS)
      target_link_libraries(uno_dependencies INTERFACE ${HIGHS})
      target_compile_definitions(uno_dependencies INTERFACE HAS_HIGHS)
      get_filename_component(HIGHS_DIRECTORY ${HIGHS} DIRECTORY)
      target_include_directories(uno_dependencies SYSTEM INTERFACE ${HIGHS_DIRECTORY}/../include/highs)
      message(STATUS "uno_build: found HIGHS (legacy raw-library lookup)")
   endif()
endif()
if(HIGHS)
   target_sources(compiled_uno_files PRIVATE
      "${UNO_SOURCE_DIR}/uno/ingredients/subproblem_solvers/HiGHS/HiGHSSolver.cpp"
      "${UNO_SOURCE_DIR}/uno/ingredients/subproblem_solvers/HiGHS/HiGHSWorkspace.cpp")
   if(ENABLE_TESTS)
      target_sources(run_unotest PRIVATE
         "${UNO_SOURCE_DIR}/unotest/functional_tests/HiGHSSolverTests.cpp")
   endif()
endif()

set(_mumps_found FALSE)
find_package(mumps CONFIG QUIET)
if(TARGET mumps::libdmumps)
   set(_mumps_libs mumps::libdmumps mumps::libmumps_common mumps::pord)
   if(TARGET mumps::mpiseq)
      list(APPEND _mumps_libs mumps::mpiseq)
   else()
      find_package(MPI REQUIRED)
      list(APPEND _mumps_libs MPI::MPI_CXX MPI::MPI_Fortran)
      target_compile_definitions(uno_dependencies INTERFACE HAS_MPI MUMPS_PARALLEL)
      find_package(BLACS REQUIRED)
      target_link_libraries(uno_dependencies INTERFACE ${BLACS_LIBRARY})
      target_include_directories(uno_dependencies SYSTEM INTERFACE ${BLACS_INCLUDE_DIRS})
      find_package(ScaLAPACK REQUIRED)
      target_link_libraries(uno_dependencies INTERFACE ${ScaLAPACK_LIBRARY})
      target_include_directories(uno_dependencies SYSTEM INTERFACE ${ScaLAPACK_INCLUDE_DIRS})
   endif()
   link_uno_target(mumps ${_mumps_libs})
   set(_mumps_found TRUE)
   unset(_mumps_libs)
else()
   find_package(MUMPS)
   if(MUMPS_LIBRARY)
      target_link_libraries(uno_dependencies INTERFACE ${MUMPS_LIBRARY} ${MUMPS_COMMON_LIBRARY} ${MUMPS_PORD_LIBRARY})
      target_compile_definitions(uno_dependencies INTERFACE HAS_MUMPS)
      target_include_directories(uno_dependencies SYSTEM INTERFACE ${MUMPS_INCLUDE_DIR})
      if(NOT MUMPS_MPISEQ_LIBRARY)
         find_package(MPI REQUIRED)
         target_link_libraries(uno_dependencies INTERFACE MPI::MPI_CXX MPI::MPI_Fortran)
         target_compile_definitions(uno_dependencies INTERFACE HAS_MPI MUMPS_PARALLEL)
         find_package(BLACS REQUIRED)
         target_link_libraries(uno_dependencies INTERFACE ${BLACS_LIBRARY})
         target_include_directories(uno_dependencies SYSTEM INTERFACE ${BLACS_INCLUDE_DIRS})
         find_package(ScaLAPACK REQUIRED)
         target_link_libraries(uno_dependencies INTERFACE ${ScaLAPACK_LIBRARY})
         target_include_directories(uno_dependencies SYSTEM INTERFACE ${ScaLAPACK_INCLUDE_DIRS})
      else()
         link_to_uno(MUMPS_MPISEQ_LIBRARY ${MUMPS_MPISEQ_LIBRARY})
      endif()
      set(_mumps_found TRUE)
   endif()
endif()
if(_mumps_found)
   target_sources(compiled_uno_files PRIVATE
      "${UNO_SOURCE_DIR}/uno/ingredients/subproblem_solvers/MUMPS/MUMPSSolver.cpp")
   if(ENABLE_TESTS)
      target_sources(run_unotest PRIVATE
         "${UNO_SOURCE_DIR}/unotest/functional_tests/MUMPSSolverTests.cpp")
   endif()
   if(NOT METIS AND NOT TARGET METIS::metis)
      find_package(METIS REQUIRED)
      target_link_libraries(uno_dependencies INTERFACE ${METIS_LIBRARY})
      target_include_directories(uno_dependencies SYSTEM INTERFACE ${METIS_INCLUDE_DIRS})
   endif()
   if(NOT OpenMP_FOUND)
      find_package(OpenMP REQUIRED)
   endif()
   target_link_libraries(uno_dependencies INTERFACE OpenMP::OpenMP_CXX)
   message(STATUS "uno_build: found MUMPS")
endif()

find_library(SPRAL NAMES spral)
if(SPRAL)
   target_sources(compiled_uno_files PRIVATE
      "${UNO_SOURCE_DIR}/uno/ingredients/subproblem_solvers/SSIDS/SSIDSSolver.cpp")
   if(ENABLE_TESTS)
      target_sources(run_unotest PRIVATE
         "${UNO_SOURCE_DIR}/unotest/functional_tests/SSIDSSolverTests.cpp")
   endif()
   link_to_uno(spral ${SPRAL})
endif()

function(resolve_library_paths input_list output_list)
   set(result)
   foreach(entry IN LISTS ${input_list})
      if(entry MATCHES "^-l")
         list(APPEND result "${entry}")
      else()
         get_filename_component(absolute_entry "${entry}" ABSOLUTE)
         list(APPEND result "${absolute_entry}")
      endif()
   endforeach()
   set(${output_list} "${result}" PARENT_SCOPE)
endfunction()

find_package(OpenBLAS CONFIG QUIET)
if(TARGET OpenBLAS::OpenBLAS)
   target_link_libraries(uno_dependencies INTERFACE OpenBLAS::OpenBLAS)
   message(STATUS "uno_build: using OpenBLAS::OpenBLAS for BLAS and LAPACK")
else()
   if(LAPACK_LIBRARIES)
      resolve_library_paths(LAPACK_LIBRARIES ABSOLUTE_LAPACK_LIBRARIES)
      target_link_libraries(uno_dependencies INTERFACE ${ABSOLUTE_LAPACK_LIBRARIES})
   else()
      find_package(LAPACK REQUIRED)
      target_link_libraries(uno_dependencies INTERFACE LAPACK::LAPACK)
   endif()
   if(BLAS_LIBRARIES)
      resolve_library_paths(BLAS_LIBRARIES ABSOLUTE_BLAS_LIBRARIES)
      target_link_libraries(uno_dependencies INTERFACE ${ABSOLUTE_BLAS_LIBRARIES})
   else()
      find_package(BLAS REQUIRED)
      target_link_libraries(uno_dependencies INTERFACE BLAS::BLAS)
   endif()
endif()

if(AUXILIARY_LIBRARIES)
   resolve_library_paths(AUXILIARY_LIBRARIES ABSOLUTE_AUXILIARY_LIBRARIES)
   target_link_libraries(uno_dependencies INTERFACE ${ABSOLUTE_AUXILIARY_LIBRARIES})
endif()

set(FORTRAN_LIBS ${CMAKE_Fortran_IMPLICIT_LINK_LIBRARIES})
if(APPLE)
   list(REMOVE_ITEM FORTRAN_LIBS gcc emutls_w heapt_w)
endif()
if(MINGW)
   list(REMOVE_ITEM FORTRAN_LIBS gcc_s)
endif()
target_link_libraries(uno_dependencies INTERFACE ${FORTRAN_LIBS})
target_link_libraries(compiled_uno_files PUBLIC uno_dependencies)

add_library(uno_static STATIC $<TARGET_OBJECTS:compiled_uno_files>)
target_link_libraries(uno_static PUBLIC uno_dependencies)
set_target_properties(uno_static PROPERTIES OUTPUT_NAME "uno" POSITION_INDEPENDENT_CODE ON)
add_library(uno::uno_static ALIAS uno_static)

if(ENABLE_TESTS)
   include(FetchContent)
   FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
   )
   set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
   set(INSTALL_GMOCK OFF CACHE BOOL "" FORCE)
   set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
   FetchContent_MakeAvailable(googletest)
   message(STATUS "uno_build: fetched GoogleTest")

   target_link_libraries(run_unotest PUBLIC GTest::gtest uno_static uno_dependencies ${CMAKE_DL_LIBS})
   if(MSVC)
      set_target_properties(run_unotest PROPERTIES MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
   endif()
   include(GoogleTest)
   gtest_discover_tests(run_unotest DISCOVERY_MODE PRE_TEST)
endif()
