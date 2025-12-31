FetchContent_Declare(
    uno
    GIT_REPOSITORY https://github.com/cvanaret/Uno
    GIT_TAG        main
    EXCLUDE_FROM_ALL
)

FetchContent_Populate(uno)

file(GLOB UNO_SOURCE_FILES
   ${uno_SOURCE_DIR}/interfaces/C/Uno_C_API.cpp
   ${uno_SOURCE_DIR}/uno/Uno.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/constraint_relaxation_strategies/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/constraint_relaxation_strategies/relaxed_problems/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/globalization_mechanisms/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/globalization_strategies/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/globalization_strategies/switching_methods/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/globalization_strategies/switching_methods/filter_methods/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/globalization_strategies/switching_methods/filter_methods/filters/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/globalization_strategies/switching_methods/funnel_methods/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/hessian_models/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/inequality_handling_methods/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/inequality_handling_methods/inequality_constrained_methods/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/inequality_handling_methods/interior_point_methods/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/inequality_handling_methods/interior_point_methods/barrier_problems/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/inertia_correction_strategies/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/subproblem/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/subproblem_solvers/*.cpp
   ${uno_SOURCE_DIR}/uno/model/*.cpp
   ${uno_SOURCE_DIR}/uno/optimization/*.cpp
   ${uno_SOURCE_DIR}/uno/options/*.cpp
   ${uno_SOURCE_DIR}/uno/tools/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/subproblem_solvers/MA27/MA27Solver.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/subproblem_solvers/MA57/MA57Solver.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/subproblem_solvers/BQPD/*.cpp
   ${uno_SOURCE_DIR}/uno/ingredients/subproblem_solvers/MUMPS/MUMPSSolver.cpp
)

enable_language(Fortran)
include(FortranCInterface)
FortranCInterface_HEADER(${CMAKE_BINARY_DIR}/include/fortran_interface.h
        MACRO_NAMESPACE "FC_"
        SYMBOL_NAMESPACE "FC_")

add_library(unolib ${UNO_SOURCE_FILES})
target_compile_definitions(unolib PUBLIC HAS_MA57 HAS_MA27 HAS_BQPD HAS_MUMPS)
target_include_directories(unolib PUBLIC ${uno_SOURCE_DIR}/interfaces/C)
target_include_directories(unolib PRIVATE ${CMAKE_BINARY_DIR}/include ${uno_SOURCE_DIR}/uno)
target_link_libraries(unolib PUBLIC bqpd_sparse libdmumps libmumps_common coinhsl)
if (WIN32)
    target_compile_options(unolib PRIVATE "$<$<COMPILE_LANGUAGE:C>:/utf-8>" "$<$<COMPILE_LANGUAGE:CXX>:/utf-8>")
endif()