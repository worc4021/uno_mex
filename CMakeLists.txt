cmake_minimum_required(VERSION 3.25)
project(UnoMex)

include(GNUInstallDirs)
set(MKL_LINK static)
set(MKL_THREADING sequential)
set(MKL_INTERFACE lp64)
find_package(MKL REQUIRED)


include(FetchContent)


FetchContent_Declare(
    MexUtilities
    GIT_REPOSITORY https://github.com/worc4021/MexUtilities
    GIT_TAG        main
    EXCLUDE_FROM_ALL
)

FetchContent_Declare(
    BQPD
    GIT_REPOSITORY https://github.com/worc4021/bqpd_lib
    GIT_TAG        main
    EXCLUDE_FROM_ALL
)

FetchContent_Declare(
    MUMPS
    GIT_REPOSITORY https://github.com/worc4021/MUMPS_cmake
    GIT_TAG        main
    EXCLUDE_FROM_ALL
)

FetchContent_Declare(
    coinhsl
    GIT_REPOSITORY https://github.com/worc4021/coinhsl
    GIT_TAG        main
    EXCLUDE_FROM_ALL
)

FetchContent_MakeAvailable(MexUtilities BQPD MUMPS coinhsl)

add_subdirectory(uno)

find_package(Matlab REQUIRED)

add_executable(hs015 src/hs015.cpp)
target_link_libraries(hs015 PRIVATE unolib)

# matlab_add_mex(
#     NAME uno_mex
#     SRC src/main.cpp
#     LINK_TO MexUtilities::MexUtilities unolib
#     R2018a
# )
# target_include_directories(uno_mex PRIVATE include)

# matlab_add_mex(
#     NAME uno_options
#     SRC src/get_options.cpp
#     LINK_TO MexUtilities::MexUtilities unolib
#     R2018a
# )
# target_include_directories(uno_options PRIVATE include)