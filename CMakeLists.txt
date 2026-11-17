# ==========================================
# BLOK 1: Metryczka projektu i standard C++
# ==========================================
cmake_minimum_required(VERSION 3.16)
project(PlanarArmIKPID LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ==========================================
# BLOK 2: Wbudowany menedżer paczek CMake'a
# ==========================================
include(FetchContent)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE 
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)


# ==========================================
# BLOK 3: Twój program (co ma zostać skompilowane)
# ==========================================
add_executable(arm_sim
    src/main.cpp
)


# ==========================================
# BLOK 4: Podpinanie nagłówków (.hpp) i bibliotek
# ==========================================
target_include_directories(arm_sim PRIVATE include ${raylib_SOURCE_DIR}/src)

# Linkujemy (sklejamy) nasz program z pobranymi wyżej bibliotekami
target_link_libraries(arm_sim PRIVATE Eigen3::Eigen raylib)