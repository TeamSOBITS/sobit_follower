# Install script for directory: /home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/AdolcForward"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/AlignedVector3"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/ArpackSupport"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/AutoDiff"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/BVH"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/EulerAngles"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/FFT"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/IterativeSolvers"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/KroneckerProduct"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/LevenbergMarquardt"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/MatrixFunctions"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/MoreVectorization"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/MPRealSupport"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/NonLinearOptimization"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/NumericalDiff"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/OpenGLSupport"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/Polynomials"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/Skyline"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/SparseExtra"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/SpecialFunctions"
    "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/Splines"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/home/iwami/eigen-eigen-b3f3d4950030/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/iwami/eigen-eigen-b3f3d4950030/build/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

