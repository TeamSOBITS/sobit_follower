# Install script for directory: /home/iwami/eigen-eigen-b3f3d4950030/Eigen

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/Cholesky"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/CholmodSupport"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/Core"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/Dense"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/Eigen"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/Eigenvalues"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/Geometry"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/Householder"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/IterativeLinearSolvers"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/Jacobi"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/LU"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/MetisSupport"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/OrderingMethods"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/PaStiXSupport"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/PardisoSupport"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/QR"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/QtAlignedMalloc"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/SPQRSupport"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/SVD"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/Sparse"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/SparseCholesky"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/SparseCore"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/SparseLU"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/SparseQR"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/StdDeque"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/StdList"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/StdVector"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/SuperLUSupport"
    "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/UmfPackSupport"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/iwami/eigen-eigen-b3f3d4950030/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

