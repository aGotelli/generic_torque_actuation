cmake_minimum_required(VERSION 3.5)

project(generic_torque_actuation VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


#   External utilities to configure the package
include(GNUInstallDirs)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})


#   Give different names for debug and Release
set(CMAKE_RELEASE_POSTFIX "")
set(CMAKE_DEBUG_POSTFIX "-debug")



#
# Dependencies
#
find_package(mpi_cmake_modules REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(blmc_drivers REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

#If you want to use the Aurora tracker
set(VTK_DIR "/usr/local/include/vtk-8.2")
find_package(VTK QUIET)



add_subdirectory(src)




#   Install the library using the default routine
include(cmake/installation_module.cmake)
