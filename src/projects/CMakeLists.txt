#--------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

#--------------------------------------------------------------

# Enable creation of "application bundles" on MacOSX.
# This is necessary for Irrlicht-based tests and optional for OpenGL or non-graphics tests.
# For simplicity and consistency, we enable here MACOSX_BUNDLE for all subsequent programs.

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
    set(MACOSX_BUNDLE_INFO_PLIST "${PROJECT_SOURCE_DIR}/cmake/MacOSXBundleInfo.plist.in")
endif()

#--------------------------------------------------------------

message(STATUS "")
message(STATUS "==== Projects ====")
message(STATUS "")

option(BUILD_PROJECTS_BASE "Build base Chrono module projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_BASE)
if(BUILD_PROJECTS_BASE)
  add_subdirectory(base)
endif()

option(BUILD_PROJECTS_VISUALIZATION "Build visualization projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_VISUALIZATION)
if(BUILD_PROJECTS_VISUALIZATION)
  add_subdirectory(visualization)
endif()

option(BUILD_PROJECTS_POSTPROCESS "Build Chrono::Postprocess projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_POSTPROCESS)
if(BUILD_PROJECTS_POSTPROCESS)
  add_subdirectory(postprocess)
endif()

option(BUILD_PROJECTS_HDF5 "Build HDF5 projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_HDF5)
if(BUILD_PROJECTS_HDF5)
  add_subdirectory(hdf5)
endif()

option(BUILD_PROJECTS_CONTACT "Build contact projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_CONTACT)
if(BUILD_PROJECTS_CONTACT)
  add_subdirectory(contact)
endif()

option(BUILD_PROJECTS_GRANULAR "Build granular dynamics projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_GRANULAR)
if(BUILD_PROJECTS_GRANULAR)
  add_subdirectory(granular)
endif()

option(BUILD_PROJECTS_FSI "Build Chrono::FSI projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_FSI)
if(BUILD_PROJECTS_FSI)
  add_subdirectory(fsi)
endif()

option(BUILD_PROJECTS_GEOMECHANICS "Build geomechanics projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_GEOMECHANICS)
if(BUILD_PROJECTS_GEOMECHANICS)
  add_subdirectory(geomechanics)
endif()

option(BUILD_PROJECTS_VEHICLE "Build Chrono::Vehicle projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_VEHICLE)
if(BUILD_PROJECTS_VEHICLE)
  add_subdirectory(vehicle)
endif()

option(BUILD_PROJECTS_DISTRIBUTED "Build Chrono::Distributed projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_DISTRIBUTED)
if(BUILD_PROJECTS_DISTRIBUTED)
  add_subdirectory(distributed)
endif()

#option(BUILD_PROJECTS_WVP "Build WVP (wheeled vehicle platform) projects" ON)
#mark_as_advanced(FORCE BUILD_PROJECTS_WVP)
#if(BUILD_PROJECTS_WVP)
#  add_subdirectory(wvp_benchmark)
#endif()

option(BUILD_PROJECTS_FEDA "Build FED-alpha projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_FEDA)
if(BUILD_PROJECTS_FEDA)
  add_subdirectory(feda_benchmark)
endif()

option(BUILD_PROJECTS_M113a "Build M113a projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_M113a)
if(BUILD_PROJECTS_M113a)
  add_subdirectory(m113a_benchmark) 
endif()

option(BUILD_PROJECTS_CORRIDOR "Build AV corridor projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_CORRIDOR)
if(BUILD_PROJECTS_CORRIDOR)
  add_subdirectory(corridor)
endif()

option(BUILD_PROJECTS_FEA "Build FEA projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_FEA)
if(BUILD_PROJECTS_FEA)
  add_subdirectory(fea)
endif()

option(BUILD_PROJECTS_BENCHMARKS "Build benchmark projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_BENCHMARKS)
if(BUILD_PROJECTS_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

option(BUILD_PROJECTS_SYNCHRONO "Build Synchrono projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_SYNCHRONO)
if(BUILD_PROJECTS_SYNCHRONO)
  add_subdirectory(synchrono)
endif()

option(BUILD_PROJECTS_POLARIS "Build Polaris projects" ON)
mark_as_advanced(FORCE BUILD_PROJECTS_POLARIS)
if(BUILD_PROJECTS_POLARIS)
  add_subdirectory(polaris)
endif()
