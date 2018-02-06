#--------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib64")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib64" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib64")
endif("${isSystemDir}" STREQUAL "-1")

#--------------------------------------------------------------

message(STATUS "==== Test programs ====")

add_subdirectory(filesystem)
add_subdirectory(hdf5)

if(ENABLE_MODULE_IRRLICHT)
    add_subdirectory(collision)
    add_subdirectory(render)
endif()

if(ENABLE_MODULE_FEA AND ENABLE_MODULE_IRRLICHT)
    add_subdirectory(collision_FEA)
endif()

if(ENABLE_MODULE_POSTPROCESS)
    add_subdirectory(postprocess)
endif()

if(ENABLE_MODULE_PARALLEL)
    add_subdirectory(parallel)
    add_subdirectory(box_collision)
    add_subdirectory(rolling)
endif()

if(ENABLE_MODULE_PARALLEL AND ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_OPENGL)
    add_subdirectory(body_force)
endif()

if(ENABLE_MODULE_VEHICLE AND ENABLE_MODULE_PARALLEL)
    add_subdirectory(go_nogo)
endif()

if(ENABLE_MODULE_VEHICLE AND ENABLE_MODULE_IRRLICHT)
	add_subdirectory(vehicle)
    add_subdirectory(steering_mechanism)
endif()

if(ENABLE_MODULE_VEHICLE)
    add_subdirectory(wvp_benchmark)
	add_subdirectory(go_nogo_SCM)
endif()

