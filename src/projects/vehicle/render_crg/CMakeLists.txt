#=============================================================================
# CMake configuration file for the CRGTerrain demos.
# These example programs require Irrlicht run-time visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT OR NOT HAVE_OPENCRG)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all demos

SET(DEMOS
    test_VEH_render_crg
    )

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# Create the executables

FOREACH(DEMO ${DEMOS})

    MESSAGE(STATUS "...add ${DEMO}")

    ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp)
    SOURCE_GROUP("" FILES ${DEMO}.cpp)
    SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                          COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    TARGET_LINK_LIBRARIES(${DEMO}
                          ChronoEngine
                          ChronoEngine_irrlicht
                          ChronoEngine_vehicle
                          ChronoEngine_vehicle_irrlicht
                          ChronoModels_vehicle)
    INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()
