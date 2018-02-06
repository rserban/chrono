# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

SET(DEMOS
    test_steering_input
    test_steering_compliance
    test_steering_hmmwv
)

# ------------------------------------------------------------------------------

SET(CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
SET(LIBRARIES ChronoEngine ChronoEngine_vehicle ChronoModels_vehicle ChronoEngine_irrlicht)

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

if(ENABLE_MODULE_MKL)
  SET(CXX_FLAGS "${CXX_FLAGS} ${CH_MKL_CXX_FLAGS}")
  SET(LIBRARIES ${LIBRARIES} ChronoEngine_mkl)

  INCLUDE_DIRECTORIES(${CH_MKL_INCLUDES})
endif()

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

MESSAGE(STATUS "Test programs for collision...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})

ENDFOREACH(PROGRAM)

