if(NOT_CH_ENABLE_MODULE_PARSERS OR NOT CH_USE_URDF)
   return()
endif()   

if(NOT_CH_ENABLE_MODULE_VEHICLE)
   return()
endif()   

if(NOT_CH_ENABLE_MODULE_VSG)
   return()
endif()   

#--------------------------------------------------------------
# List of all Parsers demos

set(PROGRAMS 
    test_ROBOSIMIAN_locomotion
    test_ROBOSIMIAN_locomotion_var
)

#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries
list(APPEND LIBS Chrono_core)
list(APPEND LIBS ChronoModels_robot)
list(APPEND LIBS Chrono_parsers)
list(APPEND LIBS Chrono_vehicle)
list(APPEND LIBS Chrono_vsg)

if(CH_ENABLE_MODULE_FSI_SPH)
  list(APPEND LIBS Chrono_fsisph)
  list(APPEND LIBS Chrono_fsisph_vsg)
endif()

#--------------------------------------------------------------

# Add executables

message(STATUS "Add test programs for RoboSimian")

foreach(PROGRAM ${PROGRAMS})
  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
  if(APPLE)
    set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
  endif()
  if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_link_libraries(${PROGRAM} ${LIBS})
endforeach()
