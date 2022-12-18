if (_CMAKE_EXTERNAL_PROJECT_CATKIN_INCLUDED_)
  return()
endif()
set(_CMAKE_EXTERNAL_PROJECT_CATKIN_INCLUDED_ TRUE)

include(CMakeParseArguments)

@[if DEVELSPACE]@
# cmake dir in develspace
set(cmake_external_project_catkin_CMAKE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)")
@[else]@
# cmake dir in installspace
set(cmake_external_project_catkin_CMAKE_DIR "@(PKG_CMAKE_DIR)")
@[end if]@

macro(ExternalProject_blub)
message(STATUS "blalllalalala")
endmacro()

macro(ExternalProject_CatkinInstall PROJECT_NAME)
  message(STATUS "project:  ${PROJECT_NAME}")

  ExternalProject_Get_Property(${PROJECT_NAME} BINARY_DIR)
  message(STATUS "binary_dir: ${BINARY_DIR}")

  set(CATKIN_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
  find_program (CATKIN_TOOLS_AVAILABLE NAMES catkin)

  # check if we install the catkin package
  set(CATKIN_INSTALL "False")
  if (DEFINED CATKIN_TOOLS_AVAILABLE)
    message(STATUS "catkin is available (${CATKIN_TOOLS_AVAILABLE})")
    exec_program("catkin config  | grep 'Install Packages:' | tr -s ' ' | cut -c19-"
      OUTPUT_VARIABLE CATKIN_INSTALL
      )
  else ()
    message(AUTHOR_WARNING "catkin tools not installed; add another approach to check for enabled install;")
  endif ()

  message(STATUS "CATKIN_TOOLS_AVAILABLE: ${CATKIN_TOOLS_AVAILABLE}")
  message(STATUS "CATKIN_INSTALL_PREFIX: ${CATKIN_INSTALL_PREFIX}")
  message(STATUS "CATKIN_INSTALL: ${CATKIN_INSTALL}")

  if ("${CATKIN_INSTALL}" STREQUAL "True")
    message(STATUS "Package marked for installation.")
    ExternalProject_Add_Step(${PROJECT_NAME} 
      catkin_install
      DEPENDEES build
      COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_INSTALL_PREFIX} ${BINARY_DIR} && ${CMAKE_COMMAND} --build ${BINARY_DIR} --target install
      COMMENT "Installing the external project..."
      )
  else ()
    message(STATUS "Package not installed.")
  endif ()

endmacro()
