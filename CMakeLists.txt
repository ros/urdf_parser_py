cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project(urdfdom_py)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/display_urdf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(SETUP_PY "${CMAKE_CURRENT_SOURCE_DIR}/setup.py")
install(CODE "execute_process(COMMAND \"${PYTHON}\" \"${SETUP_PY}\" build --build-base \"${CMAKE_CURRENT_BINARY_DIR}/pybuild\" install --install-layout deb --prefix \"${CMAKE_INSTALL_PREFIX}\"
              WORKING_DIRECTORY \"${CMAKE_CURRENT_SOURCE_DIR}\")")

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_urdf.py)
endif()
