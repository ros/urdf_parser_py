cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(urdfdom_py)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/display_urdf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_urdf.py)
  catkin_add_nosetests(test/test_urdf_error.py)
endif()
