cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
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

  # Force CMake to find python again in case it was called before to find python 2
  unset(PYTHON_EXECUTABLE CACHE)
  unset(PYTHONINTERP_FOUND CACHE)
  find_package(PythonInterp 3 REQUIRED)
  add_test(NAME test_py3_syntax COMMAND "${PYTHON_EXECUTABLE}" "-m" "flake8" "--select" "E999")
endif()
