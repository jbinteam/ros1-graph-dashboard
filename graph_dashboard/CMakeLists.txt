cmake_minimum_required(VERSION 3.0.2)
project(graph_dashboard)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/scan
    scripts/serve
    scripts/bench_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # Not catkin_add_nosetests(): the tests use pytest fixtures (tmp_path,
  # monkeypatch) that nose cannot supply. Register pytest with catkin's own
  # test-target machinery instead, so `catkin_make run_tests` collects the
  # results like any other catkin test.
  catkin_run_tests_target("pytest" "unit" "pytest-unit.xml"
    COMMAND
      "${PYTHON_EXECUTABLE} -m pytest ${CMAKE_CURRENT_SOURCE_DIR}/test \
       --junitxml=${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/pytest-unit.xml"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()
