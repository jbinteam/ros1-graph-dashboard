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
  DESTINATION ${CATKIN_PACKAGE_BIN_DIR}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
