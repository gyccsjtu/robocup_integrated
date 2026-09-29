cmake_minimum_required(VERSION 3.0.2)
project(robocup_training_worlds)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/generate_training_city.py
  scripts/validate_training_city.py
  scripts/rasterize_metadata.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
