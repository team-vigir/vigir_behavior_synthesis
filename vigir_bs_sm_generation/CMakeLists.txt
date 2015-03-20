cmake_minimum_required(VERSION 2.8.3)
project(vigir_bs_sm_generation)

find_package(catkin REQUIRED COMPONENTS message_generation)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# generate srv
# add_service_files(FILES 
#   CompileLTL.srv
# )

# opt arg DEPENDENCIES ...
# generate_messages()

# specify catkin-specific information
# INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package
# LIBRARIES - The exported libraries from the project
# CATKIN_DEPENDS - Other catkin projects that this project depends on
# DEPENDS - Non-catkin CMake projects that this project depends on
# CFG_EXTRAS - Additional configuration options 
catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime)

# use add_library() or add_executable() as required
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

# install executables
# install(PROGRAMS bin/start_behavior
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

