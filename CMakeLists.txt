cmake_minimum_required(VERSION 2.8.3)
project(ros_type_introspection)

# Build flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


if(NOT NO_CATKIN)
   find_package(catkin REQUIRED COMPONENTS
      smclib
   )

   catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ros_type_introspection
      CATKIN_DEPENDS
      DEPENDS
   )
   set(EXTRA_LIBRARIES ${catkin_LIBRARIES})
   include_directories(${catkin_INCLUDE_DIRS})
else(NOT NO_CATKIN)
   set(CATKIN_PACKAGE_LIB_DESTINATION lib)
   set(CATKIN_PACKAGE_INCLUDE_DESTINATION include)
   if(ROS_INCLUDE_PATH)
      include_directories(${ROS_INCLUDE_PATH})
   else(ROS_INCLUDE_PATH)
      find_package(PkgConfig REQUIRED)
      pkg_check_modules(rostime REQUIRED rostime)
      include_directories(${rostime_INCLUDE_DIRS})
   endif(ROS_INCLUDE_PATH)
endif(NOT NO_CATKIN)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
    include
)

## Declare a C++ library
add_library(ros_type_introspection STATIC

   src/ros_type.cpp
   src/ros_field.cpp
   src/stringtree_leaf.cpp
   src/ros_message.cpp
   src/substitution_rule.cpp
   src/ros_introspection.cpp
 )

if (COMMON_CXX_PATH)
   include_directories(${COMMON_CXX_PATH})
   target_compile_definitions(ros_type_introspection PUBLIC ENABLE_COMMON_CXX)
else (COMMON_CXX_PATH)
   set (EXTRA_LIBRARIES ${EXTRA_LIBRARIES} absl_any absl_strings absl_base absl_throw_delegate)
endif (COMMON_CXX_PATH)

if (DISABLE_SERIALIZATION)
   target_compile_definitions(ros_type_introspection PUBLIC DISABLE_SERIALIZATION)
endif (DISABLE_SERIALIZATION)

target_link_libraries(ros_type_introspection ${EXTRA_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Mark executables and/or libraries for installation
install(TARGETS ros_type_introspection 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".git" EXCLUDE
)

#############
## Testing ##
#############

