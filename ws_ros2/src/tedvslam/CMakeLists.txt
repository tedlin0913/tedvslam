cmake_minimum_required(VERSION 3.22)
project(tedvslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread)
endif()

# C++ standard default to C++ 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()


set(G2O_ROOT /usr/local/include/g2o)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find_package(cv_bridge REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# glog
# find_package(Glog REQUIRED)
# include_directories(${GLOG_INCLUDE_DIRS})

# find_package(Boost REQUIRED)
# include_directories(${Boost_INCLUDE_DIRS})

# gtest
# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})

# gflags
# find_package(GFlags REQUIRED)
# include_directories(${GFLAGS_INCLUDE_DIRS})

# # csparse
# find_package(CSparse REQUIRED)
# include_directories(${CSPARSE_INCLUDE_DIR})

find_package(fmt REQUIRED)
# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_LIST_DIR}/3rdParty/DBoW2/include)

# Set dependencies
set(dependencies
  # ${GLOG_LIBRARIES}
  ${OpenCV_LIBS}
  # cv_bridge
  ${EIGEN3_LIBS}
  ${Sophus_LIBRARIES}
  ${Pangolin_LIBRARIES}
  # g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
  # ${CMAKE_CURRENT_LIST_DIR}/3rdParty/DBoW2/build/libDBoW2.so
  # ${GFLAGS_LIBRARIES}
  fmt::fmt
  pthread
  # ${CSPARSE_LIBRARY}
  # Boost::boost
)

# set(LIB_NAME tedslamnode)

# add_library(${LIB_NAME}
# SHARED
#   src/frontend.cpp
#   src/backend.cpp
#   src/config.cpp
#   src/feature.cpp
#   src/dataset.cpp
#   src/map.cpp
#   src/viewer.cpp
#   src/mappoint.cpp
#   src/visual_odometry.cpp
#   src/frame.cpp
#   src/camera.cpp
# )

# target_include_directories(${LIB_NAME}
#   PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
# )
# target_link_libraries(${LIB_NAME}
#   PUBLIC
#     ${dependencies}
#   # ${CMAKE_CURRENT_LIST_DIR}/3rdParty/g2o/build/lib/libg2o.so
# )

# ament_export_libraries(${LIB_NAME})
# ament_export_include_directories(include)


# find_package(${LIB_NAME} REQUIRED)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/frontend.cpp
  src/backend.cpp
  src/config.cpp
  src/feature.cpp
  src/dataset.cpp
  src/map.cpp
  src/viewer.cpp
  src/mappoint.cpp
  src/visual_odometry.cpp
  src/frame.cpp
  src/camera.cpp
)

target_include_directories(${PROJECT_NAME} 
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)


# ament_export_libraries(b_library)
# ament_export_include_directories(include)

# ament_target_dependencies(${PROJECT_NAME} 
#   # Pangolin 
#   rclcpp 
#   # pthread 
#   fmt 
#   G2O 
#   ${EIGEN3_LIBS}
#   ${Sophus_LIBRARIES}
#   ${Pangolin_LIBRARIES}
#   ${OpenCV_LIBS}
# )
target_link_libraries(${PROJECT_NAME}
PUBLIC
  ${LIB_NAME}
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBS}
  # cv_bridge
  ${EIGEN3_LIBS}
  ${Sophus_LIBRARIES}
  ${Pangolin_LIBRARIES}
  # g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
  # ${CMAKE_CURRENT_LIST_DIR}/3rdParty/DBoW2/build/libDBoW2.so
  # ${GFLAGS_LIBRARIES}
  fmt::fmt
  pthread
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
