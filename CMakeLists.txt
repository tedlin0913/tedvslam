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

set(CMAKE_CXX_FLAGS "-std=c++17")

set(G2O_ROOT /usr/local/include/g2o)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

# find dependencies
# find_package(ament_cmake REQUIRED)

# find_package(rclcpp REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find_package(cv_bridge REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# find_package(Boost REQUIRED)
# include_directories(${Boost_INCLUDE_DIRS})

# gtest
# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

find_package(spdlog REQUIRED)



find_package(fmt REQUIRED)
# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_LIST_DIR}/external/DBoW2/include)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 0.8.0
)
FetchContent_MakeAvailable(yaml-cpp)

include(FetchContent)
FetchContent_Declare(
    fast_csv
    GIT_REPOSITORY https://github.com/ben-strasser/fast-cpp-csv-parser.git
    GIT_TAG        master  # Use the latest version, or specify a specific tag/commit
)
FetchContent_MakeAvailable(fast_csv)

add_library(${PROJECT_NAME}
SHARED
  src/config.cpp
  src/feature.cpp
  src/dataset.cpp
  src/frame.cpp
  src/mappoint.cpp
  
  src/keyframe.cpp
  src/camera.cpp
  src/map.cpp


  src/frontend.cpp
  src/backend.cpp
  src/viewer.cpp

  src/visual_odometry.cpp


)

target_include_directories(${PROJECT_NAME} 
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<BUILD_INTERFACE:${fast_csv_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# target_include_directories(csv_example PRIVATE ${fast_csv_SOURCE_DIR})

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

include_directories("/usr/include/suitesparse")
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

target_link_libraries(${PROJECT_NAME}
PUBLIC
  # ${LIB_NAME}
  # ${rclcpp_LIBRARIES}
  ${OpenCV_LIBS}
  # cv_bridge
  ${EIGEN3_LIBS}
  ${Sophus_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${G2O_LIBS}
  # g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
  # ${CMAKE_CURRENT_LIST_DIR}/3rdParty/DBoW2/build/libDBoW2.so
  ${GFLAGS_LIBRARIES}
  ${CSPARSE_LIBRARY}
  GL GLU GLEW glut
  fmt::fmt
  pthread
  yaml-cpp::yaml-cpp
  spdlog::spdlog
)

add_subdirectory(app)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
