cmake_minimum_required(VERSION 3.16)

project(mems_ins_gnss_loose
    VERSION 1.0.0
    DESCRIPTION "MEMS INS/GNSS Integrated Navigation System"
    LANGUAGES CXX
)

# ============================================================================
# Build Options
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Compiler options
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-g -O0)
    else()
        add_compile_options(-O3 -DNDEBUG)
    endif()
endif()

# Output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ============================================================================
# Dependencies
# ============================================================================

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# ============================================================================
# Library
# ============================================================================

add_library(gins-lib
    ins/pins.cpp
    utils/earth_model.cpp
    utils/logger.cpp
    io/nav_result_saver.cpp
    filter/gi_loose_filter.cpp
    filter/gins_config.cpp
    estimator/pose_estimator.cpp
    measurement/imu_time_aligner.cpp
)

target_include_directories(gins-lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(gins-lib
    PUBLIC
        Eigen3::Eigen
        Sophus::Sophus
        yaml-cpp::yaml-cpp
        spdlog::spdlog
)

# ============================================================================
# Executables
# ============================================================================

# INS evaluation tool
add_executable(eval_ins eval/ins-eval.cpp)
target_link_libraries(eval_ins PRIVATE gins-lib)

# Estimator evaluation tool
add_executable(estimator_eval eval/estimator_eval.cpp)
target_link_libraries(estimator_eval PRIVATE gins-lib)

# Pose evaluation tool
add_executable(pose_eval eval/pose_evaluator.cpp)
target_link_libraries(pose_eval PRIVATE gins-lib)

# ============================================================================
# Installation (optional)
# ============================================================================

# Uncomment the following lines if you want to install the project
# install(TARGETS gins-lib
#     EXPORT mems_ins_gpsTargets
#     LIBRARY DESTINATION lib
#     ARCHIVE DESTINATION lib
# )
# 
# install(TARGETS eval_ins estimator_eval pose_eval
#     RUNTIME DESTINATION bin
# )
# 
# install(EXPORT mems_ins_gpsTargets
#     FILE mems_ins_gpsTargets.cmake
#     NAMESPACE mems_ins_gps::
#     DESTINATION lib/cmake/mems_ins_gps
# )



