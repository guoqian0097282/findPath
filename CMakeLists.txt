cmake_minimum_required(VERSION 2.8.3)
project(FindPathTest)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++17 -w)
set(CMAKE_CXX_FLAGS " -Wall -Wextra ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "Debug")


find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)


# # for x86 build
set(OsqpEigen_DIR ${CMAKE_SOURCE_DIR}/3rdparty/AFA_LIB/x86/osqp-eigen-0.4.1/lib/cmake/OsqpEigen)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/AFA_LIB/x86/osqp-eigen-0.4.1/include/OsqpEigen)
set(gflags_DIR ${CMAKE_SOURCE_DIR}/3rdparty/AFA_LIB/x86/osqp-eigen-0.4.1/lib/cmake/OsqpEigen)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/AFA_LIB/x86/gflags-2.2.2/include/gflags)
set(osqp_DIR ${CMAKE_SOURCE_DIR}/3rdparty/AFA_LIB/x86/osqp-0.5.0/lib/cmake/osqp)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/AFA_LIB/x86/osqp-0.5.0/include)

find_package(osqp REQUIRED)
find_package(gflags REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(
        include
        include/speed/
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
        src/tools/tools.cpp
        src/tools/spline.cpp
        src/path_optimizer.cpp
        src/tools/collision_checker.cpp
        src/reference_path_smoother/reference_path_smoother.cpp
        src/tools/Map.cpp
        src/tools/car_geometry.cpp
        src/solver/base_solver.cpp
        src/data_struct/date_struct.cpp
        src/data_struct/reference_path_impl.cpp
        src/data_struct/reference_path.cpp
        src/data_struct/vehicle_state_frenet.cpp
        src/config/planning_flags.cpp
        src/reference_path_smoother/tension_smoother.cpp
        src/reference_path_smoother/tension_smoother_2.cpp
        src/tools/time_recorder.cpp
        
        src/grid_map_core/GridMap.cpp
        src/grid_map_core/GridMapMath.cpp
        src/grid_map_core/SubmapGeometry.cpp
        src/grid_map_core/BufferRegion.cpp
        src/grid_map_core/Polygon.cpp
        src/grid_map_core/CubicInterpolation.cpp
        src/grid_map_core/iterators/GridMapIterator.cpp
        src/grid_map_core/iterators/SubmapIterator.cpp
        src/grid_map_core/iterators/CircleIterator.cpp
        src/grid_map_core/iterators/EllipseIterator.cpp
        src/grid_map_core/iterators/SpiralIterator.cpp
        src/grid_map_core/iterators/PolygonIterator.cpp
        src/grid_map_core/iterators/LineIterator.cpp
        src/grid_map_core/iterators/SlidingWindowIterator.cpp

        src/tinyspline_ros/tinyspline.c
        src/tinyspline_ros/tinysplinecpp.cpp
        src/bezier.cpp
        src/curve.cpp

        src/speed/box2d.cpp
        src/speed/discretized_path.cpp
        src/speed/dp_st_cost.cpp
        src/speed/frenet_frame_path.cpp
        src/speed/gridded_path_time_graph.cpp
        src/speed/line_segment2d.cpp
        src/speed/linear_interpolation.cpp
        src/speed/math_utils.cpp
        src/speed/obstacle.cpp
        src/speed/path_data.cpp
        src/speed/piecewise_jerk_problem.cpp
        src/speed/piecewise_jerk_speed_problem.cpp
        src/speed/planning_gflags.cpp
        src/speed/polygon2d.cpp
        src/speed/speed_data.cpp
        src/speed/speed_limit.cpp
        src/speed/speed_optimizer.cpp
        src/speed/speed_profile_generator.cpp
        src/speed/st_boundary.cpp
        src/speed/st_graph_data.cpp
        src/speed/st_graph_point.cpp
        src/speed/st_obstacles_processor.cpp
        src/speed/st_point.cpp
        src/speed/util.cpp
        src/speed/vec2d.cpp
        )
target_link_libraries(${PROJECT_NAME}  gflags OsqpEigen::OsqpEigen osqp::osqp
        )

add_executable(final
        src/test/final.cpp)
target_link_libraries(final
        ${PROJECT_NAME} ${OpenCV_LIBRARIES} 
        )

add_executable(hough_main
        src/test/hough_main.cpp)
target_link_libraries(hough_main
        ${PROJECT_NAME} ${OpenCV_LIBRARIES}
        )
        

add_executable(advance_filter
        src/test/advanced_main.cpp)
target_link_libraries(advance_filter
        ${PROJECT_NAME} ${OpenCV_LIBRARIES}
        )

# add_executable(avoidObstacle_main
#         src/test/avoidObstacle_main.cpp)
# target_link_libraries(avoidObstacle_main
#         ${PROJECT_NAME} ${OpenCV_LIBRARIES}
#         )
add_executable(many_hough
        src/test/many_hough.cpp)
target_link_libraries(many_hough
        ${PROJECT_NAME} ${OpenCV_LIBRARIES}
        )

add_executable(regularI_main
        src/test/regularI_main.cpp)
target_link_libraries(regularI_main
        ${PROJECT_NAME} ${OpenCV_LIBRARIES}
        )


add_executable(main
        src/test/main.cpp)
target_link_libraries(main
        ${PROJECT_NAME} ${OpenCV_LIBRARIES}
        )