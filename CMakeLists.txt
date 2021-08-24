cmake_minimum_required(VERSION 3.15)

project(embree_curve_test)
find_package(embree 3.0 REQUIRED)

set(SOURCE
    curve_data.h
    curve_data.cpp
    main.cpp
)

add_executable(embree_curve_test ${SOURCE})
target_compile_features(embree_curve_test PRIVATE cxx_std_17)
target_include_directories(embree_curve_test PRIVATE ${EMBREE_INCLUDE_DIRS})
target_link_libraries(embree_curve_test PRIVATE ${EMBREE_LIBRARIES})