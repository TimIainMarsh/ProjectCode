set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(CMAKE_AUTOMOC ON)

project(main)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(includes)
INCLUDE_DIRECTORIES(source)

SET(SOURCE
    main.cpp
    src/timedate.cpp
    src/Functions.cpp
    src/cloudoperations.cpp
    src/segmentor.cpp
    src/projection.cpp
)

SET(HEADERS

    includes/timedate.h
    includes/Functions.h
    includes/cloudoperations.h
    includes/constants.h
    includes/segmentor.h
    includes/projection.h
)


ADD_EXECUTABLE(
    main
    ${SOURCE}
    ${HEADERS}
    ${PCL_DIR}

)

target_link_libraries(

    ${PROJECT_NAME}
    ${PCL_LIBRARIES}
)

