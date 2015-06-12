
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(main)

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(includes)
INCLUDE_DIRECTORIES(src)

SET(SOURCE
  main.cpp

  src/timedate.cpp
  src/displayptcloud.cpp
)

SET(HEADERS

  includes/timedate.h
  includes/displayptcloud.h
)

ADD_EXECUTABLE(main ${SOURCE} ${HEADERS} ${PCL_DIR})



target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} )
