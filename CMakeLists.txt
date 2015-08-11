set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-dev -pipe -std=c++14 -fopenmp -fPIC ")

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(CMAKE_AUTOMOC ON)

project(main)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake")

#find_package(Qt5Widgets REQUIRED)

INCLUDE_DIRECTORIES(includes)
INCLUDE_DIRECTORIES(source)
#INCLUDE_DIRECTORIES(UI)

#SET(UI_Files
#     UI/mainwindow.ui

#)

SET(SOURCE
    main.cpp
    src/timedate.cpp
    src/displayptcloud.cpp
    src/cloudoperations.cpp
#    src/constants.cpp
#     src/mainwindow.cpp
)

SET(HEADERS

    includes/timedate.h
    includes/displayptcloud.h
    includes/cloudoperations.h
    includes/constants.h
#    includes/mainwindow.h
)

#QT5_WRAP_UI(UIsrc ${UI_Files})
#QT5_WRAP_CPP(MOCSrcs ${QT_WRAP})

ADD_EXECUTABLE(
    main
    ${SOURCE}
    ${HEADERS}
    ${PCL_DIR}
#    ${UIsrc}
#    ${MOCSrcs}
#    ${UI_RESOURCES}

)

target_link_libraries(

    ${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${Qt5Core_QTMAIN_LIBRARIES}
)

#qt5_use_modules(main Widgets)
