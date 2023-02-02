cmake_minimum_required(VERSION 3.5)

list(APPEND CMAKE_PREFIX_PATH /opt/ros/noetic/)

project(husky_gui VERSION 0.1 LANGUAGES CXX)

add_compile_options(-std=c++14)

set(CMAKE_PROJECT_NAME husky_gui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC_SEARCH_PATHS src)
set(CMAKE_AUTOUIC ON) #this will autogenerate ui_mainwindow.h
set(CMAKE_AUTOMOC ON) #this will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON) #this needs to be on so cmake will know what to do with QT resource Files
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_FLAGS "-fpermissive")

set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5PrintSupport
  OpenCV
  )

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::PrintSupport
  Qt5::Core
  Qt5::Gui
  Qt5::Network
  ${OpenCV_LIBS}
  )


set(ROS_DEPENDS
  roscpp
  rviz
  cv_bridge
  image_transport
  )

find_package(QT NAMES Qt6 Qt5 Quick Gui REQUIRED COMPONENTS Widgets Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Network)
find_package( OpenCV REQUIRED )

###
###  Add Non ROS dependencies
###

foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

# Add any required ros packages here
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDS})

catkin_package(
  INCLUDE_DIRS
    include
    include/
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${ROS_DEPENDS}
  DEPENDS
    ${EXTERNAL_DEPENDS}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRECTORIES}
  #home/vasilis/catkin_ws/src/ogre
)

set(PROJECT_SOURCES
        ./include/husky_gui2/husky_gui.h
        ./include/husky_gui2/logger.h
        ./include/husky_gui2/public_params.h
        ./include/husky_gui2/qnode.h
        ./include/husky_gui2/ros_topics.h
        src/husky_gui.cpp
        #src/image_subscriber.cpp
        src/logger.cpp
        src/public_params.cpp
        src/main.cpp
        src/mainwindow.ui
        src/qnode.cpp
        src/ros_topics.cpp
        package.xml
        arrows.qrc
        visual_cues.qrc
    )

###########
## Build ##
###########

#file(GLOB HDRS
#  "include/${PROJECT_NAME}/*.h"
#)
#file(GLOB SRCS
#  "src/*.cpp"
#  "src/*.hpp"
#)
#file(GLOB NODES
#  "nodes/*.cpp"
#)

#find all the qt UI stuff
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/mainwindow/*.h)
file(GLOB_RECURSE UI
    "src/*.ui"
    #"include/${PROJECT_NAME}/*.ui"
)

message("Adding ${PROJECT_NAME} UI files: " )
foreach(source ${UI})
    message("  - " ${source})
endforeach(source)

#make them into headers
qt5_wrap_ui (UIS_HDRS  ${UI})

message("Generating ${PROJECT_NAME} UI Headers: " )
foreach(source ${UI_HDRS})
    message("  - " ${source})
endforeach(source)

## find all resource files
#file(GLOB QRC
#    "rc/*.qrc"
#)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(husky_gui
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET husky_gui APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(${CMAKE_PROJECT_NAME} SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(${CMAKE_PROJECT_NAME}
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Network
    ${catkin_LIBRARIES}    #ATTENTION: this links the gui with ROS and other libraries - if missing, 'undefined ros::" errors occur
    ${OpenCV_LIBS}
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS ${CMAKE_PROJECT_NAME}
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(TARGETS ${NODE_LIST}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

###
### compile
###

message("building ${PROJECT_NAME} node: ")
foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message("  - " ${NODE_NAME})

  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS} ${UI} ${QRC})
  add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries (${NODE_NAME} ${catkin_LIBRARIES}  ${EXTERNAL_DEPENDS_LIB})
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")

  list(APPEND NODE_LIST ${NODE_NAME})

endforeach(NODE_FILE)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(husky_gui)
endif()
