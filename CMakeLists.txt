cmake_minimum_required(VERSION 2.8.3)
project(ud_cursor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs std_msgs message_generation ) 

add_message_files(
  FILES
  UDCursor.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

catkin_package(
    ##INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy message_runtime ud_imarker
    DEPENDS ach QtCore QtGui )

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
# add_definitions(-DQT_NO_KEYWORDS)


## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  src/ud_cursor_tool.h
)


## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
 src/ud_cursor_tool.cpp
  ${MOC_FILES}
)

include_directories( include ud_imarker/include)

## Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



add_library( ${PROJECT_NAME} ${SOURCE_FILES} )


## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
##target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ach huboparams)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

