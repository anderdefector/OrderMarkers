cmake_minimum_required(VERSION 2.8)

project( DisplayImage )

find_package( OpenCV REQUIRED )
#find_package(Qt5 COMPONENTS Widgets REQUIRED OpenGL REQUIRED)
#find_package(Qt5 COMPONENTS OpenGL REQUIRED)
#find_package(OpenGL REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} )
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(triangles SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/marker/triangles.cc)
add_library(lambda SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/marker/lambdaMatrix.cc)
add_library(follow SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/marker/follow.cc)

add_library(reconoce SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/marker/reconoce.cc)

add_library(eigen SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/homography/eigen.cc)
add_library(homografia SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/homography/homografia.cc)
add_library(matrices SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/homography/matrices.cc)

add_executable(DisplayImage displayimage.cpp )


target_link_libraries( DisplayImage ${OpenCV_LIBS} reconoce triangles lambda follow  homografia eigen matrices )

#target_link_libraries( main ${OpenCV_LIBS}  Qt5::Widgets Qt5::OpenGL ${OPENGL_INCLUDE_DIRS} )
