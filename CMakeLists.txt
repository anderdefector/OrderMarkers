cmake_minimum_required(VERSION 2.8)


project( DisplayImage )
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

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

#add_library(procesa SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/procesa/procesa.cpp)
add_library(procesaRobot SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/procesa/procesaRobot.cpp)


add_executable(Servidor Servidor.cpp )

target_link_libraries( Servidor Threads::Threads procesaRobot ${OpenCV_LIBS}  reconoce triangles lambda follow  homografia eigen matrices )

#target_link_libraries( main ${OpenCV_LIBS}  Qt5::Widgets Qt5::OpenGL ${OPENGL_INCLUDE_DIRS} )
