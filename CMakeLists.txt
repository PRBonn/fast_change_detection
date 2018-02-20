cmake_minimum_required(VERSION 2.8.3)
project(fast_change_detection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -DNDEBUG -O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS glow)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt5Gui REQUIRED) 
find_package(Qt5OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

set(CMAKE_INCLUDE_CURRENT_DIR ON) # needs to be activated for qt generated files in build directory.
set(CMAKE_AUTOMOC ON)

option(USE_OpenMP "Use OpenMP" OFF)
if(USE_OpenMP)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS src/
  LIBRARIES fastcd
  CATKIN_DEPENDS glow
)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${QT5_INCLUDE_DIRS} 
                    ${GLEW_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS} 
                    /usr/include/eigen3
                    src)

set(FASTCD_SHADER_SRC ${CMAKE_BINARY_DIR}/fastcd_shaders.cpp)
set(VISUALIZER_SHADER_SRC ${CMAKE_BINARY_DIR}/visualizer_shaders.cpp)

compile_shaders(${FASTCD_SHADER_SRC}
src/fastcd/shaders/project_mesh.vert
src/fastcd/shaders/project_mesh.frag
)

compile_shaders(${VISUALIZER_SHADER_SRC}
src/visualizer/shaders/colored_vertices.vert
src/visualizer/shaders/colored_vertices.frag
src/visualizer/shaders/draw_mesh.vert
src/visualizer/shaders/draw_mesh.frag
src/visualizer/shaders/render_points.vert
src/visualizer/shaders/render_points.frag
)

add_library(fastcd
src/fastcd/camera.cpp
src/fastcd/image.cpp
src/fastcd/processed_image.cpp
src/fastcd/image_sequence.cpp
src/fastcd/point_covariance2d.cpp
src/fastcd/point_covariance3d.cpp
src/fastcd/depth_projector.cpp
src/fastcd/mesh.cpp
src/fastcd/regions3d_projector.cpp
src/fastcd/change_detector.cpp
src/fastcd/regions_matcher_hist.cpp

${FASTCD_SHADER_SRC}
)

target_link_libraries(fastcd  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(fastcd_example src/example.cpp
                              src/utils/obj_reader.cpp
                              src/visualizer/visualizer.cpp
                              ${VISUALIZER_SHADER_SRC}
                              )

target_link_libraries(fastcd_example fastcd ${OPENGL_LIBRARIES} ${catkin_LIBRARIES} ${GLEW_LIBRARIES} Qt5::Widgets Qt5::OpenGL)

# Generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile @ONLY)
  add_custom_target(doc ALL
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
  )
endif(DOXYGEN_FOUND)