cmake_minimum_required(VERSION 2.8.3)

project(manifoldReconstructor)  
   
#----------------------- CMAKE MODULES ------------------------ 

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)  
  
# ----------------------- LIBRARIES ---------------------------
find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)

# ----------------------- INCLUDES ----------------------------
include_directories(  ${EIGEN3_INCLUDE_DIR}
                      ${GMPXX_INCLUDE_DIR}
                      ${OpenCV_INCLUDE_DIRS} 
                      ${CGAL_INCLUDE_DIRS}
                      external/glm/glm/
                      external/
                      include/manifoldReconstructor/
                      include/test/
                      ) 
# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# ------------------------ MESSAGES ----------------------------

# ------------------------ BUILD -------------------------------

file(GLOB_RECURSE meshRec_SOURCE 
                  src/*.cpp
                  src/cam_parsers/*.cpp
                  src/utilities/*.cpp
                  test/*.cpp
                  )

add_executable(manifoldReconstructor ${meshRec_SOURCE})
target_link_libraries(manifoldReconstructor ${OpenCV_LIBS} 
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARIES}  )


# ------------------------ INSTALLATION ------------------------


# --------------------------------------------------------------


