cmake_minimum_required(VERSION 3.5)
project(space_rover)

# set MuJoCo CMake Modules
set(CMAKE_MODULE_PATH cmake)
set(CMAKE_BUILD_TYPE Debug)

# set C/C++ Standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic) # when you can't find build error, try this
  # add_compile_options(-Wall -Wpedantic)
  add_compile_options(-Wpedantic)
endif()

################################################################################
# Find shared library of MuJoCo
################################################################################
if(NOT TARGET ${PROJECT_NAME})
  add_library(${PROJECT_NAME} SHARED IMPORTED)
  set_target_properties(${PROJECT_NAME}
    PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libmujoco.so)
endif()


################################################################################
# Find Python3 casadi
################################################################################
find_package(Python3 COMPONENTS Development NumPy REQUIRED)
find_package(PythonLibs REQUIRED)

# sorce 파일들을 다른 패키지로 빼면 여기서 수정이 필요하다
file(GLOB_RECURSE SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/controller/*cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utilities/*.cpp

)

add_executable(simulation ${SOURCES})
target_link_libraries(simulation mujoco glfw pthread rt casadi ipopt ${CMAKE_DL_LIBS} Python3::Python Python3::NumPy ${Python3_LIBRARY})

target_include_directories(simulation
PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mujoco
  ${CMAKE_CURRENT_SOURCE_DIR}/include/math
  ${CMAKE_CURRENT_SOURCE_DIR}/include/robot
  ${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory
  ${CMAKE_CURRENT_SOURCE_DIR}/include/controller
  ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities


  /home/ycm/YCM/third-party/casadi/build/lib
  /home/ycm/YCM/third-party/casadi/build
  /home/ycm/YCM/third-party/casadi
  /home/ycm/YCM/third-party/Ipopt/src/Common
  /home/ycm/YCM/third-party/Ipopt/src/Interfaces
  /home/ycm/YCM/third-party/Ipopt/src/LinAlg

  $<INSTALL_INTERFACE:include>

)
