cmake_minimum_required(VERSION 3.0.0)
project(BentPipeMesh)

# c++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
#add_compile_options(-Wall -Wextra -pedantic -Werror)
add_compile_options(-DOUTPUTEXP)

# header and src files
include_directories(include)
file(GLOB CADSOURCE "CAD3D/*.cpp")

ADD_CUSTOM_TARGET(debug
  COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Debug ${CMAKE_SOURCE_DIR}
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target all
  COMMENT "Switch CMAKE_BUILD_TYPE to Debug"
  )

ADD_CUSTOM_TARGET(release
  COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release ${CMAKE_SOURCE_DIR}
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target all
  COMMENT "Switch CMAKE_BUILD_TYPE to Release"
  )

# build shared library
add_library(LIBPROCESSING SHARED ${CADSOURCE})

# process executable
add_executable(mesh mesh.cpp PipeMesh.cpp)
target_link_libraries(mesh LIBPROCESSING)

include(CTest)
if(BUILD_TESTING)
  add_executable(check_bent_pipe checks/bent_pipe.cpp)
  target_include_directories(check_bent_pipe PRIVATE ${CMAKE_SOURCE_DIR})
  add_test(NAME bent_pipe COMMAND check_bent_pipe)
  add_executable(check_cad3d_compatibility checks/cad3d_compatibility.cpp)
  target_include_directories(check_cad3d_compatibility PRIVATE ${CMAKE_SOURCE_DIR})
  target_link_libraries(check_cad3d_compatibility LIBPROCESSING)
  add_test(NAME cad3d_compatibility COMMAND check_cad3d_compatibility
    ${CMAKE_SOURCE_DIR}/InputMesh/inFoil.xml ${CMAKE_BINARY_DIR}/cad3d_roundtrip.xml)
endif()
