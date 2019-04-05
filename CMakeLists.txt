cmake_minimum_required(VERSION 3.6)
project(simple-raytracer CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
#set(CMAKE_CXX_FLAGS "-O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")

#set(CMAKE_PREFIX_PATH "/mnt/DeepDrive/Projects/libtorch-10.0;/mnt/DeepDrive/Projects/yaml-cpp-yaml-cpp-0.6.2/install")
set(CMAKE_PREFIX_PATH "/mnt/DeepDrive/Projects/yaml-cpp-yaml-cpp-0.6.2/install;/mnt/DeepDrive/Projects/pytorch/torch")

find_package(GLFW 3.0.0)
if (GLFW_FOUND)
    add_definitions( -DGLFW_FOUND )
    message("- GLFW Located")
    include_directories(${GLFW_INCLUDE_DIR})
else()
    message("- GLFW not located\n->Windowed Mode Disabled")
endif()
if(GLFW_FOUND AND (GLFW_VERSION VERSION_EQUAL 3.0 OR GLFW_VERSION VERSION_GREATER 3.0))
    add_definitions( -DGLFW_VERSION_3 )
endif()

find_package(OpenGL)
if (OPENGL_FOUND)
    add_definitions( -DOPENGL_FOUND )
    message("- OpenGL Located")
    include_directories(${OPENGL_INCLUDE_DIR})
else()
    message("- OpenGL not located\n->Windowed Mode Disabled")
endif()

find_package(PNG)
if (PNG_FOUND)
    add_definitions( -DPNG_FOUND )
    message("- LibPNG Located")
    include_directories(${PNG_INCLUDE_DIRS})
    add_definitions(${PNG_DEFINITIONS})
else()
    message("- LibPNG not located\n->Image Mode Disabled")
endif()

set(YAMLCPP_USE_STATIC_LIBS TRUE)
find_package(YamlCpp REQUIRED)
message("YamlCpp found\n->Include dir: ${YAMLCPP_INCLUDE_DIR}\n->Library: ${YAMLCPP_LIBRARY}")

find_package(Boost COMPONENTS system filesystem REQUIRED)

find_package(X11 REQUIRED)
if (X11_FOUND)
    message("- X11 Located")
endif()

find_package(Torch REQUIRED)
message("Torch found\n->Include dir: ${TORCH_INCLUDE_DIRS}\n->Library: ${TORCH_LIBRARIES}")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


#link_directories(${YAMLCPP_INCLUDE_DIR})

set(PROJECT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/Source)

file(GLOB SOURCE_FILES
        "${PROJECT_SOURCE_DIR}/*.h"
        "${PROJECT_SOURCE_DIR}/*.cpp"

        "${PROJECT_SOURCE_DIR}/Includes/glad/*.h"
        "${PROJECT_SOURCE_DIR}/Includes/glad/*.c"

        "${PROJECT_SOURCE_DIR}/Geometry/*.h"
        "${PROJECT_SOURCE_DIR}/Geometry/*.cpp"

        "${PROJECT_SOURCE_DIR}/Lighting/*.h"
        "${PROJECT_SOURCE_DIR}/Lighting/*.cpp"

        "${PROJECT_SOURCE_DIR}/Util/*.h"
        "${PROJECT_SOURCE_DIR}/Util/*.cpp"
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES} Source/Geometry/Mesh.cpp Source/Geometry/Mesh.h)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${PROJECT_SOURCE_DIR}/Includes
        ${PROJECT_SOURCE_DIR}
        )

if (TARGET Eigen3::Eigen)
    target_link_libraries (${PROJECT_NAME} PUBLIC Eigen3::Eigen)
    message("Eigen dir found at " ${EIGEN3_INCLUDE_DIR})
else()
    message(WARNING, "No Eigen found")
endif (TARGET Eigen3::Eigen)

target_include_directories(${PROJECT_NAME} PUBLIC ${TORCH_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${TORCH_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${YAMLCPP_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC
#        ${YAMLCPP_LIBRARY}
        ${PNG_LIBRARIES}
        ${GLFW_LIBRARIES}
        )

target_include_directories(${PROJECT_NAME} PUBLIC ${BOOST_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        )

target_include_directories(${PROJECT_NAME} PUBLIC "/mnt/DeepDrive/Projects/libigl/include")
target_link_libraries(${PROJECT_NAME} PUBLIC
        "/mnt/DeepDrive/Projects/libigl/cmake-build-release/libglad.a"
        "/mnt/DeepDrive/Projects/libigl/cmake-build-release/libglfw3.a"
        "/mnt/DeepDrive/Projects/yaml-cpp-yaml-cpp-0.6.2/install/lib/libyaml-cpp.so"
        -lX11
        )
target_link_libraries(${PROJECT_NAME} PUBLIC ${CMAKE_DL_LIBS})

