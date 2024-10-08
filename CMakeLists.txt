cmake_minimum_required(VERSION 3.5)

project(TinyBlender VERSION 0.1 LANGUAGES CXX)

cmake_policy(SET CMP0071 NEW)
cmake_policy(SET CMP0100 NEW)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(Boost_DIR E:/Libraries/boost_1_85_0/out/lib/cmake/Boost-1.85.0)
set(Boost_NO_SYSTEM_PATHS ON)

# add our macro directory to cmake search path
# add our macro directory to cmake search path
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_SOURCE_DIR}/cmake/cmake-library/finders ${CMAKE_SOURCE_DIR}/cmake/cmake-library/VCI )

set(PROJECT_RESOURCES
    ${CMAKE_SOURCE_DIR}/resources/i18n/TinyBlender_en_US.ts
    ${CMAKE_SOURCE_DIR}/resources/tinyblender.qrc
)

add_compile_definitions(_USE_MATH_DEFINES)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Widgets LinguistTools OpenGLWidgets Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets LinguistTools OpenGLWidgets Network)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
message(STATUS "Boost_INCLUDE_DIRS: " ${Boost_INCLUDE_DIRS})

add_subdirectory(source/3rdparty/SARibbon)
include_directories(source/3rdparty/SARibbon/src/SARibbonBar)

add_subdirectory(required/glew-cmake)
set(GLEW_TARGET libglew_shared)

add_subdirectory(required/OpenMesh)
include_directories(${OPENMESH_INCLUDE_DIR})
add_subdirectory(required/ACG)

include_directories(source)

add_subdirectory(source)
