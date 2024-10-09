project(Log)

message(STATUS "Sub Project: " ${PROJECT_NAME})

# Recursively find all source and header files
file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

file(GLOB ${PROJECT_NAME}_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

# Create the library from source files
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
add_library(TB::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Include header files in the library (for IDEs and clarity)
target_sources(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_HEADERS})

# Specify the include directory for this target
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
