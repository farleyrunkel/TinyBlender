cmake_minimum_required(VERSION 3.5)

project(TinyBlender VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets LinguistTools OpenGLWidgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets LinguistTools OpenGLWidgets)

add_subdirectory(source/3rdparty/SARibbon/)
include_directories(source/3rdparty/SARibbon/src/SARibbonBar)

add_subdirectory(source/core)
include_directories(${MODULE_TBCORE_INCLUDE})
message(STATUS "MODULE_TBCORE_INCLUDE: " ${MODULE_TBCORE_INCLUDE})

add_subdirectory(source/interface)
include_directories(${MODULE_TBINTERFACE_INCLUDE})
message(STATUS "MODULE_TBINTERFACE_INCLUDE: " ${MODULE_TBINTERFACE_INCLUDE})

add_subdirectory(source/primitives)
include_directories(${MODULE_TBPRIMITIVES_INCLUDE})
message(STATUS "MODULE_TBPRIMITIVES_INCLUDE: " ${MODULE_TBPRIMITIVES_INCLUDE})

# Set the main project source
set(PROJECT_SOURCES source/main.cpp)

set(PROJECT_RESOURCES
    resources/i18n/TinyBlender_en_US.ts
    resources/tinyblender.qrc
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(TinyBlender
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        ${PROJECT_RESOURCES}
    )

# Define target properties for Android with Qt 6 as:
#    set_property(TARGET TinyBlender APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    if(ANDROID)
        add_library(TinyBlender SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(TinyBlender
            ${PROJECT_SOURCES}
        )
    endif()

    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

target_link_libraries(TinyBlender PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(TinyBlender PRIVATE Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
target_link_libraries(TinyBlender PRIVATE SARibbonBar)

target_link_libraries(TinyBlender PRIVATE tbcore)
target_link_libraries(TinyBlender PRIVATE tbprimitives)

target_compile_options(TinyBlender PRIVATE /Zc:__cplusplus)


# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.TinyBlender)
endif()
set_target_properties(TinyBlender PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS TinyBlender
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(TinyBlender)
endif()
