

# Set the main project source
set(PROJECT_SOURCES TinyBlender.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(common)
add_subdirectory(core)
add_subdirectory(objects)
add_subdirectory(primitives)
add_subdirectory(Log)

qt_add_executable(${PROJECT_NAME}
    MANUAL_FINALIZATION
    ${PROJECT_SOURCES}
    ${PROJECT_RESOURCES}
)
qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
target_link_libraries(${PROJECT_NAME} PRIVATE SARibbonBar)
target_link_libraries(${PROJECT_NAME} PRIVATE ACG)

target_link_libraries(${PROJECT_NAME} PRIVATE TB::Core)
target_link_libraries(${PROJECT_NAME} PRIVATE TB::Common)
target_link_libraries(${PROJECT_NAME} PRIVATE TB::Primitives)
target_link_libraries(${PROJECT_NAME} PRIVATE TB::Log)

target_link_libraries(${PROJECT_NAME} PRIVATE TB::ObjectOpenMesh)

target_precompile_headers(${PROJECT_NAME} PRIVATE pch.h)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.${PROJECT_NAME})
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(${PROJECT_NAME})
endif()
