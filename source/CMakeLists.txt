

# Set the main project source
set(PROJECT_SOURCES TinyBlender.cpp)

add_subdirectory(common)
add_subdirectory(core)
add_subdirectory(objects)
add_subdirectory(primitives)

# Append the OpenMesh sources to the main project sources list
list(APPEND PROJECT_SOURCES ${TBCOMMON_SOURCES})

qt_add_executable(TinyBlender
    MANUAL_FINALIZATION
    ${PROJECT_SOURCES}
    ${PROJECT_RESOURCES}
)
qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})

target_link_libraries(TinyBlender PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
target_link_libraries(TinyBlender PRIVATE SARibbonBar)

target_link_libraries(TinyBlender PRIVATE core)
target_link_libraries(TinyBlender PRIVATE common)
target_link_libraries(TinyBlender PRIVATE primitives)

target_link_libraries(TinyBlender PRIVATE OpenMeshObject)

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
