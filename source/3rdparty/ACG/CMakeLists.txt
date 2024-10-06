include (VCICommon)

project (ACG
         VERSION 3.1.0
         LANGUAGES CXX )

# Disable Library installation when not building ACG on its own but as part of another project!
if ( ${CMAKE_PROJECT_NAME} MATCHES "TinyBlender")
  set(VCI_NO_LIBRARY_INSTALL true)
else()
  vci_qt ()
endif()

if (NOT QT_FOUND)
  message(FATAL ERROR "Qt not found")
endif ()

vci_openmp()

# check for OpenGL, GLEW as our required dependencies
find_package (OpenGL)
if (NOT OPENGL_FOUND)
  message (FATAL_ERROR "OpengGL not found!")
endif ()

find_package (OpenMesh)
if (NOT OPENMESH_FOUND)
    message (FATAL_ERROR "OpenMesh not found!")
endif ()

set(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
                 ${CMAKE_CURRENT_SOURCE_DIR}/ShaderUtils
                 ${CMAKE_CURRENT_BINARY_DIR}
                 ${OPENGL_INCLUDE_DIR} )

set(ADDITIONAL_LINK_LIBRARIES "" )

#===================================================================
# ACG Library files
#===================================================================
set ( headers
    Geometry/Algorithms.hh
    Geometry/AlgorithmsAngleT.hh
    Geometry/AlgorithmsAngleT_impl.hh
    Geometry/GPUCacheOptimizer.hh
    Geometry/Spherical.hh
    Geometry/Triangulator.hh
    Config/ACGDefines.hh
    Geometry/Types/PlaneT.hh
    Geometry/Types/PlaneType.hh
    Geometry/Types/QuadricT.hh
    Geometry/bsp/BSPImplT.hh
    Geometry/bsp/BSPImplT_impl.hh
    Geometry/bsp/BSPTreeNode.hh
    Geometry/bsp/TriangleBSPCoreT.hh
    Geometry/bsp/TriangleBSPCoreT_impl.hh
    Geometry/bsp/TriangleBSPT.hh
    GL/AntiAliasing.hh
    GL/ColorStack.hh
    GL/ColorTranslator.hh
    GL/DrawMesh.hh
    GL/DrawMeshT_impl.hh
    GL/FBO.hh
    GL/FilterKernels.hh
    GL/GLError.hh
    GL/GLFormatInfo.hh
    GL/GLPrimitives.hh
    GL/GLState.hh
    GL/GLTrackball.hh
    GL/IRenderer.hh
    GL/MeshCompiler.hh
    GL/PBuffer.hh
    GL/RenderObject.hh
    GL/ScreenQuad.hh
    GL/ShaderCache.hh
    GL/ShaderGenerator.hh
    GL/VertexDeclaration.hh
    GL/acg_glew.hh
    GL/gl.hh
    GL/globjects.hh
    GL/removedEnums.hh
    GL/stipple_alpha.hh
    Math/BSplineBasis.hh
    Math/BezierCurveT.hh
    Math/BezierCurveT_impl.hh
    Math/DualQuaternionT.hh
    Math/DualQuaternionT_impl.hh
    Math/GLMatrixT.hh
    Math/GLMatrixT_impl.hh
    Math/Matrix3x3T.hh
    Math/Matrix4x4T.hh
    Math/Matrix4x4T_impl.hh
    Math/QuaternionT.hh
    Math/VectorT.hh
    QtWidgets/QtApplication.hh
    QtWidgets/QtClippingDialog.hh
    QtWidgets/QtColorChooserButton.hh
    QtWidgets/QtColorTranslator.hh
    QtWidgets/QtCoordFrameDialog.hh
    QtWidgets/QtFileDialog.hh
    QtWidgets/QtFlapBox.hh
    QtWidgets/QtHistogramWidget.hh
    QtWidgets/QtLasso.hh
    QtWidgets/QtLessAnnoyingDoubleSpinBox.hh
    QtWidgets/QtMaterialDialog.hh
    QtWidgets/QtPrincipalAxisDialog.hh
    QtWidgets/QtSceneGraphWidget.hh
    QtWidgets/QtShaderDialog.hh
    QtWidgets/QtTextureDialog.hh
    QtWidgets/QtWheel.hh
    Scenegraph/ArrowNode.hh
    Scenegraph/BaseNode.hh
    Scenegraph/BoundingBoxNode.hh
    Scenegraph/CartesianClippingNode.hh
    Scenegraph/ClippingNode.hh
    Scenegraph/CoordFrameNode.hh
    Scenegraph/CoordsysNode.hh
    Scenegraph/DrawModes.hh
    Scenegraph/EnvMapNode.hh
    Scenegraph/FloatingSubtreeNode.hh
    Scenegraph/GlutPrimitiveNode.hh
    Scenegraph/GridNode.hh
    Scenegraph/LightSourceNode.hh
    Scenegraph/LineNode.hh
    Scenegraph/ManipulatorNode.hh
    Scenegraph/MaterialNode.hh
    Scenegraph/MeshNode2T.hh
    Scenegraph/MeshNode2T_impl.hh
    Scenegraph/MeshNodeDeprecatedT.hh
    Scenegraph/MeshNodeDeprecatedT_impl.hh
    Scenegraph/OBJNode.hh
    Scenegraph/OSDTransformNode.hh
    Scenegraph/PickTarget.hh
    Scenegraph/PointNode.hh
    Scenegraph/PrincipalAxisNode.hh
    Scenegraph/PrincipalAxisNodeT_impl.hh
    Scenegraph/QuadNode.hh
    Scenegraph/ResourceManagerNode.hh
    Scenegraph/SceneGraph.hh
    Scenegraph/SceneGraphAnalysis.hh
    Scenegraph/SeparatorNode.hh
    Scenegraph/ShaderNode.hh
    Scenegraph/SliceNode.hh
    Scenegraph/StatusNodesT.hh
    Scenegraph/StatusNodesT_impl.hh
    Scenegraph/StencilRefNode.hh
    Scenegraph/StripProcessorT.hh
    Scenegraph/StripProcessorT_impl.hh
    Scenegraph/SwitchNode.hh
    Scenegraph/TextNode.hh
    Scenegraph/Texture3DNode.hh
    Scenegraph/TextureNode.hh
    Scenegraph/TrackballNode.hh
    Scenegraph/TransformNode.hh
    Scenegraph/TranslationManipulatorNode.hh
    Scenegraph/TriStripNodeDeprecatedT.hh
    Scenegraph/TriangleNode.hh
    QtScenegraph/QtManipulatorNode.hh
    QtScenegraph/QtTranslationManipulatorNode.hh
    ShaderUtils/GLSLShader.hh
    ShaderUtils/UniformPool.hh
    Utils/ColorCoder.hh
    Utils/ColorConversion.hh
    Utils/ColorGenerator.hh
    Utils/HaltonColors.hh
    Utils/HeapT.hh
    Utils/Histogram.hh
    Utils/HuePartitioningColors.hh
    Utils/IColorCoder.hh
    Utils/ImageConversion.hh
    Utils/LinearTwoColorCoder.hh
    Utils/Noncopyable.hh
    Utils/NumLimitsT.hh
    Utils/Profile.hh
    Utils/Progress.hh
    Utils/SmartPointer.hh
    Utils/StopWatch.hh
    Utils/Tracing.hh
    Utils/VSToolsT.hh
    )

set (sources
    Geometry/Algorithms.cc
    Geometry/GPUCacheOptimizer.cc
    Geometry/Triangulator.cc
    Geometry/Types/PlaneType.cc
    GL/AntiAliasing.cc
    GL/ColorStack.cc
    GL/ColorTranslator.cc
    GL/DrawMesh.cc
    GL/DrawMeshT_impl.cc
    GL/FBO.cc
    GL/FilterKernels.cc
    GL/GLError.cc
    GL/GLFormatInfo.cc
    GL/GLPrimitives.cc
    GL/GLState.cc
    GL/GLTrackball.cc
    GL/IRenderer.cc
    GL/MeshCompiler.cc
    GL/PBuffer.cc
    GL/RenderObject.cc
    GL/ScreenQuad.cc
    GL/ShaderCache.cc
    GL/ShaderGenerator.cc
    GL/VertexDeclaration.cc
    GL/gl.cc
    GL/globjects.cc
    GL/stipple_alpha.cc
    Math/BSplineBasis.cc
    QtWidgets/QtApplication.cc
    QtWidgets/QtClippingDialog.cc
    QtWidgets/QtColorChooserButton.cc
    QtWidgets/QtColorTranslator.cc
    QtWidgets/QtCoordFrameDialog.cc
    QtWidgets/QtFileDialog.cc
    QtWidgets/QtFlapBox.cc
    QtWidgets/QtHistogramWidget.cc
    QtWidgets/QtLasso.cc
    QtWidgets/QtLessAnnoyingDoubleSpinBox.cc
    QtWidgets/QtMaterialDialog.cc
    QtWidgets/QtPrincipalAxisDialog.cc
    QtWidgets/QtSceneGraphWidget.cc
    QtWidgets/QtShaderDialog.cc
    QtWidgets/QtTextureDialog.cc
    QtWidgets/QtWheel.cc
    Scenegraph/ArrowNode.cc
    Scenegraph/BaseNode.cc
    Scenegraph/BoundingBoxNode.cc
    Scenegraph/CartesianClippingNode.cc
    Scenegraph/ClippingNode.cc
    Scenegraph/CoordFrameNode.cc
    Scenegraph/CoordsysNode.cc
    Scenegraph/DrawModes.cc
    Scenegraph/EnvMapNode.cc
    Scenegraph/FloatingSubtreeNode.cc
    Scenegraph/GlutPrimitiveNode.cc
    Scenegraph/GridNode.cc
    Scenegraph/LightSourceNode.cc
    Scenegraph/LineNode.cc
    Scenegraph/LineNodeGLCompat.cc
    Scenegraph/ManipulatorNode.cc
    Scenegraph/MaterialNode.cc
    Scenegraph/MeshNode2.cc
    Scenegraph/MeshNode2T_impl.cc
    Scenegraph/OBJNode.cc
    Scenegraph/OSDTransformNode.cc
    Scenegraph/PointNode.cc
    Scenegraph/PrincipalAxisNode.cc
    Scenegraph/QuadNode.cc
    Scenegraph/SceneGraph.cc
    Scenegraph/SceneGraphAnalysis.cc
    Scenegraph/SeparatorNode.cc
    Scenegraph/ShaderNode.cc
    Scenegraph/SliceNode.cc
    Scenegraph/StatusNodesBase.cc
    Scenegraph/StencilRefNode.cc
    Scenegraph/StripProcessor.cc
    Scenegraph/SwitchNode.cc
    Scenegraph/TextNode.cc
    Scenegraph/TextNodeGLCompat.cc
    Scenegraph/Texture3DNode.cc
    Scenegraph/TextureNode.cc
    Scenegraph/TrackballNode.cc
    Scenegraph/TransformNode.cc
    Scenegraph/TranslationManipulatorNode.cc
    Scenegraph/TriangleNode.cc
    QtScenegraph/QtManipulatorNode.cc
    QtScenegraph/QtTranslationManipulatorNode.cc
    ShaderUtils/GLSLShader.cc
    ShaderUtils/UniformPool.cc
    Utils/ColorCoder.cc
    Utils/HaltonColors.cc
    Utils/Histogram.cc
    Utils/HuePartitioningColors.cc
    Utils/ImageConversion.cc
    Utils/LinearTwoColorCoder.cc
    Utils/Profile.cc
    Utils/Tracing.cc
)

set (ui-files
    QtWidgets/QtClippingDialogUi.ui
    QtWidgets/QtCoordFrameDialogUi.ui
    QtWidgets/QtMaterialDialogUi.ui
    QtWidgets/QtPrincipalAxisDialogBaseUi.ui
    QtWidgets/QtShaderDialogUi.ui
    QtWidgets/QtTextureDialogUi.ui
    )

set (qrc
     ./ACG-resources.qrc )

# use Qwt6 if possible
find_package(QWT6)

if (QWT6_FOUND)
     set(INCLUDE_DIRS ${INCLUDE_DIRS} ${QWT6_INCLUDE_DIRS} )
     set(ADDITIONAL_LINK_LIBRARIES ${ADDITIONAL_LINK_LIBRARIES} ${QWT6_LIBRARIES})
     list(APPEND headers "QwtWidgets/QwtHistogramm.hh")
     list(APPEND sources "QwtWidgets/QwtHistogramm.cc")
endif()

if (OPENMP_FOUND)
    set(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenMP_CXX_INCLUDE_DIR} )
    set(ADDITIONAL_LINK_LIBRARIES ${ADDITIONAL_LINK_LIBRARIES} ${OpenMP_libomp_LIBRARY})
endif()

include_directories (
  ..
  ${INCLUDE_DIRS}
  ../glew-cmake/include
)

# generate dllexport macros on windows
if (WIN32)
  add_definitions(-DACGDLL )
  remove_definitions(-DUSEACG)
endif ()

source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${sources} ${headers} ${ui-files} ${qrc})

if (QT_FOUND)
  vci_add_library (ACG SHARED ${sources} ${headers} ${qrc})
  target_include_directories(ACG PUBLIC
                                    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
                                    $<INSTALL_INTERFACE:include>)

endif()

if ( NOT WIN32 )
  set_target_properties (ACG PROPERTIES VERSION ${ACG_VERSION_MAJOR}.${ACG_VERSION_MINOR}
                                      SOVERSION ${ACG_VERSION_MAJOR}.${ACG_VERSION_MINOR})
endif()

if (NOT VCI_PROJECT_MACOS_BUNDLE OR NOT APPLE)
  install (TARGETS ACG DESTINATION ${VCI_PROJECT_LIBDIR})
endif ()

# display results
vci_print_configure_header (ACG "ACG")

# Allow targets depending on ACG find the autogenerated header files
# cf OpenFlipper/cmake/type.cmake


# cf https://www.graphics.rwth-aachen.de:9000/OpenFlipper-Free/OpenFlipper-Free/-/issues/176
# CMake's AUTOUIC automatically adds the proper include path to the target, however
# we need a PUBLIC include directory - so we do this manually for now:

get_property(_isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if ( _isMultiConfig )
  target_include_directories( ACG PUBLIC
                              ${CMAKE_CURRENT_BINARY_DIR}/ACG_autogen/include_$<CONFIG>/)
else()
  target_include_directories( ACG PUBLIC
                               ${CMAKE_CURRENT_BINARY_DIR}/ACG_autogen/include )
endif()

target_link_libraries ( ACG  ${OPENMESH_LIBRARIES}
                             ${OPENGL_LIBRARIES}
                             ${ADDITIONAL_LINK_LIBRARIES}
                             ${QT_TARGET}::Core ${QT_TARGET}::Widgets ${QT_TARGET}::OpenGL ${QT_TARGET}::Network
                             ${GLEW_TARGET} )

if (QT6_FOUND)
  target_link_libraries( ACG ${QT_TARGET}::OpenGLWidgets )
endif()


#===================================================================
# Setting up the unit tests
#===================================================================

include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG f8d7d77c06936315286eb55f8de22cd23c188571 # v1.14.0
)
if(WIN32)
    # avoid linking errors, cf https://stackoverflow.com/questions/12540970/how-to-make-gtest-build-mdd-instead-of-mtd-by-default-using-cmake
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif()
FetchContent_MakeAvailable(googletest)

if ( NOT DEFINED ACG_BUILD_UNIT_TESTS )
  set( ACG_BUILD_UNIT_TESTS false CACHE BOOL "Enable or disable unit tests on ACG library." )
endif()

if ( ACG_BUILD_UNIT_TESTS )

  enable_testing()

  set ( TEST_SOURCES
        tests/Algorithm/DBSCAN_test.cc
        tests/Geometry/Algorithms_ClosestPoints_tests.cc
        tests/Geometry/Algorithms_test.cc
        tests/Geometry/BSP/BSP_test.cc
        tests/Geometry/Spherical_test.cc
        tests/Math/Matrix3x3.cc
        tests/Math/math_test.cc
        tests/MeshCompiler/MeshCompiler_test.cc
        tests/MeshCompiler/MeshCompiler_testData0.cc
        tests/MeshCompiler/MeshCompiler_testData1.cc
        tests/Utils/Histogram_test.cc
        tests/main.cc 
      )


  set(TESTED_SOURCES
        Algorithm/DBSCANT_impl.hh
      )

  include_directories(${GTEST_INCLUDE_DIRS} )
  link_directories ( ${GTEST_LIBRARY_DIR})

  if ( CMAKE_GENERATOR MATCHES "^Visual Studio 11.*" )
      add_definitions( /D _VARIADIC_MAX=10 )
  endif()

  add_executable (ACG_tests ${TEST_SOURCES})

  set(OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
  # run tests in build directory of OF
  if ( ${CMAKE_PROJECT_NAME} MATCHES "OpenFlipper" AND WIN32)
    set(OUT_DIR  ${CMAKE_BINARY_DIR}/Build)
  endif()

  set_target_properties(ACG_tests PROPERTIES
    COMPILE_FLAGS "-UNDEBUG"
    RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR}
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${OUT_DIR}
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${OUT_DIR}
  )

  target_link_libraries(ACG_tests
    GTest::gtest GTest::gtest_main ${OPENMESH_LIBRARIES} ACG ${GLEW_TARGET}
  )

  add_test(NAME AllTestsIn_ACG_tests COMMAND ${OUT_DIR}/ACG_tests)

endif()
