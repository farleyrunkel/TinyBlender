/*===========================================================================*\
 *                                                                           *
 *                              OpenFlipper                                  *
 *           Copyright (c) 2001-2015, RWTH-Aachen University                 *
 *           Department of Computer Graphics and Multimedia                  *
 *                          All rights reserved.                             *
 *                            www.openflipper.org                            *
 *                                                                           *
 *---------------------------------------------------------------------------*
 * This file is part of OpenFlipper.                                         *
 *---------------------------------------------------------------------------*
 *                                                                           *
 * Redistribution and use in source and binary forms, with or without        *
 * modification, are permitted provided that the following conditions        *
 * are met:                                                                  *
 *                                                                           *
 * 1. Redistributions of source code must retain the above copyright notice, *
 *    this list of conditions and the following disclaimer.                  *
 *                                                                           *
 * 2. Redistributions in binary form must reproduce the above copyright      *
 *    notice, this list of conditions and the following disclaimer in the    *
 *    documentation and/or other materials provided with the distribution.   *
 *                                                                           *
 * 3. Neither the name of the copyright holder nor the names of its          *
 *    contributors may be used to endorse or promote products derived from   *
 *    this software without specific prior written permission.               *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED *
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A           *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER *
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,       *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR        *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING      *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              *
 *                                                                           *
\*===========================================================================*/

#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <QFile>
#include <QTextStream>

#include <ACG/GL/gl.hh>

#include <ACG/GL/IRenderer.hh>

#include <ACG/GL/VertexDeclaration.hh>

#include <ACG/GL/ShaderCache.hh>



namespace ACG
{

void RenderObject::initFromState( GLState* _glState )
{
  culling = true;
  blending = false;
  depthTest = true;
  depthWrite = true;
  alphaTest = false;

  programPointSize = false;

  colorWriteMask[0] = colorWriteMask[1] = colorWriteMask[2] = colorWriteMask[3] = 1;

  fillMode = GL_FILL;

  depthRange = Vec2f(0.0f, 1.0f);
  depthFunc = GL_LESS;

  blendSrc = GL_ONE;
  blendDest = GL_ZERO;

  alpha = 1.0f;

  pointSize = 0.1f;

  if (_glState)
  {
    modelview = _glState->modelview();
    proj = _glState->projection();

    culling =_glState->isStateEnabled(GL_CULL_FACE);
    blending = _glState->isStateEnabled(GL_BLEND);
    depthTest = _glState->isStateEnabled(GL_DEPTH_TEST);

    _glState->getBlendFunc(&blendSrc, &blendDest);

    glGetFloatv(GL_DEPTH_RANGE, depthRange.data());

    depthFunc = _glState->depthFunc();


    alphaTest = _glState->isStateEnabled(GL_ALPHA_TEST);

    for (int i = 0; i < 3; ++i)
    {
      diffuse[i] = _glState->diffuse_color()[i];
      ambient[i] = _glState->ambient_color()[i];
      specular[i] = _glState->specular_color()[i];
      emissive[i] = _glState->base_color()[i];
    }
    shininess = _glState->shininess();


#ifdef GL_PROGRAM_POINT_SIZE
    programPointSize = _glState->isStateEnabled(GL_PROGRAM_POINT_SIZE);
#endif

    pointSize = _glState->point_size();
  }

  // get texcoord generation params
  if (_glState->isStateEnabled(GL_TEXTURE_GEN_Q))
    shaderDesc.texGenDim = 4;
  else if (_glState->isStateEnabled(GL_TEXTURE_GEN_R))
    shaderDesc.texGenDim = 3;
  else if (_glState->isStateEnabled(GL_TEXTURE_GEN_T))
    shaderDesc.texGenDim = 2;
  else if (_glState->isStateEnabled(GL_TEXTURE_GEN_S))
    shaderDesc.texGenDim = 1;
  if (shaderDesc.texGenDim)
  {
    GLint genMode;
    _glState->getTexGenMode(GL_S, GL_TEXTURE_GEN_MODE, &genMode);
    shaderDesc.texGenMode = genMode;
  }
}

void RenderObject::setupShaderGenFromDrawmode( const SceneGraph::DrawModes::DrawModeProperties* _props )
{
  if (_props)
  {
    shaderDesc.vertexColors = _props->colored();
    if (_props->textured())
      shaderDesc.addTextureType(GL_TEXTURE_2D,false,0);
    else
      shaderDesc.clearTextures();
    shaderDesc.numLights = _props->lighting() ? 0 : -1;

    switch (_props->lightStage()) {
      case SceneGraph::DrawModes::LIGHTSTAGE_SMOOTH:
        shaderDesc.shadeMode = SG_SHADE_GOURAUD;
        break;
      case SceneGraph::DrawModes::LIGHTSTAGE_PHONG:
        shaderDesc.shadeMode = SG_SHADE_PHONG;
        break;
      case SceneGraph::DrawModes::LIGHTSTAGE_UNLIT:
        shaderDesc.shadeMode = SG_SHADE_UNLIT;
        break;
      default:
        break;
    }

    if (_props->flatShaded())
      shaderDesc.shadeMode = SG_SHADE_FLAT;

    if (_props->normalSource() == SceneGraph::DrawModes::NORMAL_PER_FACE)
      shaderDesc.vertexNormalInterpolator = "flat";
    else
      shaderDesc.vertexNormalInterpolator.clear();

    if (_props->primitive() == SceneGraph::DrawModes::PRIMITIVE_WIREFRAME ||
        _props->primitive() == SceneGraph::DrawModes::PRIMITIVE_HIDDENLINE ||
        _props->primitive() == SceneGraph::DrawModes::PRIMITIVE_EDGE ||
        _props->primitive() == SceneGraph::DrawModes::PRIMITIVE_HALFEDGE)
      shaderDesc.shadeMode = SG_SHADE_UNLIT;
  }
}

void RenderObject::setMaterial( const SceneGraph::Material* _mat )
{
  for (int i = 0; i < 3; ++i)
  {
    diffuse[i] = _mat->diffuseColor()[i];
    ambient[i] = _mat->ambientColor()[i];
    specular[i] = _mat->specularColor()[i];
    emissive[i] = _mat->baseColor()[i];
  }
  shininess = _mat->shininess();
  alpha = _mat->diffuseColor()[3];

  // material node sets the alpha test function to GL_GREATER
  alphaFunc = GL_GREATER;
  alphaTest = _mat->alphaTest();
  alphaRef = _mat->alphaValue();
}


RenderObject::RenderObject()
  : priority(0),
  overlay(false),
  modelview(GLMatrixf(ACG::Vec3f(1.0, 0.0, 0.0), ACG::Vec3f(0.0, 1.0, 0.0), ACG::Vec3f(0.0, 0.0, 1.0))),
  proj(modelview),
  vertexArrayObject(0),
  vertexBuffer(0), indexBuffer(0), sysmemIndexBuffer(0),
  primitiveMode(GL_TRIANGLES), patchVertices(0), numIndices(0), indexOffset(0), indexType(GL_UNSIGNED_INT),
  numInstances(0),
  vertexDecl(0),
  culling(true), blending(false), alphaTest(false),
  depthTest(true), depthWrite(true),
  fillMode(GL_FILL), depthFunc(GL_LESS),
  alphaFunc(GL_ALWAYS), alphaRef(0.0f),
  blendSrc(GL_SRC_ALPHA), blendDest(GL_ONE_MINUS_SRC_ALPHA),
  depthRange(0.0f, 1.0f),

  clipDistanceMask(0),

  programPointSize(false),
  pointSize(0.1f),

  patchDefaultInnerLevel(1.0f, 1.0f),
  patchDefaultOuterLevel(1.0f, 1.0f, 1.0f, 1.0f),

  diffuse(0.6f, 0.6f, 0.6f), ambient(0.1f, 0.1f, 0.1f),
  specular(0.0f, 0.0f, 0.0f), emissive(0.05f, 0.05f, 0.05f),
  alpha(1.0f), shininess(100.0f),
  
  inZPrePass(true),
  depthMapUniformName(0),

  debugID(0),
  internalFlags_(0)
{
  colorWriteMask[0] = colorWriteMask[1] = colorWriteMask[2] = colorWriteMask[3] = 1;
}

RenderObject::~RenderObject() {
  uniformPool_.clear();
}

QString RenderObject::toString() const
{
  // several mappings: (int)GLEnum -> string

  const char* primitiveString[] = 
  {
    "GL_POINTS",
    "GL_LINES",
    "GL_LINE_LOOP",
    "GL_LINE_STRIP",
    "GL_TRIANGLES",
    "GL_TRIANGLE_STRIP",
    "GL_TRIANGLE_FAN",
    "GL_QUADS",
    "GL_QUAD_STRIP",
    "GL_POLYGON",
    "GL_LINES_ADJACENCY",
    "GL_LINE_STRIP_ADJACENCY",
    "GL_TRIANGLES_ADJACENCY",
    "GL_TRIANGLE_STRIP_ADJACENCY",
    "GL_PATCHES"
  };

  const char* fillModeString[] = 
  {
    "GL_POINT",
    "GL_LINE",
    "GL_FILL"
  };

  const char* depthFunString[] =
  {
    "GL_NEVER",
    "GL_LESS",
    "GL_EQUAL",
    "GL_LEQUAL",
    "GL_GREATER",
    "GL_NOTEQUAL",
    "GL_GEQUAL",
    "GL_ALWAYS"
  };

  QString result;
  QTextStream resultStrm(&result);


#if !defined(GL_VERSION_3_2)
  const GLenum maxSupportedPrimitiveMode = GL_POLYGON;
#elif !defined(GL_ARB_tessellation_shader)
  const GLenum maxSupportedPrimitiveMode = GL_TRIANGLE_STRIP_ADJACENCY;
#else
  const GLenum maxSupportedPrimitiveMode = GL_PATCHES;
#endif

  resultStrm << "name: " << QString::fromStdString(debugName)
             << "\ndebugID: " << debugID
             << "\npriority: " << priority
             << "\nprimitiveMode: " << (primitiveMode <= maxSupportedPrimitiveMode ? primitiveString[primitiveMode] : "undefined")
             << "\nfillMode: " << fillModeString[fillMode - GL_POINT]
             << "\nnumIndices: " << numIndices
             << "\nindexOffset: " << indexOffset;


#ifdef GL_ARB_tessellation_shader
  if (primitiveMode == GL_PATCHES)
    resultStrm << "\npatchVertices: " << patchVertices;
#endif

  resultStrm << "\nvao-id: " << vertexArrayObject
             << "\nvbo-id: " << vertexBuffer
             << "\nibo-id: " << indexBuffer
             << "\nsysmemIndexBuffer: " << sysmemIndexBuffer;



  resultStrm << "\n" << shaderDesc.toString();


  resultStrm << "\nmodelview: " 
    << "{" << modelview(0, 0) << ", " << modelview(0, 1) << ", " << modelview(0, 2) << ", " << modelview(0, 3) << "} "
    << "{" << modelview(1, 0) << ", " << modelview(1, 1) << ", " << modelview(1, 2) << ", " << modelview(1, 3) << "} "
    << "{" << modelview(2, 0) << ", " << modelview(2, 1) << ", " << modelview(2, 2) << ", " << modelview(2, 3) << "} "
    << "{" << modelview(3, 0) << ", " << modelview(3, 1) << ", " << modelview(3, 2) << ", " << modelview(3, 3) << "} ";

  resultStrm << "\nproj: " 
    << "{" << proj(0, 0) << ", " << proj(0, 1) << ", " << proj(0, 2) << ", " << proj(0, 3) << "} "
    << "{" << proj(1, 0) << ", " << proj(1, 1) << ", " << proj(1, 2) << ", " << proj(1, 3) << "} "
    << "{" << proj(2, 0) << ", " << proj(2, 1) << ", " << proj(2, 2) << ", " << proj(2, 3) << "} "
    << "{" << proj(3, 0) << ", " << proj(3, 1) << ", " << proj(3, 2) << ", " << proj(3, 3) << "} ";


  resultStrm << "\nculling: " << culling
    << "\nblending: " << blending
    << "\nalphaTest: " << alphaTest;


  resultStrm << "\ndepthTest: " << depthTest
    << "\ndepthWrite: " << depthWrite
    << "\ndepthFunc: " << depthFunString[depthFunc - GL_NEVER]
    << "\ndepthRange: [" << depthRange[0] << ", " << depthRange[1] << "]"
    << "\ncolorWriteMask: " << colorWriteMask[0] << ", " << colorWriteMask[1] << ", "<< colorWriteMask[2] << ", "<< colorWriteMask[2];

  resultStrm << "\nalphaFunc: " << depthFunString[alphaFunc - GL_NEVER]
    << "\nalphaRef: " << alphaRef;

  resultStrm << "\nclipDistanceMask: " << QString::number(clipDistanceMask, 2);
  resultStrm << "\nprogramPointSize: " << programPointSize;
  resultStrm << "\npointSize: " << pointSize;

  resultStrm << "\ndiffuse: [" << diffuse[0] << ", " << diffuse[1] << ", " << diffuse[2] << "]";
  resultStrm << "\nambient: [" << ambient[0] << ", " << ambient[1] << ", " << ambient[2] << "]";
  resultStrm << "\nspecular: [" << specular[0] << ", " << specular[1] << ", " << specular[2] << "]";
  resultStrm << "\nemissive: [" << emissive[0] << ", " << emissive[1] << ", " << emissive[2] << "]";

  resultStrm << "\nshininess: " << shininess;
  resultStrm << "\nalpha: " << alpha;


  resultStrm << "\ninZPrePass: " << inZPrePass;
  resultStrm << "\ndepthMapUniformName: " << depthMapUniformName;


  resultStrm << "\ninternalFlags: " << internalFlags_;

  // textures
  for (std::map<size_t, Texture>::const_iterator it = textures_.begin(); it != textures_.end(); ++it)
  {
    resultStrm << "\ntexture unit " << it->first << ": ";

    switch (it->second.type)
    {
    case GL_TEXTURE_1D: resultStrm << "GL_TEXTURE_1D"; break;
    case GL_TEXTURE_2D: resultStrm << "GL_TEXTURE_2D"; break;
    case GL_TEXTURE_3D: resultStrm << "GL_TEXTURE_3D"; break;
#ifdef GL_TEXTURE_RECTANGLE
    case GL_TEXTURE_RECTANGLE: resultStrm << "GL_TEXTURE_RECTANGLE"; break;
#endif
#ifdef GL_TEXTURE_CUBE_MAP
    case GL_TEXTURE_CUBE_MAP: resultStrm << "GL_TEXTURE_CUBE_MAP"; break;
#endif 
#ifdef GL_TEXTURE_1D_ARRAY
    case GL_TEXTURE_1D_ARRAY: resultStrm << "GL_TEXTURE_1D_ARRAY"; break;
#endif 
#ifdef GL_TEXTURE_2D_ARRAY
    case GL_TEXTURE_2D_ARRAY: resultStrm << "GL_TEXTURE_2D_ARRAY"; break;
#endif 
#ifdef GL_TEXTURE_CUBE_MAP_ARRAY
    case GL_TEXTURE_CUBE_MAP_ARRAY: resultStrm << "GL_TEXTURE_CUBE_MAP_ARRAY"; break;
#endif
#ifdef GL_TEXTURE_BUFFER
    case GL_TEXTURE_BUFFER: resultStrm << "GL_TEXTURE_BUFFER"; break;
#endif
#ifdef GL_TEXTURE_2D_MULTISAMPLE
    case GL_TEXTURE_2D_MULTISAMPLE: resultStrm << "GL_TEXTURE_2D_MULTISAMPLE"; break;
#endif
#ifdef GL_TEXTURE_2D_MULTISAMPLE_ARRAY
    case GL_TEXTURE_2D_MULTISAMPLE_ARRAY: resultStrm << "GL_TEXTURE_2D_MULTISAMPLE_ARRAY"; break;
#endif
    default: resultStrm << "unknown_type"; break;
    }

    resultStrm << " - id " << it->second.id;
  }

  if (vertexDecl)
    resultStrm << "\n" << vertexDecl->toString();

  if (!uniformPool_.empty())
    resultStrm << "\n" << uniformPool_.toString();

  return result;
}


void RenderObject::setUniform( const char *_name, GLint _value )
{
  uniformPool_.setUniform(_name, _value);
}

void RenderObject::setUniform( const char *_name, GLfloat _value )
{
  uniformPool_.setUniform(_name, _value);
}

void RenderObject::setUniform( const char *_name, const ACG::Vec2f &_value )
{
  uniformPool_.setUniform(_name, _value);
}

void RenderObject::setUniform( const char *_name, const ACG::Vec3f &_value )
{
  uniformPool_.setUniform(_name, _value);
}

void RenderObject::setUniform( const char *_name, const ACG::Vec4f &_value )
{
  uniformPool_.setUniform(_name, _value);
}

void RenderObject::setUniform( const char *_name, const ACG::GLMatrixf &_value, bool _transposed /*= false*/ )
{
  uniformPool_.setUniform(_name, _value, _transposed);
}

void RenderObject::setUniformMat3( const char *_name, const ACG::GLMatrixf &_value, bool _transposed /*= false*/ )
{
  uniformPool_.setUniform(_name, _value, _transposed);
}

void RenderObject::setUniform( const char *_name, GLint *_values, int _count )
{
  uniformPool_.setUniform(_name, _values, _count);
}

void RenderObject::setUniform( const char *_name, GLfloat *_values, int _count )
{
  uniformPool_.setUniform(_name, _values, _count);
}

void RenderObject::addUniformPool( const GLSL::UniformPool& _pool )
{
  uniformPool_.addPool(_pool);
}


void RenderObject::setupLineRendering( float _lineWidth, const Vec2f& _screenSize )
{
  shaderDesc.geometryTemplateFile = "Wireframe/geom_line2quad.tpl";

  setUniform("lineWidth", _lineWidth);
  setUniform("screenSize", _screenSize);
}

bool RenderObject::isDefaultLineObject() const
{
  return shaderDesc.geometryTemplateFile == "Wireframe/geom_line2quad.tpl" ||
    (shaderDesc.geometryTemplateFile == "Wireframe/gl42/geometry.tpl" &&
    shaderDesc.fragmentTemplateFile == "Wireframe/gl42/fragment.tpl");
}

void RenderObject::resetLineRendering()
{
  shaderDesc.geometryTemplateFile.clear();
}

void RenderObject::setupPointRendering( float _pointSize, const Vec2f& _screenSize )
{
  shaderDesc.geometryTemplateFile = "PointSize/geometry.tpl";
  shaderDesc.fragmentTemplateFile = "PointSize/fragment.tpl";

  setUniform("pointSize", _pointSize);
  setUniform("screenSize", _screenSize);
}

bool RenderObject::isDefaultPointObject() const
{
  return shaderDesc.geometryTemplateFile == "PointSize/geometry.tpl" &&
    shaderDesc.fragmentTemplateFile == "PointSize/fragment.tpl";
}

void RenderObject::resetPointRendering()
{
  shaderDesc.geometryTemplateFile.clear();
  shaderDesc.fragmentTemplateFile.clear();
}



} // namespace ACG end

