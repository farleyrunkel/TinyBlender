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


#pragma once


#include <ACG/GL/gl.hh>
#include <ACG/Math/GLMatrixT.hh>
#include <ACG/GL/ShaderGenerator.hh>
#include <ACG/ShaderUtils/UniformPool.hh>

#include <map>


namespace GLSL{
  class Program;
}

namespace ACG
{

// forward declaration
class VertexDeclaration;
class GLState;

namespace SceneGraph {
  namespace DrawModes {
    class DrawModeProperties;
  }
  class Material;
}


/** \brief Interface class between scenegraph and renderer
 *
 * RenderObject is the primary interface between scenegraph and renderer.
 *
 * Scenegraph nodes have to declare any renderable geometry objects by providing:
 * - geometry type (triangles, lines, points, triangle strips, ...)
 * - geometry buffers (vertex/index buffers,  vbo, ibo, sysmem and non-indexed supported )
 * - vertex buffer layout ( via vertex declaration, eventually declare sysmem vertex buffer here )
 * - draw mode
 * - OpenGL render states
 * - material properties
 * - texture
 *
 * Quick initialization for default values is recommended.
 * RenderObject::initFromState() grabs transforms, material and render states from a GLState.
 *
 * An OpenGL style interface for geometry setup is available via:
 * RenderObject::glBindBuffer()
 * RenderObject::glDrawArrays()
 * RenderObject::glDrawArrayElements()
 *
 * You still have to create the VertexDeclaration on your own first though.
 *
 * Note that each RenderObject corresponds to exactly one deferred draw call.
*/
struct ACGDLLEXPORT RenderObject
{
  friend class IRenderer;

  /** default constructor
   *   set all members to OpenGL default values
   */
  RenderObject();

  ~RenderObject();


  /** \brief Priority to allow sorting of objects
   *
   * The renderer sorts objects based on priority in ascending order before rendering.
   *
   * \note negative values allowed
   */
  int priority;

  /** \brief Name for logging
   *
   */
  std::string name;

  /** \brief Layer based rendering
   *
   * The renderer currently supports two layers:
   *  - scene layer
   *  - overlay layer
   *
   * Usually a render plugin operates on the scene layer only and
   * overlayed objects are rendered on top of the result.
   * For instance, meshes are rendered in the scene layer while coordsys objects are overlayed.
   *
   * \note default = false
   */
  bool overlay;

  /// Modelview transform
  GLMatrixd modelview;

  /// Projection transform
  GLMatrixd proj;


  //===========================================================================
  /** @name Geometry definition
   *
   * @{ */
  //===========================================================================

  /** \brief Use vertex array object
   * 
   * Optionally, a VAO can be used to setup rendering buffers and attribute locations.
   * If this is 0 (default), vertex-buffer, index-buffer, vertex-declarition etc. have to be provided individually.
   * Otherwise, the VAO is used instead of vertexBuffer, indexBuffer etc.
   * In this case, it is not neccessary to specify a vertex-declaration and provide vertex- and indexbuffer.
   * This is also the only way to setup a renderobject that makes use of multiple vertexbuffers!
   */
  GLuint vertexArrayObject;


  /// VBO, IBO ids,  ignored if VAO is provided
  GLuint vertexBuffer,
        indexBuffer;

  /** \brief Use system memory index buffer
   *
   * If you don't want to use an IBO, simply assign your sysmem indexbuffer address here.
   * If both indexBuffer and sysmemIndexBuffer are 0, the renderer will treat this
   * RenderObject like an unfolded vertex buffer (e.g. 3 * nTris vertex buffer for GL_TRIANGLES)
   *
   * \note  It is also possible to specify a sysmem vertex buffer by setting up the VertexDeclaration accordingly use numIndices
   */
  const void* sysmemIndexBuffer;

  /** \brief Primitive type
   *
   *  PrimitiveType must be one of the following:
   *  GL_POINTS, GL_LINE_STRIP, GL_LINE_LOOP, GL_LINES, GL_TRIANGLE_STRIP,
   *  GL_TRIANGLE_FAN, GL_TRIANGLES, GL_QUAD_STRIP, GL_QUADS, GL_POLYGON,
   *  GL_TRIANGLES_ADJACENCY, GL_LINES_ADJACENCY, GL_PATCHES
   */
  GLenum primitiveMode;

  /// patch size if primitiveMode is GL_PATCHES for rendering with tessellation shaders
  unsigned int patchVertices; // GL_PATCH_VERTICES

  /// Number indices to render
  unsigned int numIndices;

  /// Offset to first index in the index buffer or vertex buffer respectively
  unsigned int indexOffset;

  /** \brief Index element type
   *
   * Has to be GL_UNSIGNED_SHORT or GL_UNSIGNED_INT
   */
  GLenum indexType;

  /**\ brief Instancing
   *
   * Number of instances to render.
   * numinstances <= 0, disables instancing.
   */
  GLsizei numInstances;


  /// Defines the vertex buffer layout,  ignored if VAO is provided
  const VertexDeclaration* vertexDecl;

  /** @} */



  /** \brief Drawmode and other shader params
   *
   * - setup the shade mode :                         shaderDesc.shadeMode = ..  (necessary)
   * - enable lighting with all lights in the scene:  shaderDesc.numLights =  0  (recommended)
   * - disable lighting(only removes lighting code):  shaderDesc.numLights = -1
   * - manual lighting setup:                         shaderDesc.numLights = n
   *                                                  shaderDesc.lightTypes[i] = ..
   * - enable / disable texturing :                   shaderDesc.textured = ..
   * ...
   *
   */
  ShaderGenDesc shaderDesc;

  // opengl states
  //  queried from glState in initFromState()
  bool culling;
  bool blending;
  bool alphaTest;
  bool depthTest;
  bool depthWrite;

  GLenum fillMode; // GL_POINT, GL_LINE, GL_FILL,  default: GL_FILL

  GLboolean colorWriteMask[4]; // {r,g,b,a},  default: all true

//  GLenum shadeModel; // GL_FACE, GL_SMOOTH   obsolete in shader pipeline
  GLenum depthFunc;  //!< GL_LESS, GL_LEQUAL, GL_GREATER ..

  // alpha testing function
  GLenum alphaFunc; //!< GL_LESS, GL_LEQUAL, GL_GREATER ..
  float alphaRef; // reference value for alpha function

  // alpha blending
  GLenum blendSrc, blendDest; //!< glBlendFunc: GL_SRC_ALPHA, GL_ZERO, GL_ONE, GL_ONE_MINUS_SRC_ALPHA ...

  Vec2f depthRange; //!< glDepthRange: (znear, zmax)


  // enable/disable clip planes
  //  bit i decides whether the vertex shader output gl_ClipDistance[i] is enabled or disabled
  //  default: all zero (disabled)
  GLuint clipDistanceMask;

  // GL_PROGRAM_POINT_SIZE : default false
  //  if true, use gl_PointSize from shader
  //  otherwise, use pointSize from render object
  bool programPointSize; 
  
  // glPointSize(), default: 0.1
  float pointSize;

  // ---------------------------
  // default tessellation lod, if only a tess-eval, but no tess-control shader is specified
  // this is ignored otherwise

  Vec2f patchDefaultInnerLevel; // GL_PATCH_DEFAULT_INNER_LEVEL
  Vec4f patchDefaultOuterLevel; // GL_PATCH_DEFAULT_OUTER_LEVEL

  // ---------------------------
  /// material definitions
  Vec3f diffuse,
        ambient,
        specular,
        emissive;

  float alpha,
        shininess;


  /** \brief Specify whether this object should be rendered in a z-prepass
   *
   * The renderer might do a z-prepass for some rendering techniques.
   * You can control whether an object should be taken into account for a z-prepass or not.
   * For instance, scene objects should be rendered in the z-prepass, but overlays (coordsys, selection stuff..) should not.
   *
   * default: true
   */
  bool inZPrePass;

  /** \brief Uniform name of the depth map in the used shader
   *
   * If a shader used by this object requires a depth map of the scene, you can specify the name of the texture sampler uniform used here.
   * This depth map is automatically computed in a z-prepass of the scene later in the renderer and assigned to the shader.
   * It will be a 2D texture storing the values of the depth buffer (gl_FragCoord.z) in GL_TEXTURE_2D, GL_R32F format.
   * Should be set to 0 if the used shader does not require a depth map.
   *
   * default: 0
   */
  const char* depthMapUniformName;


  /** \brief Texture to be used
   *
   * eventually a more flexible texture system with user defined:
   * - texture stage binding slot (0 .. 16)
   * - texture type (1D, 2D, 3D, rect, cube)
   * - array of textures
   * assumes binding slot 0 and 2D for now
   */
  struct Texture
  {
    GLuint id;
    GLenum type;
    bool shadow;
    Texture(GLuint _id = 0, GLenum _type = GL_TEXTURE_2D, bool _shadow = false):
      id(_id),
      type(_type),
      shadow(_shadow){}
  };


  /// adds a texture to stage RenderObjects::numTextures()
  void addTexture(const Texture& _t)
  {
    addTexture(_t,numTextures());
  }

  /**
   * adds a texture to an specific stage and enables texture support in shaderDesc
   */
  void addTexture(const Texture& _t,const size_t _stage, bool _addToShaderGen = true)
  {
    if (GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS < numTextures())
    {
      std::cerr << "Texturestage " << _stage << " is too big. Allowed stages: "<< GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS << std::endl;
      return;
    }
    textures_[_stage] = _t;
    if (_addToShaderGen)
      shaderDesc.addTextureType(_t.type,_t.shadow,_stage);
  }

  ///clear all textures. Also affected on shaderDesc
  void clearTextures() {textures_.clear(); shaderDesc.clearTextures();}

  const std::map<size_t,Texture>& textures(){return textures_;}

  size_t numTextures()  {return textures_.size();}

private:
  /// holds the textures (second) and the stage id (first)
  std::map<size_t,Texture> textures_;
public:


  /// used internally for renderer debugging
  int debugID;
  std::string debugName;

  /// may be used internally by the renderer
  unsigned int internalFlags_;


  // opengl style helper function interface: 
  // provided for easier setup of RenderObjects,
  //  usage is not necessary
  inline void glBindBuffer(GLenum target, GLuint buffer)
  {
    switch (target)
    {
    case GL_ARRAY_BUFFER: vertexBuffer = buffer; break;
    case GL_ELEMENT_ARRAY_BUFFER: indexBuffer = buffer; break;
    }
  }

  inline void glDrawArrays(GLenum mode, GLint first, GLsizei count)
  {
    this->glDrawInstancedArrays(mode, first, count, 0);
  }

  inline void glDrawInstancedArrays(GLenum mode, GLint first, GLsizei count, GLsizei primcount)
  {
    indexBuffer = 0;
    sysmemIndexBuffer = 0;

    primitiveMode = mode;
    indexOffset = first;
    numIndices = count;
    numInstances = primcount;
  }

  inline void glDrawElements(GLenum mode, GLsizei count, GLenum type, const GLvoid *indices)
  {
    this->glDrawElementsInstanced(mode, count, type, indices, 0);
  }

  inline void glDrawElementsInstanced(GLenum mode, GLsizei count, GLenum type, const GLvoid *indices, GLsizei primcount)
  {
    primitiveMode = mode;
    numIndices = count;
    indexType = type;

    sysmemIndexBuffer = indices;

    numInstances = primcount;
  }

  inline void glColorMask(GLboolean r, GLboolean g, GLboolean b, GLboolean a)
  {
    colorWriteMask[0] = r; colorWriteMask[1] = g; colorWriteMask[2] = b; colorWriteMask[3] = a;
  }

  inline void glAlphaFunc(GLenum func, float ref)
  {
    alphaFunc = func;
    alphaRef = ref;
  }
  
  /** \brief Initializes a RenderObject instance.
   *
   * Grabs material and transforms automatically if a GLState is provided.
  */
  void initFromState(GLState* _glState);

  void setMaterial(const SceneGraph::Material* _mat);

  /** \brief Fills out ShaderGenDesc parameters based on Drawmode properties
  */
  void setupShaderGenFromDrawmode(const SceneGraph::DrawModes::DrawModeProperties* _props);


  /** \brief Setup rendering of thick lines
   *
   * Two default rendering methods for line thickness are available:
   *  - quad extrusion in geometry shader   (anti-aliasing,  clipping issue due to depth testing with scene)
   *  - manual rasterization via shader image load/store  (no anti-aliasing,  no clipping issue,  requires gl4.2)
   * The method used depends on renderer settings.
   *
   * Also, this will overwrite the geometry and fragment shader template.
   * Requires support for geometry shaders with glsl 150
  */
  void setupLineRendering(float _lineWidth, const Vec2f& _screenSize);

  /// Test if the object is rendered with one of the default line thickness methods
  bool isDefaultLineObject() const;

  /// Reset shader template names blocked by line rendering
  void resetLineRendering();

  /** \brief Setup rendering of circle points
   *
   * Use default quad extrusion shader.
   * This will overwrite geometry and fragment shader template.
   * Requires support for geometry shaders with glsl 150
  */
  void setupPointRendering(float _pointSize, const Vec2f& _screenSize);

  /// Test if the object is rendered with one of the default point extension methods
  bool isDefaultPointObject() const;

  /// Reset shader template names blocked by point rendering
  void resetPointRendering();



  /** Returns a text representation of the RenderObject for debugging purposes.
  */
  QString toString() const;

  /** \brief set values for int uniforms
   *
   * @param _name        Name of uniform in shader
   * @param _value       value of the type
   *
   */
  void setUniform(const char *_name, GLint _value);

  /** \brief set values for float uniforms
     *
     * @param _name        Name of uniform in shader
     * @param _value       value of the type
     *
     */
  void setUniform(const char *_name, GLfloat _value);

  /** \brief set values for Vec2f uniforms
   *
   * @param _name        Name of uniform in shader
   * @param _value       value of the type
   *
   */
  void setUniform(const char *_name, const ACG::Vec2f &_value);

  /** \brief set values for Vec3f uniforms
   *
   * @param _name        Name of uniform in shader
   * @param _value       value of the type
   *
   */
  void setUniform(const char *_name, const ACG::Vec3f &_value);

  /** \brief set values for Vec4f uniforms
   *
   * @param _name        Name of uniform in shader
   * @param _value       value of the type
   *
   */
  void setUniform(const char *_name, const ACG::Vec4f &_value);

  void setUniform(const char *_name, const ACG::GLMatrixf &_value, bool _transposed = false);
  void setUniformMat3(const char *_name, const ACG::GLMatrixf &_value, bool _transposed = false);


  void setUniform(const char *_name, GLint *_values, int _count);
  void setUniform(const char *_name, GLfloat *_values, int _count);


  /** \brief add all uniforms from a pool
   *
   * @param _pool input pool
   *
   */
  void addUniformPool(const GLSL::UniformPool& _pool);

private:
  GLSL::UniformPool uniformPool_;
};


/** \brief Interface for modifying render objects
 *
 * This class has to be implemented by a user, 
 * and could be set to nodes that allow modification of render-objects.
 * The modifier is then applied directly before adding an object to a renderer.
 * It allows low-level access to all settings in a render-objects.
*/
class ACGDLLEXPORT RenderObjectModifier
{
public:
  RenderObjectModifier(const std::string& _name = "") : name_(_name) {}
  virtual ~RenderObjectModifier() {}

  /** \brief apply the modifier
   *
   * @param _obj       modifiable render-object
   *
   */
  virtual void apply(RenderObject* _obj) = 0;

  /// Get name of the modifier
  const std::string& name() const { return name_; }

private:
  std::string name_;
};



//=============================================================================
} // namespace ACG
//=============================================================================
