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


//=============================================================================
//
//  OpenGL Objects
//
//=============================================================================


#ifndef GL_OBJECTS_HH
#define GL_OBJECTS_HH


//== INCLUDES =================================================================

// GL
#include <ACG/GL/gl.hh>
#include <ACG/GL/GLState.hh>

// C++
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <vector>

// C
#include <cstdio>
#include <cstring>

#ifdef max
#  undef max
#endif

#ifdef min
#  undef min
#endif


//== NAMESPACES ===============================================================

namespace GLSL {
  class Program; // prototype
}

namespace ACG {


//== CLASS DEFINITION =========================================================


class DisplayList
{
public:

  DisplayList() : valid(false) {}

  virtual ~DisplayList() { del(); }

  void call() { if (valid) glCallList(dlist); }

  void new_list(GLenum mode) { if(!valid) gen(); glNewList(dlist, mode); }

  void end_list() { glEndList(); }

  void del() { if(valid) glDeleteLists(dlist, 1); valid = false; }

  bool is_valid() const { return valid; }

private:

  void gen() { dlist = glGenLists(1); valid=true; }

  bool valid;
  GLuint dlist;
};


//== CLASS DEFINITION =========================================================


#if defined(GL_VERSION_1_5)

class ACGDLLEXPORT VertexBufferObject
{
public:

  VertexBufferObject(GLenum _target) : target(_target), valid(false), vbo(0u) {}

  virtual ~VertexBufferObject() { del(); }

  void del();
  bool is_valid() const { return valid; }

  void bind()   { if(!valid) gen(); ACG::GLState::bindBuffer(target, vbo); }
  void unbind() { ACG::GLState::bindBuffer(target, 0); }

  void upload(GLsizeiptr size, const GLvoid* data, GLenum usage);

  // Upload a subset of the buffer data
  void uploadSubData(GLuint _offset, GLuint _size, const GLvoid* _data );

  char* offset(unsigned int _offset) const
  {
    return reinterpret_cast<char*>(_offset);
  }

  GLuint id() const {return vbo;}

  int size();

private:

  void gen();

  GLenum target;
  bool   valid;
  GLuint vbo;

};


class GeometryBuffer : public VertexBufferObject
{
public:
  GeometryBuffer() : VertexBufferObject(GL_ARRAY_BUFFER) {}
};


class IndexBuffer : public VertexBufferObject
{
public:
  IndexBuffer() : VertexBufferObject(GL_ELEMENT_ARRAY_BUFFER) {}
};


#endif


/* Vertex array object
https://www.opengl.org/wiki/Vertex_Specification#Vertex_Array_Object

A VAO is a state collection for vertex, index buffers and vertex attribute locations.
It basically stores all vertex and index buffer related opengl states.
Maybe Vertex- or GeometryInputState is a more fitting name for this class..

extension: https://www.opengl.org/registry/specs/ARB/vertex_array_object.txt
opengl-core: 3.0


usage:

setup VAO:
vao.bind()
usual opengl setup of vertex and indexbuffer and attribute locations

draw with VAO:
vao.bind()
glDraw..()
*/
class ACGDLLEXPORT VertexArrayObject
{
public:
  VertexArrayObject();
  virtual ~VertexArrayObject();

  // check hardware support
  static bool isSupported();

  void bind();
  void unbind();

  // implicitly called on bind(), but can be used to reset the VAO
  void init();

  // opengl object id
  GLuint id() const {return id_;}

  bool is_valid() const {return id_ != 0;}

  operator GLuint() const {return id_;}

private:
  GLuint id_;

  static int supportStatus_;
};



//== CLASS DEFINITION =========================================================


class ACGDLLEXPORT Texture
{
public:

  Texture(GLenum tgt, GLenum _unit=GL_NONE);

  virtual ~Texture() { del(); }


  void bind(GLenum _unit)
  {
    if(!valid) gen();
    activate(_unit);
    ACG::GLState::bindTexture(target, texture);
  }

  void activate(GLenum _unit)
  {
    if (_unit != GL_NONE) ACG::GLState::activeTexture(_unit);
  }

  void bind() {  bind(unit); }

  void activate() {  activate(unit);  }

  void parameter(GLenum pname, GLint i)
  {
    activate();
    glTexParameteri(target, pname, i);
  }

  void parameter(GLenum pname, GLfloat f)
  {
    activate();
    glTexParameterf(target, pname, f);
  }

  void parameter(GLenum pname, GLint * ip)
  {
    activate();
    glTexParameteriv(target, pname, ip);
  }

  void parameter(GLenum pname, GLfloat * fp)
  {
    activate();
    glTexParameterfv(target, pname, fp);
  }

  void enable()
  {
    activate();
    ACG::GLState::enable(target);
  }

  void disable()
  {
    activate();
    ACG::GLState::disable(target);
  }

  void del()
  {
    if(valid) glDeleteTextures(1, &texture);
    valid = false;
  }

  void gen() { glGenTextures(1, &texture); valid = (texture > 0u ? true : valid); }

  bool is_valid() const { return valid; }

  GLuint id() const { return texture; }

  void setUnit(GLenum u) {unit = u;}
  GLenum getUnit() const { return unit; }
  GLenum getTarget() const {return target;}

  // note: might bind the texture in order to find the format
  GLint getInternalFormat();


  // check supportsClearTexture to find out whether clear is supported (ARB_clear_texture)
  // clear does not work for buffer textures!

  // clear normalized / floating point texture
  bool clear(const ACG::Vec4f& _color);

  // clear integer texture
  bool clear(const ACG::Vec4i& _color);
  bool clear(const ACG::Vec4ui& _color);


  // use texture as image load/store    (equivalent of unordered access buffers in dx11)
  //  allows data scattering operations in shader ie. random read/write access
  //  ref: https://www.opengl.org/registry/specs/ARB/shader_image_load_store.txt
  // _index zero-based image unit
  // _access access operations in shader: GL_READ_WRITE, GL_READ_ONLY, GL_WRITE_ONLY
  // requires opengl 4.2
  void bindAsImage(GLuint _index, GLenum _access);

  // test for shader_image_load_store support
  static bool supportsImageLoadStore();

  // test for texture buffer support
  static bool supportsTextureBuffer();

  // test for clear_texture support
  static bool supportsClearTexture();

  // test for hardware accelerated mip map generation
  static bool supportsGenerateMipmap();

private:

  GLenum target, unit;
  bool valid;
  GLuint texture;

protected:
  GLint internalFormat_;
};


//-----------------------------------------------------------------------------


class ACGDLLEXPORT Texture1D : public Texture
{
public:
  Texture1D(GLenum unit=GL_NONE);

  // initialize and set texture data via glTexImage1D
  void setData(GLint _level, GLint _internalFormat, GLsizei _width, GLenum _format, GLenum _type, const GLvoid* _data);

  // specify storage of texture  (OpenGL 4.2)
  //  use setData with a nullptr instead if 4.2 is not available
  void setStorage(GLsizei _levels, GLenum _internalFormat, GLsizei _width);

  // get params from glTexImage1D
  GLsizei getWidth() const {return width_;}
  GLenum getFormat() const {return format_;}
  GLenum getType() const {return type_;}

  // read data back to sysmem
  bool getData(GLint _level, void* _dst);
  bool getData(GLint _level, std::vector<char>& _dst);

private:

  GLsizei width_;
  GLenum format_, type_;
};


//-----------------------------------------------------------------------------


class ACGDLLEXPORT Texture2D : public Texture
{
public:
  Texture2D(GLenum unit=GL_NONE);

  // Enable automatic mipmap generation (either on gpu or cpu).
  // The mipmap chain is generated the next time the base level of the texture is changed.
  // So this should be called after bind() but before setData()!
  // returns true if the hardware supports this feature, false if it runs in software mode
  bool autogenerateMipMaps();

  // Disabled automatic generation of the mip map chain
  void disableAutogenerateMipMaps();

  // initialize and set texture data via glTexImage2D
  void setData(GLint _level, GLint _internalFormat, GLsizei _width, GLsizei _height, GLenum _format, GLenum _type, const GLvoid* _data, bool _mipmaps = false);

  // specify storage of texture  (OpenGL 4.2)
  //  use setData with a nullptr instead if 4.2 is not available
  void setStorage(GLsizei _levels, GLenum _internalFormat, GLsizei _width, GLsizei _height);

  // initialize and load from file
  //  supports png, jpg etc. (any format that can be loaded by qt)
  //  additionally supports dds if the gli library is available while building ACG
  bool loadFromFile(const std::string& _filename, GLenum _minFilter = GL_NEAREST_MIPMAP_LINEAR, GLenum _magFilter = GL_LINEAR);

  // initialize and fill with uniform random data in [0,1] (or [-1,1] for signed formats)
  void loadRandom(GLint _internalFormat, GLsizei _width, GLsizei _height);

  // get params from glTexImage2D
  GLsizei getWidth() const {return width_;}
  GLsizei getHeight() const {return height_;}
  GLenum getFormat() const {return format_;}
  GLenum getType() const {return type_;}

  // read data back to sysmem
  bool getData(GLint _level, void* _dst);
  bool getData(GLint _level, std::vector<char>& _dst);

  // check if there is enough mem space to allocate a texture of requested size and format
  static bool checkTextureMem(GLenum _internalFormat, GLsizei _width, GLsizei _height, 
    GLenum _format);

private:

  // build mipmap chain on cpu
  void buildMipMaps(GLenum _internalfmt,
    GLint _width,
    GLint _height,
    GLenum _format,
    GLenum _type,
    const void* _data);

private:

  GLsizei width_, height_;
  GLenum format_, type_;

  bool buildMipsCPU_;
};


//-----------------------------------------------------------------------------


class Texture3D : public Texture
{
public:
  Texture3D(GLenum unit=GL_NONE) : Texture(GL_TEXTURE_3D, unit) {}
};


//-----------------------------------------------------------------------------


#if defined(GL_ARB_texture_cube_map)

class TextureCubeMap : public Texture
{
public:
  TextureCubeMap(GLenum u=GL_NONE) : Texture(GL_TEXTURE_CUBE_MAP_ARB, u) {}
};

#elif defined(GL_EXT_texture_cube_map)

class TextureCubeMap : public Texture
{
public:
  TextureCubeMap(GLenum u=GL_NONE) : Texture(GL_TEXTURE_CUBE_MAP_EXT, u) {}
};

#endif


//-----------------------------------------------------------------------------


#if defined(GL_EXT_texture_rectangle)

class TextureRectangleEXT : public Texture
{
public:
  TextureRectangleEXT(GLenum u=GL_NONE)
    : Texture(GL_TEXTURE_RECTANGLE_EXT, u) {}
};

#endif


#if defined(GL_NV_texture_rectangle)

class TextureRectangleNV : public Texture
{
public:
  TextureRectangleNV(GLenum u=GL_NONE)
    : Texture(GL_TEXTURE_RECTANGLE_NV, u) {}
};

#endif



class ACGDLLEXPORT TextureBuffer : public Texture
{
public:
  TextureBuffer(GLenum u=GL_NONE);

  ~TextureBuffer();

  // _size  size in bytes of buffer data
  // _data  buffer data
  // _internalFormat format of buffer - http://www.opengl.org/sdk/docs/man3/xhtml/glTexBuffer.xml
  // _usage buffer usage hint - https://www.opengl.org/sdk/docs/man3/xhtml/glBufferData.xml
  void setBufferData(size_t _size, const void* _data, GLenum _internalFormat, GLenum _usage = GL_STATIC_DRAW);

  size_t getBufferSize() const {return bufferSize_;}

  GLuint getBufferId() const {return buffer_;}

  GLenum getUsage() const {return usage_;}

  GLenum getFormat() const {return fmt_;}


  // read buffer data back to sysmem
  bool getBufferData(void* _dst);
  bool getBufferData(std::vector<char>& _dst);

private:

  size_t bufferSize_;
  GLuint buffer_;
  GLenum usage_;
  GLenum fmt_;
};




//== CLASS DEFINITION =========================================================


class ProgramBase
{
public:

  ProgramBase(GLenum tgt) : valid(false), target(tgt), program(0) {}
  virtual ~ProgramBase() {}

  bool   is_valid() const { return valid; }
  GLuint id()       const { return program; }

  virtual void bind()   = 0;
  virtual void unbind() = 0;
  virtual bool load(const char* prog_text) = 0;

  bool load_file(const char* _filename)
  {
    std::ifstream ifs(_filename);
    if (!ifs)
    {
      std::cerr << "Can't open " << _filename << "\n";
      return false;
    }
    std::string prog;
    char line[255];
    while (!ifs.eof())
    {
      if (!ifs.getline(line, 255).bad())
      {
	prog += std::string(line);
	prog += '\n';
      }
    }
    ifs.close();
    return load(prog.c_str());
  }


protected:

  bool valid;
  GLenum target;
  GLuint program;
};


//== CLASS DEFINITION =========================================================


#if defined(GL_NV_vertex_program) || defined(GL_NV_fragment_program)

class ProgramBaseNV : public ProgramBase
{
public:

  ProgramBaseNV(GLenum tgt) : ProgramBase(tgt) {}
  ~ProgramBaseNV() { del(); }

  void bind();
  void unbind();

  bool load(const char* prog_text);


private:
  void gen();
  void del();
};

#endif


//== CLASS DEFINITION =========================================================


#if defined(GL_ARB_vertex_program) || defined(GL_ARB_fragment_program)

class ProgramBaseARB : public ProgramBase
{
public:

  ProgramBaseARB(GLenum tgt) : ProgramBase(tgt) {}
  ~ProgramBaseARB() { del(); }

  void bind();
  void unbind();

  bool load(const char* prog_text);

  void parameter() {}

private:
  void gen();
  void del();
};

#endif


//-----------------------------------------------------------------------------


#if defined(GL_NV_vertex_program)

class VertexProgramNV : public ProgramBaseNV
{
public:
  VertexProgramNV() : ProgramBaseNV(GL_VERTEX_PROGRAM_NV) {}
};


class VertexStateProgramNV : public ProgramBaseNV
{
public:
  VertexStateProgramNV() : ProgramBaseNV(GL_VERTEX_STATE_PROGRAM_NV) {}
};

#endif


//-----------------------------------------------------------------------------


#if defined(GL_NV_fragment_program)

class FragmentProgramNV : public ProgramBaseNV
{
public:
  FragmentProgramNV() : ProgramBaseNV(GL_FRAGMENT_PROGRAM_NV) {}
};

#endif



//-----------------------------------------------------------------------------


#if defined(GL_ARB_vertex_program)

class VertexProgramARB : public ProgramBaseARB
{
public:
  VertexProgramARB() : ProgramBaseARB(GL_VERTEX_PROGRAM_ARB) {}
};

#endif


//-----------------------------------------------------------------------------


#if defined(GL_ARB_fragment_program)

class FragmentProgramARB : public ProgramBaseARB
{
public:
  FragmentProgramARB() : ProgramBaseARB(GL_FRAGMENT_PROGRAM_ARB) {}
};

#endif


//== CLASS DEFINITION =========================================================

/*
Atomic counter for shaders:
 http://www.opengl.org/wiki/Atomic_Counter

This is a global counter that can be incremented/decremented within shaders.
Counting is atomic for all shader invocations (ie. inc/decrement is thread-safe in parallel invocations)

extension: http://www.opengl.org/registry/specs/ARB/shader_atomic_counters.txt
opengl-core: 4.2

usage:
 counter is initialized implicitly or explicitly via init(num)
 -> reset counter via set()
 -> call bind() before rendering
 -> in shader: 
        layout(binding = 0, offset = 0) uniform atomic_uint myCounter;
         ...
        
          uint counterVal = atomicCounter(myCounter);
          counterVal = atomicCounterIncrement(myCounter);
          counterVal = atomicCounterDecrement(myCounter);
*/
class ACGDLLEXPORT AtomicCounter
{
public:
  // _numCounters  number of counters in the buffer, each counter is a uint value
  AtomicCounter(int _numCounters = 1);

  virtual ~AtomicCounter();

  // create counter buffer, implicitly called for a single counter value
  void init();

  // set counter value
  void set(unsigned int _value = 0);

  // read counter values after rendering
  //  _out  ptr to array of size numCounters_, receiving the actual counter values
  void get(unsigned int* _out);

  // bind
  void bind();

  // bind to index corresponding to binding point in shader: layout (binding = _index)
  void bind(GLuint _index);

  // deactivate
  void unbind();

  // check hardware support
  static bool isSupported();

  bool isValid() const;

  GLuint getBufferId() const {return buffer_;}
  int getNumCounters() const {return numCounters_;}

private:

  int numCounters_;
  GLuint buffer_;

  static int supportStatus_;
};

//== CLASS DEFINITION =========================================================


/*
Query object (occlusion queries):

ref: https://www.opengl.org/wiki/Query_Object
opengl-core: 1.5

usage:
query.begin(GL_SAMPLES_PASSED);
.. gl-calls
query.end();
GLuint numSamples = query.result();

\note result() method synchronizes CPU and GPU, so this might stall the cpu
*/

class ACGDLLEXPORT QueryObject
{
public:

  // set the type of the query object:
  //   GL_SAMPLES_PASSED - number of samples that passed the depth test
  //   GL_ANY_SAMPLES_PASSED - return true iff there is at least one sample that passed the depth test (gl 3.3+)
  //   GL_ANY_SAMPLES_PASSED_CONSERVATIVE - return true if there might be a sample that passed the depth test (gl 4.3+)
  //   GL_TIME_ELAPSED - measure elapsed time (gl 3.3+)
  // also see: https://www.opengl.org/sdk/docs/man/docbook4/xhtml/glBeginQuery.xml
  QueryObject(GLenum _type = GL_SAMPLES_PASSED);
  virtual ~QueryObject();

  /// begin measuring the query
  void begin();

  /// stop measuring the query
  void end();

  /// check if the result is available (does not wait for gpu to finish)
  bool available() const;

  /// get the measurement of the query between the begin() end() calls (waits for the gpu)
  GLuint result() const;

private:

  GLuint id_;
  int state_; // -1 : not started,  0 : started,  1 : stopped
  GLenum type_;
};


//== CLASS DEFINITION =========================================================

/* 
Timer query:

Performance counter for profiling GPU timings

extension: https://www.opengl.org/registry/specs/ARB/timer_query.txt
opengl-core: 3.2

usage:
counter.restart();
.. gl-calls
GLuint64 ns = counter.elapsedNs();

\note elapsedX() methods synchronize CPU and GPU, so this has a performance hit on CPU side
*/
class ACGDLLEXPORT QueryCounter
{
public:
  QueryCounter();
  virtual ~QueryCounter();

  void restart();
  void stop();

  /// elapsed gpu time since restart() in nanosecs
  GLuint64 elapsedNs();

  /// elapsed gpu time in millisecs
  GLuint64 elapsedMs();

  /// elapsed gpu time in seconds
  float elapsedSecs();

  /// check hw support
  static bool isSupported();

private:

  GLuint queryObjects_[2];
  int state_; // -1 : not started,  0 : started,  1 : stopped

  static int supportStatus_;
};



//== CLASS DEFINITION =========================================================

/*
Uniform buffer object:
 https://www.opengl.org/wiki/Uniform_Buffer_Object

Grouping shader uniforms into a buffer allows to reuse 
the same set of uniforms across multiple shaders.
Also avoids having to call lots of setUniform functions.

extension: https://www.opengl.org/registry/specs/ARB/uniform_buffer_object.txt
opengl-core: 3.1

usage:
ACG::Vec4f vec0 = ..;
ubo.setUniformData(shader, "blockName", "uniformName0", vec0.data());

bind to a binding index:
 ubo.bind(idx);
 shader->setUniformBlockBinding(idx);

in shader:
uniform blockName
{
   vec4 uniformName0;
   vec4 uniformName1;
   ..
};

*/
class ACGDLLEXPORT UniformBufferObject : public VertexBufferObject
{
public:
  UniformBufferObject();

  virtual ~UniformBufferObject();

  // set data for a uniform (makes a byte-wise)
  //  if _delay is true, the buffer is only locally changed and must be updated later via upload().
  //  otherwise, the buffer is immediately updated via glBufferSubData
  void setUniformData(GLSL::Program* _prog, const char* _bufferName, const char* _uniformName, const void* _data, int _datasize, bool _delay = false);

  // upload the buffer after delayed initialization via setUniformData
  void upload();

  // use this to bind to a shader binding point
  void bind(GLuint _index);

  // check hardware support
  static bool isSupported();

  // get hw caps
  static int getMaxBindings();
  static int getMaxBlocksize();
  static int getMaxCombinedShaderBlocks();
  static int getOffsetAlignment();

private:

  // buffer data (optional)
  std::vector<char> data_;

  static void queryCaps();

  // hw caps
  static int supportStatus_;
  static int maxBlockSize_;
  static int maxBindings_;
  static int maxCombinedShaderBlocks_;
  static int offsetAlignment_;
};


//== CLASS DEFINITION =========================================================

/*
Shader storage buffer object:
 http://www.opengl.org/wiki/Shader_Storage_Buffer_Object

Similar to uniform buffer object and also texture buffer.
Can be read and written to from within shaders.
Also, the size of a SSBO is typically only bounded by the physical VRAM limitation,
so it should be used for large data sets or unknown sized arrays.

extension: https://www.opengl.org/registry/specs/ARB/shader_storage_buffer_object.txt
opengl-core: 4.3

usage:
init on application-side with data:
 ssbo.bind();
 ssbo.upload(datasize, dataptr, GL_DYNAMIC_DRAW);

bind to a binding index:
 ssbo.bind(idx);

in shader:
layout (stdxxx, binding = idx) buffer mySSBO
{
   vec4 v4;
   vec4 v_unbounded[];
};

*/
class ACGDLLEXPORT ShaderStorageBufferObject : public VertexBufferObject
{
public:
  ShaderStorageBufferObject();

  virtual ~ShaderStorageBufferObject();

  // use this to bind to a shader binding point
  void bind(GLuint _index);

  // check hardware support
  static bool isSupported();

  // get hw caps
  static int getMaxBindings();
  static int getMaxBlocksize();
  static int getMaxCombinedShaderBlocks();

private:

  static void queryCaps();

  // hw caps
  static int supportStatus_;
  static int maxBlockSize_;
  static int maxBindings_;
  static int maxCombinedShaderBlocks_;
};



//=============================================================================
} // namespace ACG
//=============================================================================
#endif // GL_OBJECTS_HH defined
//=============================================================================
