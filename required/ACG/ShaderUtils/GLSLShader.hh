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



#ifndef GLSLSHADER_H
#define GLSLSHADER_H

//==============================================================================

#include <ACG/Config/ACGDefines.hh>
#include <ACG/Math/VectorT.hh>
#include <ACG/Math/GLMatrixT.hh>
#include <ACG/GL/gl.hh>

#include <list>
#include <string>
#include <QStringList>

//==============================================================================

/** \brief This namespace contains all the classes and functions for handling
 * GLSL shader and program objects.
 */
namespace GLSL {

#define GLSL_MAX_LOGSIZE 16384

  typedef std::list<std::string> StringList;

  /** \brief A generic shader base class
  */
  class ACGDLLEXPORT Shader {

    public:
      explicit Shader(GLenum shaderType);
      virtual ~Shader();
      void setSource(const StringList& source);
      void setSource(const QStringList& source);

      // FIXME implement StringList getSource();
      bool compile(bool verbose = true);

    protected:
      GLuint m_shaderId;

      friend class Program;
  };

  typedef Shader* PtrShader;
  typedef const Shader* PtrConstShader;

  //--------------------------------------------------------------------------

  /** \brief GLSL vertex shader.
  */
  class ACGDLLEXPORT VertexShader : public Shader {

    public:
      VertexShader();
      virtual ~VertexShader();

  };

  typedef VertexShader* PtrVertexShader;
  typedef const VertexShader* PtrVertexConstShader;

  //--------------------------------------------------------------------------

  /** \brief GLSL fragment shader.
  */
  class ACGDLLEXPORT FragmentShader : public Shader {

    public:
      FragmentShader();
      virtual ~FragmentShader();
  };

  typedef FragmentShader* PtrFragmentShader;
  typedef const FragmentShader* PtrConstFragmentShader;

  //--------------------------------------------------------------------------

  /** \brief GLSL geometry shader.
  */
  class ACGDLLEXPORT GeometryShader : public Shader {

    public:
      GeometryShader();
      virtual ~GeometryShader();
  };

  typedef GeometryShader* PtrGeometryShader;
  typedef const GeometryShader* PtrConstGeometryShader;

  //--------------------------------------------------------------------------

#ifdef GL_ARB_tessellation_shader

    /** \brief GLSL tesselation control shader.
  */
  class ACGDLLEXPORT TessControlShader : public Shader {

    public:
      TessControlShader();
      virtual ~TessControlShader();
  };

  typedef TessControlShader* PtrTessControlShader;
  typedef const TessControlShader* PtrConstTessControlShader;

  //--------------------------------------------------------------------------

    /** \brief GLSL tesselation evaluation shader.
  */
  class ACGDLLEXPORT TessEvaluationShader : public Shader {

    public:
      TessEvaluationShader();
      virtual ~TessEvaluationShader();
  };

  typedef TessEvaluationShader* PtrTessEvaluationShader;
  typedef const TessEvaluationShader* PtrConstTessEvaluationShader;

#endif // GL_ARB_tessellation_shader

  //--------------------------------------------------------------------------

  /** \brief GLSL compute shader.
  */
  class ACGDLLEXPORT ComputeShader : public Shader {

    public:
      ComputeShader();
      virtual ~ComputeShader();


      // get hw caps
      struct Caps 
      {
        int maxUniformBlocks_;
        int maxTextureImageUnits_;
        int maxImageUniforms_;
        int maxSharedMemorySize_;
        int maxUniformComponents_;
        int maxAtomicCounterBufs_;
        int maxAtomicCounters_;
        int maxCombinedUniformComponents_;
        int maxWorkGroupInvocations_;
        int maxWorkGroupCount_[3];
        int maxWorkGroupSize_[3];
      };
      
      static const Caps& caps();

    private:

      static Caps caps_;
      static bool capsInitialized_;
  };

  typedef ComputeShader* PtrComputeShader;
  typedef const ComputeShader* PtrConstComputeShader;

  //--------------------------------------------------------------------------


  /** \brief GLSL program class.
  *
  * A GLSL program links together the vertex and fragment shaders.
  */
  class ACGDLLEXPORT Program {

    public:
      Program();
      virtual ~Program();



      //===========================================================================
      /** @name Compilation/Linking
       *
       * @{ */
      //===========================================================================

      void attach(PtrConstShader _shader);
      void detach(PtrConstShader _shader);
      void link();

      /** @} */

      //===========================================================================
      /** @name Localizations
       *
       * @{ */
      //===========================================================================

      int getAttributeLocation(const char *_name);
      int getUniformLocation(const char *_name);
      int getFragDataLocation(const char* _name);

      void bindAttributeLocation(unsigned int _index, const char *_name);
      void bindFragDataLocation(unsigned int _index, const char *_name);

      /** @} */

      //===========================================================================
       /** @name Uniform setters
        *
        * @{ */
       //===========================================================================

       void setUniform(const char *_name, GLint _value);
       void setUniform(const char *_name, const ACG::Vec2i &_value);
       void setUniform(const char *_name, const ACG::Vec3i &_value);
       void setUniform(const char *_name, const ACG::Vec4i &_value);

       void setUniform(const char *_name, GLuint _value);
       void setUniform(const char *_name, const ACG::Vec2ui &_value);
       void setUniform(const char *_name, const ACG::Vec3ui &_value);
       void setUniform(const char *_name, const ACG::Vec4ui &_value);

       void setUniform(const char *_name, GLfloat _value);
       void setUniform(const char *_name, const ACG::Vec2f &_value);
       void setUniform(const char *_name, const ACG::Vec3f &_value);
       void setUniform(const char *_name, const ACG::Vec4f &_value);


       void setUniform(const char *_name, const ACG::GLMatrixf &_value, bool _transposed = false);
       void setUniformMat3(const char *_name, const ACG::GLMatrixf &_value, bool _transposed = false);



       void setUniform(const char *_name, const GLint *_values, int _count);
       void setUniform(const char *_name, const GLfloat *_values, int _count);
       void setUniform(const char *_name, const ACG::Vec2f *_values, int _count);
       void setUniform(const char *_name, const ACG::Vec3f *_values, int _count);
       void setUniform(const char *_name, const ACG::Vec4f *_values, int _count);
       void setUniform(const char *_name, int _index, bool _value);

       void setUniform(const char *_name, int _index, int _value);
       void setUniform(const char *_name, int _index, float _value);

       /** @} */

       

      //===========================================================================
       /** @name Uniform buffer blocks
        *
        * @{ */
      //===========================================================================

       GLuint getUniformBlockIndex(const char *_name);

       void setUniformBlockBinding(GLuint _index, int _binding);
       void setUniformBlockBinding(const char *_name, int _binding);

       int getUniformBlockSize(GLuint _index);
       int getUniformBlockSize(const char *_name);

       void getUniformBlockOffsets(int _numUniforms, const char **_names, int *_outOffsets);


      /** @} */

      //===========================================================================
       /** @name Geometry shader parameters
        *
        * @{ */
      //===========================================================================

      void setGeometryInputType(GLint _type);
      void setGeometryOutputType(GLint _type);
      void setGeometryVertexCount(GLint _numVerticesOut);

      /** @} */

      //===========================================================================
      /** @name Enable/disable functions
       *
       * @{ */
      //===========================================================================

      void use();
      void disable();
      bool isActive();
      bool isLinked();

      /** @} */

      GLuint getProgramId();

    private:

      std::list<PtrConstShader> m_linkedShaders;
      GLint m_programId;

      GLint m_linkStatus;
  };

  typedef Program* PtrProgram;
  typedef const Program* PtrConstProgram;

  //--------------------------------------------------------------------------

  GLSL::StringList ACGDLLEXPORT loadShader(const char *filename, const GLSL::StringList *macros = 0, bool appendNewLineChar = true, GLSL::StringList *outIncludes = 0);

  GLSL::PtrVertexShader ACGDLLEXPORT loadVertexShader(const char *name, const GLSL::StringList *macros = 0, bool verbose = true);
  GLSL::PtrFragmentShader ACGDLLEXPORT loadFragmentShader(const char *name, const GLSL::StringList *macros = 0, bool verbose = true);
  GLSL::PtrGeometryShader ACGDLLEXPORT loadGeometryShader(const char *name, const GLSL::StringList *macros = 0, bool verbose = true);
  GLSL::PtrShader ACGDLLEXPORT loadTessControlShader(const char *name, const GLSL::StringList *macros = 0, bool verbose = true);
  GLSL::PtrShader ACGDLLEXPORT loadTessEvaluationShader(const char *name, const GLSL::StringList *macros = 0, bool verbose = true);
  GLSL::PtrComputeShader ACGDLLEXPORT loadComputeShader(const char *name, const GLSL::StringList *macros = 0, bool verbose = true);

  /** load shaders and create GLSL program if successful
   *
   * Shader file paths for this function are assumed to be relative
   * to the "Shader" directory as specified in   ShaderProgGenerator::getShaderDir()
  */
  GLSL::PtrProgram ACGDLLEXPORT loadProgram(const char *vertexShaderFile,
                                          const char *fragmentShaderFile,
                                          const GLSL::StringList *macros = 0,
                                          bool verbose = true);

  /** load shaders and create GLSL program if successful
   *
   * Shader file paths for this function are assumed to be relative
   * to the "Shader" directory as specified in   ShaderProgGenerator::getShaderDir()
  */
  GLSL::PtrProgram ACGDLLEXPORT loadProgram(const char *vertexShaderFile,
                                          const char *geometryShaderFile,
                                          const char *fragmentShaderFile,
                                          const GLSL::StringList *macros = 0,
                                          bool verbose = true);

  /** load shaders and create GLSL program if successful
   *
   * Shader file paths for this function are assumed to be relative
   * to the "Shader" directory as specified in   ShaderProgGenerator::getShaderDir()
  */
  GLSL::PtrProgram ACGDLLEXPORT loadProgram(const char *vertexShaderFile,
                                          const char *tessControlShaderFile,
                                          const char *tessEvaluationShaderFile,
                                          const char *geometryShaderFile,
                                          const char *fragmentShaderFile,
                                          const GLSL::StringList *macros = 0,
                                          bool verbose = true);

  /** load glsl compute shader and create GLSL program if successful
   *
   * Shader file paths for this function are assumed to be relative
   * to the "Shader" directory as specified in   ShaderProgGenerator::getShaderDir()
  */
  GLSL::PtrProgram ACGDLLEXPORT loadComputeProgram(const char *computeShaderFile,
                                                   const GLSL::StringList *macros = 0,
                                                   bool verbose = true);
}

#endif // GLSLSHADER_H
