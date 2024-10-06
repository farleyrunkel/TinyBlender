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

//==============================================================================

#include <ACG/Config/ACGDefines.hh>
#include <ACG/Math/VectorT.hh>
#include <ACG/Math/GLMatrixT.hh>
#include <ACG/ShaderUtils/GLSLShader.hh>

#include <list>
#include <string>


//==============================================================================

namespace GLSL {

  /** \brief GLSL uniform pool
  *
  * A uniform pool collects values for shader uniforms
  */
  class ACGDLLEXPORT UniformPool {

  public:
    /** \brief Constructor
    */
    UniformPool();

    /** \brief Copy Constructor
    */
    UniformPool(const UniformPool& _pool);

    virtual ~UniformPool();

    void bind(PtrProgram _prog) const;
    void bind(GLuint _prog) const;

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


    void setUniform(const char *_name, GLint *_values, int _count);
    void setUniform(const char *_name, GLfloat *_values, int _count);


    void addPool(const UniformPool& _src);

    /** \brief Clear the pool
     *
     */
    void clear();

    /** \brief returns if the pool is empty
     *
     * @return empty pool?
     */
    bool empty() const;

    /** \brief print to string for debugging
     *
     */
    QString toString() const;

    /** \brief copy
     *
     */
    UniformPool& operator =(const UniformPool& _other);

  private:
    struct UniformBase {
      std::string id;


      UniformBase() {}
      virtual ~UniformBase() {}

      virtual void bind(GLuint _progID) const {}

      virtual QString toString() const { return QString(""); }
    };

    struct UniformVecf : public UniformBase {
      ACG::Vec4f val;
      int size;

      void bind(GLuint _progID) const override;

      virtual QString toString() const override;
    };

    // separate float int vector because sizeof(int) != sizeof(float) for some compilers
    struct UniformVeci : public UniformBase {
      ACG::Vec4i val;
      int size;

      void bind(GLuint _progID) const override;

      virtual QString toString() const override;
    };

    struct UniformVecui : public UniformBase {
      ACG::Vec4ui val;
      int size;

      void bind(GLuint _progID) const override;

      virtual QString toString() const override;
    };


    struct UniformMat : UniformBase {
      ACG::Matrix4x4f val;

      bool transposed;
      int size;

      void bind(GLuint _progID) const override;

      virtual QString toString() const override;
    };

    struct UniformBuf : public UniformBase {
      float* val;
      
      bool integer;
      int size;
      
      UniformBuf();
      ~UniformBuf();

      void bind(GLuint _progID) const override;

      virtual QString toString() const override;
    };


    typedef std::list<UniformBase*> UniformList;
    typedef UniformList::iterator UniformListIt;
    
    /// list of uniform params
    UniformList pool_;

  private:

    UniformListIt findEntry(std::string _name);

    void addVecf(const UniformVecf& _vec);
    void addVeci(const UniformVeci& _vec);
    void addVecui(const UniformVecui& _vec);
    void addMatrix(const UniformMat& _mat);
    void addBuf(const char *_name, void *_values, int _count, bool _integer);
  };

  typedef UniformPool* PtrUniformPool;
  typedef const UniformPool* PtrConstUniformPool;

}

