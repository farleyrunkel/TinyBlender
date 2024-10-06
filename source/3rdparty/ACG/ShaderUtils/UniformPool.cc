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


#include <iostream>
#include <QTextStream>

#include <ACG/GL/acg_glew.hh>
#include <ACG/GL/GLError.hh>
#include "UniformPool.hh"

#ifdef WIN32
  #ifndef __MINGW32__
    #define snprintf sprintf_s
  #endif 
#endif

//==============================================================================

namespace GLSL {

  //--------------------------------------------------------------------------
  // Uniform Pool
  //--------------------------------------------------------------------------


  UniformPool::UniformPool(){
  }


  UniformPool::UniformPool(const UniformPool& _pool) {
    addPool(_pool);
  }


  /** \brief Destructor
  */
  UniformPool::~UniformPool(){
    clear();
  }


  UniformPool& UniformPool::operator =(const UniformPool& _other) {
    addPool(_other);
    return *this;
  }

  void UniformPool::clear() {
    // Delete the uniforms in that pool
    for (UniformListIt it = pool_.begin(); it != pool_.end(); ++it)
      delete (*it);

    // Clear the pool
    pool_.clear();
  }

  bool UniformPool::empty() const {
    return pool_.empty();
  }


  QString UniformPool::toString() const {
    
    QString result;
    QTextStream resultStrm(&result);

    for (UniformList::const_iterator it = pool_.begin(); it != pool_.end(); ++it) {
      resultStrm << (*it)->toString() << "\n";
    }

    return result;
  }

  /** \brief Send all stored uniforms to program
   *
   *  @param _prog receiving GLSL program 
   */
  void UniformPool::bind( PtrProgram _prog ) const {
    bind(_prog->getProgramId());
  }

  /** \brief Send all stored uniforms to program
   *
   *  @param _prog opengl program id
   */
  void UniformPool::bind( GLuint _prog ) const {
    for (UniformList::const_iterator it = pool_.begin(); it != pool_.end(); ++it) {
      (*it)->bind(_prog);
    }
  }

  /** \brief Search the pool for an existing value for a uniform name
  *
  * @param _name  Name of the uniform
  * @return iterator of uniform entry
  */
  UniformPool::UniformListIt UniformPool::findEntry( std::string _name ) {

    for (UniformListIt it = pool_.begin(); it != pool_.end(); ++it){
      if ((*it)->id.compare(_name) == 0)
        return it;
    }

    return pool_.end();
  }

  /** \brief Add all uniforms of a pool to this pool
   *
   *  @param _src source uniform pool
   */
  void UniformPool::addPool( const UniformPool& _src ){

    for (UniformList::const_iterator it = _src.pool_.begin(); it != _src.pool_.end(); ++it){

      // determine type
      const UniformVecf* pVecf = dynamic_cast<const UniformVecf*>(*it);
      const UniformVeci* pVeci = dynamic_cast<const UniformVeci*>(*it);
      const UniformVecui* pVecui = dynamic_cast<const UniformVecui*>(*it);
      const UniformMat* pMat = dynamic_cast<const UniformMat*>(*it);
      const UniformBuf* pBuf = dynamic_cast<const UniformBuf*>(*it);

      // add to our list
      if (pVecf)
        addVecf(*pVecf);

      if (pVeci)
        addVeci(*pVeci);

      if (pVecui)
        addVecui(*pVecui);

      else if (pMat)
        addMatrix(*pMat);

      else if (pBuf)
        addBuf(pBuf->id.c_str(), pBuf->val, pBuf->size, pBuf->integer);
    }
  }


  /** \brief Bind uniform float vector to shader
  *
  * @param _progID  GL Program ID
  */
  void UniformPool::UniformVecf::bind( GLuint _progID ) const {
    checkGLError2("prev opengl error");
    GLint location = glGetUniformLocation(_progID, id.c_str());
    checkGLError2(id.c_str());

    switch (size){
      case 1:
        glUniform1fv(location, 1, val.data());
        break;
      case 2:
        glUniform2fv(location, 1, val.data());
        break;
      case 3:
        glUniform3fv(location, 1, val.data());
        break;
      case 4:
        glUniform4fv(location, 1, val.data());
        break;

      default:
        std::cerr << "UniformPool::UniformVecf : invalid size "  << size << std::endl;
        break;
    }

    checkGLError2(id.c_str());
  }


  /** \brief print float vector to string
  *
  */
  QString UniformPool::UniformVecf::toString() const {
    const char* fmt = size > 1 ? "uniform vec%2 %1 = vec%2(" : "uniform float %1 = ";
    QString str = QString(fmt).arg(id.c_str()).arg(size);
    for (int i = 0; i < size; ++i) {
      str += QString::number(val[i]);
      if (i + 1 < size)
        str += ", ";
    }
    if (size > 1)
      str += ");";
    return str;
  }

  /** \brief Bind uniform int vector to shader
  *
  * @param _progID  GL Program ID
  */
  void UniformPool::UniformVeci::bind( GLuint _progID ) const {
    checkGLError2("prev opengl error");
    GLint location = glGetUniformLocation(_progID, id.c_str());
    checkGLError2(id.c_str());

    switch (size){
        case 1:
          glUniform1iv(location, 1, (GLint*)val.data());
          break;
        case 2:
          glUniform2iv(location, 1, (GLint*)val.data());
          break;
        case 3:
          glUniform3iv(location, 1, (GLint*)val.data());
          break;
        case 4:
          glUniform4iv(location, 1, (GLint*)val.data());
          break;

        default:
          std::cerr << "UniformPool::UniformVeci : invalid size "  << size << std::endl;
          break;
    }

    checkGLError2(id.c_str());
  }


  /** \brief print int vector to string
  *
  */
  QString UniformPool::UniformVeci::toString() const {
    const char* fmt = size > 1 ? "uniform ivec%2 %1 = ivec%2(" : "uniform int %1 = ";
    QString str = QString(fmt).arg(id.c_str()).arg(size);
    for (int i = 0; i < size; ++i) {
      str += QString::number(val[i]);
      if (i + 1 < size)
        str += ", ";
    }
    if (size > 1)
      str += ");";
    return str;
  }

  /** \brief Bind uniform  uint vector to shader
  *
  * @param _progID  GL Program ID
  */
  void UniformPool::UniformVecui::bind( GLuint _progID ) const {
    checkGLError2("prev opengl error");
    GLint location = glGetUniformLocation(_progID, id.c_str());
    checkGLError2(id.c_str());

    switch (size){
        case 1:
          glUniform1uiv(location, 1, (GLuint*)val.data());
          break;
        case 2:
          glUniform2uiv(location, 1, (GLuint*)val.data());
          break;
        case 3:
          glUniform3uiv(location, 1, (GLuint*)val.data());
          break;
        case 4:
          glUniform4uiv(location, 1, (GLuint*)val.data());
          break;

        default:
          std::cerr << "UniformPool::UniformVecui : invalid size "  << size << std::endl;
          break;
    }

    checkGLError2(id.c_str());
  }


  /** \brief print uint vector to string
  *
  */
  QString UniformPool::UniformVecui::toString() const {
    const char* fmt = size > 1 ? "uniform uvec%2 %1 = uvec%2(" : "uniform uint %1 = ";
    QString str = QString(fmt).arg(id.c_str()).arg(size);
    for (int i = 0; i < size; ++i) {
      str += QString::number(val[i]);
      if (i + 1 < size)
        str += ", ";
    }
    if (size > 1)
      str += ");";
    return str;
  }

  /** \brief Bind uniform matrix to shader
  *
  * @param _progID  GL Program ID
  */
  void UniformPool::UniformMat::bind( GLuint _progID ) const {
    checkGLError2("prev opengl error");
    GLint location = glGetUniformLocation(_progID, id.c_str());
    checkGLError2(id.c_str());

    switch (size){
      case 2: {
          float tmp[4];
          for (int i = 0; i < 2; ++i)
            for (int k = 0; k < 2; ++k)
              tmp[i*2+k] = val.data()[i*4+k];
          glUniformMatrix2fv(location, 1, transposed, tmp);
        } break;

      case 3: {
        float tmp[9];
          for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k)
              tmp[i*3+k] = val.data()[i*4+k];
          glUniformMatrix3fv(location, 1, transposed, tmp);
        } break;

      case 4: glUniformMatrix4fv(location, 1, transposed, val.data()); break;

      default:
        std::cerr << "UniformPool::UniformMat : invalid size "  << size << std::endl;
        break;
    }

    checkGLError2(id.c_str());
  }


  /** \brief print matrix to string
  *
  */
  QString UniformPool::UniformMat::toString() const {
    QString str = QString("uniform mat%2 %1 = {").arg(id.c_str()).arg(size);
    for (int y = 0; y < size; ++y) {
      str += "{";
      for (int x = 0; x < size; ++x) {
        str += QString::number(val(y,x));
        if (x + 1 < size)
          str += ", ";
      }
      str += "}";
      if (y + 1 < size)
        str += ", ";
    }
    str += "};";
    return str;
  }

  /** \brief Bind uniform array to shader
  *
  * @param _progID  GL Program ID
  */
  void UniformPool::UniformBuf::bind( GLuint _progID ) const {
    checkGLError2("prev opengl error");
    GLint location = glGetUniformLocation(_progID, id.c_str());
    checkGLError2(id.c_str());

    if (integer){
      glUniform1iv(location, size, (GLint*)val);
    }
    else{
      glUniform1fv(location, size, val);
    }

    checkGLError2(id.c_str());
  }


  /** \brief print buffer id to string
  *
  */
  QString UniformPool::UniformBuf::toString() const {
    QString str = QString("uniform %3 %2[%1] = {").arg(id.c_str()).arg(size).arg(integer ? "int" : "float");
    for (int y = 0; y < size; ++y) {
      if (integer)
        str += QString::number(((GLint*)val)[y]);
      else
        str += QString::number(val[y]);
      if (y + 1 < size)
        str += ", ";
    }
    str += "};";
    return str;
  }

  /** \brief Creates a copy of input data
  */
  UniformPool::UniformBuf::UniformBuf()
    : val(0), integer(false), size(0)
  {
  }

  /** \brief Free data
  */
  UniformPool::UniformBuf::~UniformBuf() {
    delete [] val;
  }

  
  /** \brief Add or update a vector type uniform in pool
  *
  * @param _vec uniform specifier
  */
  void UniformPool::addVecf( const UniformVecf& _vec ) {
    // look for existing uniform in pool
    UniformListIt it = findEntry(_vec.id);

    // storage address of uniform
    UniformVecf* dst = 0;

    if ( it == pool_.end() ){
      // create new entry
      dst = new UniformVecf;
      pool_.push_back(dst);
    }
    else{
      // use existing entry
      dst = dynamic_cast<UniformVecf*>( *it );

      if (!dst)
        std::cerr << "UniformPool::addVecf type of " << _vec.id << " incorrect." << std::endl;
    }

    if (dst) {
      // update data
      dst->id = _vec.id;
      dst->size = _vec.size;
      dst->val = _vec.val;
    }

  }

  /** \brief Add or update a vector type uniform in pool
  *
  * @param _vec uniform specifier
  */
  void UniformPool::addVeci( const UniformVeci& _vec ) {
    // look for existing uniform in pool
    UniformListIt it = findEntry(_vec.id);

    // storage address of uniform
    UniformVeci* dst = 0;

    if ( it == pool_.end() ){
      // create new entry
      dst = new UniformVeci;
      pool_.push_back(dst);
    }
    else{
      // use existing entry
      dst = dynamic_cast<UniformVeci*>( *it );

      if (!dst)
        std::cerr << "UniformPool::addVeci type of " << _vec.id << " incorrect." << std::endl;
    }

    if (dst) {
      // update data
      dst->id = _vec.id;
      dst->size = _vec.size;
      dst->val = _vec.val;
    }

  }

  /** \brief Add or update a vector type uniform in pool
  *
  * @param _vec uniform specifier
  */
  void UniformPool::addVecui( const UniformVecui& _vec ) {
    // look for existing uniform in pool
    UniformListIt it = findEntry(_vec.id);

    // storage address of uniform
    UniformVecui* dst = 0;

    if ( it == pool_.end() ){
      // create new entry
      dst = new UniformVecui;
      pool_.push_back(dst);
    }
    else{
      // use existing entry
      dst = dynamic_cast<UniformVecui*>( *it );

      if (!dst)
        std::cerr << "UniformPool::addVecui type of " << _vec.id << " incorrect." << std::endl;
    }

    if (dst) {
      // update data
      dst->id = _vec.id;
      dst->size = _vec.size;
      dst->val = _vec.val;
    }

  }

  /** \brief Add or update a matrix type uniform in pool
  *
  * @param _mat uniform specifier
  */
  void UniformPool::addMatrix( const UniformMat& _mat ) {
    // look for existing uniform in pool
    UniformListIt it = findEntry(_mat.id);

    // storage address of uniform
    UniformMat* dst = 0;

    if ( it == pool_.end() ){
      // create new entry
      dst = new UniformMat;
      pool_.push_back(dst);
    }
    else{
      // use existing entry
      dst = dynamic_cast<UniformMat*>( *it );

      if (!dst)
        std::cerr << "UniformPool::addMatrix type of " << _mat.id << " incorrect." << std::endl;
    }

    if (dst) {
      // update data
      dst->id = _mat.id;
      dst->size = _mat.size;
      dst->transposed = _mat.transposed;
      dst->val = _mat.val;
    }
  }

  /** \brief Add or update an array type uniform in pool
  *
  * @param _name Uniform name
  * @param _values array data
  * @param _count array size (in dwords)
  * @param _integer integer/float array
  */
  void UniformPool::addBuf( const char* _name, void* _values, int _count, bool _integer ) {
    // look for existing uniform in pool
    UniformListIt it = findEntry(_name);

    // storage address of uniform
    UniformBuf* dst = 0;

    if ( it == pool_.end() ){
      // create new entry
      dst = new UniformBuf();
      pool_.push_back(dst);
    }
    else{
      // use existing entry
      dst = dynamic_cast<UniformBuf*>( *it );

      if (!dst)
        std::cerr << "UniformPool::addBuf type of " << _name << " incorrect." << std::endl;
    }

    if (dst) {
      // update data
      dst->id = _name;

      if (dst->size < _count)
      {
        // resize
        delete [] dst->val;
        dst->val = new float[_count];
      }

      dst->size = _count;

      if (_values)
        memcpy(dst->val, _values, _count * sizeof(float));
    }
  }


  /** \brief Set int uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, GLint _value ) {
    // create uniform descriptor
    UniformVeci tmp;
    tmp.id = _name;
    tmp.size = 1;
    tmp.val[0] = _value;

    // add/update in pool
    addVeci(tmp);
  }

  /** \brief Set ivec2 uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, const ACG::Vec2i &_value ) {
    // create uniform descriptor
    UniformVeci tmp;
    tmp.id = _name;
    tmp.size = 2;
    tmp.val[0] = _value[0];
    tmp.val[1] = _value[1];

    // add/update in pool
    addVeci(tmp);
  }

  /** \brief Set ivec3 uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, const ACG::Vec3i &_value ) {
    // create uniform descriptor
    UniformVeci tmp;
    tmp.id = _name;
    tmp.size = 3;
    tmp.val[0] = _value[0];
    tmp.val[1] = _value[1];
    tmp.val[2] = _value[2];

    // add/update in pool
    addVeci(tmp);
  }

  /** \brief Set ivec4 uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, const ACG::Vec4i &_value ) {
    // create uniform descriptor
    UniformVeci tmp;
    tmp.id = _name;
    tmp.size = 4;
    tmp.val = _value;

    // add/update in pool
    addVeci(tmp);
  }

  /** \brief Set uint uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, GLuint _value ) {
    // create uniform descriptor
    UniformVecui tmp;
    tmp.id = _name;
    tmp.size = 1;
    tmp.val[0] = _value;

    // add/update in pool
    addVecui(tmp);
  }

  /** \brief Set uvec2 uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, const ACG::Vec2ui &_value ) {
    // create uniform descriptor
    UniformVecui tmp;
    tmp.id = _name;
    tmp.size = 2;
    tmp.val[0] = _value[0];
    tmp.val[1] = _value[1];

    // add/update in pool
    addVecui(tmp);
  }
  
  /** \brief Set uvec3 uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, const ACG::Vec3ui &_value ) {
    // create uniform descriptor
    UniformVecui tmp;
    tmp.id = _name;
    tmp.size = 3;
    tmp.val[0] = _value[0];
    tmp.val[1] = _value[1];
    tmp.val[2] = _value[2];

    // add/update in pool
    addVecui(tmp);
  }  
  
  /** \brief Set uvec4 uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, const ACG::Vec4ui &_value ) {
    // create uniform descriptor
    UniformVecui tmp;
    tmp.id = _name;
    tmp.size = 4;
    tmp.val = _value;

    // add/update in pool
    addVecui(tmp);
  }


  /** \brief Set float uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, GLfloat _value ) {
    // create uniform descriptor
    UniformVecf tmp;
    tmp.id = _name;
    tmp.size = 1;
    tmp.val[0] = _value;

    // add/update in pool
    addVecf(tmp);
  }

  /** \brief Set vec2 uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, const ACG::Vec2f &_value ) {
    // create uniform descriptor
    UniformVecf tmp;
    tmp.id = _name;
    tmp.size = 2;
    tmp.val[0] = _value[0];
    tmp.val[1] = _value[1];

    // add/update in pool
    addVecf(tmp);
  }

  /** \brief Set vec3 uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, const ACG::Vec3f &_value ) {
    // create uniform descriptor
    UniformVecf tmp;
    tmp.id = _name;
    tmp.size = 3;
    tmp.val[0] = _value[0];
    tmp.val[1] = _value[1];
    tmp.val[2] = _value[2];

    // add/update in pool
    addVecf(tmp);
  }

  /** \brief Set vec4 uniform to specified value
  *
  * @param _name  Name of the uniform
  * @param _value New value of the uniform
  */
  void UniformPool::setUniform( const char *_name, const ACG::Vec4f &_value ) {
    // create uniform descriptor
    UniformVecf tmp;
    tmp.id = _name;
    tmp.size = 4;
    tmp.val = _value;

    // add/update in pool
    addVecf(tmp);
  }

  /** \brief Set 4x4fMatrix uniform to specified value
   *
   * @param _name  Name of the uniform
   * @param _value Matrix to be set
   * @param _transposed Is the matrix transposed?
   */
  void UniformPool::setUniform( const char *_name, const ACG::GLMatrixf &_value, bool _transposed ) {
    // create uniform descriptor
    UniformMat tmp;
    tmp.id = _name;
    tmp.transposed = _transposed;
    tmp.size = 4;
    tmp.val = _value;

    // add/update in pool
    addMatrix(tmp);
  }

  /** \brief Set 3x3fMatrix uniform to specified value
   *
   * @param _name  Name of the uniform
   * @param _value Matrix to be set
   * @param _transposed Is the matrix transposed?
   */
  void UniformPool::setUniformMat3( const char *_name, const ACG::GLMatrixf &_value, bool _transposed ) {
    // create uniform descriptor
    UniformMat tmp;
    tmp.id = _name;
    tmp.transposed = _transposed;
    tmp.size = 3;
    tmp.val = _value;

    // add/update in pool
    addMatrix(tmp);
  }

  /** \brief Set int array uniform to specified values
   *
   *  @param _name Name of the uniform to be set
   *  @param _values Pointer to an array with the new values
   *  @param _count Number of values in the given array
   */
  void UniformPool::setUniform( const char *_name, GLint *_values, int _count ) {
    // add/update in pool
    addBuf(_name, _values, _count, true);
  }

  /** \brief Set float array uniform to specified values
   *
   *  @param _name Name of the uniform to be set
   *  @param _values Pointer to an array with the new values
   *  @param _count Number of values in the given array
   */
  void UniformPool::setUniform( const char *_name, GLfloat *_values, int _count ) {
    // add/update in pool
    addBuf(_name, _values, _count, false);
  }

}


//==============================================================================
