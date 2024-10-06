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


#include <list>
#include <QDateTime>
#include <ACG/Config/ACGDefines.hh>
#include <ACG/GL/ShaderGenerator.hh>

// forward declaration
namespace GLSL
{
  class Program;
}


namespace ACG {


/** \brief Cache for shaders
 *
 *
 * This class provides a cache for shaders. You can query a GLSL::Program via getProgram
 * by specifying a ShaderGenDesc. If the program has already been compiled, it is returned
 * from the cache. Otherwise it will be compiled and linked, added to the cache and returned.
 * This ensures, that the shader is not compiled and linked every time, it is used.
 *
 * \note This class is a singleton
 *
*/
class ACGDLLEXPORT ShaderCache
{
public:

  /// Destructor
  virtual ~ShaderCache();

  /** \brief Return instance of the ShaderCache singleton
   *
   * @return Instance
   */
  static ShaderCache* getInstance();


  /** \brief Query a dynamically generated program from cache
   *
   * @param _desc  Shader description
   * @param _mods  Combination of active shader modifier ids
   * @return The program (Either from cache or newly compiled and linked)
   */
  GLSL::Program* getProgram(const ShaderGenDesc* _desc, const std::vector<unsigned int>& _mods);

    /** \brief Query a dynamically generated program from cache
   *
   * @param _desc  Shader description
   * @param _mods  Combination of active shader modifier ids
   * @return The program (Either from cache or newly compiled and linked)
   */
  GLSL::Program* getProgram(const ShaderGenDesc* _desc, const std::vector<unsigned int>* _mods)
  {
    return _mods ? getProgram(_desc, *_mods) : getProgram(_desc);
  }

  /** \brief Query a dynamically generated program from cache
   *
   * @param _desc  Shader description
   * @param _mods  Combination of active shader modifier ids
   * @return The program (Either from cache or newly compiled and linked)
   */
  GLSL::Program* getProgram(const ShaderGenDesc* _desc);


  /** \brief Query a static shader program from cache
   *
   * Can be used to load a shader and have external changes automatically applied by the timestamp watchdog.
   * However, the program should be queried every time before using the program as it may have been deleted and recompiled in the meantime.
   *
   * @param _vertexShaderFile relative (from shader directory) or absolute filename of vertex shader
   * @param _tessControlShaderFile relative (from shader directory) or absolute filename of tessellation control shader, null accepted
   * @param _tessEvalShaderFile relative (from shader directory) or absolute filename of tessellation eval shader, null accepted
   * @param _geometryShaderFile relative (from shader directory) or absolute filename of geometry shader, null accepted
   * @param _fragmentShaderFile relative (from shader directory) or absolute filename of fragment shader
   * @param _macros optional list of glsl preprocessor macros, which are added directly after the #version directive (example: "#define USE_METHOD 1", "#define METHOD_PARAM 0"...)
   * @param _verbose log or suppress error output
   * @return The program (Either from cache or newly compiled and linked)
   */
  GLSL::Program* getProgram(const char* _vertexShaderFile,
    const char* _tessControlShaderFile,
    const char* _tessEvalShaderFile,
    const char* _geometryShaderFile,
    const char* _fragmentShaderFile,
    QStringList* _macros = 0, bool _verbose = true);
  
  /** \brief Query a static shader program from cache
   *
   * Can be used to load a shader and have external changes automatically applied by the timestamp watchdog.
   * However, the program should be queried every time before using the program as it may have been deleted and recompiled in the meantime.
   *
   * @param _vertexShaderFile relative (from shader directory) or absolute filename of vertex shader
   * @param _fragmentShaderFile relative (from shader directory) or absolute filename of fragment shader
   * @param _macros optional list of glsl preprocessor macros, which are added directly after the #version directive (example: "#define USE_METHOD 1", "#define METHOD_PARAM 0"...)
   * @param _verbose log or suppress error output
   * @return The program (Either from cache or newly compiled and linked)
   */
  GLSL::Program* getProgram(const char* _vertexShaderFile, const char* _fragmentShaderFile, QStringList* _macros = 0, bool _verbose = true);


    /** \brief Query a static compute shader program from cache
   *
   * Can be used to load a shader and have external changes automatically applied by the timestamp watchdog.
   * However, the program should be queried every time before using the program as it may have been deleted and recompiled in the meantime.
   *
   * @param _computeShaderFile relative (from shader directory) or absolute filename of vertex shader
   * @param _macros optional list of glsl preprocessor macros, which are added directly after the #version directive (example: "#define USE_METHOD 1", "#define METHOD_PARAM 0"...)
   * @param _verbose log or suppress error output
   * @return The program (Either from cache or newly compiled and linked)
   */
  GLSL::Program* getComputeProgram(const char* _computeShaderFile, QStringList* _macros = 0, bool _verbose = true);
  


  /** \brief Delete all cached shaders
   */
  void clearCache();

  /** \brief enable or disable checking of the time step of each file
   *
   */
  void setTimeCheck(bool _on){timeCheck_ = _on;}
  bool getTimeCheck(){return timeCheck_;}


  /** \brief Enable debug output of generated shaders to specified directory
   */
  void setDebugOutputDir(const char* _outputDir);

protected:
  ShaderCache();

  struct CacheEntry
  {
    ShaderGenDesc desc;
    std::vector<unsigned int> mods;

    // string-pointer in ShaderGenDesc may not be permanent,
    // so copy string data here
    QString strVertexTemplate;
    QString strTessControlTemplate;
    QString strTessEvaluationTemplate;
    QString strGeometryTemplate;
    QString strFragmentTemplate;

    QDateTime vertexFileLastMod;
    QDateTime tessControlFileLastMod;
    QDateTime tessEvaluationFileLastMod;
    QDateTime geometryFileLastMod;
    QDateTime fragmentFileLastMod;

    QStringList macros;
  };

  /// \brief Returns true, if the shaders have the timestamp
  bool compareTimeStamp(const CacheEntry* _a, const CacheEntry* _b);

  /// \brief Returns true, if the shaders are not equal
  int compareShaderGenDescs(const CacheEntry* _a, const CacheEntry* _b);


  typedef std::list<std::pair<CacheEntry, GLSL::Program*> > CacheList;
  
  /// cache containing dynamic shaders from ShaderProgGenerator
  CacheList cache_;

  /// cache containing static shaders loaded from files (separate from dynamic cache to reduce access time)
  CacheList cacheStatic_;

  /// cache for static compute shaders
  CacheList cacheComputeShaders_;

  bool timeCheck_;

  /// output directory for shaders in dynamic cache
  QString dbgOutputDir_;
};



//=============================================================================
} // namespace ACG
//=============================================================================
