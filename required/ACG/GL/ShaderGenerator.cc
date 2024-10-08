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



#include <ACG/GL/acg_glew.hh>
#include "ShaderGenerator.hh"
#include <cstdio>
#include <iostream>

#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QTextStream>
#include <QDateTime>
#include <QRegularExpression>

namespace ACG
{


int ShaderProgGenerator::numRegisteredModifiers_ = 0;
std::vector<ShaderModifier*> ShaderProgGenerator::registeredModifiers_;



// space naming
// OS : object space
// VS : view space
// CS : clip space


ShaderGenerator::Keywords::Keywords()
// attribute request keywords
: macro_requestPosVS("#define SG_REQUEST_POSVS"),
macro_requestPosOS("#define SG_REQUEST_POSOS"),
macro_requestTexcoord("#define SG_REQUEST_TEXCOORD"),
macro_requestVertexColor("#define SG_REQUEST_VERTEXCOLOR"),
macro_requestNormalVS("#define SG_REQUEST_NORMALVS"),
macro_requestNormalOS("#define SG_REQUEST_NORMALOS"),

// generic default attribute input keywords
//  these are extended by the correct input name by the generator for each stage
macro_inputPosVS("SG_INPUT_POSVS"),
macro_inputPosOS("SG_INPUT_POSOS"),
macro_inputPosCS("SG_INPUT_POSCS"),
macro_inputNormalVS("SG_INPUT_NORMALVS"),
macro_inputNormalOS("SG_INPUT_NORMALOS"),
macro_inputTexcoord("SG_INPUT_TEXCOORD"),
macro_inputVertexColor("SG_INPUT_VERTEXCOLOR"),

macro_outputPosVS("SG_OUTPUT_POSVS"),
macro_outputPosOS("SG_OUTPUT_POSOS"),
macro_outputPosCS("SG_OUTPUT_POSCS"),
macro_outputNormalVS("SG_OUTPUT_NORMALVS"),
macro_outputNormalOS("SG_OUTPUT_NORMALOS"),
macro_outputTexcoord("SG_OUTPUT_TEXCOORD"),
macro_outputVertexColor("SG_OUTPUT_VERTEXCOLOR"),

ioPosCS("PosCS"),
ioPosOS("PosOS"),
ioPosVS("PosVS"),
ioNormalVS("NormalVS"),
ioNormalOS("NormalOS"),
ioTexcoord("TexCoord"),
ioColor("Color"),

vs_inputPrefix("in"),
vs_outputPrefix("outVertex"),
tcs_outputPrefix("outTc"),
tes_outputPrefix("outTe"),
gs_outputPrefix("outGeometry"),
fs_outputPrefix("outFragment"),

vs_inputPosition(vs_inputPrefix + "Position"),
vs_inputNormal(vs_inputPrefix + "Normal"),
vs_inputTexCoord(vs_inputPrefix + ioTexcoord),
vs_inputColor(vs_inputPrefix + ioColor),

vs_outputPosCS(vs_outputPrefix + ioPosCS),
vs_outputPosVS(vs_outputPrefix + ioPosVS),
vs_outputPosOS(vs_outputPrefix + ioPosOS),
vs_outputTexCoord(vs_outputPrefix + ioTexcoord),
vs_outputNormalVS(vs_outputPrefix + ioNormalVS),
vs_outputNormalOS(vs_outputPrefix + ioNormalOS),
vs_outputVertexColor(vs_outputPrefix + ioColor),
fs_outputFragmentColor(fs_outputPrefix)
{
}

const ShaderGenerator::Keywords ShaderGenerator::keywords;


ShaderGenerator::ShaderGenerator()
  : version_(150), inputArrays_(false), outputArrays_(false)
{
}

ShaderGenerator::~ShaderGenerator()
{

}


void ShaderGenerator::initVertexShaderIO(const ShaderGenDesc* _desc, const DefaultIODesc* _iodesc)
{
  // set type of IO
  inputArrays_ = false;
  outputArrays_ = false;
  inputPrefix_ = keywords.vs_inputPrefix;   // inputs: inPosition, inTexCoord...
  outputPrefix_ = keywords.vs_outputPrefix; // outputs: outVertexPosition, outVertexTexCoord..

  addInput("vec4", keywords.vs_inputPosition);
  addOutput("vec4", keywords.vs_outputPosCS);

  if (_iodesc->inputNormal_)
    addInput("vec3", keywords.vs_inputNormal);

  if (_desc->textured())
  {
    std::map<size_t,ShaderGenDesc::TextureType>::const_iterator iter = _desc->textureTypes().begin();

    /// TODO Setup for multiple texture coordinates as input
    if (iter->second.type == GL_TEXTURE_3D) {
      addInput("vec3", keywords.vs_inputTexCoord);
      addOutput("vec3", keywords.vs_outputTexCoord);
    } else {
      addInput("vec2", keywords.vs_inputTexCoord);
      addOutput("vec2", keywords.vs_outputTexCoord);
    }
  }

  if (_iodesc->inputColor_)
    addInput("vec4", keywords.vs_inputColor);

  if (_iodesc->passNormalVS_)
    addStringToList("vec3 " + keywords.vs_outputNormalVS, &outputs_, _desc->vertexNormalInterpolator + " out ", ";");

  if (_iodesc->passNormalOS_)
    addStringToList("vec3 " + keywords.vs_outputNormalOS, &outputs_, _desc->vertexNormalInterpolator + " out ", ";");

  // vertex color output

  if (_desc->vertexColorsInterpolator.isEmpty())
  {
    QString strColorOut;
    if (_desc->shadeMode == SG_SHADE_FLAT)
    {
      if (!_desc->geometryTemplateFile.isEmpty())
        strColorOut = keywords.vs_outputVertexColor;
      else
      {
        // Bypass the output setter, as we have to set that directly with the flat.
        addStringToList("vec4 " + keywords.vs_outputVertexColor, &outputs_, "flat out ", "; ");
      }
    }
    else if (_desc->shadeMode == SG_SHADE_GOURAUD || _desc->vertexColors || _iodesc->inputColor_)
      strColorOut = keywords.vs_outputVertexColor;

    if (strColorOut.size())
      addOutput("vec4", strColorOut);
  }
  else
    addStringToList("vec4 " + keywords.vs_outputVertexColor, &outputs_, _desc->vertexColorsInterpolator + " out ", ";");



  // handle other requests: normals, positions, texcoords

  if (_iodesc->passPosVS_)
    addOutput("vec4", keywords.vs_outputPosVS);

  if (_iodesc->passPosOS_)
    addOutput("vec4", keywords.vs_outputPosOS);

  if (_iodesc->passTexCoord_ && !_desc->textured())
  {
    // assume 2d texcoords as default
    int texdim = 2;

    if (_desc->texGenMode && _desc->texGenDim > 0 && _desc->texGenDim <= 4 && !_desc->texGenPerFragment)
      texdim = _desc->texGenDim;

    QString texcoordType;
    if (texdim > 1)
      texcoordType =QString("vec%1").arg(texdim);
    else
      texcoordType = "float";

    addInput(texcoordType, keywords.vs_inputTexCoord);
    addOutput(texcoordType, keywords.vs_outputTexCoord);
  }


  defineIOAbstraction(_iodesc, true, false);
}

void ShaderGenerator::initTessControlShaderIO(const ShaderGenDesc* _desc, ShaderGenerator* _prevStage, const DefaultIODesc* _iodesc) 
{
  // set type of IO
  inputArrays_ = true;
  outputArrays_ = true;
  inputPrefix_ = _prevStage->outputPrefix_;
  outputPrefix_ = keywords.tcs_outputPrefix; // outputs: outTcPosition, outTcTexCoord..

  matchInputs(_prevStage, true, inputPrefix_, outputPrefix_);

  defineIOAbstraction(_iodesc, false, false);
}

void ShaderGenerator::initTessEvalShaderIO(const ShaderGenDesc* _desc, ShaderGenerator* _prevStage, const DefaultIODesc* _iodesc) 
{
  // set type of IO
  inputArrays_ = true;
  outputArrays_ = false;
  inputPrefix_ = _prevStage->outputPrefix_;
  outputPrefix_ = keywords.tes_outputPrefix; // outputs: outTePosition, outTeTexCoord..

  matchInputs(_prevStage, true, inputPrefix_, outputPrefix_);

  defineIOAbstraction(_iodesc, false, false);
}

void ShaderGenerator::initGeometryShaderIO(const ShaderGenDesc* _desc, ShaderGenerator* _prevStage, const DefaultIODesc* _iodesc) 
{
  // set type of IO
  inputArrays_ = true;
  outputArrays_ = false;
  inputPrefix_ = _prevStage->outputPrefix_;
  outputPrefix_ = keywords.gs_outputPrefix; // outputs: outGeometryPosition, outGeometryTexCoord..

  matchInputs(_prevStage, true, inputPrefix_, outputPrefix_);

  defineIOAbstraction(_iodesc, false, false);
}



void ShaderGenerator::initFragmentShaderIO(const ShaderGenDesc* _desc, ShaderGenerator* _prevStage, const DefaultIODesc* _iodesc)
{
  // set type of IO
  inputArrays_ = false;
  outputArrays_ = false;
  inputPrefix_ = _prevStage->outputPrefix_;
  outputPrefix_ = keywords.fs_outputPrefix;

  matchInputs(_prevStage, false);
  addOutput("vec4", keywords.fs_outputFragmentColor);

  defineIOAbstraction(_iodesc, false, true);
}


void ShaderGenerator::defineIOAbstraction( const DefaultIODesc* _iodesc, bool _vs, bool _fs )
{
  if (_vs)
  {
    // input name abstraction

    addIODefine(keywords.macro_inputPosOS, keywords.vs_inputPosition);

    if (_iodesc->inputTexCoord_)
      addIODefine(keywords.macro_inputTexcoord, keywords.vs_inputTexCoord);

    if (_iodesc->inputNormal_)
      addIODefine(keywords.macro_inputNormalOS, keywords.vs_inputNormal);

    if (_iodesc->inputColor_)
      addIODefine(keywords.macro_inputVertexColor, keywords.vs_inputColor);



    // output name abstraction

    addIODefine(keywords.macro_outputPosCS, keywords.vs_outputPosCS);

    if (_iodesc->passPosVS_)
      addIODefine(keywords.macro_outputPosVS, keywords.vs_outputPosVS);

    if (_iodesc->passPosOS_)
      addIODefine(keywords.macro_outputPosOS, keywords.vs_outputPosOS);

    if (_iodesc->passTexCoord_)
      addIODefine(keywords.macro_outputTexcoord, keywords.vs_outputTexCoord);

    if (_iodesc->passNormalVS_)
      addIODefine(keywords.macro_outputNormalVS, keywords.vs_outputNormalVS);

    if (_iodesc->passNormalOS_)
      addIODefine(keywords.macro_outputNormalOS, keywords.vs_outputNormalOS);

    if (_iodesc->passColor_)
      addIODefine(keywords.macro_outputVertexColor, keywords.vs_outputVertexColor);
  }
  else
  {
    if (_iodesc->passPosVS_)
    {
      addIODefine(keywords.macro_inputPosVS, inputPrefix_ + keywords.ioPosVS);
      if (!_fs)
        addIODefine(keywords.macro_outputPosVS, outputPrefix_ + keywords.ioPosVS);
    }

    if (_iodesc->passPosOS_)
    {
      addIODefine(keywords.macro_inputPosOS, inputPrefix_ + keywords.ioPosOS);
      if (!_fs)
        addIODefine(keywords.macro_outputPosOS, outputPrefix_ + keywords.ioPosOS);
    }

    addIODefine(keywords.macro_inputPosCS, inputPrefix_ + keywords.ioPosCS);
    if (!_fs)
      addIODefine(keywords.macro_outputPosCS, outputPrefix_ + keywords.ioPosCS);

    if (_iodesc->passNormalVS_)
    {
      addIODefine(keywords.macro_inputNormalVS, inputPrefix_ + keywords.ioNormalVS);
      if (!_fs)
        addIODefine(keywords.macro_outputNormalVS, outputPrefix_ + keywords.ioNormalVS);
    }

    if (_iodesc->passNormalOS_)
    {
      addIODefine(keywords.macro_inputNormalOS, inputPrefix_ + keywords.ioNormalOS);
      if (!_fs)
        addIODefine(keywords.macro_outputNormalOS, outputPrefix_ + keywords.ioNormalOS);
    }

    if (_iodesc->passTexCoord_)
    {
      addIODefine(keywords.macro_inputTexcoord, inputPrefix_ + keywords.ioTexcoord);
      if (!_fs)
        addIODefine(keywords.macro_outputTexcoord, outputPrefix_ + keywords.ioTexcoord);
    }

    if (_iodesc->passColor_)
    {
      addIODefine(keywords.macro_inputVertexColor, inputPrefix_ + keywords.ioColor);
      if (!_fs)
        addIODefine(keywords.macro_outputVertexColor, outputPrefix_ + keywords.ioColor);
    }
  }

  
}



void ShaderGenerator::initDefaultUniforms()
{
  addUniform("mat4 g_mWVP" , "  // Projection * Modelview");       // Transforms directly from Object space to NDC
  addUniform("mat4 g_mWV" , "   // Modelview matrix");             // Modelview transforms from Object to World to View coordinates
  addUniform("mat3 g_mWVIT" , " // Modelview inverse transposed"); // Modelview inverse transposed, transforms from view across World into Object coordinates
  addUniform("mat4 g_mP", "     // Projection matrix");            // Projection Matrix

  addUniform("vec3 g_vCamPos");
  addUniform("vec3 g_vCamDir");

  addUniform("vec3 g_cDiffuse");
  addUniform("vec3 g_cAmbient");
  addUniform("vec3 g_cEmissive");
  addUniform("vec3 g_cSpecular");
  addUniform("vec4 g_vMaterial");
  addUniform("vec3 g_cLightModelAmbient");
}


#define ADDLIGHT(x) addUniform( QString( QString(x) + "_%1").arg(lightIndex_)  )

void ShaderGenerator::addLight(int lightIndex_, ShaderGenLightType _light)
{
  QString sz;

  QTextStream stream(&sz);

  ADDLIGHT("vec3 g_cLightDiffuse");
  ADDLIGHT("vec3 g_cLightAmbient");
  ADDLIGHT("vec3 g_cLightSpecular");

  if (_light == SG_LIGHT_POINT ||
    _light == SG_LIGHT_SPOT)
  {
    ADDLIGHT("vec3 g_vLightPos");
    ADDLIGHT("vec3 g_vLightAtten");
  }

  if (_light == SG_LIGHT_DIRECTIONAL ||
    _light == SG_LIGHT_SPOT)
    ADDLIGHT("vec3 g_vLightDir");


  if (_light == SG_LIGHT_SPOT)
    ADDLIGHT("vec2 g_vLightAngleExp");
}



void ShaderGenerator::addStringToList(QString _str, 
                                      QStringList* _arr,
                                      QString _prefix,
                                      QString _postfix)
{
  // Construct the whole string
  QString tmp = _str;

  if (!_str.startsWith(_prefix))
    tmp = _prefix + tmp;

  if (!_str.endsWith(_postfix))
     tmp += _postfix;

  // normalize string
  //  remove tabs, double whitespace
  tmp = tmp.simplified();

  // avoid duplicates
  if (!_arr->contains(tmp))
    _arr->push_back(tmp);

}


void ShaderGenerator::addInput(const QString& _input)
{
  addStringToList(_input, &inputs_, "in ", ";");
}


void ShaderGenerator::addOutput(const QString& _output)
{
  addStringToList(_output, &outputs_, "out ", ";");
}


void ShaderGenerator::addDefine(const QString& _def)
{
  addStringToList(_def, &genDefines_, "#define ");
}


void ShaderGenerator::addIODefine(const QString& _macroName, const QString& _resolvedName)
{
  addDefine(_macroName + QString(" ") + _resolvedName);
}

void ShaderGenerator::addMacros(const QStringList& _macros)
{
  // prepend macros to the "defines" list

  // QStringList reverse_iterator:
  typedef std::reverse_iterator<QStringList::const_iterator> QStringListReverseIterator;
  QStringListReverseIterator rbegin( _macros.end() ), rend( _macros.begin() );

  for (QStringListReverseIterator it = rbegin; it != rend; ++it)
    genDefines_.push_front(*it);
}

bool ShaderGenerator::hasDefine(QString _define) const
{
  if (genDefines_.contains(_define))
    return true;

  // check trimmed strings and with startsWith()

  QString trimmedDef = _define.trimmed();

  for (QStringList::const_iterator it = genDefines_.constBegin(); it != genDefines_.constEnd(); ++it)
  {
    QString trimmedRef = it->trimmed();

    if (trimmedRef.startsWith(trimmedDef))
      return true;
  }

  // also check raw io blocks
  for (QStringList::const_iterator it = rawIO_.constBegin(); it != rawIO_.constEnd(); ++it)
  {
    QString trimmedRef = it->trimmed();

    if (trimmedRef.startsWith(trimmedDef))
      return true;
  }

  return false;
}

void ShaderGenerator::addLayout(QString _def)
{
  addStringToList(_def, &layouts_);
}


void ShaderGenerator::addUniform(QString _uniform, QString _comment)
{
  QString prefix = "";
  if (!_uniform.startsWith("uniform ") && !_uniform.contains(" uniform "))
    prefix = "uniform ";

  addStringToList(_uniform, &uniforms_, prefix, "; " + _comment );
}



void ShaderGenerator::addIOToCode(const QStringList& _cmds)
{
  QString it;
  foreach(it, _cmds)
  {
    code_.push_back(it);
    // append ; eventually

    if (!it.contains(";"))
      code_.back().append(";");
  }
}



void ShaderGenerator::buildShaderCode(QStringList* _pMainCode, const QStringList& _defaultLightingFunctions)
{
  QString glslversion = QString("#version %1").arg(version_);

  code_.push_back(glslversion);

  // provide defines
  QString it;

  foreach(it, genDefines_)
    code_.push_back(it);

  // layouts
  foreach(it, layouts_)
    code_.push_back(it);

  // IO
  addIOToCode(inputs_);
  addIOToCode(outputs_);
  addIOToCode(uniforms_);

  // eventually attach lighting functions if required
  bool requiresLightingCode = false;

  // search for references in imports
  foreach(it, imports_)
  {
    if (it.contains("LitDirLight") || it.contains("LitPointLight") || it.contains("LitSpotLight"))
      requiresLightingCode = true;
  }

  if (requiresLightingCode)
  {
    foreach(it, _defaultLightingFunctions)
      code_.push_back(it);
  }

  // provide imports
  foreach(it, imports_)
    code_.push_back(it);


  // search for lighting references in main code

  if (!requiresLightingCode)
  {
    foreach(it, (*_pMainCode))
    {
      if (it.contains("LitDirLight") || it.contains("LitPointLight") || it.contains("LitSpotLight"))
        requiresLightingCode = true;
    }

    if (requiresLightingCode)
    {
      foreach(it, _defaultLightingFunctions)
        code_.push_back(it);
    }
  }


  // add raw IO code block
  code_.append(rawIO_);


  // main function
  foreach(it, (*_pMainCode))
    code_.push_back(it);
}



void ShaderGenerator::addIncludeFile(QString _fileName)
{
  QFile file(_fileName);

  if (file.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    QTextStream fileStream(&file);
    
    // track source of include files in shader comment
    
    imports_.push_back("// ==============================================================================");
    imports_.push_back(QString("// ShaderGenerator - begin of imported file: ") + _fileName);
    

    while (!fileStream.atEnd())
    {
      QString tmpLine = fileStream.readLine();

      imports_.push_back(tmpLine.simplified());
    }
    
    
    // mark end of include file in comment
    
    imports_.push_back(QString("// ShaderGenerator - end of imported file #include \"") + _fileName);
    imports_.push_back("// ==============================================================================");
    
  }

}



void ShaderGenerator::saveToFile(const char* _fileName)
{
  QFile file(_fileName);
  if (file.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    QTextStream fileStream(&file);

    QString it;
    foreach(it, code_)
      fileStream << it << '\n';
  }
}



const QStringList& ShaderGenerator::getShaderCode()
{
  return code_;
}

void ShaderGenerator::setGLSLVersion( int _version )
{
  version_ = _version;
}

void ShaderGenerator::matchInputs(const ShaderGenerator* _previousShaderStage,
  bool _passToNextStage,
  QString _inputPrefix, 
  QString _outputPrefix)
{
  if (!_previousShaderStage)
  {
    std::cout << "error: ShaderGenerator::matchInputs called without providing input stage" << std::endl;
    return;
  }

  QString it;
  foreach(it, _previousShaderStage->outputs_)
  {
    QString input = it;

    QString outKeyword = "out ";
    QString inKeyword = "in  ";

    // replace first occurrence of "out" with "in"
    input.replace(input.indexOf(outKeyword), outKeyword.size(), inKeyword);

    // special case for array IO

    if (inputArrays_ && !_previousShaderStage->outputArrays_)
    {
      int lastNameChar = input.lastIndexOf(QRegularExpression("[a-zA-Z0-9]"));
      input.insert(lastNameChar+1, "[]");
//      input.insert(lastNameChar+1, "[gl_in.length()]");
    }


    // add to input list with duplicate check
    addStringToList(input, &inputs_);

    if (_passToNextStage)
    {
      // replace prefixes of in/outputs to avoid name collision

      QString output = input;
      output.replace(output.indexOf(_inputPrefix), _inputPrefix.size(), _outputPrefix);
      output.replace(output.indexOf(inKeyword), inKeyword.size(), outKeyword);

      // take care of arrays
      if (inputArrays_ && !outputArrays_)
      {
        int bracketStart = output.indexOf("[");
        int bracketEnd = output.indexOf("]");
        output.remove(bracketStart, bracketEnd-bracketStart+1);
      }
      else if (!inputArrays_ && outputArrays_)
      {
        int lastNameChar = output.lastIndexOf(QRegularExpression("[a-zA-Z0-9]"));
        output.insert(lastNameChar+1, "[]");
//        output.insert(lastNameChar+1, "[gl_in.length()]");
      }


      // add to output list with duplicate check
      addStringToList(output, &outputs_);
    }
  }
}

int ShaderGenerator::getNumOutputs() const
{
  return outputs_.size();
}

QString ShaderGenerator::getOutputName(int _id) const
{
  QString output = outputs_.at(_id);

  output.remove(";");
  output.remove("out ");

  int bracketStart = output.indexOf("[");
  int bracketEnd = output.lastIndexOf("]");

  if (bracketStart >= 0)
    output.remove(bracketStart, bracketEnd-bracketStart+1);

  // decompose output declaration
  QStringList decompOutput = output.split(" ");
  return decompOutput.last();
}

int ShaderGenerator::getNumInputs() const
{
  return inputs_.size();
}

QString ShaderGenerator::getInputName(int _id) const
{
  QString input = inputs_.at(_id);

  input.remove(";");
  input.remove("out ");
  
  int bracketStart = input.indexOf("[");
  int bracketEnd = input.lastIndexOf("]");

  if (bracketStart >= 0)
    input.remove(bracketStart, bracketEnd-bracketStart+1);

  // decompose output declaration
  QStringList decompInput = input.split(" ");
  return decompInput.last();
}

QString ShaderGenerator::getIOMapName(int _inId) const
{
  QString inputName = getInputName(_inId);

  // output name = input name with swapped prefix
  QString outputName = inputName;
  outputName.replace(outputName.indexOf(inputPrefix_), inputPrefix_.size(), outputPrefix_);

  return outputName;
}


ShaderGenerator::DefaultIODesc::DefaultIODesc()
  : inputTexCoord_(false),
  inputColor_(false),
  inputNormal_(false),
  passPosVS_(false), passPosOS_(false), 
  passTexCoord_(false), 
  passColor_(false),
  passNormalVS_(false), passNormalOS_(false)
{
}




QString ShaderProgGenerator::shaderDir_;
QStringList ShaderProgGenerator::lightingCode_;


ShaderProgGenerator::ShaderProgGenerator( const ShaderGenDesc* _desc )
  : vertex_(0), tessControl_(0), tessEval_(0), geometry_(0), fragment_(0)
{
  init(_desc, (ShaderModifier**)0, 0);
}

ShaderProgGenerator::ShaderProgGenerator( const ShaderGenDesc* _desc, const unsigned int* _modifierIDs, unsigned int _numActiveMods )
  : vertex_(0), tessControl_(0), tessEval_(0), geometry_(0), fragment_(0)
{
  init(_desc, _modifierIDs, _numActiveMods);
}

ShaderProgGenerator::ShaderProgGenerator(const ShaderGenDesc* _desc, const std::vector<unsigned int>& _modifierIDs)
  : vertex_(0), tessControl_(0), tessEval_(0), geometry_(0), fragment_(0)
{
  init(_desc, _modifierIDs.empty() ? 0 : &_modifierIDs[0], (unsigned int)_modifierIDs.size());
}

ShaderProgGenerator::ShaderProgGenerator(const ShaderGenDesc* _desc, const std::vector<unsigned int>* _modifierIDs)
  : vertex_(0), tessControl_(0), tessEval_(0), geometry_(0), fragment_(0)
{
  unsigned int numMods = !_modifierIDs || _modifierIDs->empty() ? 0 : (unsigned int)_modifierIDs->size();
  init(_desc, numMods ? &((*_modifierIDs)[0]) : 0, numMods);
}

ShaderProgGenerator::ShaderProgGenerator(const ShaderGenDesc* _desc, ShaderModifier* const* _modifiers, unsigned int _numActiveMods)
  : vertex_(0), tessControl_(0), tessEval_(0), geometry_(0), fragment_(0)
{
  init(_desc, _modifiers, _numActiveMods);
}

ShaderProgGenerator::ShaderProgGenerator(const ShaderGenDesc* _desc, const std::vector<ShaderModifier*>& _modifierIDs)
  : vertex_(0), tessControl_(0), tessEval_(0), geometry_(0), fragment_(0)
{
  init(_desc, _modifierIDs.empty() ? 0 : &(_modifierIDs[0]), (unsigned int)_modifierIDs.size());
}

ShaderProgGenerator::ShaderProgGenerator(const ShaderGenDesc* _desc, const std::vector<ShaderModifier*>* _modifierIDs)
  : vertex_(0), tessControl_(0), tessEval_(0), geometry_(0), fragment_(0)
{
  unsigned int numMods = !_modifierIDs || _modifierIDs->empty() ? 0 : (unsigned int)_modifierIDs->size();
  init(_desc, numMods ? &((*_modifierIDs)[0]) : 0, numMods);
}


void ShaderProgGenerator::init( const ShaderGenDesc* _desc, const unsigned int* _modifierIDs, unsigned int _numActiveMods )
{
  if (_modifierIDs && _numActiveMods)
  {
    activeMods_.resize(_numActiveMods);

    for (unsigned int i = 0; i < _numActiveMods; ++i)
      activeMods_[i] = registeredModifiers_[ _modifierIDs[i] ];
  }

  init(_desc, (ShaderModifier**)0, 0);
}

void ShaderProgGenerator::init( const ShaderGenDesc* _desc, ShaderModifier* const* _modifiers, unsigned int _numActiveMods )
{
  // mods provided by renderer are passed via parameters _modifiers, _numActiveMods
  // mods provided by individual render objects are passed via ShaderGenDesc* _desc
  // combine them
  size_t numDescMods = _desc->shaderMods.size();
  size_t numTotalMods = _numActiveMods + numDescMods;
  if (numTotalMods)
  {
    activeMods_.resize(numTotalMods);

    for (size_t i = 0; i < numDescMods; ++i)
    {
      unsigned int modID = _desc->shaderMods[i];
      activeMods_[i] = registeredModifiers_[modID];
    }

    if (_modifiers && _numActiveMods)
    {
      for (unsigned int i = 0; i < _numActiveMods; ++i)
        activeMods_[i + numDescMods] = _modifiers[i];
    }
  }




  if (shaderDir_.isEmpty())
    std::cout << "error: call ShaderProgGenerator::setShaderDir() first!" << std::endl;
  else
  {
    desc_ = *_desc;

    // We need at least version 3.2 or higher to support geometry shaders
    if ( !ACG::openGLVersionTest(3,2) )
    {
      if (!desc_.geometryTemplateFile.isEmpty())
        std::cerr << "Warning: removing geometry shader from ShaderDesc" << std::endl;

      desc_.geometryTemplateFile.clear();
    }

    // We need at least version 4.0 or higher to support tessellation
    if ( !ACG::openGLVersionTest(4, 0) )
    {
      if (!desc_.tessControlTemplateFile.isEmpty() || !desc_.tessEvaluationTemplateFile.isEmpty())
        std::cerr << "Warning: removing tessellation shader from ShaderDesc" << std::endl;

      desc_.tessControlTemplateFile.clear();
      desc_.tessEvaluationTemplateFile.clear();
    }

    // adjust glsl version to requirement

    if (!desc_.geometryTemplateFile.isEmpty())
      desc_.version = std::max(desc_.version, 150);

    if (!desc_.tessControlTemplateFile.isEmpty() || !desc_.tessEvaluationTemplateFile.isEmpty())
      desc_.version = std::max(desc_.version, 400);


    loadLightingFunctions();

    generateShaders();
  }
}


ShaderProgGenerator::~ShaderProgGenerator(void)
{
  delete vertex_;
  delete fragment_;
  delete geometry_;
  delete tessControl_;
  delete tessEval_;
}



bool ShaderProgGenerator::loadStringListFromFile(QString _fileName, QStringList* _out)
{
  bool success = false;

  QString absFilename = getAbsFilePath(_fileName);


  QFile file(absFilename);

  if (file.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    if (!file.isReadable())
      std::cout << "error: unreadable file -> \"" << absFilename.toStdString() << "\"" << std::endl;
    else
    {
      QTextStream filestream(&file);

      while (!filestream.atEnd())
      {
        QString szLine = filestream.readLine();
        _out->push_back(szLine.trimmed());
      }

      success = true;
    }

    file.close();
  }
  else
    std::cout << "error: " << file.errorString().toStdString() << " -> \"" << absFilename.toStdString() << "\"" << std::endl;

  return success;
}


void ShaderProgGenerator::loadLightingFunctions()
{
  if (lightingCode_.size()) return;
  
  static const QString lightingCodeFile = "ShaderGen/SG_LIGHTING.GLSL";

  QString fileName = shaderDir_ + QDir::separator() + QString(lightingCodeFile);

  lightingCode_.push_back("// ==============================================================================");
  lightingCode_.push_back(QString("// ShaderGenerator - default lighting functions imported from file: ") + fileName);
  
  
  // load shader code from file
  loadStringListFromFile(fileName, &lightingCode_);
  
  lightingCode_.push_back(QString("// ShaderGenerator - end of default lighting functions"));
  lightingCode_.push_back("// ==============================================================================");
}



void ShaderProgGenerator::initGenDefines(ShaderGenerator* _gen)
{
  switch (desc_.shadeMode)
  {
  case SG_SHADE_GOURAUD:
    _gen->addDefine("SG_GOURAUD 1"); break;
  case SG_SHADE_FLAT:
    _gen->addDefine("SG_FLAT 1"); break;
  case SG_SHADE_UNLIT:
    _gen->addDefine("SG_UNLIT 1"); break;
  case SG_SHADE_PHONG:
    _gen->addDefine("SG_PHONG 1"); break;

  default:
    std::cout << __FUNCTION__ << " -> unknown shade mode: " << desc_.shadeMode << std::endl;
  }

  if (desc_.twoSidedLighting)
    _gen->addDefine("TWO_SIDED_LIGHTING 1");

  if (desc_.textured())
    _gen->addDefine("SG_TEXTURED 1");

  if (desc_.vertexColors)
    _gen->addDefine("SG_VERTEX_COLOR 1");

//  if (desc_.shadeMode != SG_SHADE_UNLIT)
  if (ioDesc_.passNormalVS_)
    _gen->addDefine("SG_NORMALS 1");

  if (ioDesc_.passPosVS_)
    _gen->addDefine("SG_POSVS 1");

  if (ioDesc_.passPosOS_)
    _gen->addDefine("SG_POSOS 1");

  // # lights define
  QString strNumLights = QString("SG_NUM_LIGHTS %1").arg(desc_.numLights);
  _gen->addDefine(strNumLights);

  // light types define
  const char* lightTypeNames[] = {"SG_LIGHT_DIRECTIONAL",
    "SG_LIGHT_POINT", "SG_LIGHT_SPOT"};

  for (int i = 0; i < 3; ++i)
    _gen->addDefine(lightTypeNames[i]);


  for (int i = 0; i < desc_.numLights; ++i) {
    _gen->addDefine( QString ("SG_LIGHT_TYPE_%1 %2").arg(i).arg(lightTypeNames[desc_.lightTypes[i]]) );
  }

  _gen->addDefine("SG_ALPHA g_vMaterial.y");
  _gen->addDefine("SG_MINALPHA g_vMaterial.z");


  _gen->addMacros(desc_.macros);
}




void ShaderProgGenerator::buildVertexShader()
{
  delete vertex_;

  vertex_  = new ShaderGenerator();
  vertex_->setGLSLVersion(desc_.version);

  vertex_->initVertexShaderIO(&desc_, &ioDesc_);

  vertex_->initDefaultUniforms();


  if (desc_.texGenDim && (desc_.texGenMode == GL_OBJECT_LINEAR || desc_.texGenMode == GL_EYE_LINEAR) && !desc_.texGenPerFragment)
  {
    // application has to provide texture projection planes
    QString uniformDecl = "vec4 g_vTexGenPlane";
    if (desc_.texGenDim > 1)
      uniformDecl += "[" + QString::number(desc_.texGenDim) + "]";
    vertex_->addUniform(uniformDecl, " // texture projection planes");
  }


  // apply i/o modifiers
  for (size_t i = 0; i < activeMods_.size(); ++i)
    activeMods_[i]->modifyVertexIO(vertex_);


  initGenDefines(vertex_);



  // IO

  // when to use vertex lights
  if (desc_.shadeMode == SG_SHADE_GOURAUD ||
    desc_.shadeMode == SG_SHADE_FLAT)
  {
    for (int i = 0; i < desc_.numLights; ++i)
      vertex_->addLight(i, desc_.lightTypes[i]);
  }


  // assemble main function
  QStringList mainCode;

  if (!vertexTemplate_.size())
  {
    mainCode.push_back("void main()");
    mainCode.push_back("{");

    addVertexBeginCode(&mainCode);
    addVertexEndCode(&mainCode);

    mainCode.push_back("}");
  }
  else
  {
    // interpret loaded shader template:
    //  import #includes and replace SG_VERTEX_BEGIN/END markers

    QString it;
    foreach(it,vertexTemplate_)
    {
      if (!checkForIncludes(it, vertex_, getPathName(vertexShaderFile_)))
      {
        // str line is no include directive
        // check for SG_ markers

        if (it.contains("SG_VERTEX_BEGIN"))
          addVertexBeginCode(&mainCode);
        else
        {
          if (it.contains("SG_VERTEX_END"))
            addVertexEndCode(&mainCode);
          else
          {
            // no SG marker
            mainCode.push_back(it);
          }
        }

      }
    }

  }

  vertex_->buildShaderCode(&mainCode, lightingCode_);

}


void ShaderProgGenerator::addVertexBeginCode(QStringList* _code)
{
  // size in pixel of rendered point-lists, set by user via uniform

  _code->push_back(QString("vec4 sg_vPosPS = g_mWVP * ") + ShaderGenerator::keywords.macro_inputPosOS + QString(";"));
  _code->push_back("vec4 sg_vPosVS = g_mWV * inPosition;");
  _code->push_back("vec3 sg_vNormalVS = vec3(0.0, 1.0, 0.0);");
  _code->push_back("vec3 sg_vNormalOS = vec3(0.0, 1.0, 0.0);");

  if (desc_.vertexColors && (desc_.colorMaterialMode == GL_AMBIENT || desc_.colorMaterialMode == GL_AMBIENT_AND_DIFFUSE))
    _code->push_back(QString("vec4 sg_cColor = vec4(g_cEmissive + g_cLightModelAmbient * ")
                     + ShaderGenerator::keywords.macro_inputVertexColor
                     + QString(".rgb, SG_ALPHA * ")
                     + ShaderGenerator::keywords.macro_inputVertexColor
                     + QString(".a);"));
  else
    _code->push_back("vec4 sg_cColor = vec4(g_cEmissive + g_cLightModelAmbient * g_cAmbient, SG_ALPHA);");

  if (ioDesc_.inputNormal_)
  {
    _code->push_back("sg_vNormalVS = normalize(g_mWVIT * inNormal);");
    _code->push_back("sg_vNormalOS = normalize(inNormal);");
  }

  if (ioDesc_.inputColor_ && (desc_.shadeMode == SG_SHADE_UNLIT || desc_.colorMaterialMode == GL_EMISSION))
    _code->push_back(QString("sg_cColor = ") + ShaderGenerator::keywords.macro_inputVertexColor + QString(";"));

  // texcoord generation
  addTexGenCode(_code, false);


  // apply modifiers
  for (size_t i = 0; i < activeMods_.size(); ++i)
    activeMods_[i]->modifyVertexBeginCode(_code);
}


void ShaderProgGenerator::addVertexEndCode(QStringList* _code)
{
  if (desc_.shadeMode == SG_SHADE_GOURAUD ||
    desc_.shadeMode == SG_SHADE_FLAT)
  {
    // add lighting code here

    addLightingCode(_code);
  }

  _code->push_back("gl_Position = sg_vPosPS;");
  _code->push_back("outVertexPosCS = sg_vPosPS;");

  if (ioDesc_.passTexCoord_)
    _code->push_back("outVertexTexCoord = sg_vTexCoord;");

  if (ioDesc_.passColor_)
    _code->push_back("outVertexColor = sg_cColor;");

  if (ioDesc_.passNormalVS_)
    _code->push_back(ShaderGenerator::keywords.macro_outputNormalVS + QString(" = sg_vNormalVS;"));

  if (ioDesc_.passNormalOS_)
    _code->push_back(ShaderGenerator::keywords.macro_outputNormalOS + QString(" = sg_vNormalOS;"));

  if (ioDesc_.passPosVS_)
    _code->push_back(ShaderGenerator::keywords.macro_outputPosVS + QString(" = sg_vPosVS;"));

  if (ioDesc_.passPosOS_)
    _code->push_back(ShaderGenerator::keywords.macro_outputPosOS + QString(" = ") + ShaderGenerator::keywords.macro_inputPosOS + QString(";"));



  // apply modifiers
  for (size_t i = 0; i < activeMods_.size(); ++i)
    activeMods_[i]->modifyVertexEndCode(_code);
}


int ShaderProgGenerator::checkForIncludes(QString _str, ShaderGenerator* _gen, QString _includePath)
{
  if (_str.contains("#include "))
  {
    QString strIncludeFile = _str.remove("#include ").remove('\"').remove('<').remove('>').trimmed();

    if (strIncludeFile.isEmpty())
      std::cout << "wrong include syntax: " << _str.toStdString() << std::endl;
    else
    {
      QString fullPathToIncludeFile = _includePath + QDir::separator() + strIncludeFile;

      _gen->addIncludeFile(fullPathToIncludeFile);
    }

    return 1;
  }

  return 0;
}

int ShaderProgGenerator::checkForIncludes(QString _str, QStringList* _outImport, QString _includePath)
{
  if (_str.contains("#include "))
  {
    QString strIncludeFile = _str.remove("#include ").remove('\"').remove('<').remove('>').trimmed();

    if (strIncludeFile.isEmpty())
      std::cout << "wrong include syntax: " << _str.toStdString() << std::endl;
    else
    {
      QString fullPathToIncludeFile = _includePath + QDir::separator() + strIncludeFile;

      // unify separator chars
      fullPathToIncludeFile.replace('\\', '/');

      // get rid of ".." usage inside shader includes
      QString cleanFilepath = QDir::cleanPath(fullPathToIncludeFile);

      loadStringListFromFile(cleanFilepath, _outImport);
    }

    return 1;
  }

  return 0;
}

void ShaderProgGenerator::buildTessControlShader()
{
  // Only build a tess-control shader if enabled
  if ( desc_.tessControlTemplateFile.isEmpty() )
    return;

  // the generator provides an IO mapping function and adds default uniforms to this stage
  // - template is necessary
  // - combination/modification of tess-control shader is not supported
  // - template may call sg_MapIO(inId) somewhere in code to take care of default IO pass-through
  //         this function reads elements from gl_in[inID] and writes them to elements of gl_out[gl_InvocationID]
  //         inId can be gl_InvocationID if the patch size is not modified

  delete tessControl_;

  tessControl_  = new ShaderGenerator();
  tessControl_->setGLSLVersion(desc_.version);

  QString it;
  foreach(it, tessControlLayout_)
    tessControl_->addLayout(it);

  // find previous shader stage
  ShaderGenerator* prevStage = vertex_;

  tessControl_->initTessControlShaderIO(&desc_, prevStage, &ioDesc_);

  tessControl_->initDefaultUniforms();


  // apply i/o modifiers
  for (size_t i = 0; i < activeMods_.size(); ++i)
    activeMods_[i]->modifyTessControlIO(tessControl_);

  initGenDefines(tessControl_);



  // assemble main function
  QStringList mainCode; 

  // add simple io passthrough mapper

  {
      // Write function as macro so that compiler knows there is no index indirection (thx AMD)
      mainCode.push_back("#define sg_MapIO(inIdx) do {\\");

    // built-in IO
    mainCode.push_back("gl_out[gl_InvocationID].gl_Position = gl_in[inIdx].gl_Position;\\");
    
    // custom IO
    for (int i = 0; i < tessControl_->getNumInputs(); ++i)
    {
      QString inputName = tessControl_->getInputName(i);
      QString outputName = tessControl_->getIOMapName(i);

      QString outputAssignCode = outputName + QString("[gl_InvocationID] = ") + inputName + QString("[inIdx];\\");

      mainCode.push_back(outputAssignCode);
    }

    // Enforce semicolon when using macro
    mainCode.push_back("} while(false)");
  }


  // interpret loaded shader template:
  //  import #includes
  foreach(it,tessControlTemplate_)
  {
    if (!checkForIncludes(it, tessControl_, getPathName(tessControlShaderFile_)))
    {
      // str line is no include directive
      mainCode.push_back(it);
    }
  }

  tessControl_->buildShaderCode(&mainCode, lightingCode_);
}

void ShaderProgGenerator::buildTessEvalShader()
{
  // Only build a tess-eval shader if enabled
  if ( desc_.tessEvaluationTemplateFile.isEmpty() )
    return;

  // the generator provides default interpolation functions and adds default uniforms to this stage
  // - template is necessary
  // - combination/modification of tess-eval shader is not supported
  // - template may call sg_MapIOBarycentric() or sg_MapIOBilinear() somewhere in code to take care of default IO pass-through
  //        - barycentric interpolation can be used for triangle patches
  //        - bilinear interpolation can be used for quad patches
  //        - other interpolation schemes have to be user defined

  delete tessEval_;

  tessEval_  = new ShaderGenerator();
  tessEval_->setGLSLVersion(desc_.version);


  // find previous shader stage
  ShaderGenerator* prevStage = tessControl_;

  if (!prevStage)
    prevStage = vertex_;

  tessEval_->initTessEvalShaderIO(&desc_, prevStage, &ioDesc_);

  tessEval_->initDefaultUniforms();

  QString itLayout;
  foreach(itLayout, tessEvalLayout_)
    tessEval_->addLayout(itLayout);

  // apply i/o modifiers
  for (size_t i = 0; i < activeMods_.size(); ++i)
    activeMods_[i]->modifyTessControlIO(tessEval_);

  initGenDefines(tessEval_);


  // assemble main function
  QStringList mainCode; 

  // add simple io passthrough mapper

  {
    // barycentric interpolation

    mainCode.push_back("void sg_MapIOBarycentric()");
    mainCode.push_back("{");

    // built-in IO
    mainCode.push_back("gl_Position = gl_TessCoord.x * gl_in[0].gl_Position + gl_TessCoord.y * gl_in[1].gl_Position + gl_TessCoord.z * gl_in[2].gl_Position;");

    // custom IO
    for (int i = 0; i < tessEval_->getNumInputs(); ++i)
    {
      QString inputName = tessEval_->getInputName(i);
      QString outputName = tessEval_->getIOMapName(i);

      QString outputAssignCode = outputName + QString(" = ") +
        QString("gl_TessCoord.x*") + inputName + QString("[0] + ") +
        QString("gl_TessCoord.y*") + inputName + QString("[1] + ") +
        QString("gl_TessCoord.z*") + inputName + QString("[2];");

      mainCode.push_back(outputAssignCode);
    }

    mainCode.push_back("}");

    
    // bilinear interpolation

    mainCode.push_back("void sg_MapIOBilinear()");
    mainCode.push_back("{");

    // built-in IO
    mainCode.push_back("gl_Position = mix(  mix(gl_in[0].gl_Position, gl_in[1].gl_Position, gl_TessCoord.x), mix(gl_in[2].gl_Position, gl_in[3].gl_Position, gl_TessCoord.x), gl_TessCoord.y);");

    // custom IO
    for (int i = 0; i < tessEval_->getNumInputs(); ++i)
    {
      QString inputName = tessEval_->getInputName(i);
      QString outputName = tessEval_->getIOMapName(i);

      QString outputAssignCode = outputName + QString(" = mix( ") +
        QString("mix(") + inputName + QString("[0], ") + inputName + QString("[1], gl_TessCoord.x), ") + 
        QString("mix(") + inputName + QString("[2], ") + inputName + QString("[3], gl_TessCoord.x), gl_TessCoord.y); ");

      mainCode.push_back(outputAssignCode);
    }

    mainCode.push_back("}");
  }


  // interpret loaded shader template:
  //  replace (SG_INPUT, SG_OUTPUT) with matching io pairs
  QStringList::iterator it;
  for (it = tessEvalTemplate_.begin(); it != tessEvalTemplate_.end(); ++it)
  {
    QString line = *it;

    // replace IO line matching the pattern:
    //  SG_OUTPUT = r_expression(SG_INPUT);
    // the complete expression must be contained in a single line for this to work
    // more complex interpolation code should use #if SG_NORMALS etc.

    if (line.contains("SG_INPUT") || line.contains("SG_OUTPUT"))
    {

      QStringList resolvedCode;

      resolvedCode.push_back("// ----------------------------------------");
      resolvedCode.push_back("// ShaderGen: resolve SG_OUTPUT = expression(SG_INPUT);");

      int numOccurrences = 0;

      for (int i = 0; i < tessEval_->getNumInputs(); ++i)
      {
        QString inputName = tessEval_->getInputName(i);
        QString outputName = tessEval_->getIOMapName(i);

        // replace SG_INPUT, SG_OUTPUT with actual names
        QString resolvedLine = line;

        // avoid confusion with SG_INPUT_NORMALVS etc. naming convention
        //  resolvedLine.replace("SG_INPUT", inputName);
        //  resolvedLine.replace("SG_OUTPUT", outputName);
        // fails to do this
        
        // maybe this can be simplified with regexp
        // ie. replace SG_INPUT with inputName,  but not SG_INPUTN, SG_INPUT_ ..

        for (int k = 0; k < 2; ++k)
        {
          const QString stringToReplace = k ? "SG_OUTPUT" : "SG_INPUT";
          const int lenStringToReplace = stringToReplace.length();
          const QString replacementString = k ? outputName : inputName;

          int linePos = resolvedLine.indexOf(stringToReplace);

          while (linePos >= 0)
          {
            bool replaceOcc = true;

            int nextCharPos = linePos + lenStringToReplace;

            if (nextCharPos >= resolvedLine.size())
              nextCharPos = -1;

            if (nextCharPos > 0)
            {
              QChar nextChar = resolvedLine.at(nextCharPos);

              if (nextChar == '_' || nextChar.isDigit() || nextChar.isLetter())
              {
                // name token continues, this should not be replaced!

                linePos += lenStringToReplace;
                replaceOcc = false;
              }
            }

            // replace

            if (replaceOcc)
            {
              resolvedLine.replace(linePos, lenStringToReplace, replacementString);
              ++numOccurrences;
            }

            linePos = resolvedLine.indexOf(stringToReplace, linePos + 1);
          }
        }


        


        resolvedCode.push_back(resolvedLine);
      }

      resolvedCode.push_back("// ----------------------------------------");

      if (numOccurrences)
        mainCode.append(resolvedCode);
      else
        mainCode.push_back(line); // nothing to replace in this line
    }
    else
      mainCode.push_back(line);
  }

  tessEval_->buildShaderCode(&mainCode, lightingCode_);
}

void ShaderProgGenerator::buildGeometryShader()
{
  // Only build a geometry shader if enabled
  if ( desc_.geometryTemplateFile.isEmpty() )
    return;


  delete geometry_;

  geometry_  = new ShaderGenerator();
  geometry_->setGLSLVersion(desc_.version);


  // find previous shader stage
  ShaderGenerator* prevStage = tessEval_;

  if (!prevStage)
    prevStage = vertex_;

  geometry_->initGeometryShaderIO(&desc_, prevStage, &ioDesc_);

  geometry_->initDefaultUniforms();


  // apply i/o modifiers
  for (size_t i = 0; i < activeMods_.size(); ++i)
    activeMods_[i]->modifyGeometryIO(geometry_);

  initGenDefines(geometry_);


  // assemble main function
  QStringList mainCode; 

  // add simple io passthrough mapper

  {
      // Write function as macro so that compiler knows there is no index indirection (thx AMD)
      mainCode.push_back("#define sg_MapIO(inIdx) do {\\");

    // built-in IO
    mainCode.push_back("gl_Position = gl_in[inIdx].gl_Position;\\");
    mainCode.push_back("gl_PrimitiveID = gl_PrimitiveIDIn;\\");


    // built-in gl_ClipDistance[]
    static int maxClipDistances = -1;
    if (maxClipDistances < 0)
    {
#ifdef GL_MAX_CLIP_DISTANCES
      glGetIntegerv(GL_MAX_CLIP_DISTANCES, &maxClipDistances);
      maxClipDistances = std::min(maxClipDistances, 32); // clamp to 32 bits
#else
      maxClipDistances = 32;
#endif
    }
    for (int i = 0; i < maxClipDistances; ++i)
    {
      if (desc_.clipDistanceMask & (1 << i))
        mainCode.push_back(QString("gl_ClipDistance[%1] = gl_in[inIdx].gl_ClipDistance[%1];\\").arg(i));
    }

    // custom IO
    for (int i = 0; i < geometry_->getNumInputs(); ++i)
    {
      QString inputName = geometry_->getInputName(i);
      QString outputName = geometry_->getIOMapName(i);

      QString outputAssignCode = outputName + QString(" = ") + inputName + QString("[inIdx];\\");

      mainCode.push_back(outputAssignCode);
    }

    // Enforce semicolon when using macro
    mainCode.push_back("} while(false)");
  }


  // interpret loaded shader template:
  //  import #includes
  QString it;
  foreach(it,geometryTemplate_)
  {
    if (!checkForIncludes(it, geometry_, getPathName(geometryShaderFile_)))
    {
      // str line is no include directive
      mainCode.push_back(it);
    }
  }

  geometry_->buildShaderCode(&mainCode, lightingCode_);
}


void ShaderProgGenerator::buildFragmentShader()
{
  delete fragment_;

  fragment_  = new ShaderGenerator();
  fragment_->setGLSLVersion(desc_.version);

  // find previous shader stage
  ShaderGenerator* prevStage = geometry_;

  if (!prevStage)
    prevStage = tessEval_;
  if (!prevStage)
    prevStage = tessControl_;
  if (!prevStage)
    prevStage = vertex_;


  fragment_->initFragmentShaderIO(&desc_, prevStage, &ioDesc_);

  if (desc_.texGenDim && (desc_.texGenMode == GL_OBJECT_LINEAR || desc_.texGenMode == GL_EYE_LINEAR) && desc_.texGenPerFragment)
  {
    // application has to provide texture projection planes
    QString uniformDecl = "vec4 g_vTexGenPlane";
    if (desc_.texGenDim > 1)
      uniformDecl += "[" + QString::number(desc_.texGenDim) + "]";
    fragment_->addUniform(uniformDecl, " // texture projection planes");
  }


  fragment_->initDefaultUniforms();


  // texture sampler id
  if (desc_.textured())
  {
    for (std::map<size_t,ShaderGenDesc::TextureType>::const_iterator iter = desc_.textureTypes().begin();
        iter != desc_.textureTypes().end(); ++iter)
    {
      QString name = QString("g_Texture%1").arg(iter->first);
      QString type = "";
      switch (iter->second.type)
      {
        case GL_TEXTURE_1D: type = "sampler1D"; break;
        case GL_TEXTURE_2D: type = "sampler2D"; break;
        case GL_TEXTURE_3D: type = "sampler3D"; break;
        case GL_TEXTURE_CUBE_MAP: type = "samplerCube"; break;
#ifdef GL_ARB_texture_rectangle //ARCH_DARWIN doesn't support all texture defines with all xcode version (xcode 5.0 seems to support all)
        case GL_TEXTURE_RECTANGLE_ARB: type = "sampler2DRect"; break;
#endif
        case GL_TEXTURE_BUFFER: type = "samplerBuffer"; break;
#ifdef GL_EXT_texture_array
        case GL_TEXTURE_1D_ARRAY_EXT: type = "sampler1DArray"; break;
        case GL_TEXTURE_2D_ARRAY_EXT: type = "sampler2DArray"; break;
#endif
#ifdef GL_ARB_texture_cube_map_array
        case GL_TEXTURE_CUBE_MAP_ARRAY_ARB: type = "samplerCubeArray"; break;
#endif
#ifdef GL_ARB_texture_multisample
        case GL_TEXTURE_2D_MULTISAMPLE: type = "sampler2DMS"; break;
        case GL_TEXTURE_2D_MULTISAMPLE_ARRAY: type = "sampler2DMSArray"; break;
#endif
        default: std::cerr << "Texture Type not supported "<< iter->second.type << std::endl; break;
      }
      // todo: check if texture type supports shadowtype
      if (iter->second.shadow)
        type += "Shadow";
      fragment_->addUniform(type + " " + name);
    }
  }

  // apply i/o modifiers
  for (size_t i = 0; i < activeMods_.size(); ++i)
    activeMods_[i]->modifyFragmentIO(fragment_);


  initGenDefines(fragment_);



  // io

  // when to use fragment lights
  if (desc_.shadeMode == SG_SHADE_PHONG)
  {
    for (int i = 0; i < desc_.numLights; ++i)
      fragment_->addLight(i, desc_.lightTypes[i]);
  }

  // assemble main function
  QStringList mainCode;

  if (!fragmentTemplate_.size())
  {
    mainCode.push_back("void main()");
    mainCode.push_back("{");

    addFragmentBeginCode(&mainCode);
    addFragmentEndCode(&mainCode);

    mainCode.push_back("}");
  }
  else
  {
    // interpret loaded shader template:
    //  import #includes and replace SG_VERTEX_BEGIN/END markers
    QString it;
    foreach(it,fragmentTemplate_)
    {
      if (!checkForIncludes(it, fragment_, getPathName(fragmentShaderFile_)))
      {
        // str line is no include directive
        // check for SG_ markers

        if (it.contains("SG_FRAGMENT_BEGIN"))
          addFragmentBeginCode(&mainCode);
        else if (it.contains("SG_FRAGMENT_END"))
          addFragmentEndCode(&mainCode);
        else if (it.contains("SG_FRAGMENT_LIGHTING"))
          addLightingCode(&mainCode);
        else // no SG marker
          mainCode.push_back(it);

      }

      
    }

  }



  fragment_->buildShaderCode(&mainCode, lightingCode_);
}


void ShaderProgGenerator::addFragmentBeginCode(QStringList* _code)
{
  // support for projective texture mapping
  _code->push_back(QString("vec4 sg_vPosCS = ") + ShaderGenerator::keywords.macro_inputPosCS + QString(";"));
  _code->push_back("vec2 sg_vScreenPos = sg_vPosCS.xy / sg_vPosCS.w * 0.5 + vec2(0.5, 0.5);");

  _code->push_back(QString("#ifdef ") + ShaderGenerator::keywords.macro_inputPosVS);
  _code->push_back(QString("vec4 sg_vPosVS = ") + ShaderGenerator::keywords.macro_inputPosVS + QString(";"));
  _code->push_back("#endif");

  _code->push_back(QString("#ifdef ") + ShaderGenerator::keywords.macro_inputNormalVS);
  _code->push_back(QString("vec3 sg_vNormalVS = ") + ShaderGenerator::keywords.macro_inputNormalVS + QString(";"));
  _code->push_back("sg_vNormalVS = normalize(sg_vNormalVS);");
  _code->push_back("#endif");


  if (desc_.vertexColors && (desc_.colorMaterialMode == GL_AMBIENT || desc_.colorMaterialMode == GL_AMBIENT_AND_DIFFUSE))
    _code->push_back(QString("vec4 sg_cColor = vec4(g_cEmissive + g_cLightModelAmbient * ")
                     + ShaderGenerator::keywords.macro_inputVertexColor
                     + QString(".rgb, SG_ALPHA * ")
                     + ShaderGenerator::keywords.macro_inputVertexColor
                     + QString(".a);"));
  else
    _code->push_back("vec4 sg_cColor = vec4(g_cEmissive + g_cLightModelAmbient * g_cAmbient, SG_ALPHA);");

  if (desc_.shadeMode == SG_SHADE_GOURAUD ||
      desc_.shadeMode == SG_SHADE_FLAT ||
      (ioDesc_.passColor_ && (desc_.shadeMode == SG_SHADE_UNLIT || desc_.colorMaterialMode == GL_EMISSION)))
      _code->push_back(QString("sg_cColor = ") + ShaderGenerator::keywords.macro_inputVertexColor + QString(";"));

  _code->push_back(QString("if (sg_cColor.a < SG_MINALPHA) discard;"));
  if (desc_.shadeMode == SG_SHADE_PHONG)
    addLightingCode(_code);


  addTexGenCode(_code, true);

  if (desc_.textured())
  {
    std::map<size_t,ShaderGenDesc::TextureType>::const_iterator iter = desc_.textureTypes().begin();
    _code->push_back("vec4 sg_cTex = texture(g_Texture"+QString::number(iter->first)+", sg_vTexCoord);");

    for (++iter; iter != desc_.textureTypes().end(); ++iter)
      _code->push_back("sg_cTex += texture(g_Texture"+QString::number(iter->first)+", sg_vTexCoord);");

    if (desc_.textureTypes().size() > 1 && desc_.normalizeTexColors)
      _code->push_back("sg_cTex = sg_cTex * 1.0/" + QString::number(desc_.textureTypes().size()) +".0 ;");

    if (desc_.shadeMode == SG_SHADE_UNLIT)
      _code->push_back("sg_cColor += sg_cTex;");
    else
      _code->push_back("sg_cColor *= sg_cTex;");
  }

  
  // apply modifiers
  for (size_t i = 0; i < activeMods_.size(); ++i)
    activeMods_[i]->modifyFragmentBeginCode(_code);
}

void ShaderProgGenerator::addFragmentEndCode(QStringList* _code)
{
  _code->push_back(ShaderGenerator::keywords.fs_outputFragmentColor + QString(" = sg_cColor;"));

  // apply modifiers
  for (size_t i = 0; i < activeMods_.size(); ++i)
    activeMods_[i]->modifyFragmentEndCode(_code);
}



void ShaderProgGenerator::addLightingCode(QStringList* _code)
{

  ShaderModifier* lightingModifier = 0;

  // check if any modifier replaces the default lighting function
  for (size_t i = 0; i < activeMods_.size() && !lightingModifier; ++i)
  {
    if (activeMods_[i]->replaceDefaultLightingCode())
        lightingModifier = activeMods_[i];
  }

  if (!lightingModifier)
  {
    // default lighting code:

    QString buf;

    QString vertexColorString = (ioDesc_.inputColor_ && ioDesc_.passColor_) ? (ShaderGenerator::keywords.macro_inputVertexColor + QString(".xyz * ")) : "";
    QString diffuseVertexColor = (desc_.colorMaterialMode == GL_DIFFUSE || desc_.colorMaterialMode == GL_AMBIENT_AND_DIFFUSE) ? vertexColorString : "";
    QString ambientVertexColor = (desc_.colorMaterialMode == GL_AMBIENT || desc_.colorMaterialMode == GL_AMBIENT_AND_DIFFUSE) ? vertexColorString : "";
    QString specularVertexColor = (desc_.colorMaterialMode == GL_SPECULAR) ? vertexColorString : "";

    for (int i = 0; i < desc_.numLights; ++i)
    {
      ShaderGenLightType lgt = desc_.lightTypes[i];

      switch (lgt)
      {
      case SG_LIGHT_DIRECTIONAL:
//        buf.sprintf("sg_cColor.xyz += LitDirLight(sg_vPosVS.xyz, sg_vNormalVS, g_vLightDir_%d, %s g_cLightAmbient_%d, %s g_cLightDiffuse_%d,  %s g_cLightSpecular_%d);", i, ambientVertexColor, i, diffuseVertexColor, i, specularVertexColor, i);
        buf = QString("sg_cColor.xyz += LitDirLight(sg_vPosVS.xyz, sg_vNormalVS, g_vLightDir_%1, %2 g_cLightAmbient_%1, %3 g_cLightDiffuse_%1,  %4 g_cLightSpecular_%1);").arg(QString::number(i), ambientVertexColor, diffuseVertexColor, specularVertexColor);
        break;

      case SG_LIGHT_POINT:
//        buf.sprintf("sg_cColor.xyz += LitPointLight(sg_vPosVS.xyz, sg_vNormalVS,  g_vLightPos_%d, %s g_cLightAmbient_%d, %s g_cLightDiffuse_%d, %s g_cLightSpecular_%d,  g_vLightAtten_%d);", i, ambientVertexColor, i, diffuseVertexColor, i, specularVertexColor, i, i);
        buf = QString("sg_cColor.xyz += LitPointLight(sg_vPosVS.xyz, sg_vNormalVS,  g_vLightPos_%1, %2 g_cLightAmbient_%1, %3 g_cLightDiffuse_%1, %4 g_cLightSpecular_%1,  g_vLightAtten_%1);").arg(QString::number(i), ambientVertexColor, diffuseVertexColor, specularVertexColor);
        break;

      case SG_LIGHT_SPOT:
//        buf.sprintf("sg_cColor.xyz += LitSpotLight(sg_vPosVS.xyz,  sg_vNormalVS,  g_vLightPos_%d,  g_vLightDir_%d, %s g_cLightAmbient_%d, %s g_cLightDiffuse_%d, %s g_cLightSpecular_%d,  g_vLightAtten_%d,  g_vLightAngleExp_%d);", i, i, ambientVertexColor, i, diffuseVertexColor, i, specularVertexColor, i, i, i);
        buf = QString("sg_cColor.xyz += LitSpotLight(sg_vPosVS.xyz,  sg_vNormalVS,  g_vLightPos_%1,  g_vLightDir_%1, %2 g_cLightAmbient_%1, %3 g_cLightDiffuse_%1, %4 g_cLightSpecular_%1,  g_vLightAtten_%1,  g_vLightAngleExp_%1);").arg(QString::number(i), ambientVertexColor, diffuseVertexColor, specularVertexColor);
        break;

      default: break;
      }

      _code->push_back(buf);
    }

    // modify lighting color afterwards

    for (size_t i = 0; i < activeMods_.size(); ++i)
      modifyLightingCode(_code, activeMods_[i]);
  }
  else
  {
    // there exists a lighting modifier that completely replaces the default lighting shader
    modifyLightingCode(_code, lightingModifier);


    // apply remaining modifiers that do not replace the complete lighting code

    for (size_t i = 0; i < activeMods_.size(); ++i)
    {
      if (lightingModifier != activeMods_[i])
        modifyLightingCode(_code, activeMods_[i]);
    }
  }

}

void ShaderProgGenerator::modifyLightingCode( QStringList* _code, ShaderModifier* _modifier )
{
  if (!_modifier) return;

  for (int i = 0; i < desc_.numLights; ++i)
  {
    ShaderGenLightType lgt = desc_.lightTypes[i];

    _modifier->modifyLightingCode(_code, i, lgt);
  }
}


void ShaderProgGenerator::addLightingFunctions(QStringList* _code)
{
  QString it;
  foreach(it,lightingCode_)
    _code->push_back(it);
}


void ShaderProgGenerator::addTexGenCode( QStringList* _code, bool _fragmentShader )
{
  // declare local texcoord variable name as "sg_vTexCoord"
  int texcoordVarDim = 2;
  if (ioDesc_.inputTexCoord_ && 
    !desc_.textureTypes().empty() &&
    desc_.textureTypes().begin()->second.type == GL_TEXTURE_3D)
    texcoordVarDim = 3;

  bool generateTexCoord = desc_.texGenDim && desc_.texGenMode && (_fragmentShader == desc_.texGenPerFragment);
  if (generateTexCoord)
    texcoordVarDim = desc_.texGenDim;

  QString texcoordVarInit;
  if (texcoordVarDim == 1)
    texcoordVarInit = "float sg_vTexCoord";
  else
    texcoordVarInit = QString("vec%1 sg_vTexCoord").arg(texcoordVarDim);

  // init with default value: input or zero
  if (ioDesc_.inputTexCoord_ && !generateTexCoord)
    texcoordVarInit += QString("= ") + ShaderGenerator::keywords.macro_inputTexcoord + QString(";");
  else if (0 <= texcoordVarDim && texcoordVarDim <= 4)
  {
    QString zeroVecDefs[] = 
    {
      ";",
      "= 0;",
      "= vec2(0,0);",
      "= vec3(0,0,0);",
      "= vec4(0,0,0,0);"
    };
    texcoordVarInit += zeroVecDefs[texcoordVarDim];
  }

  _code->push_back(texcoordVarInit);


  // texcoord generation
  // https://www.opengl.org/wiki/Mathematics_of_glTexGen
  if (generateTexCoord)
  {

    const char* texGenCoordString[] = { "x", "y", "z", "w" };

    switch (desc_.texGenMode)
    {
    case GL_OBJECT_LINEAR:
    {
      for (int i = 0; i < desc_.texGenDim; ++i)
      {
        QString assignmentInstrString;
        assignmentInstrString = "sg_vTexCoord";
        if (desc_.texGenDim > 1)
        {
          assignmentInstrString +=".";
          assignmentInstrString += texGenCoordString[i];
        }
        assignmentInstrString += " = dot(";
        assignmentInstrString += ShaderGenerator::keywords.macro_inputPosOS;
        assignmentInstrString += ", g_vTexGenPlane";
        if (desc_.texGenDim > 1)
        {
          assignmentInstrString += "[";
          assignmentInstrString += QString::number(i);
          assignmentInstrString += "]";
        }
        assignmentInstrString += ");";
        _code->push_back(assignmentInstrString);
      }
    } break;

    case GL_EYE_LINEAR:
    {
      for (int i = 0; i < desc_.texGenDim; ++i)
      {
        QString assignmentInstrString;
        assignmentInstrString = "sg_vTexCoord";
        if (desc_.texGenDim > 1)
        {
          assignmentInstrString += ".";
          assignmentInstrString += texGenCoordString[i];
        }
        assignmentInstrString += " = dot(sg_vPosVS, g_vTexGenPlane";
        if (desc_.texGenDim > 1)
        {
          assignmentInstrString += "[";
          assignmentInstrString += QString::number(i);
          assignmentInstrString += "]";
        }
        assignmentInstrString += ");";
        _code->push_back(assignmentInstrString);
      }

    } break;

    case GL_SPHERE_MAP:
    {
      _code->push_back("vec3 sg_vPosVS_unit = normalize(sg_vPosVS.xyz);");
      _code->push_back("vec3 sg_TexGenRefl = reflect(sg_vPosVS_unit, sg_vNormalVS);");
      _code->push_back("vec3 sg_TexGenRefl2 = sg_TexGenRefl; sg_TexGenRefl2.z += 1.0;");
      _code->push_back("float sg_TexGenMRcp = 0.5 * inversesqrt(dot(sg_TexGenRefl2, sg_TexGenRefl2));");
      for (int i = 0; i < desc_.texGenDim; ++i)
      {        
        _code->push_back(QString ("sg_vTexCoord.%1 = sg_TexGenRefl.%2 * sg_TexGenMRcp + 0.5;").arg(texGenCoordString[i]).arg(texGenCoordString[i]));
      }
    } break;

    case GL_NORMAL_MAP:
    {
      for (int i = 0; i < desc_.texGenDim; ++i)
      {       
        _code->push_back( QString ("sg_vTexCoord.%1 = sg_vNormalVS.%2;").arg(texGenCoordString[i]).arg(texGenCoordString[i]) );
      }
    } break;

    case GL_REFLECTION_MAP:
    {
      _code->push_back("vec3 sg_vPosVS_unit = normalize(sg_vPosVS.xyz);");
      _code->push_back("vec3 sg_TexGenRefl = reflect(sg_vPosVS_unit, sg_vNormalVS);");
      for (int i = 0; i < desc_.texGenDim; ++i)
      {

        _code->push_back( QString ("sg_vTexCoord.%1 = sg_TexGenRefl.%2;").arg(texGenCoordString[i]).arg(texGenCoordString[i]) );
      }
    } break;

    default: break;
    }
  }
}


void ShaderProgGenerator::generateShaders()
{
  // import template source from files
  loadShaderTemplateFromFile();

  // check what needs to be passed down from vertex shader

  if (desc_.shadeMode != SG_SHADE_UNLIT)
    ioDesc_.inputNormal_ = true;

  if (desc_.textured())
  {
    ioDesc_.inputTexCoord_ = true;
    ioDesc_.passTexCoord_ = true;
  }

  // clamp generated texcoord dimension
  int maxTexGenDim = 4;

  switch (desc_.texGenMode)
  {
  case GL_EYE_LINEAR:
  case GL_OBJECT_LINEAR: maxTexGenDim = 4; break;
    
  case GL_SPHERE_MAP: maxTexGenDim = 2; break;
  
  case GL_NORMAL_MAP:
  case GL_REFLECTION_MAP: maxTexGenDim = 3; break;
  
  default: maxTexGenDim = 0; break;
  }

  desc_.texGenDim = std::max(std::min(desc_.texGenDim, maxTexGenDim), 0);


  if (desc_.texGenDim && desc_.texGenMode)
  {
    // pass generated texcoord from vertex to fragment shader
    if (!desc_.texGenPerFragment)
      ioDesc_.passTexCoord_ = true;

    // some modes require normal vectors
    if (desc_.texGenMode == GL_REFLECTION_MAP || desc_.texGenMode == GL_SPHERE_MAP || desc_.texGenMode == GL_NORMAL_MAP)
      ioDesc_.inputNormal_ = true;

    // pass data to the fragment shader as required for the generation
    if (desc_.texGenPerFragment)
    {
      switch (desc_.texGenMode)
      {
      case GL_OBJECT_LINEAR: ioDesc_.passPosOS_ = true; break;
      case GL_EYE_LINEAR: ioDesc_.passPosVS_ = true; break;
      case GL_SPHERE_MAP: ioDesc_.passPosVS_ = ioDesc_.passNormalVS_ = true; break;
      case GL_NORMAL_MAP: ioDesc_.passNormalVS_ = true; break;
      case GL_REFLECTION_MAP: ioDesc_.passPosVS_ = ioDesc_.passNormalVS_ = true; break;
      default: break;
      }
    }
  }


  if (desc_.vertexColors)
    ioDesc_.inputColor_ = true;

  if (desc_.shadeMode == SG_SHADE_PHONG)
  {
    ioDesc_.passNormalVS_ = true;
    ioDesc_.passPosVS_ = true;
  }

  if (desc_.shadeMode == SG_SHADE_FLAT || desc_.shadeMode == SG_SHADE_GOURAUD || desc_.vertexColors)
    ioDesc_.passColor_ = true;


  // scan macros of modifiers for attribute requests,
  // done by adding modifier io to an empty dummy
  ShaderGenerator dummy;

  for (size_t i = 0; i < activeMods_.size(); ++i)
  {
    ShaderModifier* mod = activeMods_[i];

    mod->modifyVertexIO(&dummy);
    mod->modifyTessControlIO(&dummy);
    mod->modifyTessEvalIO(&dummy);
    mod->modifyGeometryIO(&dummy);
    mod->modifyFragmentIO(&dummy);
  }
  // scan requested inputs from modifiers

  if (dummy.hasDefine(ShaderGenerator::keywords.macro_requestPosVS))
    ioDesc_.passPosVS_ = true;
  if (dummy.hasDefine(ShaderGenerator::keywords.macro_requestTexcoord))
  {
    ioDesc_.inputTexCoord_ = true;
    ioDesc_.passTexCoord_ = true;
  }
  if (dummy.hasDefine(ShaderGenerator::keywords.macro_requestVertexColor))
  {
    ioDesc_.inputColor_ = true;
    ioDesc_.passColor_ = true;
  }
  if (dummy.hasDefine(ShaderGenerator::keywords.macro_requestNormalVS))
  {
    ioDesc_.inputNormal_ = true;
    ioDesc_.passNormalVS_ = true;
  }
  if (dummy.hasDefine(ShaderGenerator::keywords.macro_requestNormalOS))
  {
    ioDesc_.inputNormal_ = true;
    ioDesc_.passNormalOS_ = true;
  }
  if (dummy.hasDefine(ShaderGenerator::keywords.macro_requestPosOS))
    ioDesc_.passPosOS_ = true;
  




  // assemble shader codes

  buildVertexShader();
  buildTessControlShader();
  buildTessEvalShader();
  buildGeometryShader();
  buildFragmentShader();
}


const QStringList& ShaderProgGenerator::getVertexShaderCode()
{
  return vertex_->getShaderCode();
}

const QStringList& ShaderProgGenerator::getTessControlShaderCode()
{
  return tessControl_->getShaderCode();
}

const QStringList& ShaderProgGenerator::getTessEvaluationShaderCode()
{
  return tessEval_->getShaderCode();
}

const QStringList& ShaderProgGenerator::getGeometryShaderCode()
{
  return geometry_->getShaderCode();
}

const QStringList& ShaderProgGenerator::getFragmentShaderCode()
{
  return fragment_->getShaderCode();
}


void ShaderProgGenerator::saveVertexShToFile(const char* _fileName)
{
  vertex_->saveToFile(_fileName);
}

void ShaderProgGenerator::saveGeometryShToFile(const char* _fileName)
{
  geometry_->saveToFile(_fileName);
}

void ShaderProgGenerator::saveFragmentShToFile(const char* _fileName)
{
  fragment_->saveToFile(_fileName);
}


void ShaderProgGenerator::loadShaderTemplateFromFile()
{
  if (!desc_.vertexTemplateFile.isEmpty())
  {
    loadStringListFromFile(desc_.vertexTemplateFile, &vertexTemplate_);
    scanShaderTemplate(vertexTemplate_, desc_.vertexTemplateFile);
  }
  if (!desc_.fragmentTemplateFile.isEmpty())
  {
    loadStringListFromFile(desc_.fragmentTemplateFile, &fragmentTemplate_);
    scanShaderTemplate(fragmentTemplate_, desc_.fragmentTemplateFile);
  }
  if (!desc_.geometryTemplateFile.isEmpty())
  {
    loadStringListFromFile(desc_.geometryTemplateFile, &geometryTemplate_);
    scanShaderTemplate(geometryTemplate_, desc_.geometryTemplateFile);
  }
  if (!desc_.tessControlTemplateFile.isEmpty())
  {
    loadStringListFromFile(desc_.tessControlTemplateFile, &tessControlTemplate_);
    scanShaderTemplate(tessControlTemplate_, desc_.tessControlTemplateFile, &tessControlLayout_);
  }
  if (!desc_.tessEvaluationTemplateFile.isEmpty())
  {
    loadStringListFromFile(desc_.tessEvaluationTemplateFile, &tessEvalTemplate_);
    scanShaderTemplate(tessEvalTemplate_, desc_.tessEvaluationTemplateFile, &tessEvalLayout_);
  }


  vertexShaderFile_   = desc_.vertexTemplateFile;
  tessControlShaderFile_ = desc_.tessControlTemplateFile;
  tessEvalShaderFile_ = desc_.tessEvaluationTemplateFile;
  geometryShaderFile_ = desc_.geometryTemplateFile;
  fragmentShaderFile_ = desc_.fragmentTemplateFile;
}

void ShaderProgGenerator::scanShaderTemplate(QStringList& _templateSrc, QString _templateFilename, QStringList* _outLayoutDirectives)
{
  // interpret loaded shader template:
  //  import #includes

  QString filePath = getPathName(_templateFilename);

  QStringList::iterator it;
  for (it = _templateSrc.begin(); it != _templateSrc.end(); ++it)
  {
    QStringList import;

    if (checkForIncludes(*it, &import, filePath))
    {
      // line is include directive

      // remove line from source
      it = _templateSrc.erase(it);

      int offset = it - _templateSrc.begin();

      // insert imported file

      QString importLine;
      foreach(importLine, import)
      {
        it = _templateSrc.insert(it, importLine);
        ++it;
      }

      // included file might recursively include something again
      // -> scan included file
      it = _templateSrc.begin() + offset;
    }
    else
    {
      QString trimmedLine = it->trimmed();

      // scan and adjust glsl version
      QByteArray lineBytes = trimmedLine.toUtf8();

      if (trimmedLine.startsWith("#version "))
      {
        QStringList tokens = trimmedLine.split(' ');

        if (tokens.size() > 1)
        {
          // templateVersion
          bool convOk = false;
          int templateVersion = tokens.at(1).toInt(&convOk);

          if (convOk)
          {
            desc_.version = std::max(templateVersion, desc_.version);

            // remove version line from template since this is added later in the build functions
            it = _templateSrc.erase(it);
          }
        }
      }
      // scan layout() directive
      else if (trimmedLine.startsWith("layout(") || trimmedLine.startsWith("layout ("))
      {
        if (_outLayoutDirectives)
        {
          _outLayoutDirectives->push_back(trimmedLine);
          // layout() will be inserted later at the correct position in the build functions
          // - must be placed before shader IO declaration to make tess-control shaders compilable on ati
          it = _templateSrc.erase(it);
        }
      }
      else
      {
        // scan requested inputs

        if (trimmedLine.startsWith(ShaderGenerator::keywords.macro_requestPosVS))
          ioDesc_.passPosVS_ = true;
        else if (trimmedLine.startsWith(ShaderGenerator::keywords.macro_requestTexcoord))
        {
          ioDesc_.inputTexCoord_ = true;
          ioDesc_.passTexCoord_ = true;
        }
        else if (trimmedLine.startsWith(ShaderGenerator::keywords.macro_requestVertexColor))
        {
          ioDesc_.inputColor_ = true;
          ioDesc_.passColor_ = true;
        }
        else if (trimmedLine.startsWith(ShaderGenerator::keywords.macro_requestNormalVS))
        {
          ioDesc_.inputNormal_ = true;
          ioDesc_.passNormalVS_ = true;
        }
        else if (trimmedLine.startsWith(ShaderGenerator::keywords.macro_requestNormalOS))
        {
          ioDesc_.inputNormal_ = true;
          ioDesc_.passNormalOS_ = true;
        }
        else if (trimmedLine.startsWith(ShaderGenerator::keywords.macro_requestPosOS))
          ioDesc_.passPosOS_ = true;
        else if (trimmedLine.startsWith("SG_FRAGMENT_LIGHTING"))
        {
          // shader template performs lighting in fragment shader
          // -> forced phong shading
          desc_.shadeMode = SG_SHADE_PHONG;
        }
      }

    }
  }

}

QString ShaderProgGenerator::getPathName(QString _strFileName)
{
  QFileInfo fileInfo(getAbsFilePath(_strFileName));
  return fileInfo.absolutePath();
}

QString ShaderProgGenerator::getAbsFilePath(QString _strFileName)
{
  QString absFilename;
  if ( QDir(_strFileName).isRelative() )
    absFilename = getShaderDir() + QDir::separator() + _strFileName;
  else
    absFilename = _strFileName;

  return QDir::cleanPath(absFilename);
}

void ShaderProgGenerator::setShaderDir( QString _dir )
{
  shaderDir_ = _dir;
}

QString ShaderProgGenerator::getShaderDir()
{
  return shaderDir_ + QString("/");
}


unsigned int ShaderProgGenerator::registerModifier( ShaderModifier* _modifier )
{
  if (!_modifier) return 0;

  // redundancy check
  for (int i = 0; i < numRegisteredModifiers_; ++i)
  {
    if (registeredModifiers_[i] == _modifier) 
    {
//      std::cout << "warning: trying to re-register shader modifier " << _modifier->getID() << std::endl;
      return registeredModifiers_[i]->getID();
    }
  }

  _modifier->modifierID_ = (unsigned int)(numRegisteredModifiers_++);

  registeredModifiers_.push_back(_modifier);
  return _modifier->modifierID_;
}

ShaderModifier* ShaderProgGenerator::getActiveModifier( int _i )
{
  if (_i >= 0 && _i <= int(activeMods_.size()))
    return activeMods_[_i];

  // invalid _i
  return 0;
}

int ShaderProgGenerator::getNumActiveModifiers() const
{
  return int(activeMods_.size());
}


bool ShaderProgGenerator::hasGeometryShader() const
{
  return !desc_.geometryTemplateFile.isEmpty();
}

bool ShaderProgGenerator::hasTessControlShader() const
{
  return !desc_.tessControlTemplateFile.isEmpty();
}

bool ShaderProgGenerator::hasTessEvaluationShader() const
{
  return !desc_.tessEvaluationTemplateFile.isEmpty();
}


//=============================================================================

ShaderModifier::ShaderModifier( void )
: modifierID_(0)
{}

ShaderModifier::~ShaderModifier( void )
{}


class ShaderModifierFile : public ShaderModifier
{
public:

  ShaderModifierFile()
    : version_(0) 
  {}

  virtual ~ShaderModifierFile() 
  {}

  void modifyVertexIO(ShaderGenerator* _shader) override { modifyIO(0, _shader); }
  void modifyTessControlIO(ShaderGenerator* _shader) override { modifyIO(1, _shader); }
  void modifyTessEvalIO(ShaderGenerator* _shader) override { modifyIO(2, _shader); }
  void modifyGeometryIO(ShaderGenerator* _shader) override { modifyIO(3, _shader); }
  void modifyFragmentIO(ShaderGenerator* _shader) override { modifyIO(4, _shader); }


  void modifyVertexBeginCode(QStringList* _code) override { _code->append(vertexBeginCode_); }
  void modifyVertexEndCode(QStringList* _code) override { _code->append(vertexEndCode_); }
  void modifyFragmentBeginCode(QStringList* _code) override { _code->append(fragmentBeginCode_); }
  void modifyFragmentEndCode(QStringList* _code) override { _code->append(fragmentEndCode_); }

  const QString& filename() const {return filename_;}
  const QDateTime& filetime() const {return filetime_;}
  void filetime(const QDateTime& _newtime) {filetime_ = _newtime;}

  void clear()
  {
    version_ = 0;

    for (int i = 0; i < 5; ++i)
      io_[i].clear();

    vertexBeginCode_.clear();
    vertexEndCode_.clear();
    fragmentBeginCode_.clear();
    fragmentEndCode_.clear();
  }

  static ShaderModifierFile* loadFromFile(QString _filename)
  {
    ShaderModifierFile* res = 0;

    // get timestamp
    QString absFilename = ShaderProgGenerator::getAbsFilePath(_filename);
    QDateTime lastmod = QFileInfo(absFilename).lastModified();

    // check cache
    QHash<QString, ShaderModifierFile>::iterator cacheEntry = fileCache_.find(_filename);

    bool reload = false;
    bool firstLoad = false;

    if (cacheEntry != fileCache_.end())
    {
      // fetch from cache
      res = &cacheEntry.value();

      if (lastmod != res->filetime())
      {
        res->clear();
        reload = true;
      }
    }
    else
    {
      // load new modifier
      reload = true;
      firstLoad = true;
    }

    if (reload)
    {
      QStringList lines;
      if (ShaderProgGenerator::loadStringListFromFile(_filename, &lines))
      {
        // new cache entry
        if (firstLoad)
          res = &fileCache_[_filename];

        res->loadBlocks(lines);
        res->filetime(lastmod);

        // also register to generator
        if (firstLoad)
          ShaderProgGenerator::registerModifier(res);
      }
    }

    return res;
  }

private:


  void loadBlocks(const QStringList& _lines)
  {
    static const char* markers [] = 
    {
      "VertexIO:",
      "TessControlIO:",
      "TessEvalIO:",
      "GeometryIO:",
      "FragmentIO:",
      "VertexBeginCode:",
      "VertexEndCode:",
      "FragmentBeginCode:",
      "FragmentEndCode:"
    };
    const int numMarkers = sizeof(markers) / sizeof(markers[0]);

    QStringList* blockTargets [] = 
    {
      io_ + 0,
      io_ + 1,
      io_ + 2,
      io_ + 3,
      io_ + 4,
      &vertexBeginCode_,
      &vertexEndCode_,
      &fragmentBeginCode_,
      &fragmentEndCode_
    };

    assert(sizeof(blockTargets) / sizeof(blockTargets[0]) == numMarkers);


    // current block in file, points to one of io_[idx], vertexBeginCode_, ...
    QStringList* curBlock_ = 0;


    int curLine = 0;

    for (QStringList::const_iterator it = _lines.begin(); it != _lines.end(); ++it, ++curLine)
    {
      if (it->isEmpty())
        continue;

      // read glsl version
      if (version_ <= 0 && it->startsWith("#version "))
      {
        const int offset = strlen("#version ");
        version_ = atoi(it->toLatin1().data() + offset);
      }
      else
      {
        // read code blocks

        bool blockMarker = false;

        for (int i = 0; i < numMarkers && !blockMarker; ++i)
        {
          if ( it->startsWith(markers[i]) ) 
          {
            // new block start
            curBlock_ = blockTargets[i];
            blockMarker = true;
          }
        }

        if (!blockMarker)
        {
          if (curBlock_) // code belongs to some block
            curBlock_->push_back(*it);
          else // wrong file structure
            std::cerr << "ShaderModifierFile::loadBlocks - line belongs to unknown block in file " << filename_.toLatin1().data() << " at line " << curLine << std::endl;
        }
      }
    }
  }

  void modifyIO(int _stage, ShaderGenerator* _shader)
  {
    if (version_ > 0)
      _shader->setGLSLVersion(version_);

    _shader->addRawIOBlock(io_[_stage]);
  }

private:

  QString filename_;

  QDateTime filetime_; 

  // glsl version
  int version_;

  // io mods
  QStringList io_[5];

  // code mods
  QStringList vertexBeginCode_,
    vertexEndCode_,
    fragmentBeginCode_,
    fragmentEndCode_;


  // loaded modifiers
  static QHash<QString, ShaderModifierFile> fileCache_;
};

QHash<QString, ShaderModifierFile> ShaderModifierFile::fileCache_;


ShaderModifier* ShaderModifier::loadFromFile(QString _filename)
{
  return ShaderModifierFile::loadFromFile(_filename);
}


//=============================================================================


QString ShaderGenDesc::toString() const
{
  // mapping (int)ShaderGenMode -> string
  const char* shadeModeString[] = 
  {
    "SG_SHADE_UNLIT",
    "SG_SHADE_FLAT",
    "SG_SHADE_GOURAUD",
    "SG_SHADE_PHONG"
  };

  QString res;
  QTextStream resStrm(&res);

  resStrm << "version: " << version;

  resStrm << "\nshaderDesc.numLights: " << numLights;

  if (numLights)
  {
    resStrm << "\nshaderDesc.lightTypes[]: {";

    for (int i = 0; i < numLights; ++i)
    {
      switch (lightTypes[i]) 
      {
      case SG_LIGHT_DIRECTIONAL: resStrm << "DIRECTIONAL"; break;
      case SG_LIGHT_POINT: resStrm << "POINT"; break;
      case SG_LIGHT_SPOT: resStrm << "SPOT"; break;
      default: resStrm << "UNDEFINED"; break;
      }

      if (i + 1 < numLights)
        resStrm << ", ";
      else
        resStrm << "}";
    }
  }
  resStrm << "\nshaderDesc.shadeMode: " << shadeModeString[shadeMode];
  resStrm << "\nshaderDesc.twoSidedLighting: " << (twoSidedLighting ? "Yes" : "No");
  resStrm << "\nshaderDesc.vertexColors: " << vertexColors;
  resStrm << "\nshaderDesc.textured(): " << textured();
  for (std::map<size_t,TextureType>::const_iterator iter = textureTypes_.begin(); iter != textureTypes_.end();++iter)
  {
    resStrm << "\nTexture stage: " << iter->first;
    resStrm << "\nTexture Type: ";
    switch (iter->second.type)
    {
        case GL_TEXTURE_1D: resStrm << "GL_TEXTURE_1D"; break;
        case GL_TEXTURE_2D: resStrm << "GL_TEXTURE_2D"; break;
        case GL_TEXTURE_3D: resStrm << "GL_TEXTURE_3D"; break;
        case GL_TEXTURE_CUBE_MAP: resStrm << "GL_TEXTURE_CUBE_MAP"; break;
#ifdef GL_ARB_texture_rectangle //ARCH_DARWIN doesn't support all texture defines with all xcode version (xcode 5.0 seems to support all)
        case GL_TEXTURE_RECTANGLE_ARB: resStrm << "GL_TEXTURE_RECTANGLE"; break;
#endif
        case GL_TEXTURE_BUFFER: resStrm << "GL_TEXTURE_BUFFER"; break;
#ifdef GL_EXT_texture_array
        case GL_TEXTURE_1D_ARRAY_EXT: resStrm << "GL_TEXTURE_1D_ARRAY"; break;
        case GL_TEXTURE_2D_ARRAY_EXT: resStrm << "GL_TEXTURE_2D_ARRAY"; break;
#endif
#ifdef GL_ARB_texture_cube_map_array
        case GL_TEXTURE_CUBE_MAP_ARRAY_ARB: resStrm << "GL_TEXTURE_CUBE_MAP_ARRAY"; break;
#endif
#ifdef GL_ARB_texture_multisample
        case GL_TEXTURE_2D_MULTISAMPLE: resStrm << "GL_TEXTURE_2D_MULTISAMPLE"; break;
        case GL_TEXTURE_2D_MULTISAMPLE_ARRAY: resStrm << "GL_TEXTURE_2D_MULTISAMPLE_ARRAY"; break;
#endif
        default: std::cerr << "Texture Type with number "<< iter->second.type << " on stage "<< iter->first << " is not supported "  << std::endl; break;
    }

    resStrm  << "\nShadowTexture: " <<  iter->second.shadow;
  }

  resStrm << "\nshaderDesc.texGenDim: " << texGenDim;

  switch (texGenMode)
  {
  case GL_OBJECT_LINEAR: resStrm << "\nshaderDesc.texGenMode: GL_OBJECT_LINEAR"; break;
  case GL_EYE_LINEAR: resStrm << "\nshaderDesc.texGenMode: GL_EYE_LINEAR"; break;
  case GL_SPHERE_MAP: resStrm << "\nshaderDesc.texGenMode: GL_SPHERE_MAP"; break;
  case GL_NORMAL_MAP: resStrm << "\nshaderDesc.texGenMode: GL_NORMAL_MAP"; break;
  case GL_REFLECTION_MAP: resStrm << "\nshaderDesc.texGenMode: GL_REFLECTION_MAP"; break;
  default: resStrm << "\nshaderDesc.texGenMode: unknown"; break;
  }
  
  resStrm << "\nshaderDesc.texGenPerFragment: " << texGenPerFragment;

  if (!vertexTemplateFile.isEmpty())
    resStrm << "\nshaderDesc.vertexTemplateFile: " << vertexTemplateFile;

  if (!tessControlTemplateFile.isEmpty())
    resStrm << "\nshaderDesc.tessControlTemplateFile: " << tessControlTemplateFile;

  if (!tessEvaluationTemplateFile.isEmpty())
    resStrm << "\nshaderDesc.tessEvaluationTemplateFile: " << tessEvaluationTemplateFile;

  if (!geometryTemplateFile.isEmpty())
    resStrm << "\nshaderDesc.geometryTemplateFile: " << geometryTemplateFile;

  if (!fragmentTemplateFile.isEmpty())
    resStrm << "\nshaderDesc.fragmentTemplateFile: " << fragmentTemplateFile;

  return res;
}



} // namespace ACG
//=============================================================================
