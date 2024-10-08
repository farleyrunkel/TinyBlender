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
#include <cstdio>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <QFile>
#include <QTextStream>

#include <ACG/GL/gl.hh>

#include <ACG/GL/IRenderer.hh>

#include <ACG/GL/VertexDeclaration.hh>
#include <ACG/GL/GLError.hh>

#include <ACG/GL/ShaderCache.hh>
#include <ACG/GL/ScreenQuad.hh>
#include <ACG/GL/FBO.hh>
#include <ACG/GL/globjects.hh>



namespace ACG
{

int IRenderer::maxClipDistances_ = -1;

IRenderer::IRenderer()
: numLights_(0), 
renderObjects_(0), 
depthMapUsed_(false), 
curViewerID_(0),
prevFbo_(0),
prevDrawBuffer_(GL_BACK),
prevFboSaved_(false),
prevVAO_(0),
depthCopyShader_(0),
errorDetectionLevel_(1),
coreProfile_(false),
enableLineThicknessGL42_(false)
{
  prevViewport_[0] = 0;
  prevViewport_[1] = 0;
  prevViewport_[2] = 0;
  prevViewport_[3] = 0;

  // set global ambient scale to default OpenGL value
  globalLightModelAmbient_ = ACG::Vec3f(0.2f, 0.2f, 0.2f);
}


IRenderer::~IRenderer()
{
  delete depthCopyShader_;

  // free depth map fbos
  for (std::map<int, ACG::FBO*>::iterator it = depthMaps_.begin(); it != depthMaps_.end(); ++it)
    delete it->second;
}

void IRenderer::addRenderObject(ACG::RenderObject* _renderObject)
{
  // avoid null-ptr access
  if (_renderObject->debugName.empty())
    _renderObject->debugName = "<unnamed>";

  if (_renderObject->name.empty())
    _renderObject->name = _renderObject->debugName;

  // do some more checks for error detection
  if (!_renderObject->vertexDecl && !_renderObject->vertexArrayObject)
    std::cout << "error: missing vertex declaration in renderobject: " << _renderObject->debugName << std::endl;
  else
  {
    if (errorDetectionLevel_ > 0)
    {
      // commonly encountered rendering errors

      if (!_renderObject->numIndices)
        std::cout << "warning: numIndices is 0 in renderobject: " << _renderObject->debugName << std::endl;

      //  Why is my object invisible/black?
      if (!_renderObject->depthWrite && 
        !_renderObject->colorWriteMask[0] && !_renderObject->colorWriteMask[1] &&
        !_renderObject->colorWriteMask[2] && !_renderObject->colorWriteMask[3])
        std::cout << "warning: depth write and color write disabled in renderobject: " << _renderObject->debugName << std::endl;

      // Why is gl_PointSize not working in shader?
#ifndef GL_PROGRAM_POINT_SIZE
      if (_renderObject->programPointSize)
        std::cout << "warning: GL_PROGRAM_POINT_SIZE requested but missing in opengl headers!" << std::endl;
#endif

      if (errorDetectionLevel_ > 1 && _renderObject->shaderDesc.shadeMode == SG_SHADE_UNLIT)
      {
        if (_renderObject->emissive.max() < 1e-3f)
          std::cout << "warning: unlit object rendered with black emissive color: " << _renderObject->debugName << std::endl;
        else
        {
          // rendering with clear color

          float clearColor[4];
          glGetFloatv(GL_COLOR_CLEAR_VALUE, clearColor);

          if (checkEpsilon(clearColor[0] - _renderObject->emissive[0]) &&
            checkEpsilon(clearColor[1] - _renderObject->emissive[1]) &&
            checkEpsilon(clearColor[2] - _renderObject->emissive[2]))
          {
            std::cout << "warning: unlit object rendered with clear color as emissive color: " << _renderObject->debugName << std::endl;
            std::cout << "         Should this be intentional, disable color writing instead via obj->glColorMask(0,0,0,0)" << std::endl;
          }
        }
      }

      if (_renderObject->textures().size())
      {
        // Why are my textures sampled as black?

        // mipmap enabled, but no mipmap chain provided?

        for (std::map<size_t,RenderObject::Texture>::const_iterator it = _renderObject->textures().begin();
          it != _renderObject->textures().end(); ++it)
        {
          if (it->second.type == GL_TEXTURE_BUFFER)
            continue; // skip texture buffers, for which testing for mipmaps would generate an error

          glBindTexture(it->second.type, it->second.id);

          GLint minFilter = GL_NONE;
          glGetTexParameteriv(it->second.type, GL_TEXTURE_MIN_FILTER, &minFilter);

          if (minFilter == GL_NEAREST_MIPMAP_LINEAR ||
            minFilter == GL_NEAREST_MIPMAP_NEAREST ||
            minFilter == GL_LINEAR_MIPMAP_LINEAR ||
            minFilter == GL_LINEAR_MIPMAP_NEAREST)
          {
            GLint maxLevel = 0;
            glGetTexParameteriv(it->second.type, GL_TEXTURE_MAX_LEVEL, &maxLevel);

            GLint texWidth = 0;
            glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &texWidth);

            GLint maxLod = -1;
            for (GLint lod = 0; lod < maxLevel && maxLod < 0; ++lod)
            {
              GLint lodWidth;
              glGetTexLevelParameteriv(GL_TEXTURE_2D, lod, GL_TEXTURE_WIDTH, &lodWidth);

              if (lodWidth <= 0 || lodWidth == GL_INVALID_VALUE)
                maxLod = lod-1;
            }

            if (maxLod <= 0 && texWidth > 1)
            {
              std::cout << "warning: texture is sampled with mipmapping, but no mipchain is present: " << _renderObject->debugName << " texid: " << it->second.id << std::endl;
              std::cout << "         automatically disabling mipmapping!!" << std::endl;

              GLint correctedFilter = GL_LINEAR;

              if (minFilter == GL_NEAREST_MIPMAP_LINEAR ||
                minFilter == GL_LINEAR_MIPMAP_LINEAR)
                correctedFilter = GL_LINEAR;
              else
                correctedFilter = GL_NEAREST;

              glTexParameteri(it->second.type, GL_TEXTURE_MIN_FILTER, correctedFilter);
            }
          }
        }

      }

      if (errorDetectionLevel_ > 1)
      {
        // Why is there nothing written to the depth-buffer?
        // this might very well be intentional, so put it in higher level

        if (_renderObject->depthWrite && !_renderObject->depthTest)
        {
          std::cout << "warning: trying to write to depth buffer with depth-testing disabled does not work in: " << _renderObject->debugName << std::endl;
          std::cout << "         If depth-writing was intended, enable depth-testing and set depth-func to GL_ALWAYS" << std::endl;
        }
      }

      if (!_renderObject->vertexArrayObject)
      {
        // Why are there gl errors and/or nothing gets drawn?
        if (_renderObject->shaderDesc.shadeMode != SG_SHADE_UNLIT)
        {
          // check for normals
          if (!_renderObject->vertexDecl->findElementByUsage(VERTEX_USAGE_NORMAL))
            std::cout << "warning: missing normals for lighting in renderobject: " << _renderObject->debugName << std::endl
                      << "         Set shadeMode to SG_SHADE_UNLIT or provide normals!" << std::endl;
        }

        if (_renderObject->shaderDesc.textured())
        {
          // check for texcoords
          if (!_renderObject->vertexDecl->findElementByUsage(VERTEX_USAGE_TEXCOORD))
            std::cout << "warning: missing texcoords for textured mode in renderobject: " << _renderObject->debugName << std::endl;
        }

        if (_renderObject->shaderDesc.vertexColors)
        {
          // check for vertex colors
          if (!_renderObject->vertexDecl->findElementByUsage(VERTEX_USAGE_COLOR))
            std::cout << "warning: missing colors for vertexcolor mode in renderobject: " << _renderObject->debugName << std::endl;
        }
      }


      // Why is alpha blending not work?
      if (fabsf(_renderObject->alpha - 1.0f) > 1e-3f && !(_renderObject->alphaTest || _renderObject->blending))
        std::cout << "warning: alpha value != 1 but no alpha blending or testing enabled in renderobject: " << _renderObject->debugName << std::endl;


#ifdef GL_ARB_tessellation_shader
      // Trying to render with tessellation shaders?
      const bool tessellationActive = !_renderObject->shaderDesc.tessControlTemplateFile.isEmpty() || !_renderObject->shaderDesc.tessEvaluationTemplateFile.isEmpty();
      bool tryToFixPatchInfo = false;
      if (tessellationActive && _renderObject->primitiveMode != GL_PATCHES)
      {
        std::cout << "error: tessellation shaders are not used with GL_PATCHES primitiveType in renderobject: " << _renderObject->debugName << std::endl;
        tryToFixPatchInfo = true;
      }

      if (tessellationActive && !_renderObject->patchVertices)
      {
        std::cout << "error: undefined patch size for tessellation in renderobject: " << _renderObject->debugName << std::endl;
        tryToFixPatchInfo = true;
      }

      if (tryToFixPatchInfo)
      {
        if (_renderObject->primitiveMode == GL_POINTS)
        {
          _renderObject->primitiveMode = GL_PATCHES;
          _renderObject->patchVertices = 1;
          std::cout << "warning: attempting to draw with patchVertices = 1 for tessellation in renderobject: " << _renderObject->debugName << std::endl;
        }
        else if (_renderObject->primitiveMode == GL_LINES)
        {
          _renderObject->primitiveMode = GL_PATCHES;
          _renderObject->patchVertices = 2;
          std::cout << "warning: attempting to draw with patchVertices = 2 for tessellation in renderobject: " << _renderObject->debugName << std::endl;
        }
        else if (_renderObject->primitiveMode == GL_TRIANGLES)
        {
          _renderObject->primitiveMode = GL_PATCHES;
          _renderObject->patchVertices = 3;
          std::cout << "warning: attempting to draw with patchVertices = 3 for tessellation in renderobject: " << _renderObject->debugName << std::endl;
        }
        else if (_renderObject->primitiveMode == GL_QUADS)
        {
          _renderObject->primitiveMode = GL_PATCHES;
          _renderObject->patchVertices = 4;
          std::cout << "warning: attempting to draw with patchVertices = 4 for tessellation in renderobject: " << _renderObject->debugName << std::endl;
        }
      }

#endif
    }


    renderObjects_.push_back(*_renderObject);


    ACG::RenderObject* p = &renderObjects_.back();

    // apply modifiers
    size_t numMods = renderObjectModifiers_.size();
    for (size_t i = 0; i < numMods; ++i)
    {
      if (renderObjectModifiers_[i])
        renderObjectModifiers_[i]->apply(p);
    }

    if (!p->shaderDesc.numLights)
      p->shaderDesc.numLights = numLights_;

    else if (p->shaderDesc.numLights < 0 || p->shaderDesc.numLights >= SG_MAX_SHADER_LIGHTS)
      p->shaderDesc.numLights = 0;

    p->internalFlags_ = 0;

    // precompile shader
#ifdef GL_VERSION_3_2
    GLSL::Program* shaderProg = ACG::ShaderCache::getInstance()->getProgram(&p->shaderDesc);
#endif


    // check primitive type and geometry shader
    if (errorDetectionLevel_ > 1 && p->shaderDesc.geometryTemplateFile.length())
    {
#ifdef GL_VERSION_3_2
      GLint geomInputType = 0;
      glGetProgramiv(shaderProg->getProgramId(), GL_GEOMETRY_INPUT_TYPE, &geomInputType);


      if (geomInputType != GLint(p->primitiveMode))
      {

        switch (geomInputType)
        {
        case GL_POINTS: std::cout << "warning: primitive mode is incompatible with points-geometryshader in renderobject: " << _renderObject->debugName << std::endl; break;
        
        case GL_LINES: 
          {
            if (p->primitiveMode != GL_LINE_STRIP && p->primitiveMode != GL_LINE_LOOP)
              std::cout << "warning: primitive mode is incompatible with lines-geometryshader in renderobject: " << _renderObject->debugName << std::endl;
          } break;

        case GL_LINES_ADJACENCY:
          {
            if (p->primitiveMode != GL_LINE_STRIP_ADJACENCY)
              std::cout << "warning: primitive mode is incompatible with lines_adjacency-geometryshader in renderobject: " << _renderObject->debugName << std::endl;
          } break;

        case GL_TRIANGLES:
          {
            if (p->primitiveMode != GL_TRIANGLE_STRIP && p->primitiveMode != GL_TRIANGLE_FAN)
              std::cout << "warning: primitive mode is incompatible with triangles-geometryshader in renderobject: " << _renderObject->debugName << std::endl;
          } break;
          
        case GL_TRIANGLES_ADJACENCY:
          {
            if (p->primitiveMode != GL_TRIANGLE_STRIP_ADJACENCY)
              std::cout << "warning: primitive mode is incompatible with triangles_adjacency-geometryshader in renderobject: " << _renderObject->debugName << std::endl;
          } break;

        default: std::cout << "warning: unknown input_type for geometryshader in renderobject: " << _renderObject->debugName << std::endl;
        }
        
        

      }


#else
      if (_renderObject->isDefaultLineObject())
      {
        if (_renderObject->primitiveMode != GL_LINES && _renderObject->primitiveMode != GL_LINE_STRIP && _renderObject->primitiveMode != GL_LINE_LOOP)
          std::cout << "warning: primitive mode is incompatible with lines-geometryshader in renderobject: " << _renderObject->debugName << std::endl;
      }
      else if (_renderObject->isDefaultPointObject())
      {
        if (_renderObject->primitiveMode != GL_POINTS)
          std::cout << "warning: primitive mode is incompatible with points-geometryshader in renderobject: " << _renderObject->debugName << std::endl;
      }
#endif
    }

  }
}




void IRenderer::collectRenderObjects( ACG::GLState* _glState, ACG::SceneGraph::DrawModes::DrawMode _drawMode, ACG::SceneGraph::BaseNode* _sceneGraphRoot )
{
  // collect light sources
//  collectLightNodes(_sceneGraphRoot);
  numLights_ = 0; // reset light counter

//  // flush render objects
//  for (size_t i = 0; i < renderObjects_.size(); ++i)
//  {
//    renderObjects_[i].uniformPool_.clear();
//  }
  renderObjects_.clear();

  if (!_sceneGraphRoot) return;

  // default material needed
  ACG::SceneGraph::Material defMat;
  defMat.baseColor(ACG::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
  defMat.ambientColor(ACG::Vec4f(0.2f, 0.2f, 0.2f, 1.0f));
  defMat.diffuseColor(ACG::Vec4f(0.6f, 0.6f, 0.6f, 1.0f));
  defMat.specularColor(ACG::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
  defMat.shininess(1.0f);
  //  defMat.alphaValue(1.0f);

  // collect renderables
  traverseRenderableNodes(_glState, _drawMode, *_sceneGraphRoot, defMat);
}


/// Stack element used in IRenderer::traverseRenderableNodes
namespace {
class ScenegraphTraversalStackEl {
    public:
        ScenegraphTraversalStackEl(ACG::SceneGraph::BaseNode *_node,
                const ACG::SceneGraph::Material *_material) :
            node(_node), material(_material),
            subtree_index_start(0), leave(false) {}

        ACG::SceneGraph::BaseNode *node;
        const ACG::SceneGraph::Material* material;
        size_t subtree_index_start;
        bool leave;
};
}

void IRenderer::traverseRenderableNodes( ACG::GLState* _glState, ACG::SceneGraph::DrawModes::DrawMode _drawMode, ACG::SceneGraph::BaseNode &_node, const ACG::SceneGraph::Material &_mat )
{
    renderObjectSource_.clear();
    overlayObjectSource_.clear();

    if (_node.status() == ACG::SceneGraph::BaseNode::HideSubtree)
        return;

    std::vector<ScenegraphTraversalStackEl> stack;
    // That's roughly the minimum size every scenegraph requries.
    stack.reserve(32);
    stack.push_back(ScenegraphTraversalStackEl(&_node, &_mat));
    while (!stack.empty()) {
        ScenegraphTraversalStackEl &cur = stack.back();
        auto cur_idx = stack.size() - 1;
        ACG::SceneGraph::DrawModes::DrawMode nodeDM = cur.node->drawMode();
        if (nodeDM == ACG::SceneGraph::DrawModes::DEFAULT)
          nodeDM = _drawMode;

        if (!cur.leave) {
            /*
             * Stuff that happens before processing cur.node's children.
             */
            if ( cur.node->status() != ACG::SceneGraph::BaseNode::HideNode )
                cur.node->enter(this, *_glState, nodeDM);

            cur.subtree_index_start = renderObjects_.size();

            // fetch material (Node itself can be a material node, so we have to
            // set that in front of the nodes own rendering
            ACG::SceneGraph::MaterialNode* matNode =
                    dynamic_cast<ACG::SceneGraph::MaterialNode*>(cur.node);
            if (matNode)
              cur.material = &matNode->material();

            if (cur.node->status() != ACG::SceneGraph::BaseNode::HideNode)
                cur.node->getRenderObjects(this, *_glState, nodeDM, cur.material);

            // keep track of which node added objects
            size_t numAddedObjects = renderObjects_.size() - renderObjectSource_.size();
            renderObjectSource_.insert(renderObjectSource_.end(), numAddedObjects, cur.node);

            auto cur_mat = cur.material; // make a copy so we can avoid use-after-free on stack.push_back
            // Process children?
            if (cur.node->status() != ACG::SceneGraph::BaseNode::HideChildren) {
                // Process all children which are second pass
                for (ACG::SceneGraph::BaseNode::ChildIter
                        cIt = cur.node->childrenBegin(),
                        cEnd = cur.node->childrenEnd(); cIt != cEnd; ++cIt) {
                  if (((*cIt)->traverseMode() &
                          ACG::SceneGraph::BaseNode::SecondPass) &&
                          (*cIt)->status() != ACG::SceneGraph::BaseNode::HideSubtree)
                      stack.emplace_back(*cIt, cur_mat);
                }

                // Process all children which are not second pass
                for (ACG::SceneGraph::BaseNode::ChildIter
                        cIt = cur.node->childrenBegin(),
                        cEnd = cur.node->childrenEnd(); cIt != cEnd; ++cIt) {
                  if ((~(*cIt)->traverseMode() &
                          ACG::SceneGraph::BaseNode::SecondPass) &&
                          (*cIt)->status() != ACG::SceneGraph::BaseNode::HideSubtree)
                      stack.emplace_back(*cIt, cur_mat);
                }
            }

            // Next time process the other branch of this if statement.
            // 'cur' is now invalidated due to push_back, access via stack
            stack[cur_idx].leave = true;

        } else {
            /*
             * Stuff that happens after processing cur.node's children.
             */
            current_subtree_objects_ = RenderObjectRange(
                    renderObjects_.begin() + cur.subtree_index_start,
                    renderObjects_.end());

            if (cur.node->status() != ACG::SceneGraph::BaseNode::HideNode )
                cur.node->leave(this, *_glState, nodeDM);
            stack.pop_back();
        }
    }
}

void IRenderer::prepareRenderingPipeline(ACG::GLState* _glState, ACG::SceneGraph::DrawModes::DrawMode _drawMode, ACG::SceneGraph::BaseNode* _scenegraphRoot)
{
  // save default VAO
#ifdef GL_ARB_vertex_array_object
  glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &prevVAO_);
#endif

  coreProfile_ = !_glState->compatibilityProfile();

  // grab view transform from glstate
  viewMatrix_ = _glState->modelview();
  camPosWS_ = Vec3f( viewMatrix_(0,3), viewMatrix_(1,3), viewMatrix_(2,3) );
  camDirWS_ = Vec3f( viewMatrix_(0,2), viewMatrix_(1,2), -viewMatrix_(2,2) ); // mind the z flip

//   printf("pos: %f %f %f\ndir: %f %f %f\n", camPosWS_[0], camPosWS_[1], camPosWS_[2],
//     camDirWS_[0], camDirWS_[1], camDirWS_[2]);

  // First, all render objects get collected.
  collectRenderObjects(_glState, _drawMode, _scenegraphRoot);

  // ==========================================================
  // Sort renderable objects based on their priority
  // Filter for overlay, lines etc.
  // ==========================================================

  size_t numRenderObjects = 0,
    numOverlayObjects = 0,
    numLineObjects = 0;

  for (std::vector<ACG::RenderObject>::const_iterator it = renderObjects_.begin();
          it != renderObjects_.end(); ++it) {
      if (!it->overlay && !(it->isDefaultLineObject() && enableLineThicknessGL42_))
          numRenderObjects++;
      if (it->overlay)
          numOverlayObjects++;
      if (enableLineThicknessGL42_ && it->isDefaultLineObject())
          numLineObjects++;
  }

  /*
   * Neither clear() nor resize() ever decreases the capacity of
   * a vector. So it has no adverse impact on performance to clear
   * the vectors here.
   */
  sortedObjects_.clear();
  sortedObjects_.reserve(numRenderObjects);

  overlayObjects_.clear();
  overlayObjects_.reserve(numOverlayObjects);

  lineGL42Objects_.clear();
  lineGL42Objects_.reserve(numLineObjects);


  sortListObjects_.clear();
  sortListObjects_.reserve(numRenderObjects);

  sortListOverlays_.clear();
  sortListOverlays_.reserve(numOverlayObjects);

  // init sorted objects array
  for (size_t i = 0; i < renderObjects_.size(); ++i)
  {
    if (renderObjects_[i].overlay)
    {
      overlayObjects_.push_back(&renderObjects_[i]);
      sortListOverlays_.push_back(i);
    }
    else if (enableLineThicknessGL42_ && numLineObjects && renderObjects_[i].isDefaultLineObject())
    {
      renderObjects_[i].shaderDesc.geometryTemplateFile = "Wireframe/gl42/geometry.tpl";
      renderObjects_[i].shaderDesc.fragmentTemplateFile = "Wireframe/gl42/fragment.tpl";

      // disable color write to fbo, but allow RenderObject to control depth write
      renderObjects_[i].glColorMask(0,0,0,0);

//      sortedObjects_[sceneObjectOffset++] = &renderObjects_[i];
      lineGL42Objects_.push_back(&renderObjects_[i]);
    }
    else
    {
      sortedObjects_.push_back(&renderObjects_[i]);
      sortListObjects_.push_back(i);
    }
  }

  sortRenderObjects();


  // ---------------------------
  // Initialize the render state
  // ---------------------------
  // gl cleanup

  if (_glState->compatibilityProfile())
  {
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_INDEX_ARRAY);
  }

  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glDepthMask(GL_TRUE);

  // save active fbo and viewport
  saveInputFbo();

if(_glState->compatibilityProfile())
{
  // get global ambient factor
  GLfloat lightModelAmbient[4];
  glGetFloatv(GL_LIGHT_MODEL_AMBIENT, lightModelAmbient);
  globalLightModelAmbient_ = ACG::Vec3f(lightModelAmbient[0], lightModelAmbient[1], lightModelAmbient[2]);
}

  // search list of render object for objects requiring the scene depth map
  bool requiresSceneDepths = false;

  for (size_t i = 0; i < renderObjects_.size(); ++i)
  {
    if (renderObjects_[i].depthMapUniformName)
      requiresSceneDepths = true;
  }

  // render scene depth map
  if (requiresSceneDepths)
    renderDepthMap(curViewerID_, _glState->viewport_width(), _glState->viewport_height());
}



void IRenderer::finishRenderingPipeline(bool _drawOverlay)
{
#ifdef GL_ARB_vertex_array_object
  glBindVertexArray(prevVAO_);
#endif

  // draw thick lines
  if (enableLineThicknessGL42_)
    renderLineThicknessGL42();

  if (_drawOverlay)
  {
    const int numOverlayObj = overlayObjects_.size();
    // two-pass overlay rendering:
    // 1. clear depth buffer at pixels of overlay objects
    // 2. render overlay with correct depth-testing

    // 1. pass: clear depth to max value at overlay footprint
    for (int i = 0; i < numOverlayObj; ++i)
    {
      RenderObject* obj = getOverlayRenderObject(i);

      if (obj->depthTest && obj->depthFunc != GL_ALWAYS)
      {
        float depthMax = 1.0f;

        if (obj->depthFunc == GL_GREATER || obj->depthFunc == GL_GEQUAL)
          depthMax = 0.0f;

        // save depth setting of renderobject
        Vec2f depthRange = obj->depthRange;
        bool depthWrite = obj->depthWrite;
        GLenum depthFunc = obj->depthFunc;

        // reset depth to maximal distance by using depth range
        obj->depthRange = Vec2f(depthMax, depthMax);
        obj->depthWrite = true;
        obj->depthFunc = GL_ALWAYS;

        renderObject(obj);

        // restore depth setting
        obj->depthRange = depthRange;
        obj->depthWrite = depthWrite;
        obj->depthFunc = depthFunc;
      }

    }

    // 2. render overlay with correct depth-testing
    for (int i = 0; i < numOverlayObj; ++i)
    {
      RenderObject* obj = getOverlayRenderObject(i);
      renderObject(obj);
    }

  }

#ifdef GL_ARB_vertex_array_object
  glBindVertexArray(prevVAO_);
#endif

  for (int i = 0; i < maxClipDistances_; ++i)
    glDisable(GL_CLIP_DISTANCE0 + i);

  glDisable(GL_PROGRAM_POINT_SIZE);

  glDepthMask(1);
  glColorMask(1,1,1,1);

  glUseProgram(0);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  // Renderer check:
  // Print a warning if the currently active fbo / viewport is not the same as the saved fbo.
  // Restore to previous fbo and viewport if not done already.

  GLint curFbo;
  GLint curViewport[4];
  GLint curDrawBuf;
  glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &curFbo);
  glGetIntegerv(GL_VIEWPORT, curViewport);
  glGetIntegerv(GL_DRAW_BUFFER, &curDrawBuf);
  
  if (curFbo != prevFbo_)
  {
    std::cout << "warning: input and output fbo are not the same in renderer implementation" << std::endl;
    restoreInputFbo();
  }

  if (curDrawBuf != prevDrawBuffer_)
  {
    std::cout << "warning: input and output draw-buffer are not the same in renderer implementation" << std::endl;
    restoreInputFbo();
  }

  if (curViewport[0] != prevViewport_[0] ||
    curViewport[1] != prevViewport_[1] ||
    curViewport[2] != prevViewport_[2] ||
    curViewport[3] != prevViewport_[3])
  {
    std::cout << "warning: input and output viewport are not the same in renderer implementation" << std::endl;
    restoreInputFbo();
  }
}


void IRenderer::saveInputFbo()
{
  // save active fbo
  saveActiveFbo(&prevFbo_, prevViewport_, &prevDrawBuffer_);
  prevFboSaved_ = true;
}

void IRenderer::restoreInputFbo()
{
  if (prevFboSaved_)
    restoreFbo(prevFbo_, prevViewport_, prevDrawBuffer_);
}

void IRenderer::saveActiveFbo( GLint* _outFboId, GLint* _outViewport, GLint* _outDrawBuffer ) const
{
  // save active fbo
  glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, _outFboId);
  glGetIntegerv(GL_VIEWPORT, _outViewport);
  glGetIntegerv(GL_DRAW_BUFFER, _outDrawBuffer);
}

void IRenderer::restoreFbo( GLint _fboId, const GLint* _outViewport, GLint _drawBuffer ) const
{
  glBindFramebuffer(GL_FRAMEBUFFER, _fboId);
  glDrawBuffer(_drawBuffer);
  glViewport(_outViewport[0], _outViewport[1], _outViewport[2], _outViewport[3]);
}

void IRenderer::clearInputFbo( const ACG::Vec4f& clearColor )
{
  glClearColor(clearColor[0], clearColor[1], clearColor[2], 1.0f);

  // glClear will affect the whole back buffer, not only the area in viewport.
  // Temporarily use glScissor to only clear the viewport area in the back buffer.
  if (!prevFboSaved_)
  {
    GLint oldViewport[4];
    glGetIntegerv(GL_VIEWPORT, oldViewport);
    glScissor(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);
  }
  else
    glScissor(prevViewport_[0], prevViewport_[1], prevViewport_[2], prevViewport_[3]);

  glEnable(GL_SCISSOR_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // disable scissors again, draw calls only affect the area in glViewport anyway
  glDisable(GL_SCISSOR_TEST);
}

namespace {
struct RenderObjectComparator {
    explicit RenderObjectComparator(const std::vector<ACG::RenderObject>& _vec) : vec_(_vec) {
    }

    bool operator() (int a, int b) {
        return (vec_[a].priority < vec_[b].priority);
    }

    const std::vector<ACG::RenderObject>& vec_;
};
}

void IRenderer::sortRenderObjects()
{
  size_t numObjs = sortListObjects_.size();
  size_t numOverlays = sortListOverlays_.size();

  RenderObjectComparator cmpOp(renderObjects_);

  std::sort(sortListObjects_.begin(), sortListObjects_.end(), cmpOp);
  std::sort(sortListOverlays_.begin(), sortListOverlays_.end(), cmpOp);

  // apply sorting list
  std::vector<ACG::SceneGraph::BaseNode*> temp;
  renderObjectSource_.swap(temp);


  renderObjectSource_.resize(numObjs, 0);
  overlayObjectSource_.resize(numOverlays, 0);

  for (size_t i = 0; i < numObjs; ++i)
  {
    int objID = sortListObjects_[i];
    sortedObjects_[i] = &renderObjects_[objID];

    renderObjectSource_[i] = temp[objID];
  }

  for (size_t i = 0; i < numOverlays; ++i)
  {
    int objID = sortListOverlays_[i];
    overlayObjects_[i] = &renderObjects_[objID];

    overlayObjectSource_[i] = temp[objID];
  }
}



void IRenderer::bindObjectVBO(ACG::RenderObject* _obj,
                                       GLSL::Program*     _prog)
{
  _prog->use();


#ifdef GL_ARB_vertex_array_object
  if (_obj->vertexArrayObject)
    glBindVertexArray(_obj->vertexArrayObject);
#endif

  if (!_obj->vertexArrayObject)
  {
    //////////////////////////////////////////////////////////////////////////
    // NOTE:
    //  always bind buffers before glVertexAttribPointer calls!!
    //  freeze in glDrawElements guaranteed (with no error message whatsoever)
    glBindBuffer(GL_ARRAY_BUFFER, _obj->vertexBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _obj->indexBuffer);


    // activate vertex declaration
    _obj->vertexDecl->activateShaderPipeline(_prog);

  }
}


void IRenderer::bindObjectUniforms( ACG::RenderObject* _obj, GLSL::Program* _prog )
{
  // transforms
  ACG::GLMatrixf mvp = _obj->proj * _obj->modelview;
  ACG::GLMatrixf mvIT = _obj->modelview;
  mvIT.invert();
  mvIT.transpose();

  _prog->setUniform("g_mWVP", mvp);
  _prog->setUniform("g_mWV", _obj->modelview);
  _prog->setUniformMat3("g_mWVIT", mvIT);
  _prog->setUniform("g_mP", _obj->proj);

  _prog->setUniform("g_vCamPos", camPosWS_);
  _prog->setUniform("g_vCamDir", camDirWS_);

  // material
  
  _prog->setUniform("g_cEmissive", _obj->emissive);

  _prog->setUniform("g_cDiffuse", _obj->diffuse);
  _prog->setUniform("g_cAmbient", _obj->ambient);
  _prog->setUniform("g_cSpecular", _obj->specular);


  float alphaRef = 0.f;
  if (_obj->alphaTest && _obj->alphaFunc == GL_GREATER) {
    alphaRef = _obj->alphaRef;
  }
  ACG::Vec4f materialParams(_obj->shininess, _obj->alpha, alphaRef, 0.0f);
  _prog->setUniform("g_vMaterial", materialParams);

  _prog->setUniform("g_cLightModelAmbient", globalLightModelAmbient_);

  // Additional Uniforms defined in the render Object
  if ( !_obj->uniformPool_.empty() )
    _obj->uniformPool_.bind(_prog);

  // textures:

  // maybe consider using bindless textures some time in the future
  // to get rid of the old and problematic glActiveTexture(), glBindTexture() state machine usage:
  // https://www.opengl.org/registry/specs/ARB/bindless_texture.txt

  int maxTextureStage = 0;
  for (std::map<size_t,RenderObject::Texture>::const_iterator iter = _obj->textures().begin();
      iter != _obj->textures().end();++iter)
  {
    //check for valid texture id
    const size_t texture_stage = iter->first;
    const RenderObject::Texture tex = iter->second;
    if (!tex.id)
      continue;

    glActiveTexture(GL_TEXTURE0 + (GLenum)texture_stage);
    glBindTexture(iter->second.type, tex.id);
    _prog->setUniform(QString("g_Texture%1").arg(texture_stage).toStdString().c_str(), (int)texture_stage);

    maxTextureStage = std::max(maxTextureStage, (int)texture_stage);
  }


  // scene depth texture
  if (_obj->depthMapUniformName)
  {
    // bind to (hopefully) unused texture stage
    int depthMapSlot = maxTextureStage + 1;

    _prog->setUniform(_obj->depthMapUniformName, depthMapSlot);
    glActiveTexture(GL_TEXTURE0 + depthMapSlot);
    glBindTexture(GL_TEXTURE_2D, depthMaps_[curViewerID_]->getAttachment(GL_COLOR_ATTACHMENT0));
  }


  // lights
  for (int i = 0; i < numLights_; ++i)
  {
    LightData* lgt = lights_ + i;

    auto idx_str = std::to_string(i);

    _prog->setUniform(("g_cLightDiffuse_" + idx_str).c_str(), lgt->diffuse);
    _prog->setUniform(("g_cLightAmbient_" + idx_str).c_str(), lgt->ambient);
    _prog->setUniform(("g_cLightSpecular_" + idx_str).c_str(), lgt->specular);

    if (lgt->ltype == ACG::SG_LIGHT_POINT || lgt->ltype == ACG::SG_LIGHT_SPOT)
    {
      _prog->setUniform(("g_vLightPos_" + idx_str).c_str(), lgt->pos);
      _prog->setUniform(("g_vLightAtten_" + idx_str).c_str(), lgt->atten);
    }

    if (lgt->ltype == ACG::SG_LIGHT_DIRECTIONAL || lgt->ltype == ACG::SG_LIGHT_SPOT)
    {
      _prog->setUniform(("g_vLightDir_" + idx_str).c_str(), lgt->dir);
    }

    if (lgt->ltype == ACG::SG_LIGHT_SPOT)
    {
      _prog->setUniform(("g_vLightAngleExp_" + idx_str).c_str(), lgt->spotCutoffExponent);
    }
  }
}

void IRenderer::bindObjectRenderStates(ACG::RenderObject* _obj)
{
  if (_obj->culling)
    glEnable(GL_CULL_FACE);
  else
    glDisable(GL_CULL_FACE);

  if (_obj->blending)
    glEnable(GL_BLEND);
  else
    glDisable(GL_BLEND);

  if (!coreProfile_)
  {
    if (_obj->alphaTest)
    {
      glEnable(GL_ALPHA_TEST);
      glAlphaFunc(_obj->alphaFunc, _obj->alphaRef);
    }
    else
      glDisable(GL_ALPHA_TEST);
  }

  if (_obj->depthTest)
    glEnable(GL_DEPTH_TEST);
  else
    glDisable(GL_DEPTH_TEST);


  glDepthMask(_obj->depthWrite ? GL_TRUE : GL_FALSE);

  glColorMask(_obj->colorWriteMask[0], _obj->colorWriteMask[1], _obj->colorWriteMask[2], _obj->colorWriteMask[3]);

  glDepthFunc(_obj->depthFunc);

  glDepthRange(_obj->depthRange[0], _obj->depthRange[1]);

  //  ACG::GLState::shadeModel(_obj->shadeModel);

  ACG::GLState::blendFunc(_obj->blendSrc, _obj->blendDest);

  if (maxClipDistances_ < 0)
  {
    glGetIntegerv(GL_MAX_CLIP_DISTANCES, &maxClipDistances_);
    maxClipDistances_ = std::min(maxClipDistances_, 32); // clamp to 32 bits
  }

  for (int i = 0; i < maxClipDistances_; ++i)
  {
    if (_obj->clipDistanceMask & (1 << i))
      glEnable(GL_CLIP_DISTANCE0 + i);
    else
      glDisable(GL_CLIP_DISTANCE0 + i);
  }

#ifdef GL_PROGRAM_POINT_SIZE
  if (_obj->programPointSize)
    glEnable(GL_PROGRAM_POINT_SIZE);
  else
    glDisable(GL_PROGRAM_POINT_SIZE);
#endif

  glPointSize(_obj->pointSize);
}

void IRenderer::drawObject(ACG::RenderObject* _obj)
{
  if (_obj->numIndices)
  {
    // indexed drawing?
    bool noIndices = true;
    if (_obj->indexBuffer || _obj->sysmemIndexBuffer)
      noIndices = false;

    glPolygonMode(GL_FRONT_AND_BACK, _obj->fillMode);

    // tessellation info
    bool tessellationActive = !(_obj->shaderDesc.tessControlTemplateFile.isEmpty() && _obj->shaderDesc.tessEvaluationTemplateFile.isEmpty());
#ifdef GL_ARB_tessellation_shader
    if (tessellationActive)
    {
      glPatchParameteri(GL_PATCH_VERTICES, _obj->patchVertices);

      if (_obj->shaderDesc.tessControlTemplateFile.isEmpty())
      {
        glPatchParameterfv(GL_PATCH_DEFAULT_INNER_LEVEL, _obj->patchDefaultInnerLevel.data());
        glPatchParameterfv(GL_PATCH_DEFAULT_OUTER_LEVEL, _obj->patchDefaultOuterLevel.data());
      }
    }
#else
    if (tessellationActive)
      std::cout << "error: tessellation shaders cannot be used with the outdated glew version" << std::endl;
#endif

    if (noIndices) {
      if (_obj->numInstances <= 0)
        glDrawArrays(_obj->primitiveMode, _obj->indexOffset, _obj->numIndices);
      else
        glDrawArraysInstanced(_obj->primitiveMode, _obj->indexOffset, _obj->numIndices, _obj->numInstances);
    }
    else
    {
      // ------------------------------------------
      // index offset stuff not tested
      int indexSize = 0;
      switch (_obj->indexType)
      {
      case GL_UNSIGNED_INT: indexSize = 4; break;
      case GL_UNSIGNED_SHORT: indexSize = 2; break;
      default: indexSize = 1; break;
      }

      if (_obj->numInstances <= 0)
        glDrawElements(_obj->primitiveMode, _obj->numIndices, _obj->indexType,
          ((const char*)_obj->sysmemIndexBuffer) + _obj->indexOffset * indexSize);
      else
        glDrawElementsInstanced(_obj->primitiveMode, _obj->numIndices, _obj->indexType,
          ((const char*)_obj->sysmemIndexBuffer) + _obj->indexOffset * indexSize, _obj->numInstances);
    }
  }
}

void IRenderer::renderObject(ACG::RenderObject* _obj, 
                                      GLSL::Program* _prog,
                                      bool _constRenderStates,
                                      const std::vector<unsigned int>* _shaderModifiers)
{
  // select shader from cache
  GLSL::Program* prog = _prog ? _prog : ACG::ShaderCache::getInstance()->getProgram(&_obj->shaderDesc, _shaderModifiers);


  bindObjectVBO(_obj, prog);

  // ---------------------------------------
  // set shader uniforms

  bindObjectUniforms(_obj, prog);

  // render states

  if (!_constRenderStates)
    bindObjectRenderStates(_obj);

  // ----------------------------
  // OpenGL draw call

  drawObject(_obj);


  ACG::glCheckErrors();

  if (_obj->vertexDecl)
  {
    // deactivate vertex declaration to avoid errors
    _obj->vertexDecl->deactivateShaderPipeline(prog);
  }


#ifdef GL_ARB_vertex_array_object
  if (_obj->vertexArrayObject)
    glBindVertexArray(prevVAO_);
#endif

}


void IRenderer::addLight(const LightData& _light)
{
  if (numLights_ < SG_MAX_SHADER_LIGHTS)
  {
    lights_[numLights_] = _light;

    // normalize direction
    if (_light.ltype != SG_LIGHT_POINT)
      lights_[numLights_].dir.normalize();

    ++numLights_;
  }
}

void IRenderer::addRenderObjectModifier(RenderObjectModifier* _mod)
{
  renderObjectModifiers_.push_back(_mod);
}

void IRenderer::removeRenderObjectModifier(RenderObjectModifier* _mod)
{
  for (int i = int(renderObjectModifiers_.size()) - 1; i >= 0; --i)
  {
    if (renderObjectModifiers_[i] == _mod)
      renderObjectModifiers_.erase(renderObjectModifiers_.begin() + i);
  }
}

int IRenderer::getNumRenderObjects() const {
    return sortedObjects_.size();
}

int IRenderer::getNumLights() const
{
  return numLights_;
}


ACG::RenderObject* IRenderer::getRenderObject( int i )
{
  if (sortedObjects_.empty())
    return &renderObjects_[i];
  
  return sortedObjects_[i];
}

ACG::RenderObject* IRenderer::getOverlayRenderObject( int i )
{
  if (overlayObjects_.empty())
    return &renderObjects_[i];

  return overlayObjects_[i];
}


ACG::SceneGraph::BaseNode* IRenderer::getRenderObjectNode( int i )
{
  return renderObjectSource_[i];
}

ACG::SceneGraph::BaseNode* IRenderer::getOverlayRenderObjectNode( int i )
{
  return overlayObjectSource_[i];
}


ACG::RenderObject* IRenderer::getLineGL42RenderObject( int i )
{
  if (lineGL42Objects_.empty())
    return 0;

  return lineGL42Objects_[i];
}

IRenderer::LightData* IRenderer::getLight( int i )
{
  return &lights_[i];
}


void IRenderer::dumpRenderObjectsToFile(const char* _fileName, ACG::RenderObject** _sortedList) const
{
  QFile fileOut(_fileName);
  if (fileOut.open(QFile::WriteOnly | QFile::Truncate))
  {
    QTextStream outStrm(&fileOut);
    for (int i = 0; i < getNumRenderObjects(); ++i)
    {
      if (_sortedList)
        outStrm << "\n" << _sortedList[i]->toString();
      else
        outStrm << "\n" << renderObjects_[i].toString();
    }

    fileOut.close();
  }
}


QString IRenderer::dumpCurrentRenderObjectsToString(ACG::RenderObject** _list, bool _outputShaders, std::vector<ACG::ShaderModifier*>* _modifiers) {

  QString objectString;

  QTextStream outStrm(&objectString);

  for (int i = 0; i < getNumRenderObjects(); ++i)
  {
    const RenderObject* obj = _list ? _list[i] : &renderObjects_[i];

    if (obj) {
      outStrm << "\n" << obj->toString();

      if ( _outputShaders ) {

        outStrm << "\n";

        outStrm << obj->shaderDesc.toString();

        ShaderProgGenerator progGen(&(obj->shaderDesc), _modifiers);

        outStrm << "\n---------------------vertex-shader--------------------\n\n";
        for (int nr = 0; nr < progGen.getVertexShaderCode().size(); ++nr)
          outStrm << progGen.getVertexShaderCode()[nr] << "\n";
        outStrm << "\n---------------------end-vertex-shader--------------------\n\n";

        if (progGen.hasTessControlShader())
        {
          outStrm << "\n---------------------tesscontrol-shader--------------------\n\n";
          for (int nr = 0; nr < progGen.getTessControlShaderCode().size(); ++nr)
            outStrm << progGen.getTessControlShaderCode()[nr] << "\n";
          outStrm << "\n---------------------end-tesscontrol-shader--------------------\n\n";
        }

        if (progGen.hasTessControlShader())
        {
          outStrm << "\n---------------------tesseval-shader--------------------\n\n";
          if (progGen.hasTessEvaluationShader())
            for (int nr = 0; nr < progGen.getTessEvaluationShaderCode().size(); ++nr)
              outStrm << progGen.getTessEvaluationShaderCode()[nr] << "\n";
          outStrm << "\n---------------------end-tesseval-shader--------------------\n\n";
        }

        if (progGen.hasGeometryShader())
        {
          outStrm << "\n---------------------geometry-shader--------------------\n\n";
          for (int nr = 0; nr < progGen.getGeometryShaderCode().size(); ++nr)
            outStrm << progGen.getGeometryShaderCode()[nr] << "\n";
          outStrm << "\n---------------------end-geometry-shader--------------------\n\n";
        }


        outStrm << "\n---------------------fragment-shader--------------------\n\n";
        for (int nr = 0; nr < progGen.getFragmentShaderCode().size(); ++nr)
          outStrm << progGen.getFragmentShaderCode()[nr] << "\n";
        outStrm << "\n---------------------end-fragment-shader--------------------\n\n";
      }

    } 

  }

  return objectString;
}

void IRenderer::copyDepthToBackBuffer( GLuint _depthTex, float _scale /*= 1.0f*/ )
{
  if (!_depthTex) return;
#ifdef __APPLE__
  if(ACG::openGLVersion(3,3))
  {
      if (!depthCopyShader_)
        depthCopyShader_ = GLSL::loadProgram("ScreenQuad/screenquad.glsl", "ScreenQuad/depth_copy_330.glsl");
  }
  else
#endif
  {
  if (!depthCopyShader_)
    depthCopyShader_ = GLSL::loadProgram("ScreenQuad/screenquad.glsl", "ScreenQuad/depth_copy.glsl");
  }

  if (depthCopyShader_)
  {
    // save important opengl states
    GLint curFbo;
    GLint curViewport[4];
    GLint curDrawBuffer;
    saveActiveFbo(&curFbo, curViewport, &curDrawBuffer);

    GLboolean colorMask[4], depthMask;
    glGetBooleanv(GL_COLOR_WRITEMASK, colorMask);
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);

    GLboolean depthTestEnable;
    GLint depthFunc;
    glGetBooleanv(GL_DEPTH_TEST, &depthTestEnable);
    glGetIntegerv(GL_DEPTH_FUNC, &depthFunc);

    // write to depth buffer of input fbo
    restoreInputFbo();

    depthCopyShader_->use();
    depthCopyShader_->setUniform("offset", ACG::Vec2f(0.0f, 0.0f));
    depthCopyShader_->setUniform("size", ACG::Vec2f(1.0f, 1.0f));
    depthCopyShader_->setUniform("DepthTex", 0); // depth tex at texture slot 0
    depthCopyShader_->setUniform("DepthSign", _scale);

    // write to depth buffer only, disable writing to color buffer
    glColorMask(0,0,0,0);
    glDepthMask(1);

    // depth test enabled + pass always 
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _depthTex);

    ACG::ScreenQuad::draw(depthCopyShader_);


    // restore important opengl states

    restoreFbo(curFbo, curViewport, curDrawBuffer);

    glColorMask(colorMask[0], colorMask[1], colorMask[2], colorMask[3]);
    glDepthMask(depthMask);

    if (!depthTestEnable)
      glDisable(GL_DEPTH_TEST);

    if (depthFunc != GL_ALWAYS)
      glDepthFunc(depthFunc);

    glBindTexture(GL_TEXTURE_2D, 0);
  }
}


// depth map computation with a modifier

IRenderer::DepthMapPass IRenderer::DepthMapPass::instance;

void IRenderer::DepthMapPass::modifyFragmentEndCode(QStringList* _code)
{
  _code->push_back("outFragment = gl_FragCoord.zzzz;");
}

void IRenderer::renderDepthMap(int _viewerID, int _width, int _height)
{
  ACG::FBO* fbo = depthMaps_[_viewerID];

  // setup fbo for depth map
  if (!fbo)
  {
    fbo = depthMaps_[_viewerID] = new ACG::FBO();

    fbo->bind();
    fbo->attachTexture2D(GL_COLOR_ATTACHMENT0, _width, _height, GL_R32F, GL_RED);
    fbo->addDepthBuffer(_width, _height);
  }
  else
  {
    fbo->bind();
    fbo->resize(_width, _height);
  }

  // make sure modifier is registered
  ACG::ShaderProgGenerator::registerModifier(&DepthMapPass::instance);

  /* note: 
    It is possible to directly read the depth buffer if it was attached as a texture.
    Then, we would disable the color write mask and just write to the depth buffer in this function.

    However, doing this has shown that the actual depth values written to the depth buffer highly depend on the gpu driver.
    Or, at least sampling from a texture with format GL_DEPTH_COMPONENT returns driver dependent values.
    For instance, in the amd-gl driver sampling the depth texture returns the value of gl_FragDepth (as expected).
    But in the nvidia driver, sampling directly from the depth buffer returns something different!

    Therefore we render the value of gl_FragDepth to a custom floating-point texture bound to a color slot in order to get driver independent behavior.
  */

  fbo->bind();
  glDrawBuffer(GL_COLOR_ATTACHMENT0);
  glViewport(0, 0, _width, _height);

  // set depth to max
  glColorMask(1,1,1,1);
  glDepthMask(1);

  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // render z-prepass
  for (int i = 0; i < getNumRenderObjects(); ++i)
  {
    RenderObject* obj = getRenderObject(i);

    if (obj->inZPrePass)
    {
      // apply depth map modifier to get the depth pass shader
      GLSL::Program* depthPassShader = ShaderCache::getInstance()->getProgram(&obj->shaderDesc, DepthMapPass::instance);

      // temporarily prevent read/write access to the same texture (the depth map)
      const char* depthMapUniformName = obj->depthMapUniformName;
      obj->depthMapUniformName = 0;

      if (depthMapUniformName)
      {
        depthPassShader->use();
        depthPassShader->setUniform(depthMapUniformName, 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, 0);
      }

      // we are interested in the depth value only, so temporarily modify the write mask to allow writing to the red channel
      // (for instance, an object might not write a color value, but a depth value)
      const GLboolean redWriteMask = obj->colorWriteMask[0];
      obj->colorWriteMask[0] = obj->depthWrite;

      renderObject(obj, depthPassShader);

      // reset modified render object states
      obj->depthMapUniformName = depthMapUniformName;
      obj->colorWriteMask[0]= redWriteMask;
    }
  }

  // restore input fbo state

  fbo->unbind();

  restoreInputFbo();
}


void IRenderer::setViewerID(int _viewerID)
{
  curViewerID_ = _viewerID;
}


void IRenderer::renderLineThicknessGL42()
{
#ifdef GL_ARB_shader_image_load_store
  // quad extrusion has depth clipping issue
  // GL4.2 method: manually rasterize thick lines into load/store texture, thereby avoiding clipping

  // experimental technique, needs some improvements for rasterization
  /* possible improvement: 
    0. init depth buffer with scene objects
    1. extract visible line object (startpos, endpos) into imageBuffer after depth-test in fragment shader
    1a. line segment id can be found via atomic counter in geometry shader
    1b. use imageAtomicMin/max to find start and end pixel pos of each line segment
         atomic ops work with uint only, thus each segment has 4 texels in the imageBuffer,  offsets are segmentID * 4 + {0,1,2,3}
    2. read imageBuffer in geometry shader and do usual quad extrusion, now without depth testing
        => get hw rasterization and msaa
  */

  //  using image2D does not work: texture always reads 0, even with glGetTexImage2D
  //  imageBuffer works
  //  - check with different gpu

  const int numLines = lineGL42Objects_.size();

  // configs
  const bool useBufferTexture = true;  // imageBuffer or image2D
  const bool useIntegerTexture = true; // color as R32U or RGBA32F



  if (numLines)
  {

    // macro configs for shader
    QStringList macros;

    if (useBufferTexture)
      macros.push_back("#define IMAGE_BUFFER");
    if (useIntegerTexture)
      macros.push_back("#define FMT_UINT");


    // get random access write buffer
    //  32bit uint per viewport pixel, stores rgba8
    Texture2D* lineColorImg2D = 0;
    TextureBuffer* lineColorImgBuf = 0;

    size_t w = static_cast<size_t>(prevViewport_[2]), h = static_cast<size_t>(prevViewport_[3]);
    size_t lineBPP = static_cast<size_t>(useIntegerTexture ? 4 : 16); // bytes per pixel


    if (useBufferTexture)
    {
      lineColorImgBuf = dynamic_cast<TextureBuffer*>(lineColorBuffers_[curViewerID_]);

      if (!lineColorImgBuf)
      {
        lineColorImgBuf = new TextureBuffer(GL_TEXTURE0);
        lineColorBuffers_[curViewerID_] = lineColorImgBuf;
      }

      // resize
      if (lineColorImgBuf->getBufferSize() != w * h * lineBPP)
        lineColorImgBuf->setBufferData(w*h*lineBPP, 0, GL_R32UI, GL_DYNAMIC_DRAW);
    }
    else
    {
      lineColorImg2D = dynamic_cast<Texture2D*>(lineColorBuffers_[curViewerID_]);

      if (!lineColorImg2D)
      {
        // allocate and store in map
        lineColorImg2D = new Texture2D(GL_TEXTURE0);
        lineColorBuffers_[curViewerID_] = lineColorImg2D;
      }

      // resize
      if (lineColorImg2D->getWidth() != static_cast<int>(w) || lineColorImg2D->getHeight() != static_cast<int>(h))
        lineColorImg2D->setData(0,
          useIntegerTexture ? GL_R32UI : GL_RGBA32F,
          w, h, 
          useIntegerTexture ? GL_RED_INTEGER : GL_RGBA, 
          useIntegerTexture ? GL_UNSIGNED_INT : GL_FLOAT, 
          0);  
    }
    

    


    glViewport(0, 0, w, h);



    // ---------------------------
    //  clear line color buffer

    glColorMask(0,0,0,0);
    glDepthMask(0);
    glDisable(GL_DEPTH_TEST);

    // image binding is set to slot 0 in shader already
    if (useBufferTexture)
      lineColorImgBuf->bindAsImage(0, GL_WRITE_ONLY);
    else
      lineColorImg2D->bindAsImage(0, GL_WRITE_ONLY);

    GLSL::Program* shaderClear = ShaderCache::getInstance()->getProgram("ScreenQuad/screenquad.glsl", "Wireframe/gl42/clear.glsl", &macros);

    shaderClear->use();
//     shaderClear->setUniform("offset", Vec2f(0,0));
//     shaderClear->setUniform("size", Vec2f(1,1));
    shaderClear->setUniform("screenSize", Vec2f(w,h));

    ScreenQuad::draw(shaderClear);

    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
//    GLDebug::dumpTexture2D(lineColorBuf->id(), 0, lineColorBuf->getFormat(), lineColorBuf->getType(), lineBPP*w*h, "c:/dbg/lines_clear.dds");
//    GLDebug::dumpBufferData(GL_TEXTURE_BUFFER, lineColorBuf2->getBufferId(), "c:/dbg/lines_clear.bin");

    // ---------------------------
    // 1. pass
    //  render into line color buffer via imageStore,

    for (int i = 0; i < numLines; ++i)
    {
      RenderObject* obj = getLineGL42RenderObject(i);

      renderObject(obj);
    }

    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
//    GLDebug::dumpTexture2D(lineColorBuf->id(), 0, lineColorBuf->getFormat(), lineColorBuf->getType(), lineBPP*w*h, "c:/dbg/lines_image.dds");
//    GLDebug::dumpBufferData(GL_TEXTURE_BUFFER, lineColorBuf2->getBufferId(), "c:/dbg/lines_image.bin");


    // ---------------------------
    // 2. pass
    //  composition of line colors and fbo

    restoreInputFbo();

    // image binding is set to slot 0 in shader already
    if (useBufferTexture)
      lineColorImgBuf->bindAsImage(0, GL_READ_ONLY);
    else
      lineColorImg2D->bindAsImage(0, GL_READ_ONLY);


    glColorMask(1,1,1,1);
    glDisable(GL_DEPTH_TEST);

    // enable alpha blending
    glEnable(GL_BLEND);
    ACG::GLState::blendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    GLSL::Program* shaderComposite = ShaderCache::getInstance()->getProgram("ScreenQuad/screenquad.glsl", "Wireframe/gl42/composite.glsl", &macros);

    shaderComposite->use();
//     shaderComposite->setUniform("offset", Vec2f(0,0));
//     shaderComposite->setUniform("size", Vec2f(1,1));
    shaderComposite->setUniform("screenSize", Vec2f(w,h));

    ScreenQuad::draw(shaderComposite);


  }
#endif
}

void IRenderer::setLineThicknessRenderingGL42( bool _enable )
{
  if (Texture::supportsImageLoadStore() && Texture::supportsTextureBuffer())
    enableLineThicknessGL42_ = _enable;
}

void IRenderer::setErrorDetectionLevel( int _level )
{
  errorDetectionLevel_ = std::max(_level, 0); // clamp to [0,n]
}

int IRenderer::getErrorDetectionLevel() const
{
  return errorDetectionLevel_;
}

} // namespace ACG end

