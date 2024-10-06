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
#include <ACG/GL/RenderObject.hh>

#include <ACG/Scenegraph/SceneGraph.hh>
#include <ACG/Scenegraph/MaterialNode.hh>



namespace GLSL{
  class Program;
}

namespace ACG
{

// forward declaration
class VertexDeclaration;
class GLState;
class FBO;
class Texture;
class Texture2D;

namespace SceneGraph {
  namespace DrawModes {
    class DrawModeProperties;
  }
  class Material;
}

class ACGDLLEXPORT IRenderer
{
public:
  IRenderer();
  virtual ~IRenderer();


public:
  //=========================================================================
  // Callbacks for the scenegraph nodes
  //=========================================================================

  /** \brief Callback for the scenegraph nodes, which send new render objects via this function.
   *
   * AddRenderObject is typically called by a scenegraph nodes during the collection of renderable
   * objects. A renderobject is a collection of opengl states, buffers and parameters,
   * that correspond to exactly one draw call.
   *
   * Creates another RenderObject instance internally, safe for temporary local RenderObject instances.
   *
   * \note The VertexDeclaration address must be permanently valid.
   *
   * @param _renderObject Newly added render object
  */
  virtual void addRenderObject(RenderObject* _renderObject);

  struct LightData
  {
    LightData()
      : ltype(ACG::SG_LIGHT_DIRECTIONAL),
      diffuse(1.0f, 1.0f, 1.0f), ambient(1.0f, 1.0f, 1.0f), specular(1.0f, 1.0f, 1.0f),
      pos(0.0f, 0.0f, 0.0f), dir(1.0f, 0.0f, 0.0f), atten(1.0f, 0.0f, 0.0f), 
      spotCutoffExponent(0.0f, 0.0f)
    { }

    ACG::ShaderGenLightType ltype; // directional, spot- or pointlight
    ACG::Vec3f diffuse, ambient, specular; // light color factor
    ACG::Vec3f pos, dir; // position, direction in view-space
    ACG::Vec3f atten; // (constant, linear, quadratic) attenuation
    ACG::Vec2f spotCutoffExponent; // (cutoff angle, exponent) for spotlights
  };

  class RenderObjectRange {
      public:
          RenderObjectRange() {}
          RenderObjectRange(std::vector<ACG::RenderObject>::iterator begin,
                  std::vector<ACG::RenderObject>::iterator end) :
                      begin_(begin), end_(end) {}

          std::vector<ACG::RenderObject>::iterator begin() const {
              return begin_;
          }

          std::vector<ACG::RenderObject>::iterator end() const {
              return end_;
          }

      private:
          std::vector<ACG::RenderObject>::iterator begin_, end_;
  };

  /** \brief Callback for the scenegraph nodes, which send new lights to the renderer via this function
   *
   * Scenegraph nodes are able to add new light sources to the renderer with this function.
   * To so, the node should implement getRenderObjects() and call addLight in there.
   * LightNodes use this for example.
   *
   * @param _light Newly added light
  */
  virtual void addLight(const LightData& _light);

  /** \brief Callback for the scenegraph nodes, which adds a render object modifier to the renderer via this function
  *
  * This function is needed whenever a scenegraph node wants to set certain rendering parameters for all its child nodes.
  * A scenegraph node does not have direct access to render object created within its subtree.
  * Instead, it can implement a render object modifier and attach it to the renderer.
  * This modifier is applied to all render objects added to this renderer until it gets removed again.
  * 
  * A good place to call this function is in the derived BaseNode::enter() function.
  *
  * @param _mod address of the modifier. It has to remain in memory until it gets removed from the renderer!
  */
  virtual void addRenderObjectModifier(RenderObjectModifier* _mod);

  /** \brief Callback for the scenegraph nodes, which removes a render object modifier from the renderer
  *
  * Remove a previously added modifier. RenderObjects added to the renderer afterwards will not be affected anymore.
  *
  * A good place to call this function is in the derived BaseNode::leave() function.
  *
  * @param _mod address of the modifier.
  */
  virtual void removeRenderObjectModifier(RenderObjectModifier* _mod);

  //=========================================================================
  // Render object collection and OpenGL setup for shader-based rendering
  //=========================================================================
protected:
  /** \brief Prepares renderer and OpenGL for any draw-related calls including
   *
   * Prepares renderer and OpenGL for any draw-related calls including:
   *  - collecting renderobjects ( collectRenderObjects() )
   *  - sorting renderobjects ( sortRenderObjects() )
   *  - resetting OpenGL state machine for shader-based rendering
   *
   * @param _glState pointer to glstate
   * @param _drawMode default drawmode
   * @param _scenegraphRoot root node of scenegraph
  */
  virtual void prepareRenderingPipeline(ACG::GLState* _glState, ACG::SceneGraph::DrawModes::DrawMode _drawMode, ACG::SceneGraph::BaseNode* _scenegraphRoot);

  /** \brief Traverse the scenegraph to collect render information
   *
   * Traverses the scenegraph and calls the getRenderObject function of each node.
   * Each node can then add multiple renderobjects via addRenderObject to this renderer.
   *
   * Also collects all light sources in the scenegraph.
   * The currently active list of renderobjects is invalidated too.
   */
  virtual void collectRenderObjects(ACG::GLState* _glState, ACG::SceneGraph::DrawModes::DrawMode _drawMode, ACG::SceneGraph::BaseNode* _sceneGraphRoot);


  /** Calls getRenderObjects on each node of the scenegraph recursively.
    */

  /** \brief Scene graph traversal for render object collection
   *
   * Calls getRenderObjects on each node of the scenegraph recursively. Each node then triggers the callbacks.
   */
  void traverseRenderableNodes(ACG::GLState* _glState, ACG::SceneGraph::DrawModes::DrawMode _drawMode, ACG::SceneGraph::BaseNode &_node, const ACG::SceneGraph::Material &_mat);



  //=========================================================================
  // Sorting
  //=========================================================================
protected:

    /** \brief Sort the renderobjects by priority
     *
     * Sort array of renderobjects by priority and store the result in sortedObjects_.
    */
    virtual void sortRenderObjects();


  //=========================================================================
  // Rendering
  //=========================================================================
protected:

  /** \brief Render one renderobject
   *
   * Fully prepares opengl for a renderobject and executes the draw call.
   * This combines bindObjectVBO, bindObjectUniforms...
   *
   * Optionally render-states may not be changed, in case depth-peeling or
   * similar global shader operations may require a fixed state setting.
   *
   * Shader modifiers can be combined and applied to this pass.
   */
  virtual void renderObject(ACG::RenderObject* _obj, GLSL::Program* _prog = 0, bool _constRenderStates = false, const std::vector<unsigned int>* _shaderModifiers = 0);

  /** \brief Binding VBOs (First state function)
   *
   * This is the first function called by renderObject().
   *
   * It binds vertex, index buffer and vertex format of a
   * render object.
   */
  virtual void bindObjectVBO(ACG::RenderObject* _obj,
      GLSL::Program*     _prog);

  /** \brief Binding Uniforms (Second state function)
   *
   * This is the second function called by renderObject().
   *
   * Set common shader constants like model-view-projection matrix,
   * material colors and light params.
   */
  virtual void bindObjectUniforms(ACG::RenderObject* _obj,
      GLSL::Program*     _prog);

  /** \brief Binding Render state (Third state function)
   *
   * This is the third function called by renderObject().
   *
   * Prepare the opengl state machine for a renderobject draw call.
   *
   * This includes any glEnable/glDisable states, depth-cmp functions, blend equation..
   */
  virtual void bindObjectRenderStates(ACG::RenderObject* _obj);


  /** \brief Executes the opengl draw call for one object (Fourth function)
   *
   * This is the fourth function called by renderObject().
   *
   * Executes one draw call for the given render object
   */
  virtual void drawObject(ACG::RenderObject* _obj);


  //=========================================================================
  // Restore OpenGL State
  //=========================================================================

protected:


  /** \brief Draw overlay objects and reset OpenGL state
   *
   * Optionally renders overlay objects.
   * Resets critical OpenGL states to default to prevent crashes.
   * - deactivate framebuffer
   * - disable shaders
   * - disable vbo
  */
  virtual void finishRenderingPipeline(bool _drawOverlay = true);

  /** \brief Save input Fbo configuration (fbo id + viewport)
  */
  virtual void saveInputFbo();

  /** \brief Restore the previously saved input Fbo configuration (fbo id + viewport)
  */
  virtual void restoreInputFbo();

  /** \brief Save active Fbo configuration (fbo id + viewport)
   * @param _outFboId [out] pointer to address receiving the currently active fbo id
   * @param _outViewport [out] pointer to address of 4 GLint values receiving the currently active viewport
   * @param _outDrawBuffer [out] pointer to address receiving the currently active drawbuffer target (ie GL_BACK, GL_COLOR_ATTACHMENTi ..)
  */
  virtual void saveActiveFbo(GLint* _outFboId, GLint* _outViewport, GLint* _outDrawBuffer) const;

  /** \brief Restore a previously saved input Fbo configuration (fbo id + viewport)
   * @param _fboId fbo id
   * @param _outViewport [in] pointer to address of 4 GLint values receiving the currently active viewport
   * @param _drawBuffer draw-buffer target of fbo
  */
  virtual void restoreFbo(GLint _fboId, const GLint* _outViewport, GLint _drawBuffer) const;

  /** \brief Clear input Fbo.
   *
   * Clears color and depth buffer of input Fbo (using a scissor test to only clear viewport area).
   * @param _clearColor clear color
  */
  virtual void clearInputFbo(const ACG::Vec4f& _clearColor);

  //=========================================================================
  // Other Convenience
  //=========================================================================

protected:

  /** \brief Copy texture to depth buffer
   *
   * Copies depth values from a texture to the input depth buffer.
   * These are the changes made to the OpenGL state, which are not restored upon return:
   * - active texture stage is 0 with _depthTex bound
   * - internal shader program active
   * - internal screen quad vbo active
   * - filling mode is set to glPolygonMode(GL_FRONT, GL_FILL)
   *
   * @param _depthTex texture containing depth values
   * @param _scale scaling factor for the depth values from _depthTex
  */
  virtual void copyDepthToBackBuffer(GLuint _depthTex, float _scale = 1.0f);

  
  /** \brief Render the depth map of the scene.
   *
   * When setting up a shader of a render-object, a scenegraph node can use the depth map for custom rendering techniques.
   * If at least one render object calls this function, the depth map is later rendered before the scene pass.
   * and bound to the shader of an renderobject in bindObjectUniforms according to the provided "depthMapUniformName".
   * Depth Map format: GL_TEXTURE_2D, GL_R32F, depth value = gl_FragCoord.z
   * This function is automatically called in prepareRenderingPipeline() when required.
   * 
   * @param _viewerID id of viewport, 
   * @param _width viewport width
   * @param _height viewport height
  */
  virtual void renderDepthMap(int _viewerID, int _width, int _height);


  //=========================================================================
  // Internal shader modifiers
  //=========================================================================
protected:

  // depth map modifier: writes gl_FragCoord.z to red color channel
  class DepthMapPass : public ShaderModifier
  {
  public:
    void modifyFragmentEndCode(QStringList* _code);

    static DepthMapPass instance;
  };


  //=========================================================================
  // Debugging
  //=========================================================================
public:

  /** \brief Debugging function to dump list of render objects into a file
   *
   * Dump list of render objects to text file.
   * @param _fileName name of text file to write to
   * @param _sortedList dump sorted render objects in order, may be 0 to use the unsorted list instead
   */
  void dumpRenderObjectsToFile(const char* _fileName, ACG::RenderObject** _sortedList = 0) const;

  /** \brief Outputs the current render objects to the string
   *
   * @param _list          Pointer to a list of render objects used for output (defaults to unsorted internal list)
   * @param _outputShaders Output the shaders used for the objects?
   * @param _modifiers     Pointer to vector of registered ShaderModifier
   * @return Render object data in a QString
   */
  virtual QString dumpCurrentRenderObjectsToString(ACG::RenderObject** _list = 0, bool _outputShaders = false, std::vector<ACG::ShaderModifier*>* _modifiers = 0);

  /** \brief Control error detection for checking render objects 
   *
   * Levels:
   *  0 - disable error detection
   *  1 - fast check (obvious stuff that does not require much time to check)
   *  2 - intensive check (do additional check, possibly false positives and impact on performance)
   * 
   * Default level is 1
   *
   * @param _level   detection level
   */
  void setErrorDetectionLevel(int _level);

  /// Get error detection level
  int getErrorDetectionLevel() const;

  //=========================================================================
  // Variables
  //=========================================================================
protected:

  /// Get the number of collected render objects (not including overlay objects or gl4.2 line objects)
  int getNumRenderObjects() const;

  /// Get the number of current light sources
  int getNumLights() const;

  /// Get render objects in the sorted list by index (not including overlay objects)
  ACG::RenderObject* getRenderObject(int i);

  /// Get render objects in the sorted list by index (only overlay objects)
  ACG::RenderObject* getOverlayRenderObject(int i);

  /// Get render objects in the sorted list by index (only line objects rendered with gl4.2)
  ACG::RenderObject* getLineGL42RenderObject(int i);

  /// Get light by index
  LightData* getLight(int i);


  /// Get global ambient light contribution from GL_LIGHT_MODEL_AMBIENT
  const ACG::Vec3f& getGlobalAmbientScale() const {return globalLightModelAmbient_;}


  /// Get node that emitted the render object in the sorted list by index (not including overlay objects)
  ACG::SceneGraph::BaseNode* getRenderObjectNode(int i);

  /// Get node that emitted the render object in the sorted list by index (only overlay objects)
  ACG::SceneGraph::BaseNode* getOverlayRenderObjectNode(int i);



  /** Enable/disable line thickness rendering with opengl4.2
   *
   * Instead of quad extrusion in a geometry shader, thick lines are manually rasterized in a load/store image.
   * This image is then composited with the back buffer.
   * If the gpu does not support image load/store, the default quad extrusion is used anyway and this function has no effect.
   *
   * @param _enable  enable/disable
   */
  void setLineThicknessRenderingGL42(bool _enable);


  //=========================================================================
  // Internally called by OpenFlipper core
  //=========================================================================
public:

  /** \brief Set currently active viewer id
   *
   * If the scenegraph makes use of the z-prepass feature and the application uses multiple viewports,
   * the currently active viewport should be set prior rendering.
   * Otherwise, this function can be ignored.
   *
   * @param _viewerID  unique id of the current viewport (i.e. ViewerProperties::viewerID() )
   */
  void setViewerID(int _viewerID);



  /** \brief Tell renderer to use core or compatibility profile
   *
   * @param _enable  renderer should use opengl in core profile mode
   */
  void setCoreProfileMode(bool _enable) {coreProfile_ = _enable;}

  /**
   * During traversal of the scene graph this method returns the range of
   * render objects that has been collected in the current subtree.
   *
   * This method is exclusively intended to be called in the leave() function
   * of scene graph nodes. Calling it from anywhere else will yield an
   * undefined result with potentially invalid iterators.
   */
  const RenderObjectRange &getCollectedSubtreeObjects() const {
      return current_subtree_objects_;
  }

protected:
  /// Number of Lights
  int numLights_;

  /// Light sources ( Filled by addLight() )
  LightData lights_[SG_MAX_SHADER_LIGHTS];

  /// ambient color scale for global light:
  ///  this is set via glLightModel(GL_LIGHT_MODEL_AMBIENT, scale)
  ACG::Vec3f globalLightModelAmbient_;

  /// array of renderobjects, filled by addRenderObject()
  std::vector<ACG::RenderObject> renderObjects_;

  /// map sortedID -> original renderObjectID
  std::vector<int> sortListObjects_;
  /// map sortedID -> original renderObjectID
  std::vector<int> sortListOverlays_;

  /// sorted list of renderobjects without overlay objects (sorted in rendering order)
  std::vector<ACG::RenderObject*> sortedObjects_;

  /// sorted list of overlay-only renderobjects (sorted in rendering order)
  std::vector<ACG::RenderObject*> overlayObjects_;

  /// active render object modifiers
  std::vector<ACG::RenderObjectModifier*> renderObjectModifiers_;

  /// source node that added the render object (map renderObjectID -> scenegraph node)
  std::vector<ACG::SceneGraph::BaseNode*> renderObjectSource_;

  /// source node that added the overlay render object (map renderObjectID -> scenegraph node)
  std::vector<ACG::SceneGraph::BaseNode*> overlayObjectSource_;

  /**
   * Stores fbo containing a depth map for each viewport.
   * The depth map is computed in a z-prepass if at least one RenderObject makes use of the scene depth map.
   * (optional convenience feature)
   */
  std::map<int, ACG::FBO*> depthMaps_;

  /// true if at least one renderobject requires a scene depthmap, false otherwise
  bool depthMapUsed_;

  /// currently active viewer ID as specified in prepareRenderObjects()
  int curViewerID_;

  /// view transform
  GLMatrixf viewMatrix_;

  /// cam position in world-space
  Vec3f camPosWS_;

  /// direction the camera is looking to in world-space
  Vec3f camDirWS_;

  /// previous fbo
  GLint prevFbo_;

  /// previous drawbuffer target (ie GL_BACK, GL_COLOR_ATTACHMENTi...)
  GLint prevDrawBuffer_;

  /// previous viewport
  GLint prevViewport_[4];

  /// flag indicating a that saveCurrentFbo() has been called prior restoreFbo()
  bool prevFboSaved_;

  /// previously bound VAO
  GLint prevVAO_;

  /// shader copies depth of the first front layer to the back buffer
  GLSL::Program* depthCopyShader_;

  /// error-detection level for checking render objects
  int errorDetectionLevel_;


  /// core profile mode
  bool coreProfile_;


  /// max number of clip distance outputs in a vertex shader
  static int maxClipDistances_;

  RenderObjectRange current_subtree_objects_;
private:

  //=========================================================================
  // Default rendering of thick lines
  //=========================================================================

  /** \brief Enable/disable gl4.2 based technique for rendering thick lines
   *
   *  If enabled, default line objects are not returned via getRenderObject(),
   *   but are rendered in finishRenderingPipeline() just before drawing overlays.
  */
  bool enableLineThicknessGL42_;

  /// default line render objects that are rendered with gl4.2
  std::vector<RenderObject*> lineGL42Objects_;

  /// map from viewport id to line buffer
  std::map< int, Texture* > lineColorBuffers_;

  void renderLineThicknessGL42();


};



//=============================================================================
} // namespace ACG
//=============================================================================
