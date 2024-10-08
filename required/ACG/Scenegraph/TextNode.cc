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
//  CLASS TextNode - IMPLEMENTATION
//
//=============================================================================



//== INCLUDES =================================================================

#include <ACG/GL/acg_glew.hh>

#include "TextNode.hh"
#include "../Utils/ImageConversion.hh"


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ==========================================================

// static members
#ifdef WIN32
// fonts in windows are drawn wider
QFont TextNode::qfont_ = QFont("Helvetica", 20);
#else
QFont TextNode::qfont_ = QFont("Helvetica", 30);
#endif
GLuint TextNode::texture_ = 0;
int TextNode::imageWidth_ = 0;
int TextNode::imageHeight_ = 0;
QFontMetrics TextNode::fontMetric_ = (QFontMetrics(TextNode::qfont_));
bool TextNode::initialised_ = false;
std::map< char, std::pair<unsigned int, unsigned int> > TextNode::charToIndex_ = TextNode::createMap();
QColor TextNode::color_ = QColor(255, 0, 0);


//----------------------------------------------------------------------------


TextNode::
TextNode( BaseNode*    _parent,
          const std::string&  _name,
          TextMode     _textMode,
          bool         _alwaysOnTop)
  : BaseNode(_parent, _name),
    size_(1.0),
    pixelSize_(12),
    textMode_(_textMode),
    vbo_(0),
    vertexBuffer_(0),
    oldVboSize_(0),
    blendEnabled_(false),
    texture2dEnabled_(false),
    cullFaceEnabled_(false),
    depthEnabled_(false),
    alwaysOnTop_(_alwaysOnTop),
    alphaTest_(false),
    alphaTestValue_(0.5f),
    alphaTestFunc_(GL_GREATER),
    blendSrc_(0),
    blendDest_(0),
    lastScale_(0.f)
{
  updateFont();
  vertexDecl_.addElement(GL_FLOAT, 3, ACG::VERTEX_USAGE_POSITION);
  vertexDecl_.addElement(GL_FLOAT, 2, ACG::VERTEX_USAGE_TEXCOORD);
  updateVBO();
}



//----------------------------------------------------------------------------


TextNode::
~TextNode()
{
  glDeleteBuffers(1, &vbo_);
}


//----------------------------------------------------------------------------



void
TextNode::
boundingBox(Vec3d& /*_bbMin*/, Vec3d& /*_bbMax*/)
{
}


//----------------------------------------------------------------------------


DrawModes::DrawMode
TextNode::
availableDrawModes() const
{
  return ( DrawModes::POINTS |
           DrawModes::POINTS_SHADED |
           DrawModes::POINTS_COLORED );
}


//----------------------------------------------------------------------------


void
TextNode::
setRenderingMode(TextMode _textMode) {
  textMode_ = _textMode;
}



//----------------------------------------------------------------------------

void
TextNode::
setAlwaysOnTop(bool _alwaysOnTop)
{
  alwaysOnTop_ = _alwaysOnTop;
}


//----------------------------------------------------------------------------

bool
TextNode::
alwaysOnTop()
{
  return alwaysOnTop_;
}


//----------------------------------------------------------------------------


TextNode::TextMode
TextNode::
renderingMode() {
  return textMode_;
}



//----------------------------------------------------------------------------


void
TextNode::
setText(std::string _text) {
  text_ = _text; updateVBO();
}



//----------------------------------------------------------------------------


void
TextNode::
setSize(const double _size) {
  size_ = _size; updateVBO();
}


//----------------------------------------------------------------------------


std::map< char, std::pair<unsigned int, unsigned int> >
TextNode::
createMap() {
  std::map< char, std::pair<unsigned int, unsigned int> > m;
  unsigned char c = ' ';
  for (unsigned int i = 0; i < rows_; ++i) {
    for (unsigned int j = 0; j < columns_; ++j, ++c) {
      m[c] = std::make_pair(j, i);
    }
  }

  return m;
}


//----------------------------------------------------------------------------


void
TextNode::
enter(GLState& _state, const DrawModes::DrawMode& _drawmode) {
  if(_state.compatibilityProfile())
    enterCompat(_state,_drawmode);
  else
  {
  if (text_.empty())
    return;

  // store current gl state
  cullFaceEnabled_ = glIsEnabled(GL_CULL_FACE);
  blendEnabled_ = glIsEnabled(GL_BLEND);
  depthEnabled_ = glIsEnabled(GL_DEPTH_TEST);

  glGetIntegerv(GL_BLEND_SRC, &blendSrc_);
  glGetIntegerv(GL_BLEND_DST, &blendDest_);

  // set texture and drawing states
  ACG::GLState::disable(GL_CULL_FACE);
  ACG::GLState::enable(GL_BLEND);
  ACG::GLState::blendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  if (alwaysOnTop_)
    ACG::GLState::disable(GL_DEPTH_TEST);
  }
}



//----------------------------------------------------------------------------


void
TextNode::
leave(GLState& _state, const DrawModes::DrawMode& _drawmode) {
  if(_state.compatibilityProfile())
    leaveCompat(_state, _drawmode);
  else
  {
  if (text_.empty())
      return;

  // restore the GLState as it was when entering TextNode
  if (cullFaceEnabled_)
    ACG::GLState::enable(GL_CULL_FACE);  
  if (!blendEnabled_)
    ACG::GLState::disable(GL_BLEND);
  if (depthEnabled_)
    ACG::GLState::enable(GL_DEPTH_TEST);
  else
    ACG::GLState::disable(GL_DEPTH_TEST);  

  ACG::GLState::blendFunc(blendSrc_, blendDest_);
  }
}



//----------------------------------------------------------------------------


void
TextNode::
draw(GLState& _state, const DrawModes::DrawMode& /*_drawMode*/)
{
  if (!text_.empty()) {
    bindVBO();

    // do not rotate the quads in this case
    if (textMode_ == SCREEN_ALIGNED || textMode_ == SCREEN_ALIGNED_STATIC_SIZE)
      applyScreenAligned(_state);


    _state.push_modelview_matrix();
    _state.scale(size_);
    glDrawArrays(GL_TRIANGLES, 0, int(text_.size() * 6) );
    _state.pop_modelview_matrix();

    if (textMode_ == SCREEN_ALIGNED || textMode_ == SCREEN_ALIGNED_STATIC_SIZE) {
      _state.pop_modelview_matrix();
    }
    unbindVBO();
  }
}


//----------------------------------------------------------------------------


quint32
TextNode::nearestPowerOfTwo(quint32 num) {
  quint32 n = num > 0 ? num - 1 : 0;

  n |= n >> 1;
  n |= n >> 2;
  n |= n >> 4;
  n |= n >> 8;
  n |= n >> 16;
  n++;

  return n;
}


//----------------------------------------------------------------------------


void
TextNode::setFont(const QFont& _font) {
  qfont_ = QFont(_font);
  initialised_ = false;
  updateFont();
  updateVBO();
}


//----------------------------------------------------------------------------

void
TextNode::
updateFont() {

  // do not generate a new texture for every TextNode unless necessary
  if (initialised_)
    return;

  // =================================================
  // Ugly workaround to get correct font metrics:
  // 1. Setup painter with an qimage
  // 2. Get metrics from painter
  // 3. Destroy everything
  // 4. Use the metrics to setup the correct values
  // =================================================

  // The image we work on. We actually don'T use the size here.
  // It's just to get the correct metris.
  QImage tmp(128, 128, QImage::Format_ARGB32);

  // Setup our painter
  QPainter tempPainter;
  tempPainter.begin(&tmp);
  tempPainter.setRenderHints(QPainter::Antialiasing
                           | QPainter::TextAntialiasing);
  tempPainter.setFont(qfont_);
  tempPainter.setPen(color_);

  // Now get the correct metrics and store them for now and for rendering
  fontMetric_ = tempPainter.fontMetrics();

  // Stop painting and setup the image correctly from the metrics
  tempPainter.end();

/// @TODO   Remove, if it works correctly on mac
//  if ( maxFontWidth_ == 0 ) {

//      // since metric.maxWidth() returns 0 for Mac we calculate it here
//      for (char c = ' '; c < '~'; ++c) {
//          qreal width = metric.width(c) + std::abs(metric.leftBearing(c)) + std::abs(metric.rightBearing(c));
//          if (width > maxFontWidth_)
//              maxFontWidth_ = width;
//      }

//      std::cerr << "Warning! Max font width returned 0! manual computation returned " << maxFontWidth_ << std::endl;
//  }

  // Maximal height of a character this is used to set the spacing in the texture containing the characters
  qreal height = fontMetric_.height();

  // ensure that the height and width of the texture is a power of 2 for easier rendering
  int heightPow2 = nearestPowerOfTwo(fontMetric_.height());
  int widthPow2 = nearestPowerOfTwo(fontMetric_.maxWidth());

  // Calculate the final image size used as a texture containing all characters
  imageWidth_  = widthPow2 * columns_;
  imageHeight_ = heightPow2 * rows_;

  // Create an empty image
  QImage finalImage(imageWidth_, imageHeight_, QImage::Format_ARGB32);
  finalImage.fill(Qt::transparent);

  // Setup our painter on the image
  QPainter painter;
  painter.begin(&finalImage);
  painter.setRenderHints(QPainter::Antialiasing
                       | QPainter::TextAntialiasing);
  painter.setFont(qfont_);
  painter.setPen(color_);

  // characters are drawn aligned to the left into the QImage finalImage
  //  coords contains a map from a character to its coordinates in the texture.
  for (char c = ' '; c < '~'; ++c) {
    std::pair<unsigned int, unsigned int> coords = charToIndex_[c];
    painter.drawText(coords.first*widthPow2, imageHeight_ - (coords.second+1)*heightPow2, widthPow2, heightPow2, Qt::AlignLeft | Qt::AlignBottom, QString(c));
  }
  painter.end();

  // convert finalImage to an OpenGL friendly format
  finalImage = ACG::Util::convertToGLFormat(finalImage);

  // generate a new texture from finalImage
  if (!texture_)
    glGenTextures(1, &texture_);

  ACG::GLState::bindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, finalImage.width(), finalImage.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, finalImage.bits());
  glGenerateMipmap(GL_TEXTURE_2D);
  ACG::GLState::bindTexture(GL_TEXTURE_2D, 0);

  initialised_ = true;
}


//----------------------------------------------------------------------------


void
TextNode::
updateVBO() {
  if (text_.size() == 0)
    return;

  vertexBuffer_.clear();

  // Fixed values for now. The projection changes the sizes anyway.
  qreal pixelHeight = 3.0;
  qreal pixelWidth  = 3.0;

  // generate a quad for each character next to each other
  // *--*--*----*-*
  // |  |  |    | |
  // |  |  |    | |
  // *--*--*----*-*

  // The height of the rows in the font texture
  const int height = nearestPowerOfTwo(fontMetric_.height());

  // Left coordinate of current Character
  float left = 0.0f;

  for (unsigned int i = 0; i < text_.size(); ++i) {

    // ====================================================================
    // Calculate the vertex coordinates of the character
    // ====================================================================

    // Compute the width we will cover with this letter. As each letter has a different size in the texture,
    // we need to modulate the width of the quad we draw to match the texture aspect ratio.
    // Otherwise we get distortion.

    // The change is the fraction between the width of the current character to the maximal charater width in the fontset.
    float width = pixelWidth * fontMetric_.horizontalAdvance(text_[i]) / fontMetric_.maxWidth();

    // If we have a space character, we move the the maximal font width
    if ( text_[i] == ' ')
        width = pixelWidth;

    // Compute the current left and right vertex coordinates.
    float right = left + width;

    // ====================================================================
    // Calculate the texture coordinates of the current character
    // ====================================================================

    // Width and height of one character in texture space (Remember that we calculate texture coordinates here between 0 and 1)
    // We don't take the full width of the texture block, as it might contain mostly black.
    const float widthTx = (float) fontMetric_.horizontalAdvance(text_[i]) / (float) imageWidth_;
    const float heightTx = (float) height/ (float) imageHeight_;

    // Get the starting position of the character in the texture
    // note that the characters are drawn aligned to the bottom left in in the texture
    // X Coordinate
    const float leftTx = ((float) charToIndex_[text_[i]].first ) / (float) columns_;
    const float rightTx = leftTx + widthTx;

    // YCoordinate
    const float bottomTx = charToIndex_[text_[i]].second / (float) rows_;
    const float topTx = bottomTx + heightTx;

    // bottom left
    vertexBuffer_.push_back(left);
    vertexBuffer_.push_back(0.0f);
    vertexBuffer_.push_back(0.0f);

    // texture coordinates
    vertexBuffer_.push_back(leftTx);
    vertexBuffer_.push_back(bottomTx);

    // top left
    vertexBuffer_.push_back(left);
    vertexBuffer_.push_back(pixelHeight);
    vertexBuffer_.push_back(0.0f);

    // texture coordinates
    vertexBuffer_.push_back(leftTx);
    vertexBuffer_.push_back(topTx);

    // top right
    vertexBuffer_.push_back(right);
    vertexBuffer_.push_back(pixelHeight);
    vertexBuffer_.push_back(0.0f);

    // texture coordinates
    vertexBuffer_.push_back(rightTx);
    vertexBuffer_.push_back(topTx);
    
    // bottom left
    vertexBuffer_.push_back(left);
    vertexBuffer_.push_back(0.0f);
    vertexBuffer_.push_back(0.0f);

    // texture coordinates
    vertexBuffer_.push_back(leftTx);
    vertexBuffer_.push_back(bottomTx);
    
    // top right
    vertexBuffer_.push_back(right);
    vertexBuffer_.push_back(pixelHeight);
    vertexBuffer_.push_back(0.0f);

    // texture coordinates
    vertexBuffer_.push_back(rightTx);
    vertexBuffer_.push_back(topTx);

    // bottom right
    vertexBuffer_.push_back(right);
    vertexBuffer_.push_back(0.0f);
    vertexBuffer_.push_back(0.0f);

    // texture coordinates
    vertexBuffer_.push_back(rightTx);
    vertexBuffer_.push_back(bottomTx);

    // The current right is the new left coordinate (Coordinates not texture space!)
    left = right;
  }

  if (!vbo_)
    glGenBuffers(1, &vbo_);

  ACG::GLState::bindBuffer(GL_ARRAY_BUFFER, vbo_);

  if (oldVboSize_ != vertexBuffer_.size())
  {
    glBufferData( GL_ARRAY_BUFFER_ARB, vertexBuffer_.size() * sizeof(GLfloat), 0, GL_DYNAMIC_DRAW_ARB );
    oldVboSize_ = vertexBuffer_.size();
  }

    // get pointer to VBO memory
  GLfloat *data = reinterpret_cast<GLfloat*>(glMapBuffer( GL_ARRAY_BUFFER_ARB, GL_WRITE_ONLY_ARB ));

  std::copy(vertexBuffer_.begin(), vertexBuffer_.end(), data);

  glUnmapBuffer(GL_ARRAY_BUFFER_ARB);

  ACG::GLState::bindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );
}


//----------------------------------------------------------------------------


void
TextNode::
bindVBO() {
  ACG::GLState::bindBuffer(GL_ARRAY_BUFFER, vbo_);
  ACG::GLState::vertexPointer(3, GL_FLOAT, 5*sizeof(GLfloat), 0);
  ACG::GLState::enableClientState(GL_VERTEX_ARRAY);

  ACG::GLState::activeTexture(GL_TEXTURE0);
  ACG::GLState::texcoordPointer(2, GL_FLOAT, 5*sizeof(GLfloat), reinterpret_cast<void*>(3*sizeof(GLfloat)));
  ACG::GLState::enableClientState(GL_TEXTURE_COORD_ARRAY);

  ACG::GLState::bindTexture(GL_TEXTURE_2D, texture_);
}


//----------------------------------------------------------------------------


void
TextNode::
unbindVBO() {
  ACG::GLState::bindTexture(GL_TEXTURE_2D, 0);
  ACG::GLState::bindBuffer(GL_ARRAY_BUFFER, 0);
  ACG::GLState::disableClientState(GL_VERTEX_ARRAY);
  ACG::GLState::disableClientState(GL_TEXTURE_COORD_ARRAY);
}

//----------------------------------------------------------------------------

void
TextNode::
getRenderObjects(ACG::IRenderer* _renderer, ACG::GLState&  _state , const ACG::SceneGraph::DrawModes::DrawMode&  _drawMode , const ACG::SceneGraph::Material* _mat)
{
  // init base render object
  ACG::RenderObject ro;

  ro.initFromState(&_state);

  ro.debugName = std::string("TextNode: ")+name();

  // do not rotate the quads in this case
  if (textMode_ == SCREEN_ALIGNED || textMode_ == SCREEN_ALIGNED_STATIC_SIZE)
    applyScreenAligned(_state);

  _state.push_modelview_matrix();
  _state.scale(size_);
  ro.modelview = _state.modelview();
  _state.pop_modelview_matrix();

  if (textMode_ == SCREEN_ALIGNED || textMode_ == SCREEN_ALIGNED_STATIC_SIZE)
  {
    _state.pop_modelview_matrix();
  }

  ro.culling = false;
  ro.blending = true;
  ro.alpha = 0.f;

  ro.blendSrc = GL_SRC_ALPHA;
  ro.blendDest = GL_ONE_MINUS_SRC_ALPHA;

  if (alwaysOnTop_)
    ro.priority = 1;//draw after scene meshes

  // Set the buffers for rendering
  ro.vertexBuffer = vbo_;
  ro.vertexDecl   = &vertexDecl_;

  // Set Texture
  RenderObject::Texture texture;
  texture.id = texture_;
  texture.type = GL_TEXTURE_2D;
  texture.shadow = false;
  ro.addTexture(texture);

  // Set shading
  ro.shaderDesc.vertexColors = false;
  ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;

  ACG::SceneGraph::Material localMaterial;

  localMaterial.baseColor(ACG::Vec4f(0.0, 0.0, 0.0, 0.0 ));
  localMaterial.ambientColor(ACG::Vec4f(0.0, 0.0, 0.0, 0.0 ));
  localMaterial.diffuseColor(ACG::Vec4f(0.0, 0.0, 0.0, 0.0 ));
  localMaterial.specularColor(ACG::Vec4f(0.0, 0.0, 0.0, 0.0 ));
  ro.setMaterial(&localMaterial);

  ro.glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(text_.size()) * 6);
  _renderer->addRenderObject(&ro);
}

//----------------------------------------------------------------------------
void TextNode::applyScreenAligned(GLState &_state)
{
  _state.push_modelview_matrix();

  // try to get the scale factor from the parent TransformNode if it exists
  BaseNode* pParent = parent();
  double scale = 1.0;
  while (pParent) {
    TransformNode* pTrans = dynamic_cast<TransformNode*>(pParent);
    if (pTrans) {
      scale = pTrans->scale()(0,0);
      break;
    }
    pParent = pParent->parent();
  }

  // get the translation
  Vec3d projected = _state.project(Vec3d(0.0, 0.0, 0.0));
  _state.reset_modelview();
  Vec3d unprojected = _state.unproject(projected);

  _state.translate(unprojected);

  if (textMode_ == SCREEN_ALIGNED_STATIC_SIZE)
  {
    ACG::Vec3d nullProj = _state.project(Vec3d(0.0,0.0,0.0));
    //ACG::Vec3d nullUnproj = _state.unproject(nullProj);
    ACG::Vec3d heightUnproj = _state.unproject(nullProj+ACG::Vec3d(0.0,pixelSize_,0.0));
    scale *= heightUnproj.length();
    lastScale_ = scale;
  }

  _state.scale(scale);
}
//----------------------------------------------------------------------------
void TextNode::setPixelSize(const unsigned int _size)
{
  pixelSize_ = _size;
}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
