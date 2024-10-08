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
//  CLASS TextureNode - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================


#include "TextureNode.hh"
#include <ACG/Utils/ImageConversion.hh>

//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ==========================================================


TextureNode::TextureNode( BaseNode*            _parent,
                          const std::string&   _name,
                          bool                 _texture_repeat,
                          GLint                _texture_filter )
      : BaseNode( _parent, _name ),
        textures_(),
        alpha_( 0 ),
        texture_repeat_( _texture_repeat ),
        tex_mode_( GL_MODULATE ),
        texture_filter_( _texture_filter ),
        mipmapping_globally_active_(true),
        last_mipmapping_status_(true),
        mipmapping_(true),
        activeTexture_(-1),
        open_volume_mesh_texture_draw_modes_(DrawModes::getDrawMode("Faces (textured)") | DrawModes::getDrawMode("Faces (textured and shaded)") )
{
//    open_volume_mesh_texture_draw_modes_  = DrawModes::getDrawMode("Faces (textured)");
//    open_volume_mesh_texture_draw_modes_ |= DrawModes::getDrawMode("Faces (textured and shaded)");
}


//----------------------------------------------------------------------------


TextureNode::~TextureNode()
{
  for (std::vector<TextureInfo>::iterator texturesIt = textures_.begin(); texturesIt != textures_.end(); ++texturesIt) {
    delete texturesIt->tex;
  }
  textures_.clear();
}



//----------------------------------------------------------------------------


bool
TextureNode::read(const char* _filename)
{
   // load to image
   QImage image;
   if ( !image.load( _filename ) )
   {
      std::cerr << "Cannot load texture " << _filename << "\n";
      return false;
   }

   set_texture( image );

   return true;
}


//----------------------------------------------------------------------------


void
TextureNode::applyGLSettings(  )
{
  // GL settings
  glPixelStorei( GL_UNPACK_ROW_LENGTH,  0 );
  glPixelStorei( GL_UNPACK_SKIP_ROWS,   0 );
  glPixelStorei( GL_UNPACK_SKIP_PIXELS, 0 );
  glPixelStorei( GL_UNPACK_ALIGNMENT,   1 );
  glPixelStorei( GL_PACK_ROW_LENGTH,    0 );
  glPixelStorei( GL_PACK_SKIP_ROWS,     0 );
  glPixelStorei( GL_PACK_SKIP_PIXELS,   0 );
  glPixelStorei( GL_PACK_ALIGNMENT,     1 );
}

void
TextureNode::applyTextureParameters( int _id )
{
  if ( texture_repeat_ ) {
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
  } else {
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );

    // Set BorderColor for Clamping
    const float borderColor[4] = {1.0, 1.0, 1.0, 1.0};
    glTexParameterfv( GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor );
  }
  
  if((mipmapping_globally_active_ && mipmapping_ && textures_[_id].mipmapAvailable) &&
     (texture_filter_ == GL_LINEAR_MIPMAP_NEAREST ||
      texture_filter_ == GL_LINEAR_MIPMAP_LINEAR  ||
      texture_filter_ == GL_NEAREST_MIPMAP_LINEAR ||
      texture_filter_ == GL_NEAREST_MIPMAP_NEAREST)) {
      
      glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
      glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, texture_filter_ );
      glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 16.0f);
      
  } else if(texture_filter_ == GL_LINEAR) {
      
      glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
      glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
  
  } else {
      glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
      glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
  }
  
}

//----------------------------------------------------------------------------

void TextureNode::enable_mipmapping() {
    
    if(!mipmapping_) {
        mipmapping_ = true;
        updateMipmaps(mipmapping_globally_active_);
    }
}

//----------------------------------------------------------------------------

void TextureNode::disable_mipmapping() {
    
    if(mipmapping_) {
        mipmapping_ = false;
        updateMipmaps(mipmapping_);
    }
}

//----------------------------------------------------------------------------

void TextureNode::setTextureDataGL (  GLuint _textureId,
                                      GLenum _target,
                                      GLint _width ,
                                      GLint _height,
                                      GLenum _format ,
                                      GLenum _type,
                                      const void * _data) {
  
  applyGLSettings();

  Texture2D* tex = textures_[_textureId].tex;

  tex->bind();

  if ( mipmapping_ )
    textures_[_textureId].mipmapAvailable = true;
  else
    textures_[_textureId].mipmapAvailable = false;

  applyTextureParameters(_textureId);

  bool mipmaps =  mipmapping_globally_active_ && mipmapping_;
  // Load the image
  if ( mipmaps )
    tex->autogenerateMipMaps();
  
  tex->setData( 0,                   // level
                GL_RGBA,             // internal format
                _width,              // width  (2^n)
                _height,             // height (2^m)
                _format,             // format
                _type,               // type
                _data,               // pointer to pixels
                mipmaps);            // mipmaps or not
  
  // Unbind until we use it
  ACG::GLState::bindTexture(GL_TEXTURE_2D,0);
}


//----------------------------------------------------------------------------

/** \brief Generate mipmaps for each texture that does not have one yet
*/
void TextureNode::updateMipmaps(bool _mipmap) {

  // Make sure we have at least on element in the textures list
  checkEmpty();

  for(unsigned int i = 1; i < textures_.size(); ++i) {

    Texture2D* tex = textures_[i].tex;

    // Bind texture
    tex->bind();

    // size in bytes of level 0 texture
    size_t bufferSize = textures_[i].tex->getWidth() * textures_[i].tex->getHeight()*4;

    if(_mipmap && bufferSize) {

      // Get pixel data out of texture memory
      GLubyte* buffer = new GLubyte[bufferSize];
//      glGetTexImage(textures_[i].target, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
      tex->getData(0, buffer);

      // Build mipmap
      tex->autogenerateMipMaps();
      tex->setData(0, tex->getInternalFormat(), tex->getWidth(), tex->getHeight(),
        tex->getFormat(), tex->getType(), buffer);

      delete [] buffer;
    }

    // Set mipmap available flag
    textures_[i].mipmapAvailable = _mipmap;

    // Update parameters such that changes apply during runtime
    applyTextureParameters(i);
  }
}

//----------------------------------------------------------------------------


void
TextureNode::set_texture(const unsigned char * _image, int _width, int _height)
{
  checkEmpty();

  // enough texture mem?
  if ( !Texture2D::checkTextureMem(GL_RGBA, _width, _height, GL_RGBA) ) {
    std::cerr << "Can't load texture";
    return;
  }

  setTextureDataGL(activeTexture_,GL_TEXTURE_2D,_width,_height,GL_RGBA,GL_UNSIGNED_BYTE,_image);

}


//----------------------------------------------------------------------------


void
TextureNode::set_texture(const QImage& _image)
{
  checkEmpty();

  // adjust texture size: 2^k * 2^l
  int tex_w, w( _image.width()  );
  int tex_h, h( _image.height() );

  for (tex_w=1; tex_w < w; tex_w <<= 1) {};
  for (tex_h=1; tex_h < h; tex_h <<= 1) {};
  if (5 * tex_w > 7 *w)   // tex_w longer than sqrt(2)*w means too much waste of storage space (7/5 = sqrt(2)-1%)
    tex_w >>= 1;
  if (5 * tex_h > 7 *h)   // tex_h longer than sqrt(2)*h means too much waste of storage space (7/5 = sqrt(2)-1%)
    tex_h >>= 1;

  // is size of input image a power of 2?
  bool isNPOT = ( tex_w != w ) || ( tex_h != h );

  // image has to be converted to GL_RGBA8 to avoid crash
  QImage textureGL;

  // eventually scale to pot image if no hw support  / core in GL 2.0
  if (!openGLVersion(2,0) && isNPOT)
  {
    // because texture will only be accessed proportionally by texture coordinates, aspect ratio is of no concern
    textureGL = ACG::Util::convertToGLFormat ( _image.scaled( tex_w, tex_h, Qt::IgnoreAspectRatio, Qt::SmoothTransformation ) );
  }
  else
  {
    // use npot texture
    tex_w = w;
    tex_h = h;
    textureGL = ACG::Util::convertToGLFormat ( _image );
  }

  // enough texture mem?
  if ( !Texture2D::checkTextureMem(GL_RGBA, tex_w, tex_h, GL_RGBA) ) {
    std::cerr << "Can't load texture TextureNode::set_texture" << std::endl;
    return;
  }

  // Set the image
  setTextureDataGL(activeTexture_ ,GL_TEXTURE_2D,tex_w,tex_h,GL_RGBA,GL_UNSIGNED_BYTE,textureGL.bits());
}

//----------------------------------------------------------------------------


void
TextureNode::set_texture(const float * _image, int _width, int _height )
{
  checkEmpty();

  // enough texture mem?
  if ( !Texture2D::checkTextureMem(GL_RGBA, _width, _height, GL_RGBA) ) {
    std::cerr << "Can't load texture TextureNode::set_texture" << std::endl;
    return;
  }

  // Set the image
  setTextureDataGL(activeTexture_,GL_TEXTURE_2D,_width,_height,GL_RGBA,GL_FLOAT,_image);
}

//----------------------------------------------------------------------------

void TextureNode::checkEmpty() {

  // No texture generated yet!
  if ( textures_.empty() ) {
    textures_.resize(1);
    activeTexture_ = 0;
//     textures_[activeTexture_].id = 0;
    Texture2D* t = new Texture2D();
    t->gen();
    textures_[activeTexture_].tex = t;
  }

}

//----------------------------------------------------------------------------

int TextureNode::available( GLuint _id  ) {
  // If the texture is found in the array return its id otherwise -1
  for ( uint i = 0 ; i < textures_.size(); ++i )
    if ( textures_[i].tex->id() == _id )
      return i;

  return -1;
}

//----------------------------------------------------------------------------

bool TextureNode::read(const char* _filename, GLuint _id ) {
  if ( available(_id) != -1 ) {
    activeTexture_ = available(_id);
    return read(_filename);
  } else
    std::cerr << "Texture with id " << _id << " not handled by this Node!!!" << std::endl;

  return false;

}


//----------------------------------------------------------------------------



void TextureNode::set_texture(const QImage& _image, GLuint _id) {

  checkEmpty();

  if ( available(_id) != -1 ) {
    activeTexture_ = available(_id);
    set_texture(_image);
  } else
    std::cerr << "Texture with id " << _id << " not handled by this Node!!!" << std::endl;

}


//----------------------------------------------------------------------------



void TextureNode::set_texture(const float * _image, int _width, int _height, GLuint _id) {

  checkEmpty();

  if ( available(_id) != -1 ) {
    activeTexture_ = available(_id);
    set_texture(_image,_width,_height);
  } else
    std::cerr << "Texture with id " << _id << " not handled by this Node!!!" << std::endl;
}



//----------------------------------------------------------------------------


void TextureNode::set_texture(const unsigned char * _image, int _width, int _height, GLuint _id) {

  checkEmpty();

  if ( available(_id) != -1 ) {
    activeTexture_ = available(_id);
    set_texture(_image,_width,_height);

  } else
    std::cerr << "Texture with id " << _id << " not handled by this Node!!!" << std::endl;

}


//----------------------------------------------------------------------------


// add QImage _image as additional texture, using face_texture_index
GLuint
TextureNode::add_texture(const QImage& _image)
{
  checkEmpty();

  textures_.resize(textures_.size()+1);  // can't push_back, because glGenTextures needs a pointer

  // Generate new texture
  textures_.back().tex = new Texture2D();

  activeTexture_ = int(textures_.size() - 1);

  // Set the image
  set_texture(_image);

  // return the id of the new texture
  return textures_.back().tex->id();
}


//----------------------------------------------------------------------------


void TextureNode::enter(GLState& _state , const DrawModes::DrawMode& _drawmode)
{
   if ( _drawmode & ( DrawModes::SOLID_TEXTURED |
                      DrawModes::SOLID_TEXTURED_SHADED |
                      DrawModes::SOLID_ENV_MAPPED |
                      DrawModes::SOLID_2DTEXTURED_FACE |
                      DrawModes::SOLID_2DTEXTURED_FACE_SHADED |
                      DrawModes::SOLID_FACES_COLORED_2DTEXTURED_FACE_SMOOTH_SHADED |
                      DrawModes::SOLID_SHADER |
                      open_volume_mesh_texture_draw_modes_ ))
   {
     if(_state.compatibilityProfile())
       ACG::GLState::enable( GL_TEXTURE_2D );
      
     mipmapping_globally_active_ = _state.mipmapping_allowed();
     
     // Check if mipmapping status has changed
     if(_state.mipmapping_allowed() != last_mipmapping_status_) {
         
             // Update mipmaps
             updateMipmaps(mipmapping_globally_active_ && mipmapping_);
             
             // Keep track of changes
             last_mipmapping_status_ = _state.mipmapping_allowed();
         
     }
     
     if ( !textures_.empty() ) {
       textures_[activeTexture_].tex->bind();
     }

     if(_state.compatibilityProfile())
       glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, tex_mode_ );

//       if (_drawmode & DrawModes::SOLID_FACES_COLORED_2DTEXTURED_FACE_SMOOTH_SHADED)
//         glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
   }
}


//----------------------------------------------------------------------------


void TextureNode::leave(GLState&  _state  , const DrawModes::DrawMode& _drawmode)
{
   if ( _drawmode & ( DrawModes::SOLID_TEXTURED |
                      DrawModes::SOLID_TEXTURED_SHADED |
                      DrawModes::SOLID_ENV_MAPPED |
                      DrawModes::SOLID_2DTEXTURED_FACE |
                      DrawModes::SOLID_2DTEXTURED_FACE_SHADED |
                      DrawModes::SOLID_SHADER |
                      open_volume_mesh_texture_draw_modes_ ))
   {
      ACG::GLState::bindTexture( GL_TEXTURE_2D, 0 );
      if(_state.compatibilityProfile())
        ACG::GLState::disable( GL_TEXTURE_2D );
   }
}

void TextureNode::enterPick(GLState& /*_state*/ , PickTarget /*_target*/, const DrawModes::DrawMode& /*_drawMode*/ ) {

}

void TextureNode::leavePick(GLState& /*_state*/, PickTarget /*_target*/, const DrawModes::DrawMode& /*_drawMode*/ ) {
}

//----------------------------------------------------------------------------


GLuint TextureNode::activeTexture()
{
  if (0 <= activeTexture_  && activeTexture_ < int(textures_.size()))
    return textures_[activeTexture_].tex->id();

  return 0;
}

//----------------------------------------------------------------------------

bool TextureNode::activateTexture(GLuint _id)
{
  int search = available(_id);

  //==========================================================================
  // If zero is given, unbind all textures
  //==========================================================================
  if ( _id == 0 ) {
    search = 0;
  }

  //==========================================================================
  // Index has not been found ... No corresponding Texture in this node
  //==========================================================================
  if ( search == -1 ) {
    std::cerr << "Texture to activate not found!" << std::endl;
    return false;
  }

  activeTexture_ = search;

  return true;
}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
