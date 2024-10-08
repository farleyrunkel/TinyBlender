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
//  CLASS DrawModes - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include "DrawModes.hh"
#include "BaseNode.hh"

#include <sstream>


//== NAMESPACES ============================================================== 

namespace ACG {
namespace SceneGraph {
namespace DrawModes {


// == Default Draw Mode initialization ======================================

DrawMode NONE                                              = DrawMode( ModeFlagSet(0)       );
DrawMode DEFAULT                                           = DrawMode( ModeFlagSet(1)       );
DrawMode POINTS                                            = DrawMode( ModeFlagSet(1) << 1  );
DrawMode POINTS_COLORED                                    = DrawMode( ModeFlagSet(1) << 2  );
DrawMode POINTS_SHADED                                     = DrawMode( ModeFlagSet(1) << 3  );
DrawMode EDGES                                             = DrawMode( ModeFlagSet(1) << 4  );
DrawMode EDGES_COLORED                                     = DrawMode( ModeFlagSet(1) << 5  );
DrawMode WIREFRAME                                         = DrawMode( ModeFlagSet(1) << 6  );
DrawMode FACES                                             = DrawMode( ModeFlagSet(1) << 7  );
DrawMode HIDDENLINE                                        = DrawMode( ModeFlagSet(1) << 8  );
DrawMode SOLID_FLAT_SHADED                                 = DrawMode( ModeFlagSet(1) << 9  );
DrawMode SOLID_SMOOTH_SHADED                               = DrawMode( ModeFlagSet(1) << 10 );
DrawMode SOLID_PHONG_SHADED                                = DrawMode( ModeFlagSet(1) << 11 );
DrawMode SOLID_FACES_COLORED                               = DrawMode( ModeFlagSet(1) << 12 );
DrawMode SOLID_POINTS_COLORED                              = DrawMode( ModeFlagSet(1) << 13 );
DrawMode SOLID_POINTS_COLORED_SHADED                       = DrawMode( ModeFlagSet(1) << 14 );
DrawMode SOLID_ENV_MAPPED                                  = DrawMode( ModeFlagSet(1) << 15 );
DrawMode SOLID_TEXTURED                                    = DrawMode( ModeFlagSet(1) << 16 );
DrawMode SOLID_TEXTURED_SHADED                             = DrawMode( ModeFlagSet(1) << 17 );
DrawMode SOLID_1DTEXTURED                                  = DrawMode( ModeFlagSet(1) << 18 );
DrawMode SOLID_1DTEXTURED_SHADED                           = DrawMode( ModeFlagSet(1) << 19 );
DrawMode SOLID_3DTEXTURED                                  = DrawMode( ModeFlagSet(1) << 20 );
DrawMode SOLID_3DTEXTURED_SHADED                           = DrawMode( ModeFlagSet(1) << 21 );
DrawMode SOLID_FACES_COLORED_FLAT_SHADED                   = DrawMode( ModeFlagSet(1) << 22 );
DrawMode SOLID_FACES_COLORED_SMOOTH_SHADED                 = DrawMode( ModeFlagSet(1) << 23 );
DrawMode SOLID_2DTEXTURED_FACE                             = DrawMode( ModeFlagSet(1) << 24 );
DrawMode SOLID_2DTEXTURED_FACE_SHADED                      = DrawMode( ModeFlagSet(1) << 25 );
DrawMode SOLID_SHADER                                      = DrawMode( ModeFlagSet(1) << 26 );
DrawMode SOLID_SMOOTH_SHADED_FEATURES                      = DrawMode( ModeFlagSet(1) << 27 );
DrawMode CELLS                                             = DrawMode( ModeFlagSet(1) << 28 );
DrawMode CELLS_COLORED                                     = DrawMode( ModeFlagSet(1) << 29 );
DrawMode HALFEDGES                                         = DrawMode( ModeFlagSet(1) << 30 );
DrawMode HALFEDGES_COLORED                                 = DrawMode( ModeFlagSet(1) << 31 );
DrawMode SOLID_FACES_COLORED_2DTEXTURED_FACE_SMOOTH_SHADED = DrawMode( ModeFlagSet(1) << 32 );
DrawMode UNUSED                                            = DrawMode( ModeFlagSet(1) << 33 );
  

//== IMPLEMENTATION ========================================================== 


/** \brief Definition of a draw mode
 *
 * This class is used to collect Information about a DrawMode in one central class.
 * It stores the name and the id of a DrawMode.
 *
 * If the DrawMode gets switched to a property based one, this class will also
 * store the properties.
 */
class DrawModeInternal {
  
  public:

    /** Initializing constructor. */
    DrawModeInternal(const std::string & _name, const DrawMode & _id, const bool _propertyBased = false) :
      name_(_name),
      id_(_id),
      propertyBased_(_propertyBased)
    {
    }

    /// Set the name of the DrawMode
    void name(const std::string& _name) {
      name_ = _name;
    }

    /// Get the name of the DrawMode
    const std::string& name() const {
      return name_;
    }
    
    const DrawMode& id() const {
      return id_;
    }

    bool propertyBased() const {
      return propertyBased_;
    }


    DrawModeProperties& properties() {
      return properties_;
    }

  private:
    std::string        name_;          ///< Human Readable Name
    DrawMode           id_;            ///< The id of the DrawMode
    bool               propertyBased_; ///< Flag if the DrawMode is property based
    DrawModeProperties properties_;    ///< The properties associated with this DrawMode
};


typedef std::vector< DrawModeInternal > VecDrawModes;

/** Vector of all currently defined DrawModes.
*/
static VecDrawModes registeredDrawModes_;

/** First free DrawMode ID for custom modes. */
static DrawMode firstFreeID_;


DrawModeProperties::DrawModeProperties(DrawModePrimitive _primitive, 
                                       DrawModeLightStage _lightStage, 
                                       DrawModeNormalSource _normalSource,
                                       DrawModeColorSource _colorSource,
                                       DrawModeTexCoordSource _texcoordSource,
                                       bool _envMapping):
envMapped_(_envMapping),
primitive_(_primitive),
lightStage_(_lightStage),
colorSource_(_colorSource),
texcoordSource_(_texcoordSource),
normalSource_(_normalSource)
{
}


DrawMode::DrawMode(size_t _index)
{
  modeFlags_.reset();
  if ( _index >= modeFlags_.size() ) {
    std::cerr << "Illegal drawMode specification from unsigned int. This should not be a bitset!!!" << std::endl;
  } else {
    modeFlags_.set(_index);
  }
  layers_.resize(1);
  layers_[0] = DrawModeProperties();
}

DrawMode::DrawMode() {
  layers_.resize(1);
  layers_[0] = DrawModeProperties();
}
    
DrawMode::DrawMode(const ModeFlagSet& _flags) :
        modeFlags_(_flags)
{
  layers_.resize(1);
  layers_[0] = DrawModeProperties();
}

DrawMode DrawMode::getFromDescription(std::string _description)
{
    DrawMode val;
    bool found = false;
    std::istringstream f(_description);
    std::string s;
    while (std::getline(f, s, '+')) {
        VecDrawModes::const_iterator modeIter, modeEnd( registeredDrawModes_.end() );
            for( modeIter = registeredDrawModes_.begin();  modeIter != modeEnd;  ++modeIter ) {
                if(modeIter->name() == s) {
                    val |= modeIter->id();
                    found = true;
                }
            }
    }
    if(!found)
        return DEFAULT;
    else return val;
}

DrawMode::operator bool() const {
  return( modeFlags_ != NONE.modeFlags_ );
}
/*
bool DrawMode::propertyBased() const {
  if ( isAtomic() ) {
    return registeredDrawModes_[getIndex()].propertyBased();
  } else {
    // Combined drawmode, iterate over all contained modes and return
    VecDrawModes::const_iterator modeIter, modeEnd( registeredDrawModes_.end() );
    for( modeIter = registeredDrawModes_.begin();  modeIter != modeEnd;  ++modeIter )
      if( ((*this) & modeIter->id()) && modeIter->propertyBased() )
        return true;

    return false;
  }
}
*/

void DrawMode::setDrawModeProperties(const DrawModeProperties* _props) {
  if (_props)
  {
    if (layers_.empty())
      layers_.push_back(*_props);
    else
      layers_[0] = *_props;
  }
  else
    layers_.erase(layers_.begin());
}

void DrawMode::setDrawModeProperties( const DrawModeProperties& _props )
{
  setDrawModeProperties(&_props);
}

bool DrawMode::operator==(const DrawMode& _mode) const {
    return modeFlags_ == _mode.modeFlags_;
  //return ((modeFlags_ & _mode.modeFlags_).any());
}

bool DrawMode::operator!=( const DrawMode& _mode2  ) const {
  return (modeFlags_ != _mode2.modeFlags_);
}

DrawMode& DrawMode::operator++() {
  if ( modeFlags_.count() != 1 ) {
    std::cerr << "Operator ++ for drawMode which is not atomic!!" << std::endl;
  }

  modeFlags_ <<= 1;
  
  return (*this);
}

DrawMode DrawMode::operator&(const DrawMode& _mode) const {
  DrawMode andMode = DrawMode(modeFlags_ & _mode.modeFlags_);

  andMode.setDrawModeProperties(getDrawModeProperties());

  for (unsigned int i = 1; i < getNumLayers(); ++i)
    andMode.addLayer(getLayer(i));

  // remove all distinct layers
  for (int i = (int)andMode.getNumLayers() - 1; i >= 0; --i)
  {
    int layerIndex = _mode.getLayerIndex(andMode.getLayer(i));

    if (layerIndex < 0)
      andMode.removeLayer(i);
  }

  return andMode;
}

DrawMode& DrawMode::operator|=( const DrawMode& _mode2  ) {
  modeFlags_ |= _mode2.modeFlags_;
  
  for (unsigned int i = 0; i < _mode2.getNumLayers(); ++i)
    addLayer(_mode2.getLayer(i));

//  assert(checkConsistency());

  return (*this);
}

DrawMode& DrawMode::operator&=( const DrawMode& _mode2  ) {
  modeFlags_ &= _mode2.modeFlags_;

  // remove all distinct layers
  for (int i = (int)getNumLayers() - 1; i >= 0; --i)
  {
    int layerIndex2 = _mode2.getLayerIndex(getLayer(i));

    if (layerIndex2 < 0)
      removeLayer(i);
  }

//  assert(checkConsistency());

  return (*this);
}

DrawMode DrawMode::operator|( const DrawMode& _mode2  ) const {
  DrawMode combined = DrawMode( modeFlags_ | _mode2.modeFlags_ );

  combined.setDrawModeProperties(getDrawModeProperties());

  for (unsigned int i = 1; i < getNumLayers(); ++i)
    combined.addLayer(getLayer(i));

  for (unsigned int i = 0; i < _mode2.getNumLayers(); ++i)
    combined.addLayer(_mode2.getLayer(i));

  return combined;
}

DrawMode DrawMode::operator^( const DrawMode& _mode2  ) const {

  DrawMode xorMode = DrawMode( modeFlags_ ^ _mode2.modeFlags_ );

  // xor on properties (curProps)

  // do xor on new temporary DrawMode
  //  internal layers of this and _mode2 must stay the same
  std::vector<const DrawModeProperties*> tmpLayers;


  // initialize tmpLayers with my own layers
  for (unsigned int i = 0; i < getNumLayers(); ++i)
  {
    const DrawModeProperties* curProps = getLayer(i);

    if (curProps)
      tmpLayers.push_back(curProps);
  }


  // xor on tmpLayers
  for (unsigned int i = 0; i < _mode2.getNumLayers(); ++i)
  {
    const DrawModeProperties* curProps = _mode2.getLayer(i);

    if (!curProps) continue;
   


    int addToVec = 1;

    // is the other layer already contained in my own list?
    for (unsigned int k = 0; addToVec && k < tmpLayers.size(); ++k)
    {
      if (!memcmp(tmpLayers[k], curProps, sizeof(DrawModeProperties)))
      {
        // yes, remove it  (layer exists in both drawmodes)
        tmpLayers.erase(tmpLayers.begin() + k);
        addToVec = 0;
      }
    }

    if (addToVec) // no, add it
      tmpLayers.push_back(curProps);
  }



  // DrawModes equal?
  if (tmpLayers.empty())
  {
    xorMode.removeLayer(0u);
    return xorMode; // return default property set to not cause exceptions
  }

  // layers not empty,
  //  copy to temporary drawmode and return

  xorMode.setDrawModeProperties(tmpLayers[0]);
  for (unsigned int i = 1; i < tmpLayers.size(); ++i)
    xorMode.addLayer(tmpLayers[i]);


//  assert(xorMode.checkConsistency());

  return xorMode;
}

DrawMode DrawMode::operator~( ) const {
  return( DrawMode(~modeFlags_) );
}    



size_t DrawMode::getIndex() const {
  if ( modeFlags_.count() == 1 ) {
    for ( size_t i = 0 ; i < modeFlags_.size() ; ++i ) 
     if ( modeFlags_[i] ) 
      return i; 
  } 
    
  return 0;
}

std::string DrawMode::description() const
{
  std::string text("");
  
  VecDrawModes::const_iterator modeIter, modeEnd( registeredDrawModes_.end() );
  for( modeIter = registeredDrawModes_.begin();  modeIter != modeEnd;  ++modeIter )
  {
    if( (*this) & modeIter->id() )
    {
      if (!text.empty()) text += "+";
      text += modeIter->name();
    }
  }
  
  return text;
}

//----------------------------------------------------------------------------

void DrawMode::filter( DrawMode _filter )
{
  modeFlags_ = (modeFlags_ | _filter.modeFlags_) ^ _filter.modeFlags_;

  for (unsigned int i = 0; i < _filter.getNumLayers(); ++i)
  {
    int idx = getLayerIndex(_filter.getLayer(i));

    removeLayer((unsigned int)idx);
  }
}



//----------------------------------------------------------------------------

void DrawMode::combine( DrawMode _mode )
{
  // XOR on bitflag
  modeFlags_ = (modeFlags_ ^ _mode.modeFlags_);

  // addLayer does redundancy check here
  for (unsigned int i = 0; i < _mode.getNumLayers(); ++i)
    addLayer(_mode.getLayer(i));

//  checkConsistency();
//  assert(checkConsistency());
}

//----------------------------------------------------------------------------

std::vector< DrawMode >
DrawMode::getAtomicDrawModes() const
{
  std::vector< DrawMode > draw_mode_ids;
  
  VecDrawModes::const_iterator modeIter, modeEnd( registeredDrawModes_.end() );
  for( modeIter = registeredDrawModes_.begin();
  modeIter != modeEnd;
  ++modeIter )
  if( (*this) & modeIter->id() )
    draw_mode_ids.push_back( modeIter->id() );
  
  return draw_mode_ids;
}


//----------------------------------------------------------------------------

bool DrawMode::isAtomic() const {
  return(modeFlags_.count() == 1 );
}
 

//----------------------------------------------------------------------------
 
bool
DrawMode::containsAtomicDrawMode( const DrawMode& _atomicDrawMode) const
{
  return (*this) & _atomicDrawMode;
}
 
//---------------------------------------------------------------------------- 
 
size_t DrawMode::maxModes() const {
  return (modeFlags_.size() );
}

size_t DrawMode::getNumLayers() const {
  return layers_.size();
}

const DrawModeProperties* DrawMode::getLayer( unsigned int i ) const {
  return (i >= layers_.size() ? 0 : &layers_[i]);
}


void DrawMode::addLayer( const DrawModeProperties* _props )
{
  if (getLayerIndex(_props) < 0 && _props)
    layers_.push_back(*_props);
}

bool DrawMode::removeLayer( unsigned int _i )
{
  if (_i < layers_.size() )
  {
    layers_.erase(layers_.begin() + _i);
    return true;
  }

  return false;
}

bool DrawMode::removeLayer( const DrawModeProperties* _prop )
{
  int layerId = getLayerIndex(_prop);

  if (layerId >= 0)
    return removeLayer((unsigned int)layerId);

  return false;
}

const DrawModeProperties* DrawMode::getDrawModeProperties() const
{
  return getLayer(0);
}


bool DrawMode::checkConsistency() const
{

  // allow at most one layer per primitive
  for (unsigned int i = 0; i < layers_.size(); ++i)
  {
    for (unsigned int k = i+1; k < layers_.size(); ++k)
    {
      if (layers_[i].primitive() == layers_[k].primitive())
        return false;
    }
  }



  // bitflag -> layer parallelism

  // points-mode in bitflag => point layer expected

  if ((*this & DrawModes::POINTS) ||
      (*this & DrawModes::POINTS_COLORED) ||
      (*this & DrawModes::POINTS_SHADED))
  {
    int pointsLayer = 0;
    for (unsigned int k = 0; k < layers_.size(); ++k)
    {
      if (layers_[k].primitive() == PRIMITIVE_POINT)
        pointsLayer++;
    }

    if (!pointsLayer)
      return false;
  } 


  return true;
}

int DrawMode::getLayerIndex( const DrawModeProperties* _prop ) const
{
  if (!_prop) return -1;

  for (unsigned int i = 0; i < layers_.size(); ++i)
  {
    if ( layers_[i] == *_prop )
      return (int)i;

    //    if (!memcmp(&layers_[i], &bla, sizeof(DrawModeProperties)))
    //      return (int)i;
  }
  return -1;
}

int DrawMode::getLayerIndexByPrimitive( DrawModePrimitive _type ) const
{
  for (unsigned int i = 0; i < layers_.size(); ++i)
  {
    if ( layers_[i].primitive() == _type )
      return (int)i;
  }
  return -1;
}

//----------------------------------------------------------------------------


void initializeDefaultDrawModes( void )
{
    static bool initialized_ = false;

    if( initialized_ )
	return;

    registeredDrawModes_.clear();

    NONE.removeLayer(0u);
    DEFAULT.removeLayer(0u);

    POINTS.                     setDrawModeProperties(DrawModeProperties(PRIMITIVE_POINT));
    POINTS_COLORED.             setDrawModeProperties(DrawModeProperties(PRIMITIVE_POINT, LIGHTSTAGE_UNLIT, NORMAL_NONE, COLOR_PER_VERTEX));
    POINTS_SHADED.              setDrawModeProperties(DrawModeProperties(PRIMITIVE_POINT, LIGHTSTAGE_SMOOTH, NORMAL_PER_VERTEX));

    EDGES.                      setDrawModeProperties(DrawModeProperties(PRIMITIVE_EDGE));
    EDGES_COLORED.              setDrawModeProperties(DrawModeProperties(PRIMITIVE_EDGE, LIGHTSTAGE_UNLIT, NORMAL_NONE, COLOR_PER_VERTEX));

    WIREFRAME.                  setDrawModeProperties(DrawModeProperties(PRIMITIVE_EDGE));
    EDGES_COLORED.              setDrawModeProperties(DrawModeProperties(PRIMITIVE_EDGE, LIGHTSTAGE_UNLIT, NORMAL_NONE, COLOR_PER_EDGE));

    WIREFRAME.                  setDrawModeProperties(DrawModeProperties(PRIMITIVE_WIREFRAME));

    FACES.                      setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON));

    HIDDENLINE.                 setDrawModeProperties(DrawModeProperties(PRIMITIVE_HIDDENLINE));

    SOLID_FLAT_SHADED.          setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_SMOOTH, NORMAL_PER_FACE));
    SOLID_SMOOTH_SHADED.        setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_SMOOTH, NORMAL_PER_VERTEX));

    SOLID_PHONG_SHADED.         setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_PHONG, NORMAL_PER_VERTEX));

    SOLID_FACES_COLORED.        setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_UNLIT, NORMAL_NONE, COLOR_PER_FACE));
    
    SOLID_POINTS_COLORED.       setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_UNLIT,   NORMAL_NONE,      COLOR_PER_VERTEX));
    SOLID_POINTS_COLORED_SHADED.setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_SMOOTH,  NORMAL_PER_VERTEX,  COLOR_PER_VERTEX));

    SOLID_ENV_MAPPED.           setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_UNLIT, NORMAL_NONE,      COLOR_NONE, TEXCOORD_PER_VERTEX, true));

    SOLID_TEXTURED.             setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_UNLIT,   NORMAL_NONE,      COLOR_NONE, TEXCOORD_PER_VERTEX));
    SOLID_TEXTURED_SHADED.      setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_SMOOTH,  NORMAL_PER_VERTEX,COLOR_NONE, TEXCOORD_PER_VERTEX));

    SOLID_1DTEXTURED.           setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_UNLIT,   NORMAL_NONE,  COLOR_NONE, TEXCOORD_PER_VERTEX));
    SOLID_1DTEXTURED_SHADED.    setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_SMOOTH,  NORMAL_PER_VERTEX,COLOR_NONE, TEXCOORD_PER_VERTEX));

    SOLID_3DTEXTURED.           setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_UNLIT,   NORMAL_NONE,  COLOR_NONE, TEXCOORD_PER_VERTEX));
    SOLID_3DTEXTURED_SHADED.    setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_SMOOTH,  NORMAL_PER_VERTEX,COLOR_NONE, TEXCOORD_PER_VERTEX));

    SOLID_FACES_COLORED_FLAT_SHADED.  setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_SMOOTH, NORMAL_PER_FACE,   COLOR_PER_FACE));
    SOLID_FACES_COLORED_SMOOTH_SHADED.setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_SMOOTH, NORMAL_PER_VERTEX, COLOR_PER_FACE));
    SOLID_FACES_COLORED_2DTEXTURED_FACE_SMOOTH_SHADED.setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_SMOOTH, NORMAL_PER_VERTEX, COLOR_PER_FACE, TEXCOORD_PER_HALFEDGE));

    SOLID_2DTEXTURED_FACE.       setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_UNLIT, NORMAL_NONE, COLOR_NONE, TEXCOORD_PER_HALFEDGE));
    SOLID_2DTEXTURED_FACE_SHADED.setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_SMOOTH, NORMAL_PER_FACE, COLOR_NONE, TEXCOORD_PER_HALFEDGE));

    SOLID_SMOOTH_SHADED_FEATURES.setDrawModeProperties(DrawModeProperties(PRIMITIVE_POLYGON, LIGHTSTAGE_SMOOTH, NORMAL_PER_HALFEDGE));

    CELLS.setDrawModeProperties(DrawModeProperties(PRIMITIVE_CELL));
    CELLS_COLORED.setDrawModeProperties(DrawModeProperties(PRIMITIVE_CELL, LIGHTSTAGE_UNLIT, NORMAL_NONE, COLOR_PER_VERTEX));

    HALFEDGES.setDrawModeProperties(DrawModeProperties(PRIMITIVE_HALFEDGE));
    HALFEDGES_COLORED.setDrawModeProperties(DrawModeProperties(PRIMITIVE_HALFEDGE, LIGHTSTAGE_UNLIT, NORMAL_NONE, COLOR_PER_HALFEDGE));


    registeredDrawModes_.push_back( DrawModeInternal( "<invalid>", NONE ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Default", DEFAULT ) );

    registeredDrawModes_.push_back( DrawModeInternal( "Points", POINTS ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Points (colored)", POINTS_COLORED ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Points (shaded)", POINTS_SHADED  ) );
    
    registeredDrawModes_.push_back( DrawModeInternal( "Edges", EDGES ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Edges Colored", EDGES_COLORED ) );
    
    registeredDrawModes_.push_back( DrawModeInternal( "Wireframe", WIREFRAME ) );
    
    registeredDrawModes_.push_back( DrawModeInternal( "Faces", FACES ) );
    
    registeredDrawModes_.push_back( DrawModeInternal( "Hiddenline", HIDDENLINE ) );
    
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (flat shaded)", SOLID_FLAT_SHADED ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (smooth shaded)", SOLID_SMOOTH_SHADED ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (Phong shaded)", SOLID_PHONG_SHADED ) );
    
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (colored per-face)", SOLID_FACES_COLORED ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (colored per-vertex)", SOLID_POINTS_COLORED ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (colored per-vertex, shaded)", SOLID_POINTS_COLORED_SHADED ) );
    
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (environment mapped)", SOLID_ENV_MAPPED ) );
    
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (textured)", SOLID_TEXTURED ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (textured, shaded)", SOLID_TEXTURED_SHADED ) );
    
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (scalar field)", SOLID_1DTEXTURED ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (scalar field, shaded)", SOLID_1DTEXTURED_SHADED ) );
    
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (3D textured)", SOLID_3DTEXTURED ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (3D textured, shaded)", SOLID_3DTEXTURED_SHADED ) );

    registeredDrawModes_.push_back( DrawModeInternal( "Solid (colored per-face, flat shaded)", SOLID_FACES_COLORED_FLAT_SHADED ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (colored per-face, smooth shaded)", SOLID_FACES_COLORED_SMOOTH_SHADED ) );

    registeredDrawModes_.push_back(DrawModeInternal("Solid (colored per-face, face textured, smooth shaded)", SOLID_FACES_COLORED_2DTEXTURED_FACE_SMOOTH_SHADED));
    

    registeredDrawModes_.push_back( DrawModeInternal( "Solid (face textured)", SOLID_2DTEXTURED_FACE ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Solid (face textured, shaded)", SOLID_2DTEXTURED_FACE_SHADED ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Shader controlled", SOLID_SHADER ) );

    registeredDrawModes_.push_back( DrawModeInternal( "Solid (smooth shaded, features)", SOLID_SMOOTH_SHADED_FEATURES ) );

    registeredDrawModes_.push_back( DrawModeInternal( "Cells", CELLS ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Cells Colored", CELLS_COLORED ) );
    
    registeredDrawModes_.push_back( DrawModeInternal( "Halfedges", HALFEDGES ) );
    registeredDrawModes_.push_back( DrawModeInternal( "Halfedges Colored", HALFEDGES_COLORED ) );

    firstFreeID_ = UNUSED;
    initialized_ = true;
}


//----------------------------------------------------------------------------


const DrawMode& addDrawMode( const std::string & _name , bool _propertyBased)
{
  // check if mode exists already
  VecDrawModes::iterator modeIter, modeEnd( registeredDrawModes_.end() );

  for( modeIter = registeredDrawModes_.begin(); modeIter != modeEnd; ++modeIter ) {
    if( _name == modeIter->name() ) {
      return modeIter->id();
    }
  }


  // add new mode
  registeredDrawModes_.push_back( DrawModeInternal( _name, firstFreeID_ , _propertyBased) );
  ++firstFreeID_;

  return registeredDrawModes_[ registeredDrawModes_.size() - 1 ].id();
}

//----------------------------------------------------------------------------

ACGDLLEXPORT
const DrawMode& addDrawMode( const std::string & _name, const DrawModeProperties& _properties)
{
  const DrawMode& drawmode = addDrawMode( _name , true );

  // Get the internal DrawMode
  VecDrawModes::iterator modeIter, modeEnd( registeredDrawModes_.end() );

  for( modeIter = registeredDrawModes_.begin(); modeIter != modeEnd; ++modeIter ) {
    if( _name == modeIter->name() ) {
      modeIter->properties() = _properties;
      return drawmode;
    }
  }

  return drawmode;
}

//----------------------------------------------------------------------------


const DrawMode& getDrawMode( const std::string & _name )
{
  // check if mode exists
  VecDrawModes::const_iterator modeIter, modeEnd( registeredDrawModes_.end() );

  for( modeIter = registeredDrawModes_.begin(); modeIter != modeEnd;  ++modeIter )
  {
    if( _name == modeIter->name() )
    {
      return modeIter->id();
    }
  }

  // the DrawMode does not exists
  return DrawModes::NONE;
}

bool drawModeExists(const std::string & _name) {
  
  // check if mode exists
  VecDrawModes::const_iterator modeIter, modeEnd( registeredDrawModes_.end() );
  
  for( modeIter = registeredDrawModes_.begin();  modeIter != modeEnd;  ++modeIter )
  {
    if( _name == modeIter->name() )
      return true;
  }
  
  // the DrawMode does not exists
  return false;  
}


DrawMode getDrawModeFromIndex( unsigned int _index ) {
  return DrawMode(_index);
}

//=============================================================================
} // namespace DrawModes
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
