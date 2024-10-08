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
//  CLASS MaterialNode
//
//=============================================================================

#ifndef ACG_MATERIAL_NODE_HH
#define ACG_MATERIAL_NODE_HH


//== INCLUDES =================================================================

#include "BaseNode.hh"
#include <string>
#include <QVariantMap>

//== NAMESPACES ===============================================================

namespace ACG {

QVariantMap ACGDLLEXPORT json_to_variant_map(QString json);

namespace SceneGraph {


//== CLASS DEFINITION =========================================================

/** \class Material

    Class to store the properties of a material.
**/

class ACGDLLEXPORT Material {

  friend class MaterialNode;

public:
    /// Default constructor
    Material() :
        baseColor_(GLState::default_base_color),
        ambientColor_(GLState::default_ambient_color),
        diffuseColor_(GLState::default_diffuse_color),
        specularColor_(GLState::default_specular_color),
        overlayColor_(GLState::default_overlay_color),
        shininess_(GLState::default_shininess),
        reflectance_(0.0),
        indexOfRefraction_(1.0),
        isRefractive_(false),
        pointSize_(1.0),
        lineWidth_(1.0),
        roundPoints_(false),
        linesSmooth_(false),
        alphaTest_(false),
        alphaClip_(0),
        blending_(false),
        blendParam1_(GL_SRC_ALPHA),
        blendParam2_(GL_ONE_MINUS_SRC_ALPHA),
        colorMaterial_(true),
        backfaceCulling_(false),
        multiSampling_(true) {};

    /// Deconstructor
    virtual ~Material() {};

    /// Copy constructor
    Material(const Material&) = default;

    static bool support_json_serialization();
    QString serializeToJson() const;
    void deserializeFromJson(const QString &json);
    void deserializeFromVariantMap(const QVariantMap &matMap);

    /** \brief Set color based on _c
     *
     *  Basecolor (which is the emitted color is set to zero)
     *  The other colors are based on the given color _c
     *
     *  @param _c Sample color used to compute the other color components (ambient, diffuse, specular)
     */
    void color(const Vec4f& _c) {
        Vec4f c(0.0,0.0,0.0,1.0);
        baseColor(c);
        overlayColor(c);
        c = _c * 0.2f;  c[3]=_c[3];  ambientColor(c);
        c = _c * 0.6f;  c[3]=_c[3];  diffuseColor(c);
        c = _c * 0.8f;  c[3]=_c[3];  specularColor(c);
    }

    /// Creates a randomized color and sets it
    void generateRandomColor() {
      color( Vec4f(0.2 + double(rand())/double(RAND_MAX)*0.8,
                   0.2 + double(rand())/double(RAND_MAX)*0.8,
                   0.2 + double(rand())/double(RAND_MAX)*0.8,
                   1.0));
    }

    /// set the base color (Sets the baseColor which is the same as the emission(const Vec4f& _c) )
    void baseColor(const Vec4f& _c) { baseColor_ = _c;}
    /// get the base color ( Same as emission() )
    const Vec4f& baseColor() const { return baseColor_; }

    /// set emission ( Same as baseColor( const Vec4f& _c )) )
    void emissionColor(const Vec4f& _c) { baseColor_ = _c;}
    /// get emission ( Same as baseColor() )
    const Vec4f& emissionColor() const { return baseColor_; }

    /// set the ambient color.
    void ambientColor(const Vec4f& _a) { ambientColor_ = _a; }
    /// get the ambient color.
    const Vec4f& ambientColor() const { return ambientColor_; }

    /// set the diffuse color.
    void diffuseColor(const Vec4f& _d) { diffuseColor_ = _d; }
    /// get the diffuse color.
    const Vec4f& diffuseColor() const { return diffuseColor_; }

    /// set the specular color
    void specularColor(const Vec4f& _s) { specularColor_ = _s; }
    /// get the specular color
    const Vec4f& specularColor() const { return specularColor_; }

    /// set the overlay color (This can be used to render overlays e.g. additional wireframes in a different color)
    void overlayColor(const Vec4f& _s) { overlayColor_ = _s; }
    /// get the overlay color (This can be used to render overlays e.g. additional wireframes in a different color)
    const Vec4f& overlayColor() const { return overlayColor_; }

    /// Set colorMaterial
    void colorMaterial( const bool _cm) { colorMaterial_ = _cm; }
    /// Enable Color Material
    void enableColorMaterial() { colorMaterial_ = true; }
    /// Disable Color Material
    void disableColorMaterial() { colorMaterial_ = false; }
    /// get colorMaterial state
    bool colorMaterial() { return colorMaterial_; }

    /// set shininess
    void shininess(float _s) { shininess_ = _s; }
    /// get shininess
    float shininess() const { return shininess_; }

    /// set reflectance ( not used in OpenGL Rendering)
    void reflectance(double _m) { reflectance_ = _m; }
    /// get reflectance ( not used in OpenGL Rendering)
    double reflectance() const { return reflectance_; }

    /// set index of refraction
    void indexOfRefraction(double _m) { indexOfRefraction_ = _m; }
    /// get index of refraction ( not used in OpenGL Rendering)
    double indexOfRefraction() const { return indexOfRefraction_; }

    /// set refractive flag
    void setRefractive(bool _r) { isRefractive_ = _r; }
    /// get refractive flag
    bool isRefractive() const {return isRefractive_;}

    /// set point size (default: 1.0)
    void pointSize(float _sz) { pointSize_ = _sz; }
    /// get point size
    float pointSize() const { return pointSize_; }

    /// set line width (default: 1.0)
    void lineWidth(float _sz) { lineWidth_ = _sz; }
    /// get line width
    float lineWidth() const { return lineWidth_; }

    /// set: round points enabled
    void roundPoints(bool _b) { roundPoints_ = _b; }
    /// get: round points enabled
    bool roundPoints() const { return roundPoints_; }

    /// set: smooth lines enabled
    void lineSmooth(bool _b) { linesSmooth_ = _b; }
    /// get: rsmooth lines enabled
    bool lineSmooth() const { return linesSmooth_; }

    /// enable alpha test (draw pixels if alpha >= _clip)
    void enableAlphaTest(float _clip) {
      alphaTest_ = true; alphaClip_ = _clip;
    }

    /// disable alpha test
    void disableAlphaTest() { alphaTest_ = false; }

    /// Return state of Alpha test
    bool alphaTest() const { return alphaTest_; };

    /// Enable Multisampling
    void enableMultisampling() {
      multiSampling_ = true;
    }

    /// enable alpha test (draw pixels if alpha >= _clip)
    void disableMultisampling() {
      multiSampling_ = false;
    }

    /// Get state of multisampling
    bool multiSampling() const {
      return multiSampling_;
    }

    /// Set state of multisampling
    void multisampling( bool _state ) {
      multiSampling_ = _state;
    }

    ///get current alpha value for alpha_test
    float alphaValue() const { return alphaClip_; };

    bool blending() const { return blending_; };

    GLenum blendingParam1() const { return blendParam1_; };
    GLenum blendingParam2() const { return blendParam2_; };

    /// enable blending with Parameters (_p1, _p2)
    void enableBlending(GLenum _p1 = GL_SRC_ALPHA,
               GLenum _p2 = GL_ONE_MINUS_SRC_ALPHA)
    { blending_ = true; blendParam1_ = _p1; blendParam2_ = _p2; }
    /// disable blending
    void disableBlending() { blending_ = false; }

    bool backfaceCulling() const { return backfaceCulling_; };

    /// enable backface culling (not active by default, see applyProperties)
    void enableBackfaceCulling() { backfaceCulling_ = true; }
    /// disable backface culling (not active by default, see applyProperties)
    void disableBackfaceCulling() { backfaceCulling_ = false; }

    bool isEmissive() const { return (baseColor_[0] > 0.f || baseColor_[1] > 0.f || baseColor_[2] > 0.f); }

protected:

    Vec4f    baseColor_;
    Vec4f    ambientColor_;
    Vec4f    diffuseColor_;
    Vec4f    specularColor_;
    Vec4f    overlayColor_;

    float    shininess_;
    double   reflectance_;
    double   indexOfRefraction_;
    bool     isRefractive_;
    float    pointSize_;
    float    lineWidth_;

    bool     roundPoints_;
    bool     linesSmooth_;

    bool     alphaTest_;
    float    alphaClip_;

    bool     blending_;
    GLenum   blendParam1_;
    GLenum   blendParam2_;

    bool     colorMaterial_;
    bool     backfaceCulling_;

    bool     multiSampling_;
};


/** \class MaterialNode MaterialNode.hh <ACG/Scenegraph/MaterialNode.hh>

    Set material and some other stuff like alphatest and blending
    for this node and all its children.
    All changes will be done in the enter() method undone
    in the leave() method.
**/

class ACGDLLEXPORT MaterialNode : public BaseNode
{
public:

  /// Apply which properties? Others will be ignored. Values may be OR'ed.
  enum ApplyProperties
  {
    /// apply nothing
    None=0,
    /// apply all properites
    All=0xffff,
    /// apply base color
    BaseColor=1,
    /// apply material (ambient, diffuse, specular, shininess)
    Material=2,
    /// apply point size
    PointSize=4,
    /// apply line width
    LineWidth=8,
    /// draw smooth (round) points using glPoint()
    RoundPoints=16,
    /// draw smooth lines using glLine()
    LineSmooth=32,
    /// use alpha test
    AlphaTest=64,
    /// use blending
    Blending=128,
    /// backface culling
    BackFaceCulling=256,
    /// Color Material ( Only when a drawmode using shading and lighting is enabled )
    ColorMaterial=512,
    /// MultiSampling
    MultiSampling=1024
  };

  /// Default constructor. Applies all properties.
  MaterialNode( BaseNode*           _parent = 0,
                const std::string&  _name = "<MaterialNode>",
                unsigned int        _applyProperties = (All & ~BackFaceCulling));

  /// Destructor.
  virtual ~MaterialNode() {};

  /// read MaterialFile
  void read( std::istream & _is);

  //===========================================================================
  /** @name Scenegraph functions
    * @{ */
  //===========================================================================

    ACG_CLASSNAME(MaterialNode);

    /// set current GL-color and GL-material
    void enter(GLState& _state, const DrawModes::DrawMode& _drawmode) override;
    /// restores original GL-color and GL-material
    void leave(GLState& _state, const DrawModes::DrawMode& _drawmode) override;


    /** \brief Do nothing in picking*/
    void enterPick(GLState& _state, PickTarget _target, const DrawModes::DrawMode& _drawMode ) override;

    /** \brief Do nothing in picking */
    void leavePick(GLState& _state, PickTarget _target, const DrawModes::DrawMode& _drawMode ) override;

    /** @} */

    //===========================================================================
    /** @name Color and material settings ( Applied to all objects below this node )
      * @{ */
    //===========================================================================

    /// set color (base, ambient, diffuse, specular) based on _c
    void set_color(const Vec4f& _c) {
      material_.color(_c);
    }

    /// Generates a random color and sets it
    void set_random_color() {
      material_.generateRandomColor();
    }

    /// set the base color ( Same as set_emission(const Vec4f& _c) )
    void set_base_color(const Vec4f& _c) { material_.baseColor(_c); }
    /// get the base color ( same as emission() )
    const Vec4f& base_color() const { return material_.baseColor(); }

    /// set emission ( same as set_base_color(const Vec4f& _c) )
    void set_emission(const Vec4f& _c) { material_.baseColor(_c); }
    /// get emission ( same as base_color() )
    const Vec4f& emission() const { return material_.baseColor(); }

    /// set the ambient color.
    void set_ambient_color(const Vec4f& _a) { material_.ambientColor(_a); }
    /// get the ambient color.
    const Vec4f& ambient_color() const { return material_.ambientColor(); }

    /// set the diffuse color.
    void set_diffuse_color(const Vec4f& _d) { material_.diffuseColor(_d); }
    /// get the diffuse color.
    const Vec4f& diffuse_color() const { return material_.diffuseColor(); }

    /// set the specular color
    void set_specular_color(const Vec4f& _s) { material_.specularColor(_s); }
    /// get the specular color
    const Vec4f& specular_color() const { return material_.specularColor(); }

    /// set the overlay color
    void set_overlay_color(const Vec4f& _s) { material_.overlayColor(_s); }
    /// get the overlay color
    const Vec4f& overlay_color() const { return material_.overlayColor(); }

    /// Set colorMaterial
    void colorMaterial( const bool _cm) { material_.colorMaterial(_cm); }
    /// Enable Color Material
    void enable_color_material() { material_.enableColorMaterial(); }
    /// Disable Color Material
    void disable_color_material() { material_.disableColorMaterial(); }
    /// get colorMaterial state
    bool colorMaterial() { return material_.colorMaterial(); }

    /// set shininess
    void set_shininess(float _s) { material_.shininess(_s); }
    /// get shininess
    float shininess() const { return material_.shininess(); }

    /// set reflectance
    void set_reflectance(double _m) { material_.reflectance(_m); }
    /// get reflectance
    double reflectance() const { return material_.reflectance(); }

    /// set index of refraction
    void set_indexOfRefraction(double _m) { material_.indexOfRefraction(_m); }
    /// get index of refraction
    double indexOfRefraction() const { return material_.indexOfRefraction(); }

    /// set refractive flag
    void set_refractive(bool _r) { material_.isRefractive_ = _r; }
    /// get refractive flag
    bool isRefractive() const {return material_.isRefractive_;}

  /** @} */

  //===========================================================================
  /** @name Point/Line controls
    * @{ */
  //===========================================================================
    /// set point size (default: 1.0)
    void set_point_size(float _sz) { material_.pointSize(_sz); }
    /// get point size
    float point_size() const { return material_.pointSize(); }

    /// set line width (default: 1.0)
    void set_line_width(float _sz) { material_.lineWidth(_sz); }
    /// get line width
    float line_width() const { return material_.lineWidth(); }

    /// set round points enabled
    void set_round_points(bool _b) { material_.roundPoints(_b); }
    /// get round points enabled
    bool round_points() const { return material_.roundPoints(); }

    /// set: smooth lines enabled
    void set_line_smooth(bool _b) { material_.lineSmooth(_b); }
    /// get: rsmooth lines enabled
    bool line_smooth() const { return material_.lineSmooth(); }

  /** @} */

  //===========================================================================
  /** @name Tests
    * @{ */
  //===========================================================================

    /// enable alpha test (draw pixels if alpha >= _clip)
    void enable_alpha_test(float _clip) { material_.enableAlphaTest(_clip); }

    /// disable alpha test
    void disable_alpha_test() { material_.disableAlphaTest(); }

    /// Return state of Alpha test
    bool alpha_test() { return material_.alphaTest(); };

  /** @} */

  //===========================================================================
  /** @name Other Rendering options
    * @{ */
  //===========================================================================

    /// Enable Multisampling
    void enable_multisampling() { material_.enableMultisampling(); }

    /// enable alpha test (draw pixels if alpha >= _clip)
    void disable_multisampling() { material_.disableMultisampling(); }

    /// Get state of multisampling
    bool multiSampling() { return material_.multiSampling(); }

    /// Set state of multisampling
    void set_multisampling( bool _state ) { material_.multisampling(_state); }

  /** @} */

  ///get current alpha value for alpha_test
  float alpha_value(){ return material_.alphaValue(); };

  bool blending() { return material_.blending(); };

  GLenum blending_param1() { return material_.blendingParam1(); };
  GLenum blending_param2() { return material_.blendingParam2(); };

  /// enable blending with Parameters (_p1, _p2)
  void enable_blending(GLenum _p1 = GL_SRC_ALPHA,
               GLenum _p2 = GL_ONE_MINUS_SRC_ALPHA) {
      material_.enableBlending(_p1, _p2);
  }
  /// disable blending
  void disable_blending() { material_.disableBlending(); }

  bool backface_culling() { return material_.backfaceCulling(); };

  /// enable backface culling (not active by default, see applyProperties)
  void enable_backface_culling() { material_.enableBackfaceCulling(); }

  /// disable backface culling (not active by default, see applyProperties)
  void disable_backface_culling() { material_.disableBackfaceCulling(); }

  /// get properties that will be applied (OR'ed ApplyProperties)
  unsigned int applyProperties() const { return applyProperties_; }

  /// set properties that will be applied (OR'ed ApplyProperties)
  void applyProperties(unsigned int _applyProperties) {
    applyProperties_ = _applyProperties;
  }

  /// Get material object reference
  ACG::SceneGraph::Material& material() { return material_; }

  /// Set material object
  void set_material(const ACG::SceneGraph::Material& _m) { material_ = _m; }

private:

    /// OR'ed ApplyProperties
    int applyProperties_;

    /// Local material class that actually stores the properties
    ACG::SceneGraph::Material material_;

    /// Material Backup
    ACG::SceneGraph::Material materialBackup_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_MATERIAL_NODE_HH defined
//=============================================================================

