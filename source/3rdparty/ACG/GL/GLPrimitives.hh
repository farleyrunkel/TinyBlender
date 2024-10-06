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
//  CLASS GLPrimitive
//
//=============================================================================

#ifndef ACG_GLPRIMITIVES_HH
#define ACG_GLPRIMITIVES_HH


//== INCLUDES =================================================================


#include <ACG/Config/ACGDefines.hh>
#include <ACG/GL/GLState.hh>
#include <ACG/Math/VectorT.hh>
#include <ACG/GL/VertexDeclaration.hh>


//== NAMESPACES ===============================================================

namespace ACG {

//== CLASS DEFINITION =========================================================

  
class ACGDLLEXPORT GLPrimitive {
public:
  GLPrimitive();
  virtual ~GLPrimitive();

  // bind vbo + gl draw call (fixed function mode)
  // use this function in compatibility profile
  void draw_primitive();

  // activate vertex declaration + gl draw call  (shader mode)
  // _program may be nullptr, in that case the attribute locations are as follows.
  //  0 : float3 position
  //  1 : float3 normal
  //  2 : float2 texcoord
  void draw_primitive(GLSL::Program* _program);

  // add to deferred draw call to renderer
  void addToRenderer_primitive(class IRenderer* _renderer, struct RenderObject* _ro);

  // Triangle or line count must be known before updateVBO.
  // A GLPrimitive can consist of either only lines or only triangles.
  // If getNumTriangles returns nonzero, its assumed to consist of tris only.
  // Otherwise, getNumLines has to return nonzero and GLPrimitive is treated as a line-list.
  virtual int getNumTriangles() = 0;
  virtual int getNumLines() {return 0;}

  // get opengl vbo id
  unsigned int getVBO();

  const VertexDeclaration* getVertexDecl() const;

  enum NormalOrientation {OUTSIDE, INSIDE};

protected:

  // calls addTriangleToVBO to fill vertex buffer
  virtual void updateVBO() = 0;

  void addTriangleToVBO(const ACG::Vec3f* _p, const ACG::Vec3f* _n, const ACG::Vec2f* _tex);
  void addLineToVBO(const ACG::Vec3f* _p, const ACG::Vec3f* _n, const ACG::Vec2f* _tex);

  void bindVBO();

  bool checkVBO();

  void unBindVBO();

  bool vboDataInvalid_;

  NormalOrientation normalOrientation_;

private:

  void updateVBOData();

  int numTris_;
  int numLines_;
  float* vboData_;
  int curTriPtr_;

  VertexDeclaration vertexDecl_;

  unsigned int vbo_;
};

//------------------------------------------------------------------------

class ACGDLLEXPORT GLSphere: public GLPrimitive {
public:

  GLSphere(int _slices, int _stacks);
  ~GLSphere();

  void draw(GLState& _state, float _radius, const ACG::Vec3f& _center = ACG::Vec3f(0.0f, 0.0f, 0.0f));

  void addToRenderer(class IRenderer* _renderer, const struct RenderObject* _base, float _radius, const ACG::Vec3f& _center = ACG::Vec3f(0.0f, 0.0f, 0.0f));

  int getNumTriangles() override;

private:

  void updateVBO() override;

  void addTriangle(int sl0, int st0, int sl1, int st1, int sl2, int st2);

  ACG::Vec3f positionOnSphere(int _sliceNumber, int _stackNumber);
  ACG::Vec2f texCoordOnSphere(int _sliceNumber, int _stackNumber);

private:

  int slices_;
  int stacks_;

};

//------------------------------------------------------------------------

class ACGDLLEXPORT GLCone: public GLPrimitive {
public:

  GLCone(int _slices, int _stacks, float _bottomRadius, float _topRadius, bool _bottomCap_, bool _topCap);
  ~GLCone();

  void draw(
      GLState& _state,
      float _height,
      const ACG::Vec3f& _center = ACG::Vec3f(0.0f, 0.0f, 0.0f),
      ACG::Vec3f _upDir = ACG::Vec3f(0.0f, 0.0f, 1.0f));


  void addToRenderer(class IRenderer* _renderer, const struct RenderObject* _base, 
    float _height,
    const ACG::Vec3f& _center = ACG::Vec3f(0.0f, 0.0f, 0.0f),
    ACG::Vec3f _upDir = ACG::Vec3f(0.0f, 0.0f, 1.0f),
    float _radiusScale = 1.0f);

  int getNumTriangles() override;

  void updateVBO() override;
  void setBottomRadius(float _bottomRadius);
  void setTopRadius(float _topRadius);
  void setNormalOrientation(NormalOrientation orienation);

private:

  void addTriangle(int sl0, int st0, int sl1, int st1, int sl2, int st2);

  ACG::Vec3f positionOnCone(int _sliceNumber, int _stackNumber);
  ACG::Vec2f texCoordOnCone(int _sliceNumber, int _stackNumber);
  ACG::Vec3f normalOnCone(int _sliceNumber, int _stackNumber);

private:

  int slices_;
  int stacks_;

  float bottomRadius_;
  float topRadius_;

  bool bottomCap_;
  bool topCap_;

};

//------------------------------------------------------------------------

class ACGDLLEXPORT GLCylinder: public GLCone {
public:
  GLCylinder(int _slices, int _stacks, float _radius, bool _bottomCap, bool _topCap);
};

//------------------------------------------------------------------------

class ACGDLLEXPORT GLPartialDisk: public GLPrimitive {
public:
  GLPartialDisk(int _slices, int _loops, float _innerRadius, float _outerRadius, float _startAngle, float _sweepAngle);

  void setInnerRadius(float _innerRadius);
  void setOuterRadius(float _outerRadius);
  int getNumTriangles() override;

  void draw(
      GLState& _state,
      const ACG::Vec3f& _center = ACG::Vec3f(0.0f, 0.0f, 0.0f),
      ACG::Vec3f _upDir = ACG::Vec3f(0.0f, 0.0f, 1.0f));

protected:
  void updateVBO() override;

private:
  int slices_;
  int loops_;
  float innerRadius_;
  float outerRadius_;
  float startAngle_;
  float sweepAngle_;
};

//------------------------------------------------------------------------

class ACGDLLEXPORT GLDisk: public GLPartialDisk {
public:
  GLDisk(int _slices, int _loops, float _innerRadius, float _outerRadius);
};

//------------------------------------------------------------------------

// axis-aligned unit cube centered at origin
class ACGDLLEXPORT GLBox: public GLPrimitive {
public:

  GLBox();
  ~GLBox();

  int getNumTriangles() override;

private:

  void updateVBO() override;
};

//------------------------------------------------------------------------

// axis-aligned unit cube centered at origin,  only lines
class ACGDLLEXPORT GLLineBox: public GLPrimitive {
public:

  GLLineBox();
  ~GLLineBox();

  int getNumTriangles() override;
  int getNumLines() override;

private:

  void updateVBO() override;
};

//------------------------------------------------------------------------

// axis-aligned Dodecahedron centered at origin
class ACGDLLEXPORT GLDodecahedron: public GLPrimitive {
public:

  GLDodecahedron();
  ~GLDodecahedron();

  int getNumTriangles() override;

private:

  void updateVBO() override;
};

//------------------------------------------------------------------------

// axis-aligned Icosahedron centered at origin
class ACGDLLEXPORT GLIcosahedron: public GLPrimitive {
public:

  GLIcosahedron();
  ~GLIcosahedron();

  int getNumTriangles() override;

private:

  void updateVBO() override;
};

//------------------------------------------------------------------------

// axis-aligned Icosahedron centered at origin
class ACGDLLEXPORT GLOctahedron: public GLPrimitive {
public:

  GLOctahedron();
  ~GLOctahedron();

  int getNumTriangles() override;

private:

  void updateVBO() override;
};

//------------------------------------------------------------------------

// axis-aligned Icosahedron centered at origin
class ACGDLLEXPORT GLTetrahedron: public GLPrimitive {
public:

  GLTetrahedron();
  ~GLTetrahedron();

  int getNumTriangles() override;

private:

  void updateVBO() override;
};

//------------------------------------------------------------------------

// axis-aligned Torus centered at origin
class ACGDLLEXPORT GLTorus: public GLPrimitive {
public:

  GLTorus(GLdouble innerRadius,
          GLdouble outerRadius,
          GLint nsides, GLint rings);
  ~GLTorus();

  int getNumTriangles() override;

private:

  void updateVBO() override;

  int rings_;
  int nsides_;
  float innerRadius_;
  float outerRadius_;

};

//------------------------------------------------------------------------

} // namespace ACG

#endif // ACG_GLPRIMITIVES_HH defined
