
#ifndef ACG_TESTS_MATH_MATRIXTESTHELPER_HH_
#define ACG_TESTS_MATH_MATRIXTESTHELPER_HH_

#include <ACG/Math/Matrix3x3T.hh>
#include <gtest/gtest.h>
#include <cmath>

template<typename Scalar>
::testing::AssertionResult areClose(OpenMesh::VectorT<Scalar, 3> a,
        OpenMesh::VectorT<Scalar, 3> b, double threshold = 1e-8) {

  if ((a-b).sqrnorm() > threshold) {
      return ::testing::AssertionFailure()
      << "ACG::Vec3d(" << a << ") and ACG::Vec3d(" << b << ") have distance "
      << (a-b).norm() << ". Threshold: " << std::sqrt(threshold);
  } else {
    return ::testing::AssertionSuccess();
  }
}

template<typename Scalar>
::testing::AssertionResult areClose(ACG::Matrix3x3T<Scalar> a, ACG::Matrix3x3T<Scalar> b, double threshold = 1e-8) {
  if ((a-b).frobeniusSquared() > threshold) {
      return ::testing::AssertionFailure()
      << "ACG::Matrix3x3T(" << a << ") and ACG::Matrix3x3T(" << b << ") have frobenius distance "
      << (a-b).frobenius() << ". Threshold: " << std::sqrt(threshold);
  } else {
    return ::testing::AssertionSuccess();
  }
}

#endif /* ACG_TESTS_MATH_MATRIXTESTHELPER_HH_ */
