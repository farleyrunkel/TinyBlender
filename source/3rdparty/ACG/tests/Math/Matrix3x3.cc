#include <gtest/gtest.h>

#include <ACG/Math/Matrix3x3T.hh>
#include "MatrixTestHelper.hh"

namespace {

template<class Scalar>
class Matrix3x3Test: public testing::Test {
    public:
        Matrix3x3Test() {}
        virtual ~Matrix3x3Test() {}

        typedef typename ACG::Matrix3x3T<Scalar> Matrix3x3;
};

typedef testing::Types<double, float> Implementations;

TYPED_TEST_SUITE(Matrix3x3Test, Implementations);

TYPED_TEST(Matrix3x3Test, access) {
    using Matrix3x3 = typename Matrix3x3Test<TypeParam>::Matrix3x3;

    Matrix3x3 m {{
        1, 2, 3,
        4, 5, 6,
        7, 8, 9 }};

    EXPECT_EQ(1, m[0]);
    EXPECT_EQ(1, m(0, 0));
    EXPECT_EQ(2, m[1]);
    EXPECT_EQ(2, m(0, 1));
    EXPECT_EQ(3, m[2]);
    EXPECT_EQ(3, m(0, 2));

    EXPECT_EQ(4, m[3]);
    EXPECT_EQ(4, m(1, 0));
    EXPECT_EQ(5, m[4]);
    EXPECT_EQ(5, m(1, 1));
    EXPECT_EQ(6, m[5]);
    EXPECT_EQ(6, m(1, 2));

    EXPECT_EQ(7, m[6]);
    EXPECT_EQ(7, m(2, 0));
    EXPECT_EQ(8, m[7]);
    EXPECT_EQ(8, m(2, 1));
    EXPECT_EQ(9, m[8]);
    EXPECT_EQ(9, m(2, 2));
}

TYPED_TEST(Matrix3x3Test, construction) {

    using Matrix3x3 = typename Matrix3x3Test<TypeParam>::Matrix3x3;
    using Vec3 = typename Matrix3x3::Vec3;

    Matrix3x3 m {{
        1, 2, 3,
        4, 5, 6,
        7, 8, 9 }};

    EXPECT_EQ(m, Matrix3x3::fromColumns(
            Vec3(1, 4, 7),
            Vec3(2, 5, 8),
            Vec3(3, 6, 9)));

    EXPECT_EQ(m, Matrix3x3::fromRows(
            Vec3(1, 2, 3),
            Vec3(4, 5, 6),
            Vec3(7, 8, 9)));
}

TYPED_TEST(Matrix3x3Test, transpose) {

    using Matrix3x3 = typename Matrix3x3Test<TypeParam>::Matrix3x3;

    Matrix3x3 m {{
        1, 2, 3,
        4, 5, 6,
        7, 8, 9 }};

    EXPECT_EQ(Matrix3x3({
        1, 4, 7,
        2, 5, 8,
        3, 6, 9}),
        m.transposed());

    m.transpose();

    EXPECT_EQ(Matrix3x3({
        1, 4, 7,
        2, 5, 8,
        3, 6, 9}),
        m);
}

TYPED_TEST(Matrix3x3Test, det) {

    using Matrix3x3 = typename Matrix3x3Test<TypeParam>::Matrix3x3;

    ASSERT_NEAR(1.0, Matrix3x3::identity().det(), 1e-8);
    ASSERT_NEAR(0.0, Matrix3x3::zero().det(), 1e-8);

    {
        Matrix3x3 m {{ 1, 2, 3, 4, 5, 6, 7, 8, 9 }};
        ASSERT_NEAR(0.0, m.det(), 1e-8);
    }
    {
        Matrix3x3 m {{
            0.540963817892, 0.63815313458, 0.232901321176,
            0.427852547119, 0.290360892496, 0.790216925959,
            0.521293721788, 0.126183253064, 0.878791594258 }};
        ASSERT_NEAR(0.0843528547941, m.det(), 1e-8);
    }
    {
        Matrix3x3 m {{
            0.55770108832, 0.698229653416, 0.213133500513,
            0.554922251751, 0.110301921695, 0.961776097631,
            0.324829998436, 0.473104184394, 0.475331624799 }};
        ASSERT_NEAR(-0.142243235281, m.det(), 1e-8);
    }
}

TYPED_TEST(Matrix3x3Test, invert_inverted) {

    using Matrix3x3 = typename Matrix3x3Test<TypeParam>::Matrix3x3;

    EXPECT_EQ(Matrix3x3::identity(), Matrix3x3::identity().inverse());
    {
        Matrix3x3 m {{
            0.828277802518, 0.265835425799, 0.764172058766,
            0.460819591568, 0.0582725933838, 0.682623689065,
            0.00010136137219, 0.549667442228, 0.71426378955 }};
        EXPECT_TRUE(areClose({{
            1.95965938707, -1.35207106222, -0.804410387772,
            1.93312797956, -3.47488258913, 1.25275115042,
            -1.48793227563, 2.67431570876, 0.436092407449 }}, m.inverse()));
        Matrix3x3 minv = m; minv.invert();
        EXPECT_EQ(m.inverse(), minv);
        EXPECT_TRUE(areClose(minv.inverse(), m));
    }
    {
        Matrix3x3 m {{
            0.177907401463, 0.801552028587, 0.537416435716,
            0.998303071804, 0.3525500305, 0.779702329831,
            0.00697248858758, 0.927937880557, 0.917278319035 }};
        EXPECT_TRUE(areClose({{
            1.3148934724, 0.777368523682, -1.43114841493,
            2.9913570641, -0.523959200394, -1.30720656662,
            -3.03611407354, 0.524139060932, 2.42345764719 }}, m.inverse()));
        Matrix3x3 minv = m; minv.invert();
        EXPECT_EQ(m.inverse(), minv);
        EXPECT_TRUE(areClose(minv.inverse(), m));
    }
}

TYPED_TEST(Matrix3x3Test, matrix_multiplication) {
    using Matrix3x3 = typename Matrix3x3Test<TypeParam>::Matrix3x3;

    EXPECT_EQ(Matrix3x3::identity(), Matrix3x3::identity() * Matrix3x3::identity());
    EXPECT_EQ(Matrix3x3::zero(), Matrix3x3::identity() * Matrix3x3::zero());
    {
        Matrix3x3 a {{
            0.0198677492323, 0.335847288905, 0.903519116051,
            0.00663887675374, 0.24203377376, 0.860915878917,
            0.159079677011, 0.51176099262, 0.754881431552 }};
        Matrix3x3 b {{
            0.685605402853, 0.550061681341, 0.963595467664,
            0.31801733629, 0.219196960585, 0.881308916676,
            0.593421354113, 0.562284336992, 0.0822415517585 }};
        EXPECT_EQ(a, a * Matrix3x3::identity());
        EXPECT_EQ(Matrix3x3::zero(), a * Matrix3x3::zero());
        EXPECT_EQ(Matrix3x3::zero(), Matrix3x3::zero() * a);
        EXPECT_TRUE(areClose({{
            0.656594233747, 0.592579839624, 0.389436497614,
            0.592408452439, 0.540784373459, 0.290506772318,
            0.719777515039, 0.62413809398, 0.666390602093 }}, a * b));
        auto ab = a; ab *= b; EXPECT_EQ(ab, a*b);
    }
    {
        Matrix3x3 a {{
            -0.69449806209, 0.782467649048, -0.665216245975,
            -0.285290453681, 0.395547604054, 0.635321636096,
            -0.463882724822, 0.197102669096, -0.972470374827 }};
        Matrix3x3 b {{
            0.510414656025, 0.738189497916, 0.9901980894,
            -0.06427439106, 0.706676712526, 0.550975704042,
            -0.468791241495, -0.70849622459, 0.313163366829 }};
        EXPECT_TRUE(areClose({{
            -0.0929270713252, 0.511583688938, -0.464891349608,
            -0.468873228703, -0.381197116857, 0.134402520046,
            0.206444398874, 0.485846099589, -0.655279102673 }}, a * b));
        auto ab = a; ab *= b; EXPECT_EQ(ab, a*b);
    }
    {
        Matrix3x3 a {{
            -4.33305043354, -2.68416159097, 6.12099479706,
            -2.56865371689, 2.45552630644, -3.71903598774,
            -1.46747079278, 4.91597319165, 1.04193027614 }};
        Matrix3x3 b {{
            9.52441107801, -0.222280392529, 8.43656371755,
            9.3234433143, -1.272702334, -2.20088570156,
            -2.76499303176, -4.57362467379, 3.2829486774 }};
        EXPECT_TRUE(areClose({{
            -83.2198899519, -23.6158419591, -10.5536114343,
            8.71215499908, 14.4552820513, -39.2843477657,
            28.9760723585, -10.6957785904, -19.7793023317 }}, a * b));
        auto ab = a; ab *= b; EXPECT_EQ(ab, a*b);
    }
}

TYPED_TEST(Matrix3x3Test, vector_multiplication) {
    using Matrix3x3 = typename Matrix3x3Test<TypeParam>::Matrix3x3;
    using Vec3 = typename Matrix3x3::Vec3;

    {
        const Matrix3x3 m {{
            0.637256867597, 0.496729074271, 0.875025689854,
            -0.0790360016434, -0.372010734739, -0.900793564733,
            -0.150797220981, -0.679495918006, -0.995588120053 }};
        const auto v = Vec3(0.0128269540114,-0.406921319051,0.42309802041);
        EXPECT_TRUE(areClose(Vec3(0.176266051606,-0.230758666314,-0.146664256512), m * v));
        EXPECT_EQ(m * v, v * m);

        EXPECT_EQ(v, Matrix3x3::identity() * v);
        EXPECT_EQ(v, v * Matrix3x3::identity());
        EXPECT_EQ(Vec3(0), Matrix3x3::zero() * v);
        EXPECT_EQ(Vec3(0), v * Matrix3x3::zero());
    }
}

} /* anonymous namespace */
