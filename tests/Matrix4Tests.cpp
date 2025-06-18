#include <numbers>
#include <stdexcept>

#include <gtest/gtest.h>

#include "Nudge/MathF.hpp"
#include "Nudge/Matrix3.hpp"
#include "Nudge/Matrix4.hpp"
#include "Nudge/Vector3.hpp"
#include "Nudge/Vector4.hpp"

using std::runtime_error;
using std::numbers::pi_v;

using testing::Test;

namespace Nudge
{
    class Matrix4Tests : public Test
    {
    public:
        // Helper method for floating point comparison
        static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
        {
            EXPECT_TRUE(MathF::Compare(expected, actual, tolerance));
        }

        static void AssertVector3Equal(const Vector3& expected, const Vector3& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.x, actual.x, tolerance);
            AssertFloatEqual(expected.y, actual.y, tolerance);
            AssertFloatEqual(expected.z, actual.z, tolerance);
        }

        static void AssertVector4Equal(const Vector4& expected, const Vector4& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.x, actual.x, tolerance);
            AssertFloatEqual(expected.y, actual.y, tolerance);
            AssertFloatEqual(expected.z, actual.z, tolerance);
            AssertFloatEqual(expected.w, actual.w, tolerance);
        }

        static void AssertMatrix4Equal(const Matrix4& expected, const Matrix4& actual, float tolerance = 0.00001f)
        {
            AssertFloatEqual(expected.m11, actual.m11, tolerance);
            AssertFloatEqual(expected.m21, actual.m21, tolerance);
            AssertFloatEqual(expected.m31, actual.m31, tolerance);
            AssertFloatEqual(expected.m41, actual.m41, tolerance);
            AssertFloatEqual(expected.m12, actual.m12, tolerance);
            AssertFloatEqual(expected.m22, actual.m22, tolerance);
            AssertFloatEqual(expected.m32, actual.m32, tolerance);
            AssertFloatEqual(expected.m42, actual.m42, tolerance);
            AssertFloatEqual(expected.m13, actual.m13, tolerance);
            AssertFloatEqual(expected.m23, actual.m23, tolerance);
            AssertFloatEqual(expected.m33, actual.m33, tolerance);
            AssertFloatEqual(expected.m43, actual.m43, tolerance);
            AssertFloatEqual(expected.m14, actual.m14, tolerance);
            AssertFloatEqual(expected.m24, actual.m24, tolerance);
            AssertFloatEqual(expected.m34, actual.m34, tolerance);
            AssertFloatEqual(expected.m44, actual.m44, tolerance);
        }

    };

    // Static Factory Method Tests
    TEST_F(Matrix4Tests, Identity_CreatesIdentityMatrix)
    {
        Matrix4 identity = Matrix4::Identity();
        AssertMatrix4Equal(Matrix4(1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f), identity);
    }

    TEST_F(Matrix4Tests, Zero_CreatesZeroMatrix)
    {
        Matrix4 zero = Matrix4::Zero();
        AssertMatrix4Equal(Matrix4(0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f), zero);
    }

    TEST_F(Matrix4Tests, Scale_ThreeParameters_CreatesScaleMatrix)
    {
        Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
        AssertMatrix4Equal(Matrix4(2.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 3.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 4.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f), scale);
    }

    TEST_F(Matrix4Tests, Scale_Vector3Parameter_CreatesScaleMatrix)
    {
        Vector3 scaleVec(2.5f, 1.5f, 3.0f);
        Matrix4 scale = Matrix4::Scale(scaleVec);
        AssertMatrix4Equal(Matrix4(2.5f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.5f, 0.0f, 0.0f,
            0.0f, 0.0f, 3.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f), scale);
    }

    TEST_F(Matrix4Tests, RotationX_ZeroDegrees_CreatesIdentityMatrix)
    {
        Matrix4 rotation = Matrix4::RotationX(0.0f);
        AssertMatrix4Equal(Matrix4::Identity(), rotation);
    }

    TEST_F(Matrix4Tests, RotationX_90Degrees_CreatesCorrectMatrix)
    {
        Matrix4 rotation = Matrix4::RotationX(90.0f);
        // 90 degrees around X: cos(90) = 0, sin(90) = 1
        // | 1   0   0  0 |
        // | 0   0  -1  0 |
        // | 0   1   0  0 |
        // | 0   0   0  1 |
        AssertMatrix4Equal(Matrix4(1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, -1.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f), rotation, 0.001f);
    }

    TEST_F(Matrix4Tests, RotationY_90Degrees_CreatesCorrectMatrix)
    {
        Matrix4 rotation = Matrix4::RotationY(90.0f);
        // 90 degrees around Y: cos(90) = 0, sin(90) = 1
        // |  0  0  1  0 |
        // |  0  1  0  0 |
        // | -1  0  0  0 |
        // |  0  0  0  1 |
        AssertMatrix4Equal(Matrix4(0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            -1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f), rotation, 0.001f);
    }

    TEST_F(Matrix4Tests, RotationZ_90Degrees_CreatesCorrectMatrix)
    {
        Matrix4 rotation = Matrix4::RotationZ(90.0f);
        // 90 degrees around Z: cos(90) = 0, sin(90) = 1
        // | 0 -1  0  0 |
        // | 1  0  0  0 |
        // | 0  0  1  0 |
        // | 0  0  0  1 |
        AssertMatrix4Equal(Matrix4(0.0f, -1.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f), rotation, 0.001f);
    }

    TEST_F(Matrix4Tests, Rotation_EulerAngles_CreatesCorrectMatrix)
    {
        Vector3 euler(90.0f, 0.0f, 0.0f); // 90 degrees around X
        Matrix4 rotation = Matrix4::Rotation(euler);
        Matrix4 expected = Matrix4::RotationX(90.0f);
        AssertMatrix4Equal(expected, rotation, 0.001f);
    }

    TEST_F(Matrix4Tests, Rotation_AxisAngle_CreatesCorrectMatrix)
    {
        Vector3 axis(0.0f, 0.0f, 1.0f); // Z-axis
        Matrix4 rotation = Matrix4::Rotation(axis, 90.0f);
        Matrix4 expected = Matrix4::RotationZ(90.0f);
        AssertMatrix4Equal(expected, rotation, 0.001f);
    }

    TEST_F(Matrix4Tests, Translation_ThreeParameters_CreatesTranslationMatrix)
    {
        Matrix4 translation = Matrix4::Translation(2.0f, 3.0f, 4.0f);
        AssertMatrix4Equal(Matrix4(1.0f, 0.0f, 0.0f, 2.0f,
            0.0f, 1.0f, 0.0f, 3.0f,
            0.0f, 0.0f, 1.0f, 4.0f,
            0.0f, 0.0f, 0.0f, 1.0f), translation);
    }

    TEST_F(Matrix4Tests, Translation_Vector3Parameter_CreatesTranslationMatrix)
    {
        Vector3 translationVec(5.0f, 6.0f, 7.0f);
        Matrix4 translation = Matrix4::Translation(translationVec);
        AssertMatrix4Equal(Matrix4(1.0f, 0.0f, 0.0f, 5.0f,
            0.0f, 1.0f, 0.0f, 6.0f,
            0.0f, 0.0f, 1.0f, 7.0f,
            0.0f, 0.0f, 0.0f, 1.0f), translation);
    }

    TEST_F(Matrix4Tests, TRS_CombinesTransformationsCorrectly)
    {
        Vector3 translation(1.0f, 2.0f, 3.0f);
        Vector3 rotation(0.0f, 0.0f, 90.0f); // 90 degrees around Z
        Vector3 scale(2.0f, 2.0f, 2.0f);

        Matrix4 trs = Matrix4::TRS(translation, rotation, scale);

        // Should be Translation * Rotation * Scale
        Matrix4 expected = Matrix4::Translation(translation) *
            Matrix4::Rotation(rotation) *
            Matrix4::Scale(scale);
        AssertMatrix4Equal(expected, trs, 0.001f);
    }

    // Constructor Tests
    TEST_F(Matrix4Tests, Constructor_Default_CreatesIdentityMatrix)
    {
        Matrix4 matrix;
        AssertMatrix4Equal(Matrix4::Identity(), matrix);
    }

    TEST_F(Matrix4Tests, Constructor_Scalar_CreatesScalarMatrix)
    {
        Matrix4 matrix(5.0f);
        AssertMatrix4Equal(Matrix4(5.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 5.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 5.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 5.0f), matrix);
    }

    TEST_F(Matrix4Tests, Constructor_SixteenFloats_CreatesMatrixWithSpecifiedValues)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        AssertMatrix4Equal(Matrix4(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f), matrix);
    }

    TEST_F(Matrix4Tests, Constructor_FourVectors_CreatesMatrixFromColumns)
    {
        Vector4 col1(1.0f, 2.0f, 3.0f, 4.0f);
        Vector4 col2(5.0f, 6.0f, 7.0f, 8.0f);
        Vector4 col3(9.0f, 10.0f, 11.0f, 12.0f);
        Vector4 col4(13.0f, 14.0f, 15.0f, 16.0f);
        Matrix4 matrix(col1, col2, col3, col4);
        AssertMatrix4Equal(Matrix4(1.0f, 5.0f, 9.0f, 13.0f,
            2.0f, 6.0f, 10.0f, 14.0f,
            3.0f, 7.0f, 11.0f, 15.0f,
            4.0f, 8.0f, 12.0f, 16.0f), matrix);
    }

    TEST_F(Matrix4Tests, Constructor_Array_CreatesMatrixFromColumnMajorArray)
    {
        float values[] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f,
                          9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f };
        Matrix4 matrix(values);
        // Column-major: [m11, m21, m31, m41, m12, m22, m32, m42, m13, m23, m33, m43, m14, m24, m34, m44]
        AssertMatrix4Equal(Matrix4(1.0f, 5.0f, 9.0f, 13.0f,
            2.0f, 6.0f, 10.0f, 14.0f,
            3.0f, 7.0f, 11.0f, 15.0f,
            4.0f, 8.0f, 12.0f, 16.0f), matrix);
    }

    TEST_F(Matrix4Tests, Constructor_Matrix3_ExtendsToMatrix4WithIdentity)
    {
        Matrix3 matrix3(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        Matrix4 matrix4(matrix3);
        AssertMatrix4Equal(Matrix4(1.0f, 2.0f, 3.0f, 0.0f,
            4.0f, 5.0f, 6.0f, 0.0f,
            7.0f, 8.0f, 9.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f), matrix4);
    }

    TEST_F(Matrix4Tests, Constructor_Copy_CreatesIdenticalMatrix)
    {
        Matrix4 original(1.5f, 2.5f, 3.5f, 4.5f,
            5.5f, 6.5f, 7.5f, 8.5f,
            9.5f, 10.5f, 11.5f, 12.5f,
            13.5f, 14.5f, 15.5f, 16.5f);
        Matrix4 copy(original);
        AssertMatrix4Equal(original, copy);
    }

    // Mathematical Operation Tests
    TEST_F(Matrix4Tests, Determinant_IdentityMatrix_ReturnsOne)
    {
        Matrix4 identity = Matrix4::Identity();
        AssertFloatEqual(1.0f, identity.Determinant());
    }

    TEST_F(Matrix4Tests, Determinant_ZeroMatrix_ReturnsZero)
    {
        Matrix4 zero = Matrix4::Zero();
        AssertFloatEqual(0.0f, zero.Determinant());
    }

    TEST_F(Matrix4Tests, Determinant_ScaleMatrix_ReturnsProduct)
    {
        Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
        AssertFloatEqual(24.0f, scale.Determinant()); // 2 * 3 * 4 * 1 = 24
    }

    TEST_F(Matrix4Tests, Determinant_SingularMatrix_ReturnsZero)
    {
        Matrix4 singular(1.0f, 2.0f, 3.0f, 4.0f,
            2.0f, 4.0f, 6.0f, 8.0f,
            1.0f, 1.0f, 1.0f, 1.0f,
            0.0f, 0.0f, 0.0f, 1.0f);
        AssertFloatEqual(0.0f, singular.Determinant(), 0.001f);
    }

    TEST_F(Matrix4Tests, Transposed_IdentityMatrix_ReturnsIdentity)
    {
        Matrix4 identity = Matrix4::Identity();
        Matrix4 transposed = identity.Transposed();
        AssertMatrix4Equal(identity, transposed);
    }

    TEST_F(Matrix4Tests, Transposed_GeneralMatrix_SwapsOffDiagonalElements)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Matrix4 transposed = matrix.Transposed();
        AssertMatrix4Equal(Matrix4(1.0f, 5.0f, 9.0f, 13.0f,
            2.0f, 6.0f, 10.0f, 14.0f,
            3.0f, 7.0f, 11.0f, 15.0f,
            4.0f, 8.0f, 12.0f, 16.0f), transposed);
    }

    TEST_F(Matrix4Tests, Transpose_ModifiesOriginalMatrix)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Matrix4 expected(1.0f, 5.0f, 9.0f, 13.0f,
            2.0f, 6.0f, 10.0f, 14.0f,
            3.0f, 7.0f, 11.0f, 15.0f,
            4.0f, 8.0f, 12.0f, 16.0f);
        matrix.Transpose();
        AssertMatrix4Equal(expected, matrix);
    }

    TEST_F(Matrix4Tests, Inverse_IdentityMatrix_ReturnsIdentity)
    {
        Matrix4 identity = Matrix4::Identity();
        Matrix4 inverse = identity.Inverse();
        AssertMatrix4Equal(identity, inverse);
    }

    TEST_F(Matrix4Tests, Inverse_ScaleMatrix_ReturnsInverseScale)
    {
        Matrix4 scale = Matrix4::Scale(2.0f, 4.0f, 8.0f);
        Matrix4 inverse = scale.Inverse();
        AssertMatrix4Equal(Matrix4::Scale(0.5f, 0.25f, 0.125f), inverse, 0.001f);
    }

    TEST_F(Matrix4Tests, Inverse_TranslationMatrix_ReturnsInverseTranslation)
    {
        Matrix4 translation = Matrix4::Translation(2.0f, 3.0f, 4.0f);
        Matrix4 inverse = translation.Inverse();
        AssertMatrix4Equal(Matrix4::Translation(-2.0f, -3.0f, -4.0f), inverse, 0.001f);
    }

    TEST_F(Matrix4Tests, Inverse_SingularMatrix_ThrowsException)
    {
        Matrix4 singular(1.0f, 2.0f, 3.0f, 4.0f,
            2.0f, 4.0f, 6.0f, 8.0f,
            1.0f, 1.0f, 1.0f, 1.0f,
            0.0f, 0.0f, 0.0f, 0.0f);

        EXPECT_THROW({
            Matrix4 inverse = singular.Inverse();
            }, runtime_error);
    }

    TEST_F(Matrix4Tests, Inverse_MultiplyByOriginal_GivesIdentity)
    {
        Matrix4 translation = Matrix4::Translation(1.0f, 2.0f, 3.0f);
        Matrix4 rotation = Matrix4::RotationZ(45.0f);
        Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
        Matrix4 matrix = (translation * rotation) * scale;
        Matrix4 inverse = matrix.Inverse();
        Matrix4 product = matrix * inverse;

        AssertMatrix4Equal(Matrix4::Identity(), product, 0.001f);
    }

    // Utility Method Tests
    TEST_F(Matrix4Tests, IsIdentity_IdentityMatrix_ReturnsTrue)
    {
        Matrix4 identity = Matrix4::Identity();
        EXPECT_TRUE(identity.IsIdentity());
    }

    TEST_F(Matrix4Tests, IsIdentity_NonIdentityMatrix_ReturnsFalse)
    {
        Matrix4 matrix = Matrix4::Scale(2.0f, 1.0f, 1.0f);
        EXPECT_FALSE(matrix.IsIdentity());
    }

    TEST_F(Matrix4Tests, IsIdentity_NearIdentityMatrix_ReturnsTrue)
    {
        Matrix4 matrix(1.00001f, 0.00001f, 0.00001f, 0.00001f,
            0.00001f, 0.99999f, 0.00001f, 0.00001f,
            0.00001f, 0.00001f, 1.00001f, 0.00001f,
            0.00001f, 0.00001f, 0.00001f, 0.99999f);
        EXPECT_TRUE(matrix.IsIdentity(0.0001f));
    }

    TEST_F(Matrix4Tests, IsZero_ZeroMatrix_ReturnsTrue)
    {
        Matrix4 zero = Matrix4::Zero();
        EXPECT_TRUE(zero.IsZero());
    }

    TEST_F(Matrix4Tests, IsZero_NonZeroMatrix_ReturnsFalse)
    {
        Matrix4 matrix(0.1f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f);
        EXPECT_FALSE(matrix.IsZero());
    }

    TEST_F(Matrix4Tests, IsZero_NearZeroMatrix_ReturnsTrue)
    {
        Matrix4 matrix(0.00001f, 0.00001f, 0.00001f, 0.00001f,
            0.00001f, 0.00001f, 0.00001f, 0.00001f,
            0.00001f, 0.00001f, 0.00001f, 0.00001f,
            0.00001f, 0.00001f, 0.00001f, 0.00001f);
        EXPECT_TRUE(matrix.IsZero(0.0001f));
    }

    TEST_F(Matrix4Tests, IsOrthogonal_IdentityMatrix_ReturnsTrue)
    {
        Matrix4 identity = Matrix4::Identity();
        EXPECT_TRUE(identity.IsOrthogonal());
    }

    TEST_F(Matrix4Tests, IsOrthogonal_RotationMatrix_ReturnsTrue)
    {
        Matrix4 rotation = Matrix4::RotationZ(45.0f);
        EXPECT_TRUE(rotation.IsOrthogonal());
    }

    TEST_F(Matrix4Tests, IsOrthogonal_ScaleMatrix_ReturnsFalse)
    {
        Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
        EXPECT_FALSE(scale.IsOrthogonal());
    }

    // Transformation Component Extraction Tests
    TEST_F(Matrix4Tests, GetTranslation_TranslationMatrix_ReturnsCorrectVector)
    {
        Matrix4 translation = Matrix4::Translation(2.0f, 3.0f, 4.0f);
        Vector3 result = translation.GetTranslation();
        AssertVector3Equal(Vector3(2.0f, 3.0f, 4.0f), result);
    }

    TEST_F(Matrix4Tests, GetTranslation_IdentityMatrix_ReturnsZeroVector)
    {
        Matrix4 identity = Matrix4::Identity();
        Vector3 result = identity.GetTranslation();
        AssertVector3Equal(Vector3(0.0f, 0.0f, 0.0f), result);
    }

    TEST_F(Matrix4Tests, GetScale_ScaleMatrix_ReturnsCorrectVector)
    {
        Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
        Vector3 result = scale.GetScale();
        AssertVector3Equal(Vector3(2.0f, 3.0f, 4.0f), result, 0.001f);
    }

    TEST_F(Matrix4Tests, GetScale_IdentityMatrix_ReturnsUnitVector)
    {
        Matrix4 identity = Matrix4::Identity();
        Vector3 result = identity.GetScale();
        AssertVector3Equal(Vector3(1.0f, 1.0f, 1.0f), result, 0.001f);
    }

    TEST_F(Matrix4Tests, GetRotation_RotationMatrix_ReturnsCorrect3x3Matrix)
    {
        Matrix4 rotation = Matrix4::RotationZ(90.0f);
        Matrix3 result = rotation.GetRotation();
        Matrix3 expected = Matrix3::RotationZ(90.0f);
        // Compare each element of the 3x3 matrices
        AssertFloatEqual(expected.m11, result.m11, 0.001f);
        AssertFloatEqual(expected.m12, result.m12, 0.001f);
        AssertFloatEqual(expected.m13, result.m13, 0.001f);
        AssertFloatEqual(expected.m21, result.m21, 0.001f);
        AssertFloatEqual(expected.m22, result.m22, 0.001f);
        AssertFloatEqual(expected.m23, result.m23, 0.001f);
        AssertFloatEqual(expected.m31, result.m31, 0.001f);
        AssertFloatEqual(expected.m32, result.m32, 0.001f);
        AssertFloatEqual(expected.m33, result.m33, 0.001f);
    }

    // Accessor Method Tests
    TEST_F(Matrix4Tests, GetColumn_Index0_ReturnsFirstColumn)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Vector4 column = matrix.GetColumn(0);
        AssertVector4Equal(Vector4(1.0f, 5.0f, 9.0f, 13.0f), column);
    }

    TEST_F(Matrix4Tests, GetColumn_Index3_ReturnsFourthColumn)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Vector4 column = matrix.GetColumn(3);
        AssertVector4Equal(Vector4(4.0f, 8.0f, 12.0f, 16.0f), column);
    }

    TEST_F(Matrix4Tests, GetColumn_InvalidIndex_ThrowsException)
    {
        Matrix4 matrix = Matrix4::Identity();
        EXPECT_THROW({
            Vector4 row = matrix.GetColumn(4);
            }, runtime_error);
    }

    TEST_F(Matrix4Tests, SetColumn_Index0_SetsFirstColumn)
    {
        Matrix4 matrix = Matrix4::Identity();
        Vector4 newColumn(5.0f, 6.0f, 7.0f, 8.0f);
        matrix.SetColumn(0, newColumn);
        AssertMatrix4Equal(Matrix4(5.0f, 0.0f, 0.0f, 0.0f,
            6.0f, 1.0f, 0.0f, 0.0f,
            7.0f, 0.0f, 1.0f, 0.0f,
            8.0f, 0.0f, 0.0f, 1.0f), matrix);
    }

    TEST_F(Matrix4Tests, SetColumn_InvalidIndex_ThrowsException)
    {
        Matrix4 matrix = Matrix4::Identity();
        Vector4 newColumn(1.0f, 2.0f, 3.0f, 4.0f);

        EXPECT_THROW({
            matrix.SetColumn(4, newColumn);
            }, runtime_error);
    }

    TEST_F(Matrix4Tests, GetRow_Index0_ReturnsFirstRow)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Vector4 row = matrix.GetRow(0);
        AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 4.0f), row);
    }

    TEST_F(Matrix4Tests, GetRow_Index3_ReturnsFourthRow)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Vector4 row = matrix.GetRow(3);
        AssertVector4Equal(Vector4(13.0f, 14.0f, 15.0f, 16.0f), row);
    }

    TEST_F(Matrix4Tests, GetRow_InvalidIndex_ThrowsException)
    {
        Matrix4 matrix = Matrix4::Identity();
        EXPECT_THROW({
            Vector4 row = matrix.GetRow(4);
            }, runtime_error);
    }

    TEST_F(Matrix4Tests, SetRow_Index0_SetsFirstRow)
    {
        Matrix4 matrix = Matrix4::Identity();
        Vector4 newRow(5.0f, 6.0f, 7.0f, 8.0f);
        matrix.SetRow(0, newRow);
        AssertMatrix4Equal(Matrix4(5.0f, 6.0f, 7.0f, 8.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f), matrix);
    }

    TEST_F(Matrix4Tests, SetRow_InvalidIndex_ThrowsException)
    {
        Matrix4 matrix = Matrix4::Identity();
        Vector4 newRow(1.0f, 2.0f, 3.0f, 4.0f);
        
        EXPECT_THROW({
            matrix.SetRow(4, newRow);
            }, runtime_error);
    }

    // String Representation Tests
    TEST_F(Matrix4Tests, ToString_FormatsCorrectly)
    {
        Matrix4 matrix(1.5f, 2.5f, 3.5f, 4.5f,
            5.5f, 6.5f, 7.5f, 8.5f,
            9.5f, 10.5f, 11.5f, 12.5f,
            13.5f, 14.5f, 15.5f, 16.5f);
        string str = matrix.ToString();
        // Basic check - should contain the numbers
        EXPECT_TRUE(str.find("1.5") != string::npos);
        EXPECT_TRUE(str.find("6.5") != string::npos);
        EXPECT_TRUE(str.find("11.5") != string::npos);
        EXPECT_TRUE(str.find("16.5") != string::npos);
    }

    // Operator Tests
    TEST_F(Matrix4Tests, Equality_SameMatrices_ReturnsTrue)
    {
        Matrix4 a(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Matrix4 b(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        EXPECT_TRUE(a == b);
    }

    TEST_F(Matrix4Tests, Equality_DifferentMatrices_ReturnsFalse)
    {
        Matrix4 a = Matrix4::Identity();
        Matrix4 b = Matrix4::Scale(2.0f, 2.0f, 2.0f);
        EXPECT_FALSE(a == b);
    }

    TEST_F(Matrix4Tests, Inequality_DifferentMatrices_ReturnsTrue)
    {
        Matrix4 a = Matrix4::Identity();
        Matrix4 b = Matrix4::Scale(2.0f, 2.0f, 2.0f);
        EXPECT_TRUE(a != b);
    }

    TEST_F(Matrix4Tests, MatrixMultiplication_IdentityMatrix_ReturnsOriginal)
    {
        Matrix4 matrix = Matrix4::Scale(2.0f, 3.0f, 4.0f);
        Matrix4 identity = Matrix4::Identity();
        Matrix4 result = matrix * identity;
        AssertMatrix4Equal(matrix, result);
    }

    TEST_F(Matrix4Tests, MatrixMultiplication_ScaleAndTranslation_CombinesCorrectly)
    {
        Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
        Matrix4 translation = Matrix4::Translation(1.0f, 2.0f, 3.0f);
        Matrix4 result = translation * scale;

        // Verify by transforming a test vector
        Vector3 testVector(1.0f, 1.0f, 1.0f);
        Vector3 transformed = result * testVector;
        AssertVector3Equal(Vector3(3.0f, 5.0f, 7.0f), transformed); // (2*1+1, 3*1+2, 4*1+3)
    }

    TEST_F(Matrix4Tests, ScalarMultiplication_MultiplyByZero_ReturnsZeroMatrix)
    {
        Matrix4 matrix = Matrix4::Identity();
        Matrix4 result = matrix * 0.0f;
        AssertMatrix4Equal(Matrix4::Zero(), result);
    }

    TEST_F(Matrix4Tests, ScalarMultiplication_MultiplyByTwo_DoublesAllElements)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Matrix4 result = matrix * 2.0f;
        AssertMatrix4Equal(Matrix4(2.0f, 4.0f, 6.0f, 8.0f,
            10.0f, 12.0f, 14.0f, 16.0f,
            18.0f, 20.0f, 22.0f, 24.0f,
            26.0f, 28.0f, 30.0f, 32.0f), result);
    }

    TEST_F(Matrix4Tests, ScalarDivision_DivideByTwo_HalvesAllElements)
    {
        Matrix4 matrix(2.0f, 4.0f, 6.0f, 8.0f,
            10.0f, 12.0f, 14.0f, 16.0f,
            18.0f, 20.0f, 22.0f, 24.0f,
            26.0f, 28.0f, 30.0f, 32.0f);
        Matrix4 result = matrix / 2.0f;
        AssertMatrix4Equal(Matrix4(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f), result);
    }

    TEST_F(Matrix4Tests, ScalarDivision_DivideByZero_ThrowsException)
    {
        Matrix4 matrix = Matrix4::Identity();

        EXPECT_THROW({
            Matrix4 result = matrix / 0.0f;
            }, runtime_error);
    }

    TEST_F(Matrix4Tests, Vector4Multiplication_IdentityMatrix_ReturnsOriginalVector)
    {
        Matrix4 identity = Matrix4::Identity();
        Vector4 vector(3.0f, 4.0f, 5.0f, 1.0f);
        Vector4 result = identity * vector;
        AssertVector4Equal(vector, result);
    }

    TEST_F(Matrix4Tests, Vector4Multiplication_ScaleMatrix_ScalesVector)
    {
        Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
        Vector4 vector(2.0f, 3.0f, 4.0f, 1.0f);
        Vector4 result = scale * vector;
        AssertVector4Equal(Vector4(4.0f, 9.0f, 16.0f, 1.0f), result);
    }

    TEST_F(Matrix4Tests, Vector4Multiplication_TranslationMatrix_TranslatesVector)
    {
        Matrix4 translation = Matrix4::Translation(1.0f, 2.0f, 3.0f);
        Vector4 vector(0.0f, 0.0f, 0.0f, 1.0f);
        Vector4 result = translation * vector;
        AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 1.0f), result);
    }

    TEST_F(Matrix4Tests, Vector3Multiplication_IdentityMatrix_ReturnsOriginalVector)
    {
        Matrix4 identity = Matrix4::Identity();
        Vector3 vector(3.0f, 4.0f, 5.0f);
        Vector3 result = identity * vector;
        AssertVector3Equal(vector, result);
    }

    TEST_F(Matrix4Tests, Vector3Multiplication_ScaleMatrix_ScalesVector)
    {
        Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
        Vector3 vector(2.0f, 3.0f, 4.0f);
        Vector3 result = scale * vector;
        AssertVector3Equal(Vector3(4.0f, 9.0f, 16.0f), result);
    }

    TEST_F(Matrix4Tests, Vector3Multiplication_TranslationMatrix_TranslatesVector)
    {
        Matrix4 translation = Matrix4::Translation(1.0f, 2.0f, 3.0f);
        Vector3 vector(2.0f, 3.0f, 4.0f);
        Vector3 result = translation * vector;
        AssertVector3Equal(Vector3(3.0f, 5.0f, 7.0f), result);
    }

    TEST_F(Matrix4Tests, Vector3Multiplication_RotationMatrix_RotatesVector)
    {
        Matrix4 rotation = Matrix4::RotationZ(90.0f);
        Vector3 vector(1.0f, 0.0f, 0.0f);
        Vector3 result = rotation * vector;
        AssertVector3Equal(Vector3(0.0f, 1.0f, 0.0f), result, 0.001f);
    }

    TEST_F(Matrix4Tests, IndexOperator_Index0_ReturnsFirstColumn)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Vector4 column = matrix[0];
        AssertVector4Equal(Vector4(1.0f, 5.0f, 9.0f, 13.0f), column);
    }

    TEST_F(Matrix4Tests, IndexOperator_Index3_ReturnsFourthColumn)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Vector4 column = matrix[3];
        AssertVector4Equal(Vector4(4.0f, 8.0f, 12.0f, 16.0f), column);
    }

    TEST_F(Matrix4Tests, IndexOperator_InvalidIndex_ThrowsException)
    {
        Matrix4 matrix = Matrix4::Identity();

        EXPECT_THROW({
            Vector4 column = matrix[4];
            }, runtime_error);
    }

    TEST_F(Matrix4Tests, Assignment_CopiesAllElements)
    {
        Matrix4 original(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Matrix4 assigned;
        assigned = original;
        AssertMatrix4Equal(original, assigned);
    }

    TEST_F(Matrix4Tests, Assignment_SelfAssignment_RemainsUnchanged)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Matrix4 original = matrix;
        matrix = matrix;
        AssertMatrix4Equal(original, matrix);
    }

    TEST_F(Matrix4Tests, GlobalScalarMultiplication_ScalarFirst_Works)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Matrix4 result = 3.0f * matrix;
        AssertMatrix4Equal(Matrix4(3.0f, 6.0f, 9.0f, 12.0f,
            15.0f, 18.0f, 21.0f, 24.0f,
            27.0f, 30.0f, 33.0f, 36.0f,
            39.0f, 42.0f, 45.0f, 48.0f), result);
    }

    // Edge Case Tests
    TEST_F(Matrix4Tests, EdgeCase_VerySmallNumbers_HandledCorrectly)
    {
        Matrix4 matrix(1e-6f, 1e-6f, 1e-6f, 1e-6f,
            1e-6f, 1e-6f, 1e-6f, 1e-6f,
            1e-6f, 1e-6f, 1e-6f, 1e-6f,
            1e-6f, 1e-6f, 1e-6f, 1e-6f);
        EXPECT_FALSE(matrix.IsZero()); // Should not be considered zero
    }

    TEST_F(Matrix4Tests, EdgeCase_VeryLargeNumbers_HandledCorrectly)
    {
        Matrix4 matrix(1e6f, 1e6f, 1e6f, 1e6f,
            1e6f, 1e6f, 1e6f, 1e6f,
            1e6f, 1e6f, 1e6f, 1e6f,
            1e6f, 1e6f, 1e6f, 1e6f);
        Matrix4 scaled = matrix * 0.5f;
        AssertMatrix4Equal(Matrix4(5e5f, 5e5f, 5e5f, 5e5f,
            5e5f, 5e5f, 5e5f, 5e5f,
            5e5f, 5e5f, 5e5f, 5e5f,
            5e5f, 5e5f, 5e5f, 5e5f), scaled, 1.0f);
    }

    TEST_F(Matrix4Tests, EdgeCase_NegativeValues_HandledCorrectly)
    {
        Matrix4 matrix(-1.0f, -2.0f, -3.0f, -4.0f,
            -5.0f, -6.0f, -7.0f, -8.0f,
            -9.0f, -10.0f, -11.0f, -12.0f,
            -13.0f, -14.0f, -15.0f, -16.0f);
        Matrix4 negated = matrix * -1.0f;
        AssertMatrix4Equal(Matrix4(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f), negated);
    }

    // Mathematical Property Tests
    TEST_F(Matrix4Tests, Property_TransposeOfTranspose_ReturnsOriginal)
    {
        Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
            5.0f, 6.0f, 7.0f, 8.0f,
            9.0f, 10.0f, 11.0f, 12.0f,
            13.0f, 14.0f, 15.0f, 16.0f);
        Matrix4 doubleTransposed = matrix.Transposed().Transposed();
        AssertMatrix4Equal(matrix, doubleTransposed);
    }

    TEST_F(Matrix4Tests, Property_DeterminantOfTranspose_EqualToOriginal)
    {
        Matrix4 matrix = Matrix4::Translation(1.0f, 2.0f, 3.0f) *
            Matrix4::RotationZ(45.0f) *
            Matrix4::Scale(2.0f, 3.0f, 4.0f);
        float detOriginal = matrix.Determinant();
        float detTransposed = matrix.Transposed().Determinant();
        AssertFloatEqual(detOriginal, detTransposed, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_InverseOfInverse_ReturnsOriginal)
    {
        Matrix4 matrix = Matrix4::Translation(1.0f, 2.0f, 3.0f) *
            Matrix4::RotationZ(30.0f) *
            Matrix4::Scale(2.0f, 3.0f, 4.0f);
        Matrix4 doubleInverse = matrix.Inverse().Inverse();
        AssertMatrix4Equal(matrix, doubleInverse, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_ScaleInverse_IsReciprocalScale)
    {
        Matrix4 scale = Matrix4::Scale(4.0f, 2.0f, 8.0f);
        Matrix4 inverse = scale.Inverse();
        Matrix4 expected = Matrix4::Scale(0.25f, 0.5f, 0.125f);
        AssertMatrix4Equal(expected, inverse, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_RotationInverse_IsNegativeRotation)
    {
        Matrix4 rotation = Matrix4::RotationZ(45.0f);
        Matrix4 inverse = rotation.Inverse();
        Matrix4 expected = Matrix4::RotationZ(-45.0f);
        AssertMatrix4Equal(expected, inverse, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_TranslationInverse_IsNegativeTranslation)
    {
        Matrix4 translation = Matrix4::Translation(3.0f, 4.0f, 5.0f);
        Matrix4 inverse = translation.Inverse();
        Matrix4 expected = Matrix4::Translation(-3.0f, -4.0f, -5.0f);
        AssertMatrix4Equal(expected, inverse, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_MatrixMultiplicationAssociativity)
    {
        Matrix4 a = Matrix4::Scale(2.0f, 2.0f, 2.0f);
        Matrix4 b = Matrix4::RotationZ(45.0f);
        Matrix4 c = Matrix4::Translation(1.0f, 2.0f, 3.0f);

        Matrix4 result1 = (a * b) * c;
        Matrix4 result2 = a * (b * c);
        AssertMatrix4Equal(result1, result2, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_IdentityIsMultiplicativeIdentity)
    {
        Matrix4 matrix = Matrix4::Translation(1.0f, 2.0f, 3.0f) *
            Matrix4::RotationY(30.0f) *
            Matrix4::Scale(2.0f, 3.0f, 4.0f);
        Matrix4 identity = Matrix4::Identity();

        Matrix4 leftProduct = identity * matrix;
        Matrix4 rightProduct = matrix * identity;

        AssertMatrix4Equal(matrix, leftProduct);
        AssertMatrix4Equal(matrix, rightProduct);
    }

    TEST_F(Matrix4Tests, Property_RotationMatricesAreOrthogonal)
    {
        Matrix4 rotationX = Matrix4::RotationX(37.0f);
        Matrix4 rotationY = Matrix4::RotationY(53.0f);
        Matrix4 rotationZ = Matrix4::RotationZ(71.0f);
        Matrix4 combinedRotation = rotationZ * rotationY * rotationX;

        EXPECT_TRUE(rotationX.IsOrthogonal());
        EXPECT_TRUE(rotationY.IsOrthogonal());
        EXPECT_TRUE(rotationZ.IsOrthogonal());
        EXPECT_TRUE(combinedRotation.IsOrthogonal());
    }

    TEST_F(Matrix4Tests, Property_TRS_DecompositionRoundTrip)
    {
        Vector3 translation(2.0f, 3.0f, 4.0f);
        Vector3 rotation(30.0f, 45.0f, 60.0f);
        Vector3 scale(1.5f, 2.0f, 2.5f);

        Matrix4 trs = Matrix4::TRS(translation, rotation, scale);

        Vector3 extractedTranslation = trs.GetTranslation();
        Vector3 extractedScale = trs.GetScale();

        AssertVector3Equal(translation, extractedTranslation, 0.001f);
        AssertVector3Equal(scale, extractedScale, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_TransformationOrder_TRS_IsCorrect)
    {
        Vector3 translation(1.0f, 0.0f, 0.0f);
        Vector3 rotation(0.0f, 0.0f, 90.0f);
        Vector3 scale(2.0f, 1.0f, 1.0f);

        Matrix4 trs = Matrix4::TRS(translation, rotation, scale);

        // Test with a unit vector
        Vector3 testVector(1.0f, 0.0f, 0.0f);
        Vector3 result = trs * testVector;

        // Expected: Scale first (2,0,0), then rotate 90 degrees around Z (0,2,0), then translate (1,2,0)
        AssertVector3Equal(Vector3(1.0f, 2.0f, 0.0f), result, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_OrthographicProjection_UnitCube_IsCorrect)
    {
        // Create orthographic projection for a 2x2x2 cube centered at origin
        Matrix4 ortho = Matrix4::Orthographic(-1.0f, 1.0f, -1.0f, 1.0f, 0.1f, 10.0f);

        // Test corner of the viewing volume
        Vector4 testPoint(1.0f, 1.0f, -0.1f, 1.0f); // At near plane, top-right
        Vector4 result = ortho * testPoint;

        // Expected: Should map to (1, 1, -1) in NDC space
        AssertVector4Equal(Vector4(1.0f, 1.0f, -1.0f, 1.0f), result, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_OrthographicProjection_AsymmetricBounds_IsCorrect)
    {
        // Create orthographic projection with asymmetric bounds
        Matrix4 ortho = Matrix4::Orthographic(0.0f, 4.0f, -2.0f, 2.0f, 1.0f, 5.0f);

        // Test center of the viewing volume
        Vector4 testPoint(2.0f, 0.0f, -3.0f, 1.0f); // Center of volume
        Vector4 result = ortho * testPoint;

        // Expected: Should map to (0, 0, 0) in NDC space
        AssertVector4Equal(Vector4(0.0f, 0.0f, 0.0f, 1.0f), result, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_LookAtMatrix_LookingDownNegativeZ_IsCorrect)
    {
        Vector3 eye(0.0f, 0.0f, 5.0f);
        Vector3 target(0.0f, 0.0f, 0.0f);
        Vector3 up(0.0f, 1.0f, 0.0f);

        Matrix4 lookAt = Matrix4::LookAt(eye, target, up);

        // Test a point that should transform to camera space
        Vector3 worldPoint(1.0f, 0.0f, 0.0f); // Point to the right of target
        Vector3 result = lookAt * worldPoint;

        // Expected: Should be at (1, 0, -5) in camera space (right, same height, in front)
        AssertVector3Equal(Vector3(1.0f, 0.0f, -5.0f), result, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_LookAtMatrix_LookingAtAngle_IsCorrect)
    {
        Vector3 eye(3.0f, 4.0f, 5.0f);
        Vector3 target(0.0f, 0.0f, 0.0f);
        Vector3 up(0.0f, 1.0f, 0.0f);

        Matrix4 lookAt = Matrix4::LookAt(eye, target, up);

        // Test that the target point transforms to origin in camera space
        Vector3 result = lookAt * target;

        // Expected: Target should be at origin with negative Z (in front of camera)
        float expectedZ = -(Vector3(3.0f, 4.0f, 5.0f).Magnitude()); // Negative distance from eye to target
        AssertVector3Equal(Vector3(0.0f, 0.0f, expectedZ), result, 0.001f);
    }

    TEST_F(Matrix4Tests, Property_PerspectiveProjection_StandardFOV_IsCorrect)
    {
        float fovY = 60.0f; // degrees
        float aspectRatio = 16.0f / 9.0f;
        float nearPlane = 0.1f;
        float farPlane = 100.0f;

        Matrix4 perspective = Matrix4::Perspective(fovY, aspectRatio, nearPlane, farPlane);

        // Test a point at the near plane center
        Vector4 testPoint(0.0f, 0.0f, -nearPlane, 1.0f);
        Vector4 result = perspective * testPoint;

        // After perspective divide (result.xyz / result.w), Z should be -1 (near plane in NDC)
        float ndcZ = result.z / result.w;
        EXPECT_EQ(-1.0f, ndcZ);
    }

    TEST_F(Matrix4Tests, Property_PerspectiveProjection_FarPlaneMapping_IsCorrect)
    {
        float fovY = 45.0f; // degrees
        float aspectRatio = 1.0f; // Square viewport
        float nearPlane = 1.0f;
        float farPlane = 10.0f;

        Matrix4 perspective = Matrix4::Perspective(fovY, aspectRatio, nearPlane, farPlane);

        // Test a point at the far plane center
        Vector4 testPoint(0.0f, 0.0f, -farPlane, 1.0f);
        Vector4 result = perspective * testPoint;

        // After perspective divide, Z should be 1 (far plane in NDC)
        float ndcZ = result.z / result.w;
        EXPECT_EQ(1.0f, ndcZ);
    }
}