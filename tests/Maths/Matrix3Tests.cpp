#include <numbers>
#include <stdexcept>

#include <gtest/gtest.h>

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Maths/Matrix2.hpp"
#include "Nudge/Maths/Matrix3.hpp"
#include "Nudge/Maths/Vector2.hpp"
#include "Nudge/Maths/Vector3.hpp"

using std::runtime_error;
using std::numbers::pi_v;

using testing::Test;

namespace Nudge
{
    class Matrix3Tests : public Test
    {
    public:
        // Helper method for floating point comparison
        static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
        {
            EXPECT_TRUE(MathF::Compare(expected, actual, tolerance));
        }

        static void AssertVector2Equal(const Vector2& expected, const Vector2& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.x, actual.x, tolerance);
            AssertFloatEqual(expected.y, actual.y, tolerance);
        }

        static void AssertVector3Equal(const Vector3& expected, const Vector3& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.x, actual.x, tolerance);
            AssertFloatEqual(expected.y, actual.y, tolerance);
            AssertFloatEqual(expected.z, actual.z, tolerance);
        }

        static void AssertMatrix3Equal(const Matrix3& expected, const Matrix3& actual, float tolerance = 0.00001f)
        {
            AssertFloatEqual(expected.m11, actual.m11, tolerance);
            AssertFloatEqual(expected.m21, actual.m21, tolerance);
            AssertFloatEqual(expected.m31, actual.m31, tolerance);
            AssertFloatEqual(expected.m12, actual.m12, tolerance);
            AssertFloatEqual(expected.m22, actual.m22, tolerance);
            AssertFloatEqual(expected.m32, actual.m32, tolerance);
            AssertFloatEqual(expected.m13, actual.m13, tolerance);
            AssertFloatEqual(expected.m23, actual.m23, tolerance);
            AssertFloatEqual(expected.m33, actual.m33, tolerance);
        }
    };

    // Static Factory Method Tests
    TEST_F(Matrix3Tests, Identity_CreatesIdentityMatrix)
    {
        Matrix3 identity = Matrix3::Identity();
        Matrix3 expected(1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f);

        AssertMatrix3Equal(expected, identity);
    }

    TEST_F(Matrix3Tests, Zero_CreatesZeroMatrix)
    {
        Matrix3 zero = Matrix3::Zero();
        Matrix3 expected(0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f);

        AssertMatrix3Equal(expected, zero);
    }

    TEST_F(Matrix3Tests, Scale_ThreeParameters_CreatesScaleMatrix)
    {
        Matrix3 scale = Matrix3::Scale(2.0f, 3.0f, 4.0f);
        Matrix3 expected(2.0f, 0.0f, 0.0f,
            0.0f, 3.0f, 0.0f,
            0.0f, 0.0f, 4.0f);

        AssertMatrix3Equal(expected, scale);
    }

    TEST_F(Matrix3Tests, Scale_Vector3Parameter_CreatesScaleMatrix)
    {
        Vector3 scaleVec(2.0f, 3.0f, 4.0f);
        Matrix3 scale = Matrix3::Scale(scaleVec);
        Matrix3 expected(2.0f, 0.0f, 0.0f,
            0.0f, 3.0f, 0.0f,
            0.0f, 0.0f, 4.0f);

        AssertMatrix3Equal(expected, scale);
    }

    TEST_F(Matrix3Tests, RotationX_ZeroDegrees_CreatesIdentityMatrix)
    {
        Matrix3 rotation = Matrix3::RotationX(0.0f);
        AssertMatrix3Equal(Matrix3::Identity(), rotation, 0.001f);
    }

    TEST_F(Matrix3Tests, RotationX_90Degrees_CreatesCorrectMatrix)
    {
        Matrix3 rotation = Matrix3::RotationX(90.0f);
        Matrix3 expected(1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, -1.0f,
            0.0f, 1.0f, 0.0f);

        AssertMatrix3Equal(expected, rotation, 0.001f);
    }

    TEST_F(Matrix3Tests, RotationX_180Degrees_CreatesCorrectMatrix)
    {
        Matrix3 rotation = Matrix3::RotationX(180.0f);
        Matrix3 expected(1.0f, 0.0f, 0.0f,
            0.0f, -1.0f, 0.0f,
            0.0f, 0.0f, -1.0f);

        AssertMatrix3Equal(expected, rotation, 0.001f);
    }

    TEST_F(Matrix3Tests, RotationY_90Degrees_CreatesCorrectMatrix)
    {
        Matrix3 rotation = Matrix3::RotationY(90.0f);
        Matrix3 expected(0.0f, 0.0f, 1.0f,
            0.0f, 1.0f, 0.0f,
            -1.0f, 0.0f, 0.0f);

        AssertMatrix3Equal(expected, rotation, 0.001f);
    }

    TEST_F(Matrix3Tests, RotationY_180Degrees_CreatesCorrectMatrix)
    {
        Matrix3 rotation = Matrix3::RotationY(180.0f);
        Matrix3 expected(-1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, -1.0f);

        AssertMatrix3Equal(expected, rotation, 0.001f);
    }

    TEST_F(Matrix3Tests, RotationZ_90Degrees_CreatesCorrectMatrix)
    {
        Matrix3 rotation = Matrix3::RotationZ(90.0f);
        Matrix3 expected(0.0f, -1.0f, 0.0f,
            1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f);

        AssertMatrix3Equal(expected, rotation, 0.001f);
    }

    TEST_F(Matrix3Tests, RotationZ_180Degrees_CreatesCorrectMatrix)
    {
        Matrix3 rotation = Matrix3::RotationZ(180.0f);
        Matrix3 expected(-1.0f, 0.0f, 0.0f,
            0.0f, -1.0f, 0.0f,
            0.0f, 0.0f, 1.0f);

        AssertMatrix3Equal(expected, rotation, 0.001f);
    }

    TEST_F(Matrix3Tests, Rotation_EulerAngles_OnlyZRotation_CreatesCorrectMatrix)
    {
        Vector3 euler(0.0f, 0.0f, 90.0f); // Only Z rotation
        Matrix3 rotation = Matrix3::Rotation(euler);
        Matrix3 expected(0.0f, -1.0f, 0.0f,
            1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f);

        AssertMatrix3Equal(expected, rotation, 0.001f);
    }

    TEST_F(Matrix3Tests, Rotation_AxisAngle_XAxis90Degrees_CreatesCorrectMatrix)
    {
        Vector3 axis(1.0f, 0.0f, 0.0f);
        Matrix3 rotation = Matrix3::Rotation(axis, 90.0f);
        Matrix3 expected(1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, -1.0f,
            0.0f, 1.0f, 0.0f);

        AssertMatrix3Equal(expected, rotation, 0.001f);
    }

    TEST_F(Matrix3Tests, Rotation_AxisAngle_ArbitraryAxis_CreatesValidRotationMatrix)
    {
        Vector3 axis(1.0f, 1.0f, 1.0f);
        Matrix3 rotation = Matrix3::Rotation(axis, 120.0f);

        // Check that it's a valid rotation matrix (orthogonal with determinant 1)
        EXPECT_TRUE(rotation.IsOrthogonal());
        AssertFloatEqual(1.0f, rotation.Determinant(), 0.001f);
    }

    TEST_F(Matrix3Tests, Translation_TwoParameters_CreatesTranslationMatrix)
    {
        Matrix3 translation = Matrix3::Translation(3.0f, 4.0f);
        Matrix3 expected(1.0f, 0.0f, 3.0f,
            0.0f, 1.0f, 4.0f,
            0.0f, 0.0f, 1.0f);

        AssertMatrix3Equal(expected, translation);
    }

    TEST_F(Matrix3Tests, Translation_Vector2Parameter_CreatesTranslationMatrix)
    {
        Vector2 transVec(3.0f, 4.0f);
        Matrix3 translation = Matrix3::Translation(transVec);
        Matrix3 expected(1.0f, 0.0f, 3.0f,
            0.0f, 1.0f, 4.0f,
            0.0f, 0.0f, 1.0f);

        AssertMatrix3Equal(expected, translation);
    }

    // Constructor Tests
    TEST_F(Matrix3Tests, Constructor_Default_CreatesIdentityMatrix)
    {
        Matrix3 matrix;
        AssertMatrix3Equal(Matrix3::Identity(), matrix);
    }

    TEST_F(Matrix3Tests, Constructor_Scalar_CreatesScalarMatrix)
    {
        Matrix3 matrix(5.0f);
        Matrix3 expected(5.0f, 0.0f, 0.0f,
            0.0f, 5.0f, 0.0f,
            0.0f, 0.0f, 5.0f);

        AssertMatrix3Equal(expected, matrix);
    }

    TEST_F(Matrix3Tests, Constructor_NineFloats_CreatesMatrixWithSpecifiedValues)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);

        // Verify column-major storage
        EXPECT_EQ(1.0f, matrix.m11);
        EXPECT_EQ(4.0f, matrix.m21);
        EXPECT_EQ(7.0f, matrix.m31);
        EXPECT_EQ(2.0f, matrix.m12);
        EXPECT_EQ(5.0f, matrix.m22);
        EXPECT_EQ(8.0f, matrix.m32);
        EXPECT_EQ(3.0f, matrix.m13);
        EXPECT_EQ(6.0f, matrix.m23);
        EXPECT_EQ(9.0f, matrix.m33);
    }

    TEST_F(Matrix3Tests, Constructor_ThreeVectors_CreatesMatrixFromColumns)
    {
        Vector3 col1(1.0f, 4.0f, 7.0f);
        Vector3 col2(2.0f, 5.0f, 8.0f);
        Vector3 col3(3.0f, 6.0f, 9.0f);
        Matrix3 matrix(col1, col2, col3);

        AssertMatrix3Equal(Matrix3(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f), matrix);
    }

    TEST_F(Matrix3Tests, Constructor_Array_CreatesMatrixFromColumnMajorArray)
    {
        float values[9] = { 1.0f, 4.0f, 7.0f, 2.0f, 5.0f, 8.0f, 3.0f, 6.0f, 9.0f };
        Matrix3 matrix(values);

        AssertMatrix3Equal(Matrix3(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f), matrix);
    }

    TEST_F(Matrix3Tests, Constructor_Matrix2_ExtendsToMatrix3WithIdentity)
    {
        Matrix2 matrix2(1.0f, 2.0f, 3.0f, 4.0f);
        Matrix3 matrix3(matrix2);
        Matrix3 expected(1.0f, 2.0f, 0.0f,
            3.0f, 4.0f, 0.0f,
            0.0f, 0.0f, 1.0f);

        AssertMatrix3Equal(expected, matrix3);
    }

    TEST_F(Matrix3Tests, Constructor_Copy_CreatesIdenticalMatrix)
    {
        Matrix3 original(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        Matrix3 copy(original);

        AssertMatrix3Equal(original, copy);
    }

    // Mathematical Operation Tests
    TEST_F(Matrix3Tests, Determinant_IdentityMatrix_ReturnsOne)
    {
        Matrix3 identity = Matrix3::Identity();
        AssertFloatEqual(1.0f, identity.Determinant());
    }

    TEST_F(Matrix3Tests, Determinant_ZeroMatrix_ReturnsZero)
    {
        Matrix3 zero = Matrix3::Zero();
        AssertFloatEqual(0.0f, zero.Determinant());
    }

    TEST_F(Matrix3Tests, Determinant_KnownMatrix_ReturnsCorrectValue)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            0.0f, 1.0f, 4.0f,
            5.0f, 6.0f, 0.0f);
        // det = 1*(1*0 - 4*6) - 2*(0*0 - 4*5) + 3*(0*6 - 1*5)
        // det = 1*(-24) - 2*(-20) + 3*(-5) = -24 + 40 - 15 = 1
        AssertFloatEqual(1.0f, matrix.Determinant());
    }

    TEST_F(Matrix3Tests, Determinant_RotationMatrix_ReturnsOne)
    {
        Matrix3 rotation = Matrix3::RotationZ(45.0f);
        AssertFloatEqual(1.0f, rotation.Determinant(), 0.001f);
    }

    TEST_F(Matrix3Tests, Determinant_ScaleMatrix_ReturnsProduct)
    {
        Matrix3 scale = Matrix3::Scale(2.0f, 3.0f, 4.0f);
        AssertFloatEqual(24.0f, scale.Determinant()); // 2 * 3 * 4 = 24
    }

    TEST_F(Matrix3Tests, Transposed_IdentityMatrix_ReturnsIdentity)
    {
        Matrix3 identity = Matrix3::Identity();
        Matrix3 transposed = identity.Transposed();
        AssertMatrix3Equal(identity, transposed);
    }

    TEST_F(Matrix3Tests, Transposed_GeneralMatrix_SwapsOffDiagonalElements)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        Matrix3 transposed = matrix.Transposed();
        Matrix3 expected(1.0f, 4.0f, 7.0f,
            2.0f, 5.0f, 8.0f,
            3.0f, 6.0f, 9.0f);

        AssertMatrix3Equal(expected, transposed);

        // Verify original is unchanged
        AssertMatrix3Equal(Matrix3(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f), matrix);
    }

    TEST_F(Matrix3Tests, Transpose_ModifiesOriginalMatrix)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        Matrix3 expected(1.0f, 4.0f, 7.0f,
            2.0f, 5.0f, 8.0f,
            3.0f, 6.0f, 9.0f);
        matrix.Transpose();
        AssertMatrix3Equal(expected, matrix);
    }

    TEST_F(Matrix3Tests, Inverse_IdentityMatrix_ReturnsIdentity)
    {
        Matrix3 identity = Matrix3::Identity();
        Matrix3 inverse = identity.Inverse();
        AssertMatrix3Equal(identity, inverse);
    }

    TEST_F(Matrix3Tests, Inverse_ScaleMatrix_ReturnsInverseScale)
    {
        Matrix3 scale = Matrix3::Scale(2.0f, 3.0f, 4.0f);
        Matrix3 inverse = scale.Inverse();
        Matrix3 expected(0.5f, 0.0f, 0.0f,
            0.0f, 1.0f / 3.0f, 0.0f,
            0.0f, 0.0f, 0.25f);

        AssertMatrix3Equal(expected, inverse, 0.001f);
    }

    TEST_F(Matrix3Tests, Inverse_MultiplyByOriginal_GivesIdentity)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            0.0f, 1.0f, 4.0f,
            5.0f, 6.0f, 0.0f);
        Matrix3 inverse = matrix.Inverse();
        Matrix3 product = matrix * inverse;

        AssertMatrix3Equal(Matrix3::Identity(), product, 0.001f);
    }

    TEST_F(Matrix3Tests, Inverse_RotationMatrix_ReturnsTranspose)
    {
        Matrix3 rotation = Matrix3::RotationZ(30.0f);
        Matrix3 inverse = rotation.Inverse();
        Matrix3 transpose = rotation.Transposed();

        AssertMatrix3Equal(transpose, inverse, 0.001f);
    }

    TEST_F(Matrix3Tests, Inverse_SingularMatrix_ThrowsException)
    {
        Matrix3 singular = Matrix3::Zero(); // Determinant = 0
        EXPECT_THROW({
            Matrix3 inverse = singular.Inverse();
            }, runtime_error);
    }

    // Utility Method Tests
    TEST_F(Matrix3Tests, IsIdentity_IdentityMatrix_ReturnsTrue)
    {
        Matrix3 identity = Matrix3::Identity();
        EXPECT_TRUE(identity.IsIdentity());
    }

    TEST_F(Matrix3Tests, IsIdentity_NonIdentityMatrix_ReturnsFalse)
    {
        Matrix3 matrix = Matrix3::Scale(2.0f, 1.0f, 1.0f);
        EXPECT_FALSE(matrix.IsIdentity());
    }

    TEST_F(Matrix3Tests, IsIdentity_NearIdentityMatrix_ReturnsTrue)
    {
        Matrix3 matrix(1.00001f, 0.00001f, 0.00001f,
            0.00001f, 0.99999f, 0.00001f,
            0.00001f, 0.00001f, 1.00001f);
        EXPECT_TRUE(matrix.IsIdentity(0.0001f));
    }

    TEST_F(Matrix3Tests, IsZero_ZeroMatrix_ReturnsTrue)
    {
        Matrix3 zero = Matrix3::Zero();
        EXPECT_TRUE(zero.IsZero());
    }

    TEST_F(Matrix3Tests, IsZero_NonZeroMatrix_ReturnsFalse)
    {
        Matrix3 matrix(0.1f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f);
        EXPECT_FALSE(matrix.IsZero());
    }

    TEST_F(Matrix3Tests, IsOrthogonal_RotationMatrix_ReturnsTrue)
    {
        Matrix3 rotation = Matrix3::RotationZ(45.0f);
        EXPECT_TRUE(rotation.IsOrthogonal());
    }

    TEST_F(Matrix3Tests, IsOrthogonal_ScaleMatrix_ReturnsFalse)
    {
        Matrix3 scale = Matrix3::Scale(2.0f, 2.0f, 2.0f);
        EXPECT_FALSE(scale.IsOrthogonal());
    }

    TEST_F(Matrix3Tests, IsOrthogonal_IdentityMatrix_ReturnsTrue)
    {
        Matrix3 identity = Matrix3::Identity();
        EXPECT_TRUE(identity.IsOrthogonal());
    }

    // Accessor Method Tests
    TEST_F(Matrix3Tests, GetColumn_ValidIndices_ReturnsCorrectColumns)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);

        Vector3 col0 = matrix.GetColumn(0);
        Vector3 col1 = matrix.GetColumn(1);
        Vector3 col2 = matrix.GetColumn(2);

        AssertVector3Equal(Vector3(1.0f, 4.0f, 7.0f), col0);
        AssertVector3Equal(Vector3(2.0f, 5.0f, 8.0f), col1);
        AssertVector3Equal(Vector3(3.0f, 6.0f, 9.0f), col2);
    }

    TEST_F(Matrix3Tests, GetColumn_InvalidIndex_ThrowsException)
    {
        Matrix3 matrix = Matrix3::Identity();
        EXPECT_THROW({
            Vector3 col = matrix.GetColumn(3);
            }, runtime_error);
        EXPECT_THROW({
            Vector3 col = matrix.GetColumn(-1);
            }, runtime_error);
    }

    TEST_F(Matrix3Tests, SetColumn_ValidIndices_SetsCorrectColumns)
    {
        Matrix3 matrix = Matrix3::Identity();
        Vector3 newColumn(10.0f, 20.0f, 30.0f);

        matrix.SetColumn(1, newColumn);

        AssertVector3Equal(newColumn, matrix.GetColumn(1));
        AssertVector3Equal(Vector3(1.0f, 0.0f, 0.0f), matrix.GetColumn(0));
        AssertVector3Equal(Vector3(0.0f, 0.0f, 1.0f), matrix.GetColumn(2));
    }

    TEST_F(Matrix3Tests, SetColumn_InvalidIndex_ThrowsException)
    {
        Matrix3 matrix = Matrix3::Identity();
        Vector3 newColumn(1.0f, 2.0f, 3.0f);

        EXPECT_THROW({
            matrix.SetColumn(3, newColumn);
            }, runtime_error);
        EXPECT_THROW({
            matrix.SetColumn(-1, newColumn);
            }, runtime_error);
    }

    TEST_F(Matrix3Tests, GetRow_ValidIndices_ReturnsCorrectRows)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);

        Vector3 row0 = matrix.GetRow(0);
        Vector3 row1 = matrix.GetRow(1);
        Vector3 row2 = matrix.GetRow(2);

        AssertVector3Equal(Vector3(1.0f, 2.0f, 3.0f), row0);
        AssertVector3Equal(Vector3(4.0f, 5.0f, 6.0f), row1);
        AssertVector3Equal(Vector3(7.0f, 8.0f, 9.0f), row2);
    }

    TEST_F(Matrix3Tests, GetRow_InvalidIndex_ThrowsException)
    {
        Matrix3 matrix = Matrix3::Identity();
        EXPECT_THROW({
            Vector3 row = matrix.GetRow(3);
            }, runtime_error);
        EXPECT_THROW({
            Vector3 row = matrix.GetRow(-1);
            }, runtime_error);
    }

    TEST_F(Matrix3Tests, SetRow_ValidIndices_SetsCorrectRows)
    {
        Matrix3 matrix = Matrix3::Identity();
        Vector3 newRow(10.0f, 20.0f, 30.0f);

        matrix.SetRow(1, newRow);

        AssertVector3Equal(newRow, matrix.GetRow(1));
        AssertVector3Equal(Vector3(1.0f, 0.0f, 0.0f), matrix.GetRow(0));
        AssertVector3Equal(Vector3(0.0f, 0.0f, 1.0f), matrix.GetRow(2));
    }

    TEST_F(Matrix3Tests, SetRow_InvalidIndex_ThrowsException)
    {
        Matrix3 matrix = Matrix3::Identity();
        Vector3 newRow(1.0f, 2.0f, 3.0f);

        EXPECT_THROW({
            matrix.SetRow(3, newRow);
            }, runtime_error);
        EXPECT_THROW({
            matrix.SetRow(-1, newRow);
            }, runtime_error);
    }

    // Operator Tests
    TEST_F(Matrix3Tests, Equality_SameMatrices_ReturnsTrue)
    {
        Matrix3 a(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        Matrix3 b(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        EXPECT_TRUE(a == b);
    }

    TEST_F(Matrix3Tests, Equality_DifferentMatrices_ReturnsFalse)
    {
        Matrix3 a = Matrix3::Identity();
        Matrix3 b = Matrix3::Zero();
        EXPECT_FALSE(a == b);
    }

    TEST_F(Matrix3Tests, Equality_NearlyIdenticalMatrices_ReturnsTrue)
    {
        Matrix3 a = Matrix3::Identity();
        Matrix3 b(1.0f + 1e-8f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f);

        EXPECT_TRUE(a == b);
    }

    TEST_F(Matrix3Tests, Inequality_DifferentMatrices_ReturnsTrue)
    {
        Matrix3 a = Matrix3::Identity();
        Matrix3 b = Matrix3::Zero();
        EXPECT_TRUE(a != b);
    }

    TEST_F(Matrix3Tests, Inequality_IdenticalMatrices_ReturnsFalse)
    {
        Matrix3 a = Matrix3::Identity();
        Matrix3 b = Matrix3::Identity();
        EXPECT_FALSE(a != b);
    }

    TEST_F(Matrix3Tests, MatrixMultiplication_IdentityMatrix_ReturnsOriginal)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        Matrix3 identity = Matrix3::Identity();

        Matrix3 result1 = matrix * identity;
        Matrix3 result2 = identity * matrix;

        AssertMatrix3Equal(matrix, result1);
        AssertMatrix3Equal(matrix, result2);
    }

    TEST_F(Matrix3Tests, MatrixMultiplication_KnownMatrices_ReturnsCorrectResult)
    {
        Matrix3 matrix1(1.0f, 2.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f);
        Matrix3 matrix2(1.0f, 0.0f, 3.0f,
            0.0f, 1.0f, 4.0f,
            0.0f, 0.0f, 1.0f);

        Matrix3 result = matrix1 * matrix2;
        Matrix3 expected(1.0f, 2.0f, 11.0f, // 1*1+2*0+0*0, 1*0+2*1+0*0, 1*3+2*4+0*1
            0.0f, 1.0f, 4.0f,
            0.0f, 0.0f, 1.0f);

        AssertMatrix3Equal(expected, result);
    }

    TEST_F(Matrix3Tests, ScalarMultiplication_MultiplyByZero_ReturnsZeroMatrix)
    {
        Matrix3 matrix = Matrix3::Identity();
        Matrix3 result = matrix * 0.0f;
        AssertMatrix3Equal(Matrix3::Zero(), result);
    }

    TEST_F(Matrix3Tests, ScalarMultiplication_MultiplyByTwo_DoublesAllElements)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        Matrix3 result = matrix * 2.0f;
        Matrix3 expected(2.0f, 4.0f, 6.0f,
            8.0f, 10.0f, 12.0f,
            14.0f, 16.0f, 18.0f);

        AssertMatrix3Equal(expected, result);
    }

    TEST_F(Matrix3Tests, ScalarDivision_DivideByTwo_HalvesAllElements)
    {
        Matrix3 matrix(2.0f, 4.0f, 6.0f,
            8.0f, 10.0f, 12.0f,
            14.0f, 16.0f, 18.0f);
        Matrix3 result = matrix / 2.0f;
        Matrix3 expected(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);

        AssertMatrix3Equal(expected, result);
    }

    TEST_F(Matrix3Tests, ScalarDivision_DivideByZero_ThrowsException)
    {
        Matrix3 matrix = Matrix3::Identity();

        EXPECT_THROW({
            Matrix3 result = matrix / 0.0f;
            }, runtime_error);
    }

    TEST_F(Matrix3Tests, Vector3Multiplication_IdentityMatrix_ReturnsOriginalVector)
    {
        Matrix3 identity = Matrix3::Identity();
        Vector3 vector(1.0f, 2.0f, 3.0f);

        Vector3 result = identity * vector;

        AssertVector3Equal(vector, result);
    }

    TEST_F(Matrix3Tests, Vector3Multiplication_TranslationMatrix_TranslatesVector)
    {
        Matrix3 translation = Matrix3::Translation(5.0f, 6.0f);
        Vector3 vector(1.0f, 2.0f, 1.0f); // Homogeneous coordinates

        Vector3 result = translation * vector;
        Vector3 expected(6.0f, 8.0f, 1.0f); // (1+5, 2+6, 1)

        AssertVector3Equal(expected, result);
    }

    TEST_F(Matrix3Tests, Vector3Multiplication_RotationMatrix_RotatesVector)
    {
        Matrix3 rotation = Matrix3::RotationZ(90.0f);
        Vector3 vector(1.0f, 0.0f, 0.0f);

        Vector3 result = rotation * vector;
        Vector3 expected(0.0f, 1.0f, 0.0f);

        AssertVector3Equal(expected, result, 0.001f);
    }

    TEST_F(Matrix3Tests, Vector3Multiplication_ScaleMatrix_ScalesVector)
    {
        Matrix3 scale = Matrix3::Scale(2.0f, 3.0f, 4.0f);
        Vector3 vector(1.0f, 1.0f, 1.0f);

        Vector3 result = scale * vector;
        Vector3 expected(2.0f, 3.0f, 4.0f);

        AssertVector3Equal(expected, result);
    }

    TEST_F(Matrix3Tests, IndexOperator_ValidIndices_ReturnsCorrectColumns)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);

        Vector3 col0 = matrix[0];
        Vector3 col1 = matrix[1];
        Vector3 col2 = matrix[2];

        AssertVector3Equal(Vector3(1.0f, 4.0f, 7.0f), col0);
        AssertVector3Equal(Vector3(2.0f, 5.0f, 8.0f), col1);
        AssertVector3Equal(Vector3(3.0f, 6.0f, 9.0f), col2);
    }

    TEST_F(Matrix3Tests, IndexOperator_InvalidIndex_ThrowsException)
    {
        Matrix3 matrix = Matrix3::Identity();
        EXPECT_THROW({
            Vector3 col = matrix[3];
            }, runtime_error);
        EXPECT_THROW({
            Vector3 col = matrix[-1];
            }, runtime_error);
    }

    TEST_F(Matrix3Tests, Assignment_CopiesAllElements)
    {
        Matrix3 original(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        Matrix3 assigned;
        assigned = original;
        AssertMatrix3Equal(original, assigned);
    }

    TEST_F(Matrix3Tests, Assignment_SelfAssignment_RemainsUnchanged)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        Matrix3 original = matrix;
        matrix = matrix;
        AssertMatrix3Equal(original, matrix);
    }

    TEST_F(Matrix3Tests, GlobalScalarMultiplication_ScalarFirst_Works)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        Matrix3 result = 3.0f * matrix;
        Matrix3 expected(3.0f, 6.0f, 9.0f,
            12.0f, 15.0f, 18.0f,
            21.0f, 24.0f, 27.0f);
        AssertMatrix3Equal(expected, result);
    }

    // Edge Case Tests
    TEST_F(Matrix3Tests, EdgeCase_VerySmallNumbers_HandlePrecisionCorrectly)
    {
        Matrix3 matrix(1e-20f, 0.0f, 0.0f,
            0.0f, 1e-20f, 0.0f,
            0.0f, 0.0f, 1e-20f);

        float det = matrix.Determinant();
        EXPECT_TRUE(std::isfinite(det));
    }

    TEST_F(Matrix3Tests, EdgeCase_ZeroAngleRotations_ProduceIdentity)
    {
        Matrix3 rotX = Matrix3::RotationX(0.0f);
        Matrix3 rotY = Matrix3::RotationY(0.0f);
        Matrix3 rotZ = Matrix3::RotationZ(0.0f);

        AssertMatrix3Equal(Matrix3::Identity(), rotX, 0.001f);
        AssertMatrix3Equal(Matrix3::Identity(), rotY, 0.001f);
        AssertMatrix3Equal(Matrix3::Identity(), rotZ, 0.001f);
    }

    TEST_F(Matrix3Tests, EdgeCase_FullCircleRotations_ProduceIdentity)
    {
        Matrix3 rotX = Matrix3::RotationX(360.0f);
        Matrix3 rotY = Matrix3::RotationY(360.0f);
        Matrix3 rotZ = Matrix3::RotationZ(360.0f);

        AssertMatrix3Equal(Matrix3::Identity(), rotX, 0.001f);
        AssertMatrix3Equal(Matrix3::Identity(), rotY, 0.001f);
        AssertMatrix3Equal(Matrix3::Identity(), rotZ, 0.001f);
    }

    // Combined Operation Tests
    TEST_F(Matrix3Tests, CombinedTransformations_ScaleRotateTranslate_WorksCorrectly)
    {
        // Create transformation: translate * rotate * scale
        Matrix3 scale = Matrix3::Scale(2.0f, 3.0f, 1.0f);
        Matrix3 rotation = Matrix3::RotationZ(90.0f);
        Matrix3 translation = Matrix3::Translation(5.0f, 6.0f);

        Matrix3 combined = translation * rotation * scale;

        // Test a point transformation
        Vector3 point(1.0f, 0.0f, 1.0f); // Homogeneous coordinates
        Vector3 result = combined * point;

        // Expected: scale(1,0) -> (2,0), rotate 90 degrees -> (0,2), translate -> (5,8)
        Vector3 expected(5.0f, 8.0f, 1.0f);

        AssertVector3Equal(expected, result, 0.001f);
    }

    TEST_F(Matrix3Tests, Property_RotationConsistency_AllAxes_MaintainOrthogonality)
    {
        Matrix3 rotX = Matrix3::RotationX(30.0f);
        Matrix3 rotY = Matrix3::RotationY(45.0f);
        Matrix3 rotZ = Matrix3::RotationZ(60.0f);

        EXPECT_TRUE(rotX.IsOrthogonal());
        EXPECT_TRUE(rotY.IsOrthogonal());
        EXPECT_TRUE(rotZ.IsOrthogonal());

        AssertFloatEqual(1.0f, rotX.Determinant(), 0.001f);
        AssertFloatEqual(1.0f, rotY.Determinant(), 0.001f);
        AssertFloatEqual(1.0f, rotZ.Determinant(), 0.001f);
    }

    TEST_F(Matrix3Tests, Property_InverseMultiplication_VariousMatrices_GivesIdentity)
    {
        // Test different types of matrices
        Matrix3 matrices[] = {
            Matrix3::Scale(2.0f, 3.0f, 4.0f),
            Matrix3::RotationX(45.0f),
            Matrix3::RotationY(30.0f),
            Matrix3::RotationZ(60.0f),
            Matrix3::Translation(3.0f, 4.0f)
        };

        for (const auto& matrix : matrices)
        {
            Matrix3 inverse = matrix.Inverse();
            Matrix3 product = matrix * inverse;
            AssertMatrix3Equal(Matrix3::Identity(), product, 0.001f);
        }
    }

    // Mathematical Property Tests
    TEST_F(Matrix3Tests, Property_TransposeOfTranspose_ReturnsOriginal)
    {
        Matrix3 matrix(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
        Matrix3 doubleTransposed = matrix.Transposed().Transposed();
        AssertMatrix3Equal(matrix, doubleTransposed);
    }

    TEST_F(Matrix3Tests, Property_DeterminantOfTranspose_EqualToOriginal)
    {
        Matrix3 matrix = Matrix3::Translation(1.0f, 2.0f) *
            Matrix3::RotationZ(45.0f) *
            Matrix3::Scale(2.0f, 3.0f, 4.0f);
        float detOriginal = matrix.Determinant();
        float detTransposed = matrix.Transposed().Determinant();
        AssertFloatEqual(detOriginal, detTransposed, 0.001f);
    }

    TEST_F(Matrix3Tests, Property_InverseOfInverse_ReturnsOriginal)
    {
        Matrix3 matrix = Matrix3::Translation(1.0f, 2.0f) *
            Matrix3::RotationZ(30.0f) *
            Matrix3::Scale(2.0f, 3.0f, 4.0f);
        Matrix3 doubleInverse = matrix.Inverse().Inverse();
        AssertMatrix3Equal(matrix, doubleInverse, 0.001f);
    }

    TEST_F(Matrix3Tests, Property_MatrixMultiplicationAssociativity)
    {
        Matrix3 a = Matrix3::Scale(2.0f, 2.0f, 2.0f);
        Matrix3 b = Matrix3::RotationZ(45.0f);
        Matrix3 c = Matrix3::Translation(1.0f, 2.0f);

        Matrix3 result1 = (a * b) * c;
        Matrix3 result2 = a * (b * c);
        AssertMatrix3Equal(result1, result2, 0.001f);
    }

    TEST_F(Matrix3Tests, Property_IdentityIsMultiplicativeIdentity)
    {
        Matrix3 matrix = Matrix3::Translation(1.0f, 2.0f) *
            Matrix3::RotationY(30.0f) *
            Matrix3::Scale(2.0f, 3.0f, 4.0f);
        Matrix3 identity = Matrix3::Identity();

        Matrix3 leftProduct = identity * matrix;
        Matrix3 rightProduct = matrix * identity;

        AssertMatrix3Equal(matrix, leftProduct);
        AssertMatrix3Equal(matrix, rightProduct);
    }
}