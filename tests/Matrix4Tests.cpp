/**
 * @file Matrix4Tests.cpp
 * @brief Unit tests for Matrix4 4x4 transformation matrix class
 */

#include <iostream>
#include <numbers>

#include "CppUnitTest.h"

#include "Nudge/MathF.hpp"
#include "Nudge/Matrix3.hpp"
#include "Nudge/Matrix4.hpp"
#include "Nudge/Vector3.hpp"
#include "Nudge/Vector4.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using std::runtime_error;
using std::numbers::pi_v;

namespace Nudge
{
    TEST_CLASS(Matrix4Tests)
    {
    public:
        // Helper method for floating point comparison
        static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
        {
            Assert::IsTrue(MathF::Compare(expected, actual, tolerance), L"Float values are not equal within tolerance");
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

        // Static Factory Method Tests
        TEST_METHOD(Identity_CreatesIdentityMatrix)
        {
            Matrix4 identity = Matrix4::Identity();
            AssertMatrix4Equal(Matrix4(1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f), identity);
        }

        TEST_METHOD(Zero_CreatesZeroMatrix)
        {
            Matrix4 zero = Matrix4::Zero();
            AssertMatrix4Equal(Matrix4(0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f), zero);
        }

        TEST_METHOD(Scale_ThreeParameters_CreatesScaleMatrix)
        {
            Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
            AssertMatrix4Equal(Matrix4(2.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 3.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 4.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f), scale);
        }

        TEST_METHOD(Scale_Vector3Parameter_CreatesScaleMatrix)
        {
            Vector3 scaleVec(2.5f, 1.5f, 3.0f);
            Matrix4 scale = Matrix4::Scale(scaleVec);
            AssertMatrix4Equal(Matrix4(2.5f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.5f, 0.0f, 0.0f,
                0.0f, 0.0f, 3.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f), scale);
        }

        TEST_METHOD(RotationX_ZeroDegrees_CreatesIdentityMatrix)
        {
            Matrix4 rotation = Matrix4::RotationX(0.0f);
            AssertMatrix4Equal(Matrix4::Identity(), rotation);
        }

        TEST_METHOD(RotationX_90Degrees_CreatesCorrectMatrix)
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

        TEST_METHOD(RotationY_90Degrees_CreatesCorrectMatrix)
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

        TEST_METHOD(RotationZ_90Degrees_CreatesCorrectMatrix)
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

        TEST_METHOD(Rotation_EulerAngles_CreatesCorrectMatrix)
        {
            Vector3 euler(90.0f, 0.0f, 0.0f); // 90 degrees around X
            Matrix4 rotation = Matrix4::Rotation(euler);
            Matrix4 expected = Matrix4::RotationX(90.0f);
            AssertMatrix4Equal(expected, rotation, 0.001f);
        }

        TEST_METHOD(Rotation_AxisAngle_CreatesCorrectMatrix)
        {
            Vector3 axis(0.0f, 0.0f, 1.0f); // Z-axis
            Matrix4 rotation = Matrix4::Rotation(axis, 90.0f);
            Matrix4 expected = Matrix4::RotationZ(90.0f);
            AssertMatrix4Equal(expected, rotation, 0.001f);
        }

        TEST_METHOD(Translation_ThreeParameters_CreatesTranslationMatrix)
        {
            Matrix4 translation = Matrix4::Translation(2.0f, 3.0f, 4.0f);
            AssertMatrix4Equal(Matrix4(1.0f, 0.0f, 0.0f, 2.0f,
                0.0f, 1.0f, 0.0f, 3.0f,
                0.0f, 0.0f, 1.0f, 4.0f,
                0.0f, 0.0f, 0.0f, 1.0f), translation);
        }

        TEST_METHOD(Translation_Vector3Parameter_CreatesTranslationMatrix)
        {
            Vector3 translationVec(5.0f, 6.0f, 7.0f);
            Matrix4 translation = Matrix4::Translation(translationVec);
            AssertMatrix4Equal(Matrix4(1.0f, 0.0f, 0.0f, 5.0f,
                0.0f, 1.0f, 0.0f, 6.0f,
                0.0f, 0.0f, 1.0f, 7.0f,
                0.0f, 0.0f, 0.0f, 1.0f), translation);
        }

        TEST_METHOD(TRS_CombinesTransformationsCorrectly)
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
        TEST_METHOD(Constructor_Default_CreatesIdentityMatrix)
        {
            Matrix4 matrix;
            AssertMatrix4Equal(Matrix4::Identity(), matrix);
        }

        TEST_METHOD(Constructor_Scalar_CreatesScalarMatrix)
        {
            Matrix4 matrix(5.0f);
            AssertMatrix4Equal(Matrix4(5.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 5.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 5.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 5.0f), matrix);
        }

        TEST_METHOD(Constructor_SixteenFloats_CreatesMatrixWithSpecifiedValues)
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

        TEST_METHOD(Constructor_FourVectors_CreatesMatrixFromColumns)
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

        TEST_METHOD(Constructor_Array_CreatesMatrixFromColumnMajorArray)
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

        TEST_METHOD(Constructor_Matrix3_ExtendsToMatrix4WithIdentity)
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

        TEST_METHOD(Constructor_Copy_CreatesIdenticalMatrix)
        {
            Matrix4 original(1.5f, 2.5f, 3.5f, 4.5f,
                5.5f, 6.5f, 7.5f, 8.5f,
                9.5f, 10.5f, 11.5f, 12.5f,
                13.5f, 14.5f, 15.5f, 16.5f);
            Matrix4 copy(original);
            AssertMatrix4Equal(original, copy);
        }

        // Mathematical Operation Tests
        TEST_METHOD(Determinant_IdentityMatrix_ReturnsOne)
        {
            Matrix4 identity = Matrix4::Identity();
            AssertFloatEqual(1.0f, identity.Determinant());
        }

        TEST_METHOD(Determinant_ZeroMatrix_ReturnsZero)
        {
            Matrix4 zero = Matrix4::Zero();
            AssertFloatEqual(0.0f, zero.Determinant());
        }

        TEST_METHOD(Determinant_ScaleMatrix_ReturnsProduct)
        {
            Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
            AssertFloatEqual(24.0f, scale.Determinant()); // 2 * 3 * 4 * 1 = 24
        }

        TEST_METHOD(Determinant_SingularMatrix_ReturnsZero)
        {
            Matrix4 singular(1.0f, 2.0f, 3.0f, 4.0f,
                2.0f, 4.0f, 6.0f, 8.0f,
                1.0f, 1.0f, 1.0f, 1.0f,
                0.0f, 0.0f, 0.0f, 1.0f);
            AssertFloatEqual(0.0f, singular.Determinant(), 0.001f);
        }

        TEST_METHOD(Transposed_IdentityMatrix_ReturnsIdentity)
        {
            Matrix4 identity = Matrix4::Identity();
            Matrix4 transposed = identity.Transposed();
            AssertMatrix4Equal(identity, transposed);
        }

        TEST_METHOD(Transposed_GeneralMatrix_SwapsOffDiagonalElements)
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

        TEST_METHOD(Transpose_ModifiesOriginalMatrix)
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

        TEST_METHOD(Inverse_IdentityMatrix_ReturnsIdentity)
        {
            Matrix4 identity = Matrix4::Identity();
            Matrix4 inverse = identity.Inverse();
            AssertMatrix4Equal(identity, inverse);
        }

        TEST_METHOD(Inverse_ScaleMatrix_ReturnsInverseScale)
        {
            Matrix4 scale = Matrix4::Scale(2.0f, 4.0f, 8.0f);
            Matrix4 inverse = scale.Inverse();
            AssertMatrix4Equal(Matrix4::Scale(0.5f, 0.25f, 0.125f), inverse, 0.001f);
        }

        TEST_METHOD(Inverse_TranslationMatrix_ReturnsInverseTranslation)
        {
            Matrix4 translation = Matrix4::Translation(2.0f, 3.0f, 4.0f);
            Matrix4 inverse = translation.Inverse();
            AssertMatrix4Equal(Matrix4::Translation(-2.0f, -3.0f, -4.0f), inverse, 0.001f);
        }

        TEST_METHOD(Inverse_SingularMatrix_ThrowsException)
        {
            Matrix4 singular(1.0f, 2.0f, 3.0f, 4.0f,
                2.0f, 4.0f, 6.0f, 8.0f,
                1.0f, 1.0f, 1.0f, 1.0f,
                0.0f, 0.0f, 0.0f, 0.0f);
            Assert::ExpectException<runtime_error>([&]() {
                Matrix4 inverse = singular.Inverse();
                });
        }

        TEST_METHOD(Inverse_MultiplyByOriginal_GivesIdentity)
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
        TEST_METHOD(IsIdentity_IdentityMatrix_ReturnsTrue)
        {
            Matrix4 identity = Matrix4::Identity();
            Assert::IsTrue(identity.IsIdentity());
        }

        TEST_METHOD(IsIdentity_NonIdentityMatrix_ReturnsFalse)
        {
            Matrix4 matrix = Matrix4::Scale(2.0f, 1.0f, 1.0f);
            Assert::IsFalse(matrix.IsIdentity());
        }

        TEST_METHOD(IsIdentity_NearIdentityMatrix_ReturnsTrue)
        {
            Matrix4 matrix(1.00001f, 0.00001f, 0.00001f, 0.00001f,
                0.00001f, 0.99999f, 0.00001f, 0.00001f,
                0.00001f, 0.00001f, 1.00001f, 0.00001f,
                0.00001f, 0.00001f, 0.00001f, 0.99999f);
            Assert::IsTrue(matrix.IsIdentity(0.0001f));
        }

        TEST_METHOD(IsZero_ZeroMatrix_ReturnsTrue)
        {
            Matrix4 zero = Matrix4::Zero();
            Assert::IsTrue(zero.IsZero());
        }

        TEST_METHOD(IsZero_NonZeroMatrix_ReturnsFalse)
        {
            Matrix4 matrix(0.1f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f);
            Assert::IsFalse(matrix.IsZero());
        }

        TEST_METHOD(IsZero_NearZeroMatrix_ReturnsTrue)
        {
            Matrix4 matrix(0.00001f, 0.00001f, 0.00001f, 0.00001f,
                0.00001f, 0.00001f, 0.00001f, 0.00001f,
                0.00001f, 0.00001f, 0.00001f, 0.00001f,
                0.00001f, 0.00001f, 0.00001f, 0.00001f);
            Assert::IsTrue(matrix.IsZero(0.0001f));
        }

        TEST_METHOD(IsOrthogonal_IdentityMatrix_ReturnsTrue)
        {
            Matrix4 identity = Matrix4::Identity();
            Assert::IsTrue(identity.IsOrthogonal());
        }

        TEST_METHOD(IsOrthogonal_RotationMatrix_ReturnsTrue)
        {
            Matrix4 rotation = Matrix4::RotationZ(45.0f);
            Assert::IsTrue(rotation.IsOrthogonal());
        }

        TEST_METHOD(IsOrthogonal_ScaleMatrix_ReturnsFalse)
        {
            Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
            Assert::IsFalse(scale.IsOrthogonal());
        }

        // Transformation Component Extraction Tests
        TEST_METHOD(GetTranslation_TranslationMatrix_ReturnsCorrectVector)
        {
            Matrix4 translation = Matrix4::Translation(2.0f, 3.0f, 4.0f);
            Vector3 result = translation.GetTranslation();
            AssertVector3Equal(Vector3(2.0f, 3.0f, 4.0f), result);
        }

        TEST_METHOD(GetTranslation_IdentityMatrix_ReturnsZeroVector)
        {
            Matrix4 identity = Matrix4::Identity();
            Vector3 result = identity.GetTranslation();
            AssertVector3Equal(Vector3(0.0f, 0.0f, 0.0f), result);
        }

        TEST_METHOD(GetScale_ScaleMatrix_ReturnsCorrectVector)
        {
            Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
            Vector3 result = scale.GetScale();
            AssertVector3Equal(Vector3(2.0f, 3.0f, 4.0f), result, 0.001f);
        }

        TEST_METHOD(GetScale_IdentityMatrix_ReturnsUnitVector)
        {
            Matrix4 identity = Matrix4::Identity();
            Vector3 result = identity.GetScale();
            AssertVector3Equal(Vector3(1.0f, 1.0f, 1.0f), result, 0.001f);
        }

        TEST_METHOD(GetRotation_RotationMatrix_ReturnsCorrect3x3Matrix)
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
        TEST_METHOD(GetColumn_Index0_ReturnsFirstColumn)
        {
            Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
                5.0f, 6.0f, 7.0f, 8.0f,
                9.0f, 10.0f, 11.0f, 12.0f,
                13.0f, 14.0f, 15.0f, 16.0f);
            Vector4 column = matrix.GetColumn(0);
            AssertVector4Equal(Vector4(1.0f, 5.0f, 9.0f, 13.0f), column);
        }

        TEST_METHOD(GetColumn_Index3_ReturnsFourthColumn)
        {
            Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
                5.0f, 6.0f, 7.0f, 8.0f,
                9.0f, 10.0f, 11.0f, 12.0f,
                13.0f, 14.0f, 15.0f, 16.0f);
            Vector4 column = matrix.GetColumn(3);
            AssertVector4Equal(Vector4(4.0f, 8.0f, 12.0f, 16.0f), column);
        }

        TEST_METHOD(GetColumn_InvalidIndex_ThrowsException)
        {
            Matrix4 matrix = Matrix4::Identity();
            Assert::ExpectException<runtime_error>([&]() {
                Vector4 column = matrix.GetColumn(4);
                });
        }

        TEST_METHOD(SetColumn_Index0_SetsFirstColumn)
        {
            Matrix4 matrix = Matrix4::Identity();
            Vector4 newColumn(5.0f, 6.0f, 7.0f, 8.0f);
            matrix.SetColumn(0, newColumn);
            AssertMatrix4Equal(Matrix4(5.0f, 0.0f, 0.0f, 0.0f,
                6.0f, 1.0f, 0.0f, 0.0f,
                7.0f, 0.0f, 1.0f, 0.0f,
                8.0f, 0.0f, 0.0f, 1.0f), matrix);
        }

        TEST_METHOD(SetColumn_InvalidIndex_ThrowsException)
        {
            Matrix4 matrix = Matrix4::Identity();
            Vector4 newColumn(1.0f, 2.0f, 3.0f, 4.0f);
            Assert::ExpectException<runtime_error>([&]() {
                matrix.SetColumn(4, newColumn);
                });
        }

        TEST_METHOD(GetRow_Index0_ReturnsFirstRow)
        {
            Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
                5.0f, 6.0f, 7.0f, 8.0f,
                9.0f, 10.0f, 11.0f, 12.0f,
                13.0f, 14.0f, 15.0f, 16.0f);
            Vector4 row = matrix.GetRow(0);
            AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 4.0f), row);
        }

        TEST_METHOD(GetRow_Index3_ReturnsFourthRow)
        {
            Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
                5.0f, 6.0f, 7.0f, 8.0f,
                9.0f, 10.0f, 11.0f, 12.0f,
                13.0f, 14.0f, 15.0f, 16.0f);
            Vector4 row = matrix.GetRow(3);
            AssertVector4Equal(Vector4(13.0f, 14.0f, 15.0f, 16.0f), row);
        }

        TEST_METHOD(GetRow_InvalidIndex_ThrowsException)
        {
            Matrix4 matrix = Matrix4::Identity();
            Assert::ExpectException<runtime_error>([&]() {
                Vector4 row = matrix.GetRow(4);
                });
        }

        TEST_METHOD(SetRow_Index0_SetsFirstRow)
        {
            Matrix4 matrix = Matrix4::Identity();
            Vector4 newRow(5.0f, 6.0f, 7.0f, 8.0f);
            matrix.SetRow(0, newRow);
            AssertMatrix4Equal(Matrix4(5.0f, 6.0f, 7.0f, 8.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f), matrix);
        }

        TEST_METHOD(SetRow_InvalidIndex_ThrowsException)
        {
            Matrix4 matrix = Matrix4::Identity();
            Vector4 newRow(1.0f, 2.0f, 3.0f, 4.0f);
            Assert::ExpectException<runtime_error>([&]() {
                matrix.SetRow(4, newRow);
                });
        }

        // String Representation Tests
        TEST_METHOD(ToString_FormatsCorrectly)
        {
            Matrix4 matrix(1.5f, 2.5f, 3.5f, 4.5f,
                5.5f, 6.5f, 7.5f, 8.5f,
                9.5f, 10.5f, 11.5f, 12.5f,
                13.5f, 14.5f, 15.5f, 16.5f);
            string str = matrix.ToString();
            // Basic check - should contain the numbers
            Assert::IsTrue(str.find("1.5") != string::npos);
            Assert::IsTrue(str.find("6.5") != string::npos);
            Assert::IsTrue(str.find("11.5") != string::npos);
            Assert::IsTrue(str.find("16.5") != string::npos);
        }

        // Operator Tests
        TEST_METHOD(Equality_SameMatrices_ReturnsTrue)
        {
            Matrix4 a(1.0f, 2.0f, 3.0f, 4.0f,
                5.0f, 6.0f, 7.0f, 8.0f,
                9.0f, 10.0f, 11.0f, 12.0f,
                13.0f, 14.0f, 15.0f, 16.0f);
            Matrix4 b(1.0f, 2.0f, 3.0f, 4.0f,
                5.0f, 6.0f, 7.0f, 8.0f,
                9.0f, 10.0f, 11.0f, 12.0f,
                13.0f, 14.0f, 15.0f, 16.0f);
            Assert::IsTrue(a == b);
        }

        TEST_METHOD(Equality_DifferentMatrices_ReturnsFalse)
        {
            Matrix4 a = Matrix4::Identity();
            Matrix4 b = Matrix4::Scale(2.0f, 2.0f, 2.0f);
            Assert::IsFalse(a == b);
        }

        TEST_METHOD(Inequality_DifferentMatrices_ReturnsTrue)
        {
            Matrix4 a = Matrix4::Identity();
            Matrix4 b = Matrix4::Scale(2.0f, 2.0f, 2.0f);
            Assert::IsTrue(a != b);
        }

        TEST_METHOD(MatrixMultiplication_IdentityMatrix_ReturnsOriginal)
        {
            Matrix4 matrix = Matrix4::Scale(2.0f, 3.0f, 4.0f);
            Matrix4 identity = Matrix4::Identity();
            Matrix4 result = matrix * identity;
            AssertMatrix4Equal(matrix, result);
        }

        TEST_METHOD(MatrixMultiplication_ScaleAndTranslation_CombinesCorrectly)
        {
            Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
            Matrix4 translation = Matrix4::Translation(1.0f, 2.0f, 3.0f);
            Matrix4 result = translation * scale;

            // Verify by transforming a test vector
            Vector3 testVector(1.0f, 1.0f, 1.0f);
            Vector3 transformed = result * testVector;
            AssertVector3Equal(Vector3(3.0f, 5.0f, 7.0f), transformed); // (2*1+1, 3*1+2, 4*1+3)
        }

        TEST_METHOD(ScalarMultiplication_MultiplyByZero_ReturnsZeroMatrix)
        {
            Matrix4 matrix = Matrix4::Identity();
            Matrix4 result = matrix * 0.0f;
            AssertMatrix4Equal(Matrix4::Zero(), result);
        }

        TEST_METHOD(ScalarMultiplication_MultiplyByTwo_DoublesAllElements)
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

        TEST_METHOD(ScalarDivision_DivideByTwo_HalvesAllElements)
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

        TEST_METHOD(ScalarDivision_DivideByZero_ThrowsException)
        {
            Matrix4 matrix = Matrix4::Identity();
            Assert::ExpectException<runtime_error>([&]() {
                Matrix4 result = matrix / 0.0f;
                });
        }

        TEST_METHOD(Vector4Multiplication_IdentityMatrix_ReturnsOriginalVector)
        {
            Matrix4 identity = Matrix4::Identity();
            Vector4 vector(3.0f, 4.0f, 5.0f, 1.0f);
            Vector4 result = identity * vector;
            AssertVector4Equal(vector, result);
        }

        TEST_METHOD(Vector4Multiplication_ScaleMatrix_ScalesVector)
        {
            Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
            Vector4 vector(2.0f, 3.0f, 4.0f, 1.0f);
            Vector4 result = scale * vector;
            AssertVector4Equal(Vector4(4.0f, 9.0f, 16.0f, 1.0f), result);
        }

        TEST_METHOD(Vector4Multiplication_TranslationMatrix_TranslatesVector)
        {
            Matrix4 translation = Matrix4::Translation(1.0f, 2.0f, 3.0f);
            Vector4 vector(0.0f, 0.0f, 0.0f, 1.0f);
            Vector4 result = translation * vector;
            AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 1.0f), result);
        }

        TEST_METHOD(Vector3Multiplication_IdentityMatrix_ReturnsOriginalVector)
        {
            Matrix4 identity = Matrix4::Identity();
            Vector3 vector(3.0f, 4.0f, 5.0f);
            Vector3 result = identity * vector;
            AssertVector3Equal(vector, result);
        }

        TEST_METHOD(Vector3Multiplication_ScaleMatrix_ScalesVector)
        {
            Matrix4 scale = Matrix4::Scale(2.0f, 3.0f, 4.0f);
            Vector3 vector(2.0f, 3.0f, 4.0f);
            Vector3 result = scale * vector;
            AssertVector3Equal(Vector3(4.0f, 9.0f, 16.0f), result);
        }

        TEST_METHOD(Vector3Multiplication_TranslationMatrix_TranslatesVector)
        {
            Matrix4 translation = Matrix4::Translation(1.0f, 2.0f, 3.0f);
            Vector3 vector(2.0f, 3.0f, 4.0f);
            Vector3 result = translation * vector;
            AssertVector3Equal(Vector3(3.0f, 5.0f, 7.0f), result);
        }

        TEST_METHOD(Vector3Multiplication_RotationMatrix_RotatesVector)
        {
            Matrix4 rotation = Matrix4::RotationZ(90.0f);
            Vector3 vector(1.0f, 0.0f, 0.0f);
            Vector3 result = rotation * vector;
            AssertVector3Equal(Vector3(0.0f, 1.0f, 0.0f), result, 0.001f);
        }

        TEST_METHOD(IndexOperator_Index0_ReturnsFirstColumn)
        {
            Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
                5.0f, 6.0f, 7.0f, 8.0f,
                9.0f, 10.0f, 11.0f, 12.0f,
                13.0f, 14.0f, 15.0f, 16.0f);
            Vector4 column = matrix[0];
            AssertVector4Equal(Vector4(1.0f, 5.0f, 9.0f, 13.0f), column);
        }

        TEST_METHOD(IndexOperator_Index3_ReturnsFourthColumn)
        {
            Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
                5.0f, 6.0f, 7.0f, 8.0f,
                9.0f, 10.0f, 11.0f, 12.0f,
                13.0f, 14.0f, 15.0f, 16.0f);
            Vector4 column = matrix[3];
            AssertVector4Equal(Vector4(4.0f, 8.0f, 12.0f, 16.0f), column);
        }

        TEST_METHOD(IndexOperator_InvalidIndex_ThrowsException)
        {
            Matrix4 matrix = Matrix4::Identity();
            Assert::ExpectException<runtime_error>([&]() {
                Vector4 column = matrix[4];
                });
        }

        TEST_METHOD(Assignment_CopiesAllElements)
        {
            Matrix4 original(1.0f, 2.0f, 3.0f, 4.0f,
                5.0f, 6.0f, 7.0f, 8.0f,
                9.0f, 10.0f, 11.0f, 12.0f,
                13.0f, 14.0f, 15.0f, 16.0f);
            Matrix4 assigned;
            assigned = original;
            AssertMatrix4Equal(original, assigned);
        }

        TEST_METHOD(Assignment_SelfAssignment_RemainsUnchanged)
        {
            Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
                5.0f, 6.0f, 7.0f, 8.0f,
                9.0f, 10.0f, 11.0f, 12.0f,
                13.0f, 14.0f, 15.0f, 16.0f);
            Matrix4 original = matrix;
            matrix = matrix;
            AssertMatrix4Equal(original, matrix);
        }

        TEST_METHOD(GlobalScalarMultiplication_ScalarFirst_Works)
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
        TEST_METHOD(EdgeCase_VerySmallNumbers_HandledCorrectly)
        {
            Matrix4 matrix(1e-6f, 1e-6f, 1e-6f, 1e-6f,
                1e-6f, 1e-6f, 1e-6f, 1e-6f,
                1e-6f, 1e-6f, 1e-6f, 1e-6f,
                1e-6f, 1e-6f, 1e-6f, 1e-6f);
            Assert::IsFalse(matrix.IsZero()); // Should not be considered zero
        }

        TEST_METHOD(EdgeCase_VeryLargeNumbers_HandledCorrectly)
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

        TEST_METHOD(EdgeCase_NegativeValues_HandledCorrectly)
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
        TEST_METHOD(Property_TransposeOfTranspose_ReturnsOriginal)
        {
            Matrix4 matrix(1.0f, 2.0f, 3.0f, 4.0f,
                5.0f, 6.0f, 7.0f, 8.0f,
                9.0f, 10.0f, 11.0f, 12.0f,
                13.0f, 14.0f, 15.0f, 16.0f);
            Matrix4 doubleTransposed = matrix.Transposed().Transposed();
            AssertMatrix4Equal(matrix, doubleTransposed);
        }

        TEST_METHOD(Property_DeterminantOfTranspose_EqualToOriginal)
        {
            Matrix4 matrix = Matrix4::Translation(1.0f, 2.0f, 3.0f) *
                Matrix4::RotationZ(45.0f) *
                Matrix4::Scale(2.0f, 3.0f, 4.0f);
            float detOriginal = matrix.Determinant();
            float detTransposed = matrix.Transposed().Determinant();
            AssertFloatEqual(detOriginal, detTransposed, 0.001f);
        }

        TEST_METHOD(Property_InverseOfInverse_ReturnsOriginal)
        {
            Matrix4 matrix = Matrix4::Translation(1.0f, 2.0f, 3.0f) *
                Matrix4::RotationZ(30.0f) *
                Matrix4::Scale(2.0f, 3.0f, 4.0f);
            Matrix4 doubleInverse = matrix.Inverse().Inverse();
            AssertMatrix4Equal(matrix, doubleInverse, 0.001f);
        }

        TEST_METHOD(Property_ScaleInverse_IsReciprocalScale)
        {
            Matrix4 scale = Matrix4::Scale(4.0f, 2.0f, 8.0f);
            Matrix4 inverse = scale.Inverse();
            Matrix4 expected = Matrix4::Scale(0.25f, 0.5f, 0.125f);
            AssertMatrix4Equal(expected, inverse, 0.001f);
        }

        TEST_METHOD(Property_RotationInverse_IsNegativeRotation)
        {
            Matrix4 rotation = Matrix4::RotationZ(45.0f);
            Matrix4 inverse = rotation.Inverse();
            Matrix4 expected = Matrix4::RotationZ(-45.0f);
            AssertMatrix4Equal(expected, inverse, 0.001f);
        }

        TEST_METHOD(Property_TranslationInverse_IsNegativeTranslation)
        {
            Matrix4 translation = Matrix4::Translation(3.0f, 4.0f, 5.0f);
            Matrix4 inverse = translation.Inverse();
            Matrix4 expected = Matrix4::Translation(-3.0f, -4.0f, -5.0f);
            AssertMatrix4Equal(expected, inverse, 0.001f);
        }

        TEST_METHOD(Property_MatrixMultiplicationAssociativity)
        {
            Matrix4 a = Matrix4::Scale(2.0f, 2.0f, 2.0f);
            Matrix4 b = Matrix4::RotationZ(45.0f);
            Matrix4 c = Matrix4::Translation(1.0f, 2.0f, 3.0f);

            Matrix4 result1 = (a * b) * c;
            Matrix4 result2 = a * (b * c);
            AssertMatrix4Equal(result1, result2, 0.001f);
        }

        TEST_METHOD(Property_IdentityIsMultiplicativeIdentity)
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

        TEST_METHOD(Property_RotationMatricesAreOrthogonal)
        {
            Matrix4 rotationX = Matrix4::RotationX(37.0f);
            Matrix4 rotationY = Matrix4::RotationY(53.0f);
            Matrix4 rotationZ = Matrix4::RotationZ(71.0f);
            Matrix4 combinedRotation = rotationZ * rotationY * rotationX;

            Assert::IsTrue(rotationX.IsOrthogonal());
            Assert::IsTrue(rotationY.IsOrthogonal());
            Assert::IsTrue(rotationZ.IsOrthogonal());
            Assert::IsTrue(combinedRotation.IsOrthogonal());
        }

        TEST_METHOD(Property_TRS_DecompositionRoundTrip)
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

        TEST_METHOD(Property_TransformationOrder_TRS_IsCorrect)
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
    };
}