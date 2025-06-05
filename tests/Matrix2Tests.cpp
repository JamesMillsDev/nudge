#include <numbers>

#include "CppUnitTest.h"

#include "Nudge/MathF.hpp"
#include "Nudge/Matrix2.hpp"
#include "Nudge/Vector2.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using std::runtime_error;
using std::numbers::pi_v;

namespace Nudge
{
    TEST_CLASS(Matrix2Tests)
    {
    public:
        // Helper method for floating point comparison
        static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
        {
            Assert::IsTrue(MathF::Compare(expected, actual, tolerance), L"Float values are not equal within tolerance");
        }

        static void AssertVector2Equal(const Vector2& expected, const Vector2& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.x, actual.x, tolerance);
            AssertFloatEqual(expected.y, actual.y, tolerance);
        }

        static void AssertMatrix2Equal(const Matrix2& expected, const Matrix2& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.m11, actual.m11, tolerance);
            AssertFloatEqual(expected.m21, actual.m21, tolerance);
            AssertFloatEqual(expected.m12, actual.m12, tolerance);
            AssertFloatEqual(expected.m22, actual.m22, tolerance);
        }

        // Static Factory Method Tests
        TEST_METHOD(Identity_CreatesIdentityMatrix)
        {
            Matrix2 identity = Matrix2::Identity();
            AssertMatrix2Equal(Matrix2(1.0f, 0.0f, 0.0f, 1.0f), identity);
        }

        TEST_METHOD(Zero_CreatesZeroMatrix)
        {
            Matrix2 zero = Matrix2::Zero();
            AssertMatrix2Equal(Matrix2(0.0f, 0.0f, 0.0f, 0.0f), zero);
        }

        TEST_METHOD(Scale_TwoParameters_CreatesScaleMatrix)
        {
            Matrix2 scale = Matrix2::Scale(2.0f, 3.0f);
            AssertMatrix2Equal(Matrix2(2.0f, 0.0f, 0.0f, 3.0f), scale);
        }

        TEST_METHOD(Scale_Vector2Parameter_CreatesScaleMatrix)
        {
            Vector2 scaleVec(2.5f, 1.5f);
            Matrix2 scale = Matrix2::Scale(scaleVec);
            AssertMatrix2Equal(Matrix2(2.5f, 0.0f, 0.0f, 1.5f), scale);
        }

        TEST_METHOD(Rotation_ZeroDegrees_CreatesIdentityMatrix)
        {
            Matrix2 rotation = Matrix2::Rotation(0.0f);
            AssertMatrix2Equal(Matrix2::Identity(), rotation);
        }

        TEST_METHOD(Rotation_90Degrees_CreatesCorrectMatrix)
        {
            Matrix2 rotation = Matrix2::Rotation(90.0f);
            // 90 degrees: cos(90) = 0, sin(90) = 1
            // | 0 -1 |
            // | 1  0 |
            AssertMatrix2Equal(Matrix2(0.0f, -1.0f, 1.0f, 0.0f), rotation, 0.001f);
        }

        TEST_METHOD(Rotation_180Degrees_CreatesCorrectMatrix)
        {
            Matrix2 rotation = Matrix2::Rotation(180.0f);
            // 180 degrees: cos(180) = -1, sin(180) = 0
            // |-1  0 |
            // | 0 -1 |
            AssertMatrix2Equal(Matrix2(-1.0f, 0.0f, 0.0f, -1.0f), rotation, 0.001f);
        }

        TEST_METHOD(Rotation_NegativeAngle_CreatesClockwiseRotation)
        {
            Matrix2 rotation = Matrix2::Rotation(-90.0f);
            // -90 degrees: cos(-90) = 0, sin(-90) = -1
            // | 0  1 |
            // |-1  0 |
            AssertMatrix2Equal(Matrix2(0.0f, 1.0f, -1.0f, 0.0f), rotation, 0.001f);
        }

        // Constructor Tests
        TEST_METHOD(Constructor_Default_CreatesIdentityMatrix)
        {
            Matrix2 matrix;
            AssertMatrix2Equal(Matrix2::Identity(), matrix);
        }

        TEST_METHOD(Constructor_Scalar_CreatesScalarMatrix)
        {
            Matrix2 matrix(5.0f);
            AssertMatrix2Equal(Matrix2(5.0f, 0.0f, 0.0f, 5.0f), matrix);
        }

        TEST_METHOD(Constructor_FourFloats_CreatesMatrixWithSpecifiedValues)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            AssertMatrix2Equal(Matrix2(1.0f, 2.0f, 3.0f, 4.0f), matrix);
        }

        TEST_METHOD(Constructor_TwoVectors_CreatesMatrixFromColumns)
        {
            Vector2 col1(1.0f, 2.0f);
            Vector2 col2(3.0f, 4.0f);
            Matrix2 matrix(col1, col2);
            AssertMatrix2Equal(Matrix2(1.0f, 3.0f, 2.0f, 4.0f), matrix);
        }

        TEST_METHOD(Constructor_Array_CreatesMatrixFromColumnMajorArray)
        {
            float values[] = { 1.0f, 2.0f, 3.0f, 4.0f };
            Matrix2 matrix(values);
            // Column-major: [m11, m21, m12, m22]
            AssertMatrix2Equal(Matrix2(1.0f, 3.0f, 2.0f, 4.0f), matrix);
        }

        TEST_METHOD(Constructor_Copy_CreatesIdenticalMatrix)
        {
            Matrix2 original(1.5f, 2.5f, 3.5f, 4.5f);
            Matrix2 copy(original);
            AssertMatrix2Equal(original, copy);
        }

        // Mathematical Operation Tests
        TEST_METHOD(Determinant_IdentityMatrix_ReturnsOne)
        {
            Matrix2 identity = Matrix2::Identity();
            AssertFloatEqual(1.0f, identity.Determinant());
        }

        TEST_METHOD(Determinant_ZeroMatrix_ReturnsZero)
        {
            Matrix2 zero = Matrix2::Zero();
            AssertFloatEqual(0.0f, zero.Determinant());
        }

        TEST_METHOD(Determinant_GeneralMatrix_ReturnsCorrectValue)
        {
            Matrix2 matrix(2.0f, 3.0f, 4.0f, 5.0f);
            // det = 2*5 - 3*4 = 10 - 12 = -2
            AssertFloatEqual(-2.0f, matrix.Determinant());
        }

        TEST_METHOD(Determinant_SingularMatrix_ReturnsZero)
        {
            Matrix2 matrix(2.0f, 4.0f, 1.0f, 2.0f);
            // det = 2*2 - 4*1 = 4 - 4 = 0
            AssertFloatEqual(0.0f, matrix.Determinant());
        }

        TEST_METHOD(Transposed_IdentityMatrix_ReturnsIdentity)
        {
            Matrix2 identity = Matrix2::Identity();
            Matrix2 transposed = identity.Transposed();
            AssertMatrix2Equal(identity, transposed);
        }

        TEST_METHOD(Transposed_GeneralMatrix_SwapsOffDiagonalElements)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 transposed = matrix.Transposed();
            AssertMatrix2Equal(Matrix2(1.0f, 3.0f, 2.0f, 4.0f), transposed);
        }

        TEST_METHOD(Transpose_ModifiesOriginalMatrix)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 expected(1.0f, 3.0f, 2.0f, 4.0f);
            matrix.Transpose();
            AssertMatrix2Equal(expected, matrix);
        }

        TEST_METHOD(Inverse_IdentityMatrix_ReturnsIdentity)
        {
            Matrix2 identity = Matrix2::Identity();
            Matrix2 inverse = identity.Inverse();
            AssertMatrix2Equal(identity, inverse);
        }

        TEST_METHOD(Inverse_ScaleMatrix_ReturnsInverseScale)
        {
            Matrix2 scale = Matrix2::Scale(2.0f, 4.0f);
            Matrix2 inverse = scale.Inverse();
            AssertMatrix2Equal(Matrix2::Scale(0.5f, 0.25f), inverse);
        }

        TEST_METHOD(Inverse_GeneralMatrix_ReturnsCorrectInverse)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 inverse = matrix.Inverse();
            // For matrix [1 2; 3 4], det = -2
            // Inverse = (1/-2) * [4 -2; -3 1] = [-2 1; 1.5 -0.5]
            AssertMatrix2Equal(Matrix2(-2.0f, 1.0f, 1.5f, -0.5f), inverse);
        }

        TEST_METHOD(Inverse_SingularMatrix_ThrowsException)
        {
            Matrix2 singular(2.0f, 4.0f, 1.0f, 2.0f); // det = 0
            Assert::ExpectException<runtime_error>([&]() {
                Matrix2 inverse = singular.Inverse();
                });
        }

        TEST_METHOD(Inverse_MultiplyByOriginal_GivesIdentity)
        {
            Matrix2 matrix(3.0f, 1.0f, 2.0f, 4.0f);
            Matrix2 inverse = matrix.Inverse();
            Matrix2 product = matrix * inverse;
            AssertMatrix2Equal(Matrix2::Identity(), product, 0.001f);
        }

        // Utility Method Tests
        TEST_METHOD(IsIdentity_IdentityMatrix_ReturnsTrue)
        {
            Matrix2 identity = Matrix2::Identity();
            Assert::IsTrue(identity.IsIdentity());
        }

        TEST_METHOD(IsIdentity_NonIdentityMatrix_ReturnsFalse)
        {
            Matrix2 matrix(2.0f, 0.0f, 0.0f, 1.0f);
            Assert::IsFalse(matrix.IsIdentity());
        }

        TEST_METHOD(IsIdentity_NearIdentityMatrix_ReturnsTrue)
        {
            Matrix2 matrix(1.00001f, 0.00001f, 0.00001f, 0.99999f);
            Assert::IsTrue(matrix.IsIdentity(.00001f));
        }

        TEST_METHOD(IsZero_ZeroMatrix_ReturnsTrue)
        {
            Matrix2 zero = Matrix2::Zero();
            Assert::IsTrue(zero.IsZero());
        }

        TEST_METHOD(IsZero_NonZeroMatrix_ReturnsFalse)
        {
            Matrix2 matrix(0.1f, 0.0f, 0.0f, 0.0f);
            Assert::IsFalse(matrix.IsZero());
        }

        TEST_METHOD(IsZero_NearZeroMatrix_ReturnsTrue)
        {
            Matrix2 matrix(0.00001f, 0.00001f, 0.00001f, 0.00001f);
            Assert::IsTrue(matrix.IsZero(.00001f));
        }

        // Accessor Method Tests
        TEST_METHOD(GetColumn_Index0_ReturnsFirstColumn)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Vector2 column = matrix.GetColumn(0);
            AssertVector2Equal(Vector2(1.0f, 3.0f), column);
        }

        TEST_METHOD(GetColumn_Index1_ReturnsSecondColumn)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Vector2 column = matrix.GetColumn(1);
            AssertVector2Equal(Vector2(2.0f, 4.0f), column);
        }

        TEST_METHOD(GetColumn_InvalidIndex_ThrowsException)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Assert::ExpectException<runtime_error>([&]() {
                Vector2 column = matrix.GetColumn(2);
                });
        }

        TEST_METHOD(SetColumn_Index0_SetsFirstColumn)
        {
            Matrix2 matrix = Matrix2::Identity();
            Vector2 newColumn(5.0f, 6.0f);
            matrix.SetColumn(0, newColumn);
            AssertMatrix2Equal(Matrix2(5.0f, 0.0f, 6.0f, 1.0f), matrix);
        }

        TEST_METHOD(SetColumn_Index1_SetsSecondColumn)
        {
            Matrix2 matrix = Matrix2::Identity();
            Vector2 newColumn(7.0f, 8.0f);
            matrix.SetColumn(1, newColumn);
            AssertMatrix2Equal(Matrix2(1.0f, 7.0f, 0.0f, 8.0f), matrix);
        }

        TEST_METHOD(SetColumn_InvalidIndex_ThrowsException)
        {
            Matrix2 matrix = Matrix2::Identity();
            Vector2 newColumn(1.0f, 2.0f);
            Assert::ExpectException<runtime_error>([&]() {
                matrix.SetColumn(2, newColumn);
                });
        }

        TEST_METHOD(GetRow_Index0_ReturnsFirstRow)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Vector2 row = matrix.GetRow(0);
            AssertVector2Equal(Vector2(1.0f, 2.0f), row);
        }

        TEST_METHOD(GetRow_Index1_ReturnsSecondRow)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Vector2 row = matrix.GetRow(1);
            AssertVector2Equal(Vector2(3.0f, 4.0f), row);
        }

        TEST_METHOD(GetRow_InvalidIndex_ThrowsException)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Assert::ExpectException<runtime_error>([&]() {
                Vector2 row = matrix.GetRow(2);
                });
        }

        TEST_METHOD(SetRow_Index0_SetsFirstRow)
        {
            Matrix2 matrix = Matrix2::Identity();
            Vector2 newRow(5.0f, 6.0f);
            matrix.SetRow(0, newRow);
            AssertMatrix2Equal(Matrix2(5.0f, 6.0f, 0.0f, 1.0f), matrix);
        }

        TEST_METHOD(SetRow_Index1_SetsSecondRow)
        {
            Matrix2 matrix = Matrix2::Identity();
            Vector2 newRow(7.0f, 8.0f);
            matrix.SetRow(1, newRow);
            AssertMatrix2Equal(Matrix2(1.0f, 0.0f, 7.0f, 8.0f), matrix);
        }

        TEST_METHOD(SetRow_InvalidIndex_ThrowsException)
        {
            Matrix2 matrix = Matrix2::Identity();
            Vector2 newRow(1.0f, 2.0f);
            Assert::ExpectException<runtime_error>([&]() {
                matrix.SetRow(2, newRow);
                });
        }

        // String Representation Tests
        TEST_METHOD(ToString_FormatsCorrectly)
        {
            Matrix2 matrix(1.5f, 2.5f, 3.5f, 4.5f);
            string str = matrix.ToString();
            // Basic check - should contain the numbers
            Assert::IsTrue(str.find("1.5") != string::npos);
            Assert::IsTrue(str.find("2.5") != string::npos);
            Assert::IsTrue(str.find("3.5") != string::npos);
            Assert::IsTrue(str.find("4.5") != string::npos);
        }

        // Operator Tests
        TEST_METHOD(Equality_SameMatrices_ReturnsTrue)
        {
            Matrix2 a(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 b(1.0f, 2.0f, 3.0f, 4.0f);
            Assert::IsTrue(a == b);
        }

        TEST_METHOD(Equality_DifferentMatrices_ReturnsFalse)
        {
            Matrix2 a(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 b(1.0f, 2.0f, 3.0f, 5.0f);
            Assert::IsFalse(a == b);
        }

        TEST_METHOD(Inequality_DifferentMatrices_ReturnsTrue)
        {
            Matrix2 a(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 b(1.0f, 2.0f, 3.0f, 5.0f);
            Assert::IsTrue(a != b);
        }

        TEST_METHOD(MatrixMultiplication_IdentityMatrix_ReturnsOriginal)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 identity = Matrix2::Identity();
            Matrix2 result = matrix * identity;
            AssertMatrix2Equal(matrix, result);
        }

        TEST_METHOD(MatrixMultiplication_GeneralCase_ReturnsCorrectProduct)
        {
            Matrix2 a(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 b(5.0f, 6.0f, 7.0f, 8.0f);
            Matrix2 result = a * b;
            // [1 2] * [5 6] = [1*5+2*7  1*6+2*8] = [19 22]
            // [3 4]   [7 8]   [3*5+4*7  3*6+4*8]   [43 50]
            AssertMatrix2Equal(Matrix2(19.0f, 22.0f, 43.0f, 50.0f), result);
        }

        TEST_METHOD(ScalarMultiplication_MultiplyByZero_ReturnsZeroMatrix)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 result = matrix * 0.0f;
            AssertMatrix2Equal(Matrix2::Zero(), result);
        }

        TEST_METHOD(ScalarMultiplication_MultiplyByTwo_DoublesAllElements)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 result = matrix * 2.0f;
            AssertMatrix2Equal(Matrix2(2.0f, 4.0f, 6.0f, 8.0f), result);
        }

        TEST_METHOD(ScalarDivision_DivideByTwo_HalvesAllElements)
        {
            Matrix2 matrix(2.0f, 4.0f, 6.0f, 8.0f);
            Matrix2 result = matrix / 2.0f;
            AssertMatrix2Equal(Matrix2(1.0f, 2.0f, 3.0f, 4.0f), result);
        }

        TEST_METHOD(ScalarDivision_DivideByZero_ThrowsException)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Assert::ExpectException<runtime_error>([&]() {
                Matrix2 result = matrix / 0.0f;
                });
        }

        TEST_METHOD(VectorMultiplication_IdentityMatrix_ReturnsOriginalVector)
        {
            Matrix2 identity = Matrix2::Identity();
            Vector2 vector(3.0f, 4.0f);
            Vector2 result = identity * vector;
            AssertVector2Equal(vector, result);
        }

        TEST_METHOD(VectorMultiplication_ScaleMatrix_ScalesVector)
        {
            Matrix2 scale = Matrix2::Scale(2.0f, 3.0f);
            Vector2 vector(4.0f, 5.0f);
            Vector2 result = scale * vector;
            AssertVector2Equal(Vector2(8.0f, 15.0f), result);
        }

        TEST_METHOD(VectorMultiplication_RotationMatrix_RotatesVector)
        {
            Matrix2 rotation = Matrix2::Rotation(90.0f);
            Vector2 vector(1.0f, 0.0f);
            Vector2 result = rotation * vector;
            AssertVector2Equal(Vector2(0.0f, 1.0f), result, 0.001f);
        }

        TEST_METHOD(VectorMultiplication_GeneralCase_ReturnsCorrectResult)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Vector2 vector(5.0f, 6.0f);
            Vector2 result = matrix * vector;
            // [1 2] * [5] = [1*5+2*6] = [17]
            // [3 4]   [6]   [3*5+4*6]   [39]
            AssertVector2Equal(Vector2(17.0f, 39.0f), result);
        }

        TEST_METHOD(IndexOperator_Index0_ReturnsFirstColumn)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Vector2 column = matrix[0];
            AssertVector2Equal(Vector2(1.0f, 3.0f), column);
        }

        TEST_METHOD(IndexOperator_Index1_ReturnsSecondColumn)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Vector2 column = matrix[1];
            AssertVector2Equal(Vector2(2.0f, 4.0f), column);
        }

        TEST_METHOD(IndexOperator_InvalidIndex_ThrowsException)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Assert::ExpectException<runtime_error>([&]() {
                Vector2 column = matrix[2];
                });
        }

        TEST_METHOD(Assignment_CopiesAllElements)
        {
            Matrix2 original(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 assigned;
            assigned = original;
            AssertMatrix2Equal(original, assigned);
        }

        TEST_METHOD(Assignment_SelfAssignment_RemainsUnchanged)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 original = matrix;
            matrix = matrix;
            AssertMatrix2Equal(original, matrix);
        }

        TEST_METHOD(GlobalScalarMultiplication_ScalarFirst_Works)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 result = 3.0f * matrix;
            AssertMatrix2Equal(Matrix2(3.0f, 6.0f, 9.0f, 12.0f), result);
        }

        // Edge Case Tests
        TEST_METHOD(EdgeCase_VerySmallNumbers_HandledCorrectly)
        {
            Matrix2 matrix(1e-6f, 1e-6f, 1e-6f, 1e-6f);
            Assert::IsFalse(matrix.IsZero()); // Should not be considered zero
        }

        TEST_METHOD(EdgeCase_VeryLargeNumbers_HandledCorrectly)
        {
            Matrix2 matrix(1e6f, 1e6f, 1e6f, 1e6f);
            Matrix2 scaled = matrix * 0.5f;
            AssertMatrix2Equal(Matrix2(5e5f, 5e5f, 5e5f, 5e5f), scaled, 1.0f);
        }

        TEST_METHOD(EdgeCase_NegativeValues_HandledCorrectly)
        {
            Matrix2 matrix(-1.0f, -2.0f, -3.0f, -4.0f);
            Matrix2 negated = matrix * -1.0f;
            AssertMatrix2Equal(Matrix2(1.0f, 2.0f, 3.0f, 4.0f), negated);
        }

        // Mathematical Property Tests
        TEST_METHOD(Property_TransposeOfTranspose_ReturnsOriginal)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 doubleTransposed = matrix.Transposed().Transposed();
            AssertMatrix2Equal(matrix, doubleTransposed);
        }

        TEST_METHOD(Property_DeterminantOfTranspose_EqualToOriginal)
        {
            Matrix2 matrix(2.0f, 3.0f, 1.0f, 4.0f);
            float detOriginal = matrix.Determinant();
            float detTransposed = matrix.Transposed().Determinant();
            AssertFloatEqual(detOriginal, detTransposed);
        }

        TEST_METHOD(Property_InverseOfInverse_ReturnsOriginal)
        {
            Matrix2 matrix(2.0f, 1.0f, 1.0f, 3.0f);
            Matrix2 doubleInverse = matrix.Inverse().Inverse();
            AssertMatrix2Equal(matrix, doubleInverse, 0.001f);
        }

        TEST_METHOD(Property_ScaleInverse_IsReciprocalScale)
        {
            Matrix2 scale = Matrix2::Scale(4.0f, 2.0f);
            Matrix2 inverse = scale.Inverse();
            Matrix2 expected = Matrix2::Scale(0.25f, 0.5f);
            AssertMatrix2Equal(expected, inverse);
        }

        TEST_METHOD(Property_RotationInverse_IsNegativeRotation)
        {
            Matrix2 rotation = Matrix2::Rotation(45.0f);
            Matrix2 inverse = rotation.Inverse();
            Matrix2 expected = Matrix2::Rotation(-45.0f);
            AssertMatrix2Equal(expected, inverse, 0.001f);
        }

        TEST_METHOD(Property_MatrixMultiplicationAssociativity)
        {
            Matrix2 a(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 b(5.0f, 6.0f, 7.0f, 8.0f);
            Matrix2 c(9.0f, 10.0f, 11.0f, 12.0f);

            Matrix2 result1 = (a * b) * c;
            Matrix2 result2 = a * (b * c);
            AssertMatrix2Equal(result1, result2, 0.001f);
        }

        TEST_METHOD(Property_IdentityIsMultiplicativeIdentity)
        {
            Matrix2 matrix(1.0f, 2.0f, 3.0f, 4.0f);
            Matrix2 identity = Matrix2::Identity();

            Matrix2 leftProduct = identity * matrix;
            Matrix2 rightProduct = matrix * identity;

            AssertMatrix2Equal(matrix, leftProduct);
            AssertMatrix2Equal(matrix, rightProduct);
        }
        
    };
}