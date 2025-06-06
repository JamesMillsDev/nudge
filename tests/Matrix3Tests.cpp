#include "CppUnitTest.h"
#include "Nudge/Matrix2.hpp"
#include "Nudge/Matrix3.hpp"
#include "Nudge/Vector2.hpp"
#include "Nudge/Vector3.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

// Add ToString specializations for MSTest
namespace Microsoft
{
	namespace VisualStudio
	{
		namespace CppUnitTestFramework
		{
			template<>
			std::wstring ToString<Nudge::Vector3>(const Nudge::Vector3& vector)
			{
				std::wostringstream oss;
				oss << L"Vector3(" << vector.x << L", " << vector.y << L", " << vector.z << L")";
				return oss.str();
			}

			template<>
			std::wstring ToString<Nudge::Vector2>(const Nudge::Vector2& vector)
			{
				std::wostringstream oss;
				oss << L"Vector2(" << vector.x << L", " << vector.y << L")";
				return oss.str();
			}

			template<>
			std::wstring ToString<Nudge::Matrix3>(const Nudge::Matrix3& matrix)
			{
				std::wostringstream oss;
				oss << L"Matrix3(\n";
				oss << L"  " << matrix.m11 << L", " << matrix.m12 << L", " << matrix.m13 << L"\n";
				oss << L"  " << matrix.m21 << L", " << matrix.m22 << L", " << matrix.m23 << L"\n";
				oss << L"  " << matrix.m31 << L", " << matrix.m32 << L", " << matrix.m33 << L"\n";
				oss << L")";
				return oss.str();
			}
		}
	}
}

namespace Nudge
{
	TEST_CLASS(Matrix3Tests)
	{
	public:
		static void AssertMatrixNear(const Matrix3& actual, const Matrix3& expected, float tolerance = FLT_EPSILON)
		{
			Assert::AreEqual(expected.m11, actual.m11, tolerance);
			Assert::AreEqual(expected.m21, actual.m21, tolerance);
			Assert::AreEqual(expected.m31, actual.m31, tolerance);
			Assert::AreEqual(expected.m12, actual.m12, tolerance);
			Assert::AreEqual(expected.m22, actual.m22, tolerance);
			Assert::AreEqual(expected.m32, actual.m32, tolerance);
			Assert::AreEqual(expected.m13, actual.m13, tolerance);
			Assert::AreEqual(expected.m23, actual.m23, tolerance);
			Assert::AreEqual(expected.m33, actual.m33, tolerance);
		}

	public:
		// ============================================================================
		// STATIC FACTORY METHODS TESTS
		// ============================================================================

		TEST_METHOD(Identity_CreatesIdentityMatrix)
		{
			Matrix3 identity = Matrix3::Identity();
			Matrix3 expected(1.0f, 0.0f, 0.0f,
			                 0.0f, 1.0f, 0.0f,
			                 0.0f, 0.0f, 1.0f);

			AssertMatrixNear(identity, expected);
		}

		TEST_METHOD(Zero_CreatesZeroMatrix)
		{
			Matrix3 zero = Matrix3::Zero();
			Matrix3 expected(0.0f, 0.0f, 0.0f,
			                 0.0f, 0.0f, 0.0f,
			                 0.0f, 0.0f, 0.0f);

			AssertMatrixNear(zero, expected);
		}

		TEST_METHOD(Scale_ThreeParameters_CreatesScaleMatrix)
		{
			Matrix3 scale = Matrix3::Scale(2.0f, 3.0f, 4.0f);
			Matrix3 expected(2.0f, 0.0f, 0.0f,
			                 0.0f, 3.0f, 0.0f,
			                 0.0f, 0.0f, 4.0f);

			AssertMatrixNear(scale, expected);
		}

		TEST_METHOD(Scale_Vector3Parameter_CreatesScaleMatrix)
		{
			Vector3 scaleVec(2.0f, 3.0f, 4.0f);
			Matrix3 scale = Matrix3::Scale(scaleVec);
			Matrix3 expected(2.0f, 0.0f, 0.0f,
			                 0.0f, 3.0f, 0.0f,
			                 0.0f, 0.0f, 4.0f);

			AssertMatrixNear(scale, expected);
		}

		TEST_METHOD(RotationX_90Degrees_CreatesCorrectRotation)
		{
			Matrix3 rotX = Matrix3::RotationX(90.0f);
			Matrix3 expected(1.0f, 0.0f, 0.0f,
			                 0.0f, 0.0f, -1.0f,
			                 0.0f, 1.0f, 0.0f);

			AssertMatrixNear(rotX, expected, 1e-5f);
		}

		TEST_METHOD(RotationX_180Degrees_CreatesCorrectRotation)
		{
			Matrix3 rotX = Matrix3::RotationX(180.0f);
			Matrix3 expected(1.0f, 0.0f, 0.0f,
			                 0.0f, -1.0f, 0.0f,
			                 0.0f, 0.0f, -1.0f);

			AssertMatrixNear(rotX, expected, 1e-5f);
		}

		TEST_METHOD(RotationY_90Degrees_CreatesCorrectRotation)
		{
			Matrix3 rotY = Matrix3::RotationY(90.0f);
			Matrix3 expected(0.0f, 0.0f, 1.0f,
			                 0.0f, 1.0f, 0.0f,
			                 -1.0f, 0.0f, 0.0f);

			AssertMatrixNear(rotY, expected, 1e-5f);
		}

		TEST_METHOD(RotationY_180Degrees_CreatesCorrectRotation)
		{
			Matrix3 rotY = Matrix3::RotationY(180.0f);
			Matrix3 expected(-1.0f, 0.0f, 0.0f,
			                 0.0f, 1.0f, 0.0f,
			                 0.0f, 0.0f, -1.0f);

			AssertMatrixNear(rotY, expected, 1e-5f);
		}

		TEST_METHOD(RotationZ_90Degrees_CreatesCorrectRotation)
		{
			Matrix3 rotZ = Matrix3::RotationZ(90.0f);
			Matrix3 expected(0.0f, -1.0f, 0.0f,
			                 1.0f, 0.0f, 0.0f,
			                 0.0f, 0.0f, 1.0f);

			AssertMatrixNear(rotZ, expected, 1e-5f);
		}

		TEST_METHOD(RotationZ_180Degrees_CreatesCorrectRotation)
		{
			Matrix3 rotZ = Matrix3::RotationZ(180.0f);
			Matrix3 expected(-1.0f, 0.0f, 0.0f,
			                 0.0f, -1.0f, 0.0f,
			                 0.0f, 0.0f, 1.0f);

			AssertMatrixNear(rotZ, expected, 1e-5f);
		}

		TEST_METHOD(Rotation_EulerAngles_OnlyZRotation_CreatesCorrectMatrix)
		{
			Vector3 euler(0.0f, 0.0f, 90.0f); // Only Z rotation
			Matrix3 rot = Matrix3::Rotation(euler);
			Matrix3 expected(0.0f, -1.0f, 0.0f,
			                 1.0f, 0.0f, 0.0f,
			                 0.0f, 0.0f, 1.0f);

			AssertMatrixNear(rot, expected, 1e-5f);
		}

		TEST_METHOD(Rotation_AxisAngle_XAxis90Degrees_CreatesCorrectMatrix)
		{
			Vector3 axis(1.0f, 0.0f, 0.0f);
			Matrix3 rot = Matrix3::Rotation(axis, 90.0f);
			Matrix3 expected(1.0f, 0.0f, 0.0f,
			                 0.0f, 0.0f, -1.0f,
			                 0.0f, 1.0f, 0.0f);

			AssertMatrixNear(rot, expected, 1e-5f);
		}

		TEST_METHOD(Rotation_AxisAngle_ArbitraryAxis_CreatesValidRotationMatrix)
		{
			Vector3 axis(1.0f, 1.0f, 1.0f);
			Matrix3 rot = Matrix3::Rotation(axis, 120.0f);

			// Check that it's a valid rotation matrix (orthogonal with determinant 1)
			Assert::IsTrue(rot.IsOrthogonal());
			Assert::AreEqual(1.0f, rot.Determinant(), 1e-5f);
		}

		TEST_METHOD(Translation_TwoParameters_CreatesTranslationMatrix)
		{
			Matrix3 trans = Matrix3::Translation(3.0f, 4.0f);
			Matrix3 expected(1.0f, 0.0f, 3.0f,
			                 0.0f, 1.0f, 4.0f,
			                 0.0f, 0.0f, 1.0f);

			AssertMatrixNear(trans, expected);
		}

		TEST_METHOD(Translation_Vector2Parameter_CreatesTranslationMatrix)
		{
			Vector2 transVec(3.0f, 4.0f);
			Matrix3 trans = Matrix3::Translation(transVec);
			Matrix3 expected(1.0f, 0.0f, 3.0f,
			                 0.0f, 1.0f, 4.0f,
			                 0.0f, 0.0f, 1.0f);

			AssertMatrixNear(trans, expected);
		}

		// ============================================================================
		// CONSTRUCTOR TESTS
		// ============================================================================

		TEST_METHOD(DefaultConstructor_CreatesIdentityMatrix)
		{
			Matrix3 matrix;
			Matrix3 expected = Matrix3::Identity();

			AssertMatrixNear(matrix, expected);
		}

		TEST_METHOD(ScalarConstructor_CreatesScalarMatrix)
		{
			Matrix3 matrix(5.0f);
			Matrix3 expected(5.0f, 0.0f, 0.0f,
			                 0.0f, 5.0f, 0.0f,
			                 0.0f, 0.0f, 5.0f);

			AssertMatrixNear(matrix, expected);
		}

		TEST_METHOD(ElementWiseConstructor_CreatesCorrectMatrix)
		{
			Matrix3 matrix(1.0f, 2.0f, 3.0f,
			               4.0f, 5.0f, 6.0f,
			               7.0f, 8.0f, 9.0f);

			// Verify column-major storage
			Assert::AreEqual(1.0f, matrix.m11);
			Assert::AreEqual(4.0f, matrix.m21);
			Assert::AreEqual(7.0f, matrix.m31);
			Assert::AreEqual(2.0f, matrix.m12);
			Assert::AreEqual(5.0f, matrix.m22);
			Assert::AreEqual(8.0f, matrix.m32);
			Assert::AreEqual(3.0f, matrix.m13);
			Assert::AreEqual(6.0f, matrix.m23);
			Assert::AreEqual(9.0f, matrix.m33);
		}

		TEST_METHOD(ColumnConstructor_CreatesCorrectMatrix)
		{
			Vector3 col1(1.0f, 4.0f, 7.0f);
			Vector3 col2(2.0f, 5.0f, 8.0f);
			Vector3 col3(3.0f, 6.0f, 9.0f);
			Matrix3 matrix(col1, col2, col3);

			AssertMatrixNear(matrix, Matrix3(1.0f, 2.0f, 3.0f,
			                                 4.0f, 5.0f, 6.0f,
			                                 7.0f, 8.0f, 9.0f));
		}

		TEST_METHOD(ArrayConstructor_CreatesCorrectMatrix)
		{
			float values[9] = { 1.0f, 4.0f, 7.0f, 2.0f, 5.0f, 8.0f, 3.0f, 6.0f, 9.0f };
			Matrix3 matrix(values);

			AssertMatrixNear(matrix, Matrix3(1.0f, 2.0f, 3.0f,
			                                 4.0f, 5.0f, 6.0f,
			                                 7.0f, 8.0f, 9.0f));
		}

		TEST_METHOD(Matrix2Constructor_ExtendsMatrix2ToMatrix3)
		{
			Matrix2 matrix2(1.0f, 2.0f, 3.0f, 4.0f);
			Matrix3 matrix3(matrix2);
			Matrix3 expected(1.0f, 2.0f, 0.0f,
			                 3.0f, 4.0f, 0.0f,
			                 0.0f, 0.0f, 1.0f);

			AssertMatrixNear(matrix3, expected);
		}

		TEST_METHOD(CopyConstructor_CreatesIdenticalMatrix)
		{
			Matrix3 original(1.0f, 2.0f, 3.0f,
			                 4.0f, 5.0f, 6.0f,
			                 7.0f, 8.0f, 9.0f);
			Matrix3 copy(original);

			AssertMatrixNear(copy, original);
		}

		// ============================================================================
		// DETERMINANT TESTS
		// ============================================================================

		TEST_METHOD(Determinant_IdentityMatrix_ReturnsOne)
		{
			Matrix3 identity = Matrix3::Identity();
			Assert::AreEqual(1.0f, identity.Determinant(), FLT_EPSILON);
		}

		TEST_METHOD(Determinant_ZeroMatrix_ReturnsZero)
		{
			Matrix3 zero = Matrix3::Zero();
			Assert::AreEqual(0.0f, zero.Determinant(), FLT_EPSILON);
		}

		TEST_METHOD(Determinant_KnownMatrix_ReturnsCorrectValue)
		{
			Matrix3 matrix(1.0f, 2.0f, 3.0f,
			               0.0f, 1.0f, 4.0f,
			               5.0f, 6.0f, 0.0f);
			// det = 1*(1*0 - 4*6) - 2*(0*0 - 4*5) + 3*(0*6 - 1*5)
			// det = 1*(-24) - 2*(-20) + 3*(-5) = -24 + 40 - 15 = 1
			Assert::AreEqual(1.0f, matrix.Determinant(), FLT_EPSILON);
		}

		TEST_METHOD(Determinant_RotationMatrix_ReturnsOne)
		{
			Matrix3 rotation = Matrix3::RotationZ(45.0f);
			Assert::AreEqual(1.0f, rotation.Determinant(), 1e-5f);
		}

		TEST_METHOD(Determinant_ScaleMatrix_ReturnsProduct)
		{
			Matrix3 scale = Matrix3::Scale(2.0f, 3.0f, 4.0f);
			Assert::AreEqual(24.0f, scale.Determinant(), FLT_EPSILON); // 2 * 3 * 4 = 24
		}

		// ============================================================================
		// TRANSPOSE TESTS
		// ============================================================================

		TEST_METHOD(Transposed_CreatesTransposedCopy)
		{
			Matrix3 original(1.0f, 2.0f, 3.0f,
			                 4.0f, 5.0f, 6.0f,
			                 7.0f, 8.0f, 9.0f);
			Matrix3 transposed = original.Transposed();
			Matrix3 expected(1.0f, 4.0f, 7.0f,
			                 2.0f, 5.0f, 8.0f,
			                 3.0f, 6.0f, 9.0f);

			AssertMatrixNear(transposed, expected);
			// Verify original is unchanged
			AssertMatrixNear(original, Matrix3(1.0f, 2.0f, 3.0f,
			                                   4.0f, 5.0f, 6.0f,
			                                   7.0f, 8.0f, 9.0f));
		}

		TEST_METHOD(Transpose_ModifiesMatrixInPlace)
		{
			Matrix3 matrix(1.0f, 2.0f, 3.0f,
			               4.0f, 5.0f, 6.0f,
			               7.0f, 8.0f, 9.0f);
			matrix.Transpose();
			Matrix3 expected(1.0f, 4.0f, 7.0f,
			                 2.0f, 5.0f, 8.0f,
			                 3.0f, 6.0f, 9.0f);

			AssertMatrixNear(matrix, expected);
		}

		TEST_METHOD(Transpose_IdentityMatrix_RemainsIdentity)
		{
			Matrix3 identity = Matrix3::Identity();
			identity.Transpose();
			AssertMatrixNear(identity, Matrix3::Identity());
		}

		// ============================================================================
		// INVERSE TESTS
		// ============================================================================

		TEST_METHOD(Inverse_IdentityMatrix_ReturnsIdentity)
		{
			Matrix3 identity = Matrix3::Identity();
			Matrix3 inverse = identity.Inverse();
			AssertMatrixNear(inverse, Matrix3::Identity());
		}

		TEST_METHOD(Inverse_InvertibleMatrix_ReturnsCorrectInverse)
		{
			Matrix3 matrix(2.0f, 0.0f, 0.0f,
			               0.0f, 3.0f, 0.0f,
			               0.0f, 0.0f, 4.0f);
			Matrix3 inverse = matrix.Inverse();
			Matrix3 expected(0.5f, 0.0f, 0.0f,
			                 0.0f, 1.0f / 3.0f, 0.0f,
			                 0.0f, 0.0f, 0.25f);

			AssertMatrixNear(inverse, expected, 1e-5f);
		}

		TEST_METHOD(Inverse_MatrixTimesInverse_GivesIdentity)
		{
			Matrix3 matrix(1.0f, 2.0f, 3.0f,
			               0.0f, 1.0f, 4.0f,
			               5.0f, 6.0f, 0.0f);
			Matrix3 inverse = matrix.Inverse();
			Matrix3 product = matrix * inverse;

			AssertMatrixNear(product, Matrix3::Identity(), 1e-5f);
		}

		TEST_METHOD(Inverse_RotationMatrix_ReturnsTranspose)
		{
			Matrix3 rotation = Matrix3::RotationZ(30.0f);
			Matrix3 inverse = rotation.Inverse();
			Matrix3 transpose = rotation.Transposed();

			AssertMatrixNear(inverse, transpose, 1e-5f);
		}

		TEST_METHOD(Inverse_SingularMatrix_ThrowsException)
		{
			Matrix3 singular = Matrix3::Zero(); // Determinant = 0
			Assert::ExpectException<std::runtime_error>([&]()
			{
				singular.Inverse();
			});
		}

		// ============================================================================
		// BOOLEAN TESTS
		// ============================================================================

		TEST_METHOD(IsIdentity_IdentityMatrix_ReturnsTrue)
		{
			Matrix3 identity = Matrix3::Identity();
			Assert::IsTrue(identity.IsIdentity());
		}

		TEST_METHOD(IsIdentity_NonIdentityMatrix_ReturnsFalse)
		{
			Matrix3 matrix(2.0f, 0.0f, 0.0f,
			               0.0f, 1.0f, 0.0f,
			               0.0f, 0.0f, 1.0f);
			Assert::IsFalse(matrix.IsIdentity());
		}

		TEST_METHOD(IsIdentity_NearIdentityMatrix_ReturnsTrue)
		{
			Matrix3 nearIdentity(1.0f + 1e-8f, 0.0f, 0.0f,
			                     0.0f, 1.0f, 0.0f,
			                     0.0f, 0.0f, 1.0f);
			Assert::IsTrue(nearIdentity.IsIdentity(1e-7f));
		}

		TEST_METHOD(IsZero_ZeroMatrix_ReturnsTrue)
		{
			Matrix3 zero = Matrix3::Zero();
			Assert::IsTrue(zero.IsZero());
		}

		TEST_METHOD(IsZero_NonZeroMatrix_ReturnsFalse)
		{
			Matrix3 matrix(1.0f, 0.0f, 0.0f,
			               0.0f, 0.0f, 0.0f,
			               0.0f, 0.0f, 0.0f);
			Assert::IsFalse(matrix.IsZero());
		}

		TEST_METHOD(IsOrthogonal_RotationMatrix_ReturnsTrue)
		{
			Matrix3 rotation = Matrix3::RotationZ(45.0f);
			Assert::IsTrue(rotation.IsOrthogonal());
		}

		TEST_METHOD(IsOrthogonal_ScaleMatrix_ReturnsFalse)
		{
			Matrix3 scale = Matrix3::Scale(2.0f, 2.0f, 2.0f);
			Assert::IsFalse(scale.IsOrthogonal());
		}

		TEST_METHOD(IsOrthogonal_IdentityMatrix_ReturnsTrue)
		{
			Matrix3 identity = Matrix3::Identity();
			Assert::IsTrue(identity.IsOrthogonal());
		}

		// ============================================================================
		// COLUMN/ROW ACCESS TESTS
		// ============================================================================

		TEST_METHOD(GetColumn_ValidIndices_ReturnsCorrectColumns)
		{
			Matrix3 matrix(1.0f, 2.0f, 3.0f,
			               4.0f, 5.0f, 6.0f,
			               7.0f, 8.0f, 9.0f);

			Vector3 col0 = matrix.GetColumn(0);
			Vector3 col1 = matrix.GetColumn(1);
			Vector3 col2 = matrix.GetColumn(2);

			Assert::AreEqual(Vector3(1.0f, 4.0f, 7.0f), col0);
			Assert::AreEqual(Vector3(2.0f, 5.0f, 8.0f), col1);
			Assert::AreEqual(Vector3(3.0f, 6.0f, 9.0f), col2);
		}

		TEST_METHOD(GetColumn_InvalidIndex_ThrowsException)
		{
			Matrix3 matrix = Matrix3::Identity();
			Assert::ExpectException<std::runtime_error>([&]()
			{
				matrix.GetColumn(3);
			});
			Assert::ExpectException<std::runtime_error>([&]()
			{
				matrix.GetColumn(-1);
			});
		}

		TEST_METHOD(SetColumn_ValidIndices_SetsCorrectColumns)
		{
			Matrix3 matrix = Matrix3::Identity();
			Vector3 newCol(10.0f, 20.0f, 30.0f);

			matrix.SetColumn(1, newCol);

			Assert::AreEqual(newCol, matrix.GetColumn(1));
			Assert::AreEqual(Vector3(1.0f, 0.0f, 0.0f), matrix.GetColumn(0));
			Assert::AreEqual(Vector3(0.0f, 0.0f, 1.0f), matrix.GetColumn(2));
		}

		TEST_METHOD(SetColumn_InvalidIndex_ThrowsException)
		{
			Matrix3 matrix = Matrix3::Identity();
			Vector3 newCol(1.0f, 2.0f, 3.0f);

			Assert::ExpectException<std::runtime_error>([&]()
			{
				matrix.SetColumn(3, newCol);
			});
			Assert::ExpectException<std::runtime_error>([&]()
			{
				matrix.SetColumn(-1, newCol);
			});
		}

		TEST_METHOD(GetRow_ValidIndices_ReturnsCorrectRows)
		{
			Matrix3 matrix(1.0f, 2.0f, 3.0f,
			               4.0f, 5.0f, 6.0f,
			               7.0f, 8.0f, 9.0f);

			Vector3 row0 = matrix.GetRow(0);
			Vector3 row1 = matrix.GetRow(1);
			Vector3 row2 = matrix.GetRow(2);

			Assert::AreEqual(Vector3(1.0f, 2.0f, 3.0f), row0);
			Assert::AreEqual(Vector3(4.0f, 5.0f, 6.0f), row1);
			Assert::AreEqual(Vector3(7.0f, 8.0f, 9.0f), row2);
		}

		TEST_METHOD(GetRow_InvalidIndex_ThrowsException)
		{
			Matrix3 matrix = Matrix3::Identity();
			Assert::ExpectException<std::runtime_error>([&]()
			{
				matrix.GetRow(3);
			});
			Assert::ExpectException<std::runtime_error>([&]()
			{
				matrix.GetRow(-1);
			});
		}

		TEST_METHOD(SetRow_ValidIndices_SetsCorrectRows)
		{
			Matrix3 matrix = Matrix3::Identity();
			Vector3 newRow(10.0f, 20.0f, 30.0f);

			matrix.SetRow(1, newRow);

			Assert::AreEqual(newRow, matrix.GetRow(1));
			Assert::AreEqual(Vector3(1.0f, 0.0f, 0.0f), matrix.GetRow(0));
			Assert::AreEqual(Vector3(0.0f, 0.0f, 1.0f), matrix.GetRow(2));
		}

		TEST_METHOD(SetRow_InvalidIndex_ThrowsException)
		{
			Matrix3 matrix = Matrix3::Identity();
			Vector3 newRow(1.0f, 2.0f, 3.0f);

			Assert::ExpectException<std::runtime_error>([&]()
			{
				matrix.SetRow(3, newRow);
			});
			Assert::ExpectException<std::runtime_error>([&]()
			{
				matrix.SetRow(-1, newRow);
			});
		}

		// ============================================================================
		// OPERATOR TESTS
		// ============================================================================

		TEST_METHOD(EqualityOperator_IdenticalMatrices_ReturnsTrue)
		{
			Matrix3 matrix1(1.0f, 2.0f, 3.0f,
			                4.0f, 5.0f, 6.0f,
			                7.0f, 8.0f, 9.0f);
			Matrix3 matrix2(1.0f, 2.0f, 3.0f,
			                4.0f, 5.0f, 6.0f,
			                7.0f, 8.0f, 9.0f);

			Assert::IsTrue(matrix1 == matrix2);
		}

		TEST_METHOD(EqualityOperator_DifferentMatrices_ReturnsFalse)
		{
			Matrix3 matrix1 = Matrix3::Identity();
			Matrix3 matrix2 = Matrix3::Zero();

			Assert::IsFalse(matrix1 == matrix2);
		}

		TEST_METHOD(EqualityOperator_NearlyIdenticalMatrices_ReturnsTrue)
		{
			Matrix3 matrix1 = Matrix3::Identity();
			Matrix3 matrix2(1.0f + 1e-8f, 0.0f, 0.0f,
			                0.0f, 1.0f, 0.0f,
			                0.0f, 0.0f, 1.0f);

			Assert::IsTrue(matrix1 == matrix2);
		}

		TEST_METHOD(InequalityOperator_DifferentMatrices_ReturnsTrue)
		{
			Matrix3 matrix1 = Matrix3::Identity();
			Matrix3 matrix2 = Matrix3::Zero();

			Assert::IsTrue(matrix1 != matrix2);
		}

		TEST_METHOD(InequalityOperator_IdenticalMatrices_ReturnsFalse)
		{
			Matrix3 matrix1 = Matrix3::Identity();
			Matrix3 matrix2 = Matrix3::Identity();

			Assert::IsFalse(matrix1 != matrix2);
		}

		TEST_METHOD(MatrixMultiplication_IdentityMatrix_ReturnsOriginal)
		{
			Matrix3 matrix(1.0f, 2.0f, 3.0f,
			               4.0f, 5.0f, 6.0f,
			               7.0f, 8.0f, 9.0f);
			Matrix3 identity = Matrix3::Identity();

			Matrix3 result1 = matrix * identity;
			Matrix3 result2 = identity * matrix;

			AssertMatrixNear(result1, matrix);
			AssertMatrixNear(result2, matrix);
		}

		TEST_METHOD(MatrixMultiplication_KnownMatrices_ReturnsCorrectResult)
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

			AssertMatrixNear(result, expected);
		}

		TEST_METHOD(ScalarMultiplication_Matrix_ReturnsScaledMatrix)
		{
			Matrix3 matrix(1.0f, 2.0f, 3.0f,
			               4.0f, 5.0f, 6.0f,
			               7.0f, 8.0f, 9.0f);
			float scalar = 2.0f;

			Matrix3 result = matrix * scalar;
			Matrix3 expected(2.0f, 4.0f, 6.0f,
			                 8.0f, 10.0f, 12.0f,
			                 14.0f, 16.0f, 18.0f);

			AssertMatrixNear(result, expected);
		}

		TEST_METHOD(ScalarMultiplication_Global_ReturnsScaledMatrix)
		{
			Matrix3 matrix(1.0f, 2.0f, 3.0f,
			               4.0f, 5.0f, 6.0f,
			               7.0f, 8.0f, 9.0f);
			float scalar = 3.0f;

			Matrix3 result = scalar * matrix;
			Matrix3 expected(3.0f, 6.0f, 9.0f,
			                 12.0f, 15.0f, 18.0f,
			                 21.0f, 24.0f, 27.0f);

			AssertMatrixNear(result, expected);
		}

		TEST_METHOD(ScalarDivision_ValidScalar_ReturnsCorrectResult)
		{
			Matrix3 matrix(2.0f, 4.0f, 6.0f,
			               8.0f, 10.0f, 12.0f,
			               14.0f, 16.0f, 18.0f);
			float scalar = 2.0f;

			Matrix3 result = matrix / scalar;
			Matrix3 expected(1.0f, 2.0f, 3.0f,
			                 4.0f, 5.0f, 6.0f,
			                 7.0f, 8.0f, 9.0f);

			AssertMatrixNear(result, expected);
		}

		TEST_METHOD(ScalarDivision_ZeroScalar_ThrowsException)
		{
			Matrix3 matrix = Matrix3::Identity();
			Assert::ExpectException<std::runtime_error>([&]()
			{
				matrix / 0.0f;
			});
		}

		TEST_METHOD(VectorMultiplication_IdentityMatrix_ReturnsOriginalVector)
		{
			Matrix3 identity = Matrix3::Identity();
			Vector3 vector(1.0f, 2.0f, 3.0f);

			Vector3 result = identity * vector;

			Assert::AreEqual(vector, result);
		}

		TEST_METHOD(VectorMultiplication_TranslationMatrix_TranslatesVector)
		{
			Matrix3 translation = Matrix3::Translation(5.0f, 6.0f);
			Vector3 vector(1.0f, 2.0f, 1.0f); // Homogeneous coordinates

			Vector3 result = translation * vector;
			Vector3 expected(6.0f, 8.0f, 1.0f); // (1+5, 2+6, 1)

			Assert::AreEqual(expected, result);
		}

		TEST_METHOD(VectorMultiplication_RotationMatrix_RotatesVector)
		{
			Matrix3 rotation = Matrix3::RotationZ(90.0f);
			Vector3 vector(1.0f, 0.0f, 0.0f);

			Vector3 result = rotation * vector;
			Vector3 expected(0.0f, 1.0f, 0.0f);

			Assert::AreEqual(expected.x, result.x, 1e-5f);
			Assert::AreEqual(expected.y, result.y, 1e-5f);
			Assert::AreEqual(expected.z, result.z, 1e-5f);
		}

		TEST_METHOD(VectorMultiplication_ScaleMatrix_ScalesVector)
		{
			Matrix3 scale = Matrix3::Scale(2.0f, 3.0f, 4.0f);
			Vector3 vector(1.0f, 1.0f, 1.0f);

			Vector3 result = scale * vector;
			Vector3 expected(2.0f, 3.0f, 4.0f);

			Assert::AreEqual(expected, result);
		}

		TEST_METHOD(ColumnAccessOperator_ValidIndices_ReturnsCorrectColumns)
		{
			Matrix3 matrix(1.0f, 2.0f, 3.0f,
			               4.0f, 5.0f, 6.0f,
			               7.0f, 8.0f, 9.0f);

			Vector3 col0 = matrix[0];
			Vector3 col1 = matrix[1];
			Vector3 col2 = matrix[2];

			Assert::AreEqual(Vector3(1.0f, 4.0f, 7.0f), col0);
			Assert::AreEqual(Vector3(2.0f, 5.0f, 8.0f), col1);
			Assert::AreEqual(Vector3(3.0f, 6.0f, 9.0f), col2);
		}

		TEST_METHOD(ColumnAccessOperator_InvalidIndex_ThrowsException)
		{
			Matrix3 matrix = Matrix3::Identity();
			Assert::ExpectException<std::runtime_error>([&]()
			{
				Vector3 col = matrix[3];
			});
			Assert::ExpectException<std::runtime_error>([&]()
			{
				Vector3 col = matrix[-1];
			});
		}

		TEST_METHOD(AssignmentOperator_CopiesCorrectly)
		{
			Matrix3 matrix1(1.0f, 2.0f, 3.0f,
			                4.0f, 5.0f, 6.0f,
			                7.0f, 8.0f, 9.0f);
			Matrix3 matrix2 = Matrix3::Identity();

			matrix2 = matrix1;

			AssertMatrixNear(matrix2, matrix1);
		}

		TEST_METHOD(AssignmentOperator_SelfAssignment_HandledCorrectly)
		{
			Matrix3 matrix(1.0f, 2.0f, 3.0f,
			               4.0f, 5.0f, 6.0f,
			               7.0f, 8.0f, 9.0f);
			Matrix3 original = matrix;

			matrix = matrix; // Self-assignment

			AssertMatrixNear(matrix, original);
		}

		// ============================================================================
		// COMBINED OPERATION TESTS
		// ============================================================================

		TEST_METHOD(CombinedTransformations_ScaleRotateTranslate_WorksCorrectly)
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

			Assert::AreEqual(expected.x, result.x, 1e-5f);
			Assert::AreEqual(expected.y, result.y, 1e-5f);
			Assert::AreEqual(expected.z, result.z, 1e-5f);
		}

		TEST_METHOD(RotationConsistency_AllAxes_MaintainOrthogonality)
		{
			Matrix3 rotX = Matrix3::RotationX(30.0f);
			Matrix3 rotY = Matrix3::RotationY(45.0f);
			Matrix3 rotZ = Matrix3::RotationZ(60.0f);

			Assert::IsTrue(rotX.IsOrthogonal());
			Assert::IsTrue(rotY.IsOrthogonal());
			Assert::IsTrue(rotZ.IsOrthogonal());

			Assert::AreEqual(1.0f, rotX.Determinant(), 1e-5f);
			Assert::AreEqual(1.0f, rotY.Determinant(), 1e-5f);
			Assert::AreEqual(1.0f, rotZ.Determinant(), 1e-5f);
		}

		TEST_METHOD(InverseMultiplication_VariousMatrices_GivesIdentity)
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
				AssertMatrixNear(product, Matrix3::Identity(), 1e-5f);
			}
		}

		// ============================================================================
		// EDGE CASE TESTS
		// ============================================================================

		TEST_METHOD(VerySmallValues_HandlePrecisionCorrectly)
		{
			Matrix3 small(1e-20f, 0.0f, 0.0f,
			              0.0f, 1e-20f, 0.0f,
			              0.0f, 0.0f, 1e-20f);

			float det = small.Determinant();
			Assert::IsTrue(std::isfinite(det));
		}

		TEST_METHOD(ZeroAngleRotations_ProduceIdentity)
		{
			Matrix3 rotX = Matrix3::RotationX(0.0f);
			Matrix3 rotY = Matrix3::RotationY(0.0f);
			Matrix3 rotZ = Matrix3::RotationZ(0.0f);

			AssertMatrixNear(rotX, Matrix3::Identity(), 1e-6f);
			AssertMatrixNear(rotY, Matrix3::Identity(), 1e-6f);
			AssertMatrixNear(rotZ, Matrix3::Identity(), 1e-6f);
		}

		TEST_METHOD(FullCircleRotations_ProduceIdentity)
		{
			Matrix3 rotX = Matrix3::RotationX(360.0f);
			Matrix3 rotY = Matrix3::RotationY(360.0f);
			Matrix3 rotZ = Matrix3::RotationZ(360.0f);

			AssertMatrixNear(rotX, Matrix3::Identity(), 1e-5f);
			AssertMatrixNear(rotY, Matrix3::Identity(), 1e-5f);
			AssertMatrixNear(rotZ, Matrix3::Identity(), 1e-5f);
		}
	};
}
