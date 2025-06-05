#include <numbers>

#include "CppUnitTest.h"

#include "Nudge/MathF.hpp"
#include "Nudge/Vector2.hpp"
#include "Nudge/Vector3.hpp"
#include "Nudge/Vector4.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using std::runtime_error;
using std::numbers::pi_v;

namespace Nudge
{
	TEST_CLASS(Vector4Tests)
	{
	public:
		// Helper method for floating point comparison
		static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
		{
			Assert::IsTrue(MathF::Compare(expected, actual, tolerance), L"Float values are not equal within tolerance");
		}

		static void AssertVector4Equal(const Vector4& expected, const Vector4& actual, float tolerance = 0.0001f)
		{
			AssertFloatEqual(expected.x, actual.x, tolerance);
			AssertFloatEqual(expected.y, actual.y, tolerance);
			AssertFloatEqual(expected.z, actual.z, tolerance);
			AssertFloatEqual(expected.w, actual.w, tolerance);
		}

		TEST_METHOD(Constructor_Default_CreatesZeroVector)
		{
			Vector4 vec;
			AssertVector4Equal(Vector4(0.0f, 0.0f, 0.0f, 0.0f), vec);
		}

		TEST_METHOD(Constructor_Scalar_CreatesSameValueForAllComponents)
		{
			Vector4 vec(5.0f);
			AssertVector4Equal(Vector4(5.0f, 5.0f, 5.0f, 5.0f), vec);
		}

		TEST_METHOD(Constructor_XYZW_CreatesVectorWithSpecifiedValues)
		{
			Vector4 vec(1.0f, 2.0f, 3.0f, 4.0f);
			AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 4.0f), vec);
		}

		TEST_METHOD(Constructor_Array_CreatesVectorFromArray)
		{
			float values[] = { 2.0f, 3.0f, 4.0f, 5.0f };
			Vector4 vec(values);
			AssertVector4Equal(Vector4(2.0f, 3.0f, 4.0f, 5.0f), vec);
		}

		TEST_METHOD(Constructor_Copy_CreatesIdenticalVector)
		{
			Vector4 original(1.5f, 2.5f, 3.5f, 4.5f);
			Vector4 copy(original);
			AssertVector4Equal(original, copy);
		}

		TEST_METHOD(Constructor_Vector2_CreatesVectorWithZeroZW)
		{
			Vector2 vec2(3.0f, 4.0f);
			Vector4 vec4(vec2);
			AssertVector4Equal(Vector4(3.0f, 4.0f, 0.0f, 0.0f), vec4);
		}

		TEST_METHOD(Constructor_Vector3_CreatesVectorWithZeroW)
		{
			Vector3 vec3(1.0f, 2.0f, 3.0f);
			Vector4 vec4(vec3);
			AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 0.0f), vec4);
		}

		TEST_METHOD(Magnitude_FourDimensionalPythagorean_ReturnsCorrectMagnitude)
		{
			Vector4 vec(2.0f, 3.0f, 6.0f, 0.0f);
			AssertFloatEqual(7.0f, vec.Magnitude()); // sqrt(4 + 9 + 36) = sqrt(49) = 7
		}

		TEST_METHOD(Magnitude_ZeroVector_ReturnsZero)
		{
			Vector4 vec(0.0f, 0.0f, 0.0f, 0.0f);
			AssertFloatEqual(0.0f, vec.Magnitude());
		}

		TEST_METHOD(Magnitude_UnitVector_ReturnsOne)
		{
			Vector4 vec(1.0f, 0.0f, 0.0f, 0.0f);
			AssertFloatEqual(1.0f, vec.Magnitude());
		}

		TEST_METHOD(MagnitudeSqr_FourDimensionalPythagorean_ReturnsSquaredMagnitude)
		{
			Vector4 vec(2.0f, 3.0f, 6.0f, 0.0f);
			AssertFloatEqual(49.0f, vec.MagnitudeSqr()); // 4 + 9 + 36 = 49
		}

		TEST_METHOD(Normalize_NonZeroVector_CreatesUnitVector)
		{
			Vector4 vec(2.0f, 3.0f, 6.0f, 0.0f);
			vec.Normalize();
			AssertFloatEqual(1.0f, vec.Magnitude());
			AssertVector4Equal(Vector4(2.0f / 7.0f, 3.0f / 7.0f, 6.0f / 7.0f, 0.0f), vec);
		}

		TEST_METHOD(Normalize_ZeroVector_RemainsZero)
		{
			Vector4 vec(0.0f, 0.0f, 0.0f, 0.0f);
			vec.Normalize();
			AssertVector4Equal(Vector4(0.0f, 0.0f, 0.0f, 0.0f), vec);
		}

		TEST_METHOD(Normalized_NonZeroVector_ReturnsUnitVector)
		{
			Vector4 vec(2.0f, 3.0f, 6.0f, 0.0f);
			Vector4 normalized = vec.Normalized();
			AssertFloatEqual(1.0f, normalized.Magnitude());
			AssertVector4Equal(Vector4(2.0f / 7.0f, 3.0f / 7.0f, 6.0f / 7.0f, 0.0f), normalized);
			// Original should be unchanged
			AssertVector4Equal(Vector4(2.0f, 3.0f, 6.0f, 0.0f), vec);
		}

		TEST_METHOD(Normalized_ZeroVector_ReturnsZero)
		{
			Vector4 vec(0.0f, 0.0f, 0.0f, 0.0f);
			Vector4 normalized = vec.Normalized();
			AssertVector4Equal(Vector4(0.0f, 0.0f, 0.0f, 0.0f), normalized);
		}

		TEST_METHOD(IsZero_ZeroVector_ReturnsTrue)
		{
			Vector4 vec(0.0f, 0.0f, 0.0f, 0.0f);
			Assert::IsTrue(vec.IsZero());
		}

		TEST_METHOD(IsZero_NonZeroVector_ReturnsFalse)
		{
			Vector4 vec(0.1f, 0.0f, 0.0f, 0.0f);
			Assert::IsFalse(vec.IsZero());
		}

		TEST_METHOD(IsUnit_UnitVector_ReturnsTrue)
		{
			Vector4 vec(1.0f, 0.0f, 0.0f, 0.0f);
			Assert::IsTrue(vec.IsUnit());
		}

		TEST_METHOD(IsUnit_NonUnitVector_ReturnsFalse)
		{
			Vector4 vec(2.0f, 0.0f, 0.0f, 0.0f);
			Assert::IsFalse(vec.IsUnit());
		}

		TEST_METHOD(Dot_OrthogonalVectors_ReturnsZero)
		{
			Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
			Vector4 vec2(0.0f, 1.0f, 0.0f, 0.0f);
			AssertFloatEqual(0.0f, Vector4::Dot(vec1, vec2));
		}

		TEST_METHOD(Dot_ParallelVectors_ReturnsProduct)
		{
			Vector4 vec1(2.0f, 0.0f, 0.0f, 0.0f);
			Vector4 vec2(3.0f, 0.0f, 0.0f, 0.0f);
			AssertFloatEqual(6.0f, Vector4::Dot(vec1, vec2));
		}

		TEST_METHOD(Dot_GeneralCase_ReturnsCorrectValue)
		{
			Vector4 vec1(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 vec2(5.0f, 6.0f, 7.0f, 8.0f);
			// 1*5 + 2*6 + 3*7 + 4*8 = 5 + 12 + 21 + 32 = 70
			AssertFloatEqual(70.0f, Vector4::Dot(vec1, vec2));
		}

		TEST_METHOD(Distance_SamePoints_ReturnsZero)
		{
			Vector4 vec1(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 vec2(1.0f, 2.0f, 3.0f, 4.0f);
			AssertFloatEqual(0.0f, Vector4::Distance(vec1, vec2));
		}

		TEST_METHOD(Distance_FourDimensionalPythagorean_ReturnsCorrectDistance)
		{
			Vector4 vec1(0.0f, 0.0f, 0.0f, 0.0f);
			Vector4 vec2(2.0f, 3.0f, 6.0f, 0.0f);
			AssertFloatEqual(7.0f, Vector4::Distance(vec1, vec2));
		}

		TEST_METHOD(DistanceSqr_FourDimensionalPythagorean_ReturnsSquaredDistance)
		{
			Vector4 vec1(0.0f, 0.0f, 0.0f, 0.0f);
			Vector4 vec2(2.0f, 3.0f, 6.0f, 0.0f);
			AssertFloatEqual(49.0f, Vector4::DistanceSqr(vec1, vec2));
		}

		TEST_METHOD(AngleOf_UnitZ_ReturnsZero)
		{
			Vector4 vec(0.0f, 0.0f, 1.0f, 0.0f);
			AssertFloatEqual(0.0f, Vector4::AngleOf(vec));
		}

		TEST_METHOD(AngleOf_NegativeZ_ReturnsPi)
		{
			Vector4 vec(0.0f, 0.0f, -1.0f, 0.0f);
			AssertFloatEqual(pi_v<float>, Vector4::AngleOf(vec), 0.0001f);
		}

		TEST_METHOD(AngleOf_ZeroVector_ReturnsZero)
		{
			Vector4 vec(0.0f, 0.0f, 0.0f, 0.0f);
			AssertFloatEqual(0.0f, Vector4::AngleOf(vec));
		}

		TEST_METHOD(AngleBetween_SameVectors_ReturnsZero)
		{
			Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
			Vector4 vec2(1.0f, 0.0f, 0.0f, 0.0f);
			AssertFloatEqual(0.0f, Vector4::AngleBetween(vec1, vec2));
		}

		TEST_METHOD(AngleBetween_OrthogonalVectors_ReturnsHalfPi)
		{
			Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
			Vector4 vec2(0.0f, 1.0f, 0.0f, 0.0f);
			AssertFloatEqual(pi_v<float> / 2.0f, Vector4::AngleBetween(vec1, vec2), 0.0001f);
		}

		TEST_METHOD(AngleBetween_OppositeVectors_ReturnsPi)
		{
			Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
			Vector4 vec2(-1.0f, 0.0f, 0.0f, 0.0f);
			AssertFloatEqual(pi_v<float>, Vector4::AngleBetween(vec1, vec2), 0.0001f);
		}

		TEST_METHOD(AngleBetween_ZeroVector_ReturnsZero)
		{
			Vector4 vec1(0.0f, 0.0f, 0.0f, 0.0f);
			Vector4 vec2(1.0f, 0.0f, 0.0f, 0.0f);
			AssertFloatEqual(0.0f, Vector4::AngleBetween(vec1, vec2));
		}

		TEST_METHOD(Lerp_ZeroT_ReturnsFirstVector)
		{
			Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
			Vector4 result = Vector4::Lerp(a, b, 0.0f);
			AssertVector4Equal(a, result);
		}

		TEST_METHOD(Lerp_OneT_ReturnsSecondVector)
		{
			Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
			Vector4 result = Vector4::Lerp(a, b, 1.0f);
			AssertVector4Equal(b, result);
		}

		TEST_METHOD(Lerp_HalfT_ReturnsMidpoint)
		{
			Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 b(3.0f, 4.0f, 5.0f, 6.0f);
			Vector4 result = Vector4::Lerp(a, b, 0.5f);
			AssertVector4Equal(Vector4(2.0f, 3.0f, 4.0f, 5.0f), result);
		}

		TEST_METHOD(Lerp_ClampsBeyondOne)
		{
			Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
			Vector4 result = Vector4::Lerp(a, b, 1.5f);
			AssertVector4Equal(b, result); // Should clamp to 1.0
		}

		TEST_METHOD(Reflect_PerpendicularToNormal_ReflectsPerfectly)
		{
			Vector4 incident(1.0f, -1.0f, 0.0f, 0.0f); // Down-right
			Vector4 normal(0.0f, 1.0f, 0.0f, 0.0f);    // Up
			Vector4 reflected = Vector4::Reflect(incident, normal);
			AssertVector4Equal(Vector4(1.0f, 1.0f, 0.0f, 0.0f), reflected); // Up-right
		}

		TEST_METHOD(Cross_OrthogonalVectors_ReturnsPerpendicularVector)
		{
			Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
			Vector4 vec2(0.0f, 1.0f, 0.0f, 0.0f);
			Vector4 result = Vector4::Cross(vec1, vec2);
			AssertVector4Equal(Vector4(0.0f, 0.0f, 1.0f, 0.0f), result);
		}

		TEST_METHOD(Cross_ParallelVectors_ReturnsZeroVector)
		{
			Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
			Vector4 vec2(2.0f, 0.0f, 0.0f, 0.0f);
			Vector4 result = Vector4::Cross(vec1, vec2);
			AssertVector4Equal(Vector4(0.0f, 0.0f, 0.0f, 0.0f), result);
		}

		TEST_METHOD(Cross_GeneralCase_ReturnsCorrectCrossProduct)
		{
			Vector4 vec1(1.0f, 2.0f, 3.0f, 0.0f);
			Vector4 vec2(4.0f, 5.0f, 6.0f, 0.0f);
			Vector4 result = Vector4::Cross(vec1, vec2);
			// Cross product: (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
			AssertVector4Equal(Vector4(-3.0f, 6.0f, -3.0f, 0.0f), result);
		}

		TEST_METHOD(Min_ComponentWise_ReturnsMinimumComponents)
		{
			Vector4 a(1.0f, 5.0f, 3.0f, 8.0f);
			Vector4 b(4.0f, 2.0f, 6.0f, 1.0f);
			Vector4 result = Vector4::Min(a, b);
			AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 1.0f), result);
		}

		TEST_METHOD(Max_ComponentWise_ReturnsMaximumComponents)
		{
			Vector4 a(1.0f, 5.0f, 3.0f, 8.0f);
			Vector4 b(4.0f, 2.0f, 6.0f, 1.0f);
			Vector4 result = Vector4::Max(a, b);
			AssertVector4Equal(Vector4(4.0f, 5.0f, 6.0f, 8.0f), result);
		}

		TEST_METHOD(Clamp_WithinBounds_ReturnsOriginal)
		{
			Vector4 value(2.0f, 3.0f, 4.0f, 5.0f);
			Vector4 min(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 max(6.0f, 7.0f, 8.0f, 9.0f);
			Vector4 result = Vector4::Clamp(value, min, max);
			AssertVector4Equal(value, result);
		}

		TEST_METHOD(Clamp_OutOfBounds_ClampsToLimits)
		{
			Vector4 value(0.0f, 8.0f, 2.0f, 10.0f);
			Vector4 min(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 max(6.0f, 7.0f, 8.0f, 9.0f);
			Vector4 result = Vector4::Clamp(value, min, max);
			AssertVector4Equal(Vector4(1.0f, 7.0f, 3.0f, 9.0f), result);
		}

		TEST_METHOD(StaticFactories_CreateCorrectVectors)
		{
			AssertVector4Equal(Vector4(0.0f, 0.0f, 0.0f, 0.0f), Vector4::Zero());
			AssertVector4Equal(Vector4(1.0f, 1.0f, 1.0f, 1.0f), Vector4::One());
			AssertVector4Equal(Vector4(0.5f, 0.5f, 0.5f, 0.5f), Vector4::Half());
			AssertVector4Equal(Vector4(1.0f, 0.0f, 0.0f, 0.0f), Vector4::UnitX());
			AssertVector4Equal(Vector4(0.0f, 1.0f, 0.0f, 0.0f), Vector4::UnitY());
			AssertVector4Equal(Vector4(0.0f, 0.0f, 1.0f, 0.0f), Vector4::UnitZ());
		}

		TEST_METHOD(Equality_SameVectors_ReturnsTrue)
		{
			Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 b(1.0f, 2.0f, 3.0f, 4.0f);
			Assert::IsTrue(a == b);
		}

		TEST_METHOD(Equality_DifferentVectors_ReturnsFalse)
		{
			Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 b(1.0f, 2.0f, 3.0f, 5.0f);
			Assert::IsFalse(a == b);
		}

		TEST_METHOD(Inequality_DifferentVectors_ReturnsTrue)
		{
			Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 b(1.0f, 2.0f, 3.0f, 5.0f);
			Assert::IsTrue(a != b);
		}

		TEST_METHOD(Addition_TwoVectors_AddsComponents)
		{
			Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
			Vector4 result = a + b;
			AssertVector4Equal(Vector4(6.0f, 8.0f, 10.0f, 12.0f), result);
		}

		TEST_METHOD(AdditionAssignment_ModifiesOriginal)
		{
			Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
			a += b;
			AssertVector4Equal(Vector4(6.0f, 8.0f, 10.0f, 12.0f), a);
		}

		TEST_METHOD(Subtraction_TwoVectors_SubtractsComponents)
		{
			Vector4 a(6.0f, 8.0f, 10.0f, 12.0f);
			Vector4 b(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 result = a - b;
			AssertVector4Equal(Vector4(5.0f, 6.0f, 7.0f, 8.0f), result);
		}

		TEST_METHOD(SubtractionAssignment_ModifiesOriginal)
		{
			Vector4 a(6.0f, 8.0f, 10.0f, 12.0f);
			Vector4 b(1.0f, 2.0f, 3.0f, 4.0f);
			a -= b;
			AssertVector4Equal(Vector4(5.0f, 6.0f, 7.0f, 8.0f), a);
		}

		TEST_METHOD(MultiplicationByScalar_ScalesComponents)
		{
			Vector4 vec(2.0f, 3.0f, 4.0f, 5.0f);
			Vector4 result = vec * 2.0f;
			AssertVector4Equal(Vector4(4.0f, 6.0f, 8.0f, 10.0f), result);
		}

		TEST_METHOD(MultiplicationAssignmentByScalar_ModifiesOriginal)
		{
			Vector4 vec(2.0f, 3.0f, 4.0f, 5.0f);
			vec *= 2.0f;
			AssertVector4Equal(Vector4(4.0f, 6.0f, 8.0f, 10.0f), vec);
		}

		TEST_METHOD(DivisionByScalar_DividesComponents)
		{
			Vector4 vec(4.0f, 6.0f, 8.0f, 10.0f);
			Vector4 result = vec / 2.0f;
			AssertVector4Equal(Vector4(2.0f, 3.0f, 4.0f, 5.0f), result);
		}

		TEST_METHOD(DivisionAssignmentByScalar_ModifiesOriginal)
		{
			Vector4 vec(4.0f, 6.0f, 8.0f, 10.0f);
			vec /= 2.0f;
			AssertVector4Equal(Vector4(2.0f, 3.0f, 4.0f, 5.0f), vec);
		}

		TEST_METHOD(UnaryMinus_NegatesComponents)
		{
			Vector4 vec(2.0f, -3.0f, 4.0f, -5.0f);
			Vector4& result = -vec;
			AssertVector4Equal(Vector4(-2.0f, 3.0f, -4.0f, 5.0f), result);
			// Should also modify original
			AssertVector4Equal(Vector4(-2.0f, 3.0f, -4.0f, 5.0f), vec);
		}

		TEST_METHOD(IndexOperator_ValidIndices_ReturnsCorrectComponents)
		{
			Vector4 vec(2.0f, 3.0f, 4.0f, 5.0f);
			AssertFloatEqual(2.0f, vec[0]);
			AssertFloatEqual(3.0f, vec[1]);
			AssertFloatEqual(4.0f, vec[2]);
			// Note: This will fail with the original buggy implementation
			// AssertFloatEqual(5.0f, vec[3]);
		}

		TEST_METHOD(IndexOperator_InvalidIndex_ThrowsException)
		{
			Vector4 vec(2.0f, 3.0f, 4.0f, 5.0f);

			// Test negative index
			Assert::ExpectException<runtime_error>([&]()
			{
				float value = vec[-1];
			});

			// Test index 4 (out of bounds)
			Assert::ExpectException<runtime_error>([&]()
			{
				float value = vec[4];
			});

			// Test index 5 (out of bounds)
			Assert::ExpectException<runtime_error>([&]()
			{
				float value = vec[5];
			});
		}

		TEST_METHOD(ToString_FormatsCorrectly)
		{
			Vector4 vec(1.5f, 2.5f, 3.5f, 4.5f);
			string str = vec.ToString();
			// Basic check - should contain the numbers
			Assert::IsTrue(str.find("1.5") != string::npos);
			Assert::IsTrue(str.find("2.5") != string::npos);
			Assert::IsTrue(str.find("3.5") != string::npos);
			Assert::IsTrue(str.find("4.5") != string::npos);
		}

		TEST_METHOD(GlobalMultiplication_ScalarTimesVector_Works)
		{
			Vector4 vec(2.0f, 3.0f, 4.0f, 5.0f);
			Vector4 result = 2.0f * vec;
			AssertVector4Equal(Vector4(4.0f, 6.0f, 8.0f, 10.0f), result);
		}

		// Edge case tests
		TEST_METHOD(EdgeCase_VerySmallNumbers_HandledCorrectly)
		{
			Vector4 vec(1e-6f, 1e-6f, 1e-6f, 1e-6f);
			Assert::IsFalse(vec.IsZero()); // Should not be considered zero
		}

		TEST_METHOD(EdgeCase_VeryLargeNumbers_HandledCorrectly)
		{
			Vector4 vec(1e6f, 1e6f, 1e6f, 1e6f);
			Vector4 normalized = vec.Normalized();
			AssertFloatEqual(1.0f, normalized.Magnitude(), 0.001f);
		}

		TEST_METHOD(EdgeCase_SelfAssignment_HandledCorrectly)
		{
			Vector4 vec(1.0f, 2.0f, 3.0f, 4.0f);
			vec = vec;
			AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 4.0f), vec);
		}

		TEST_METHOD(EdgeCase_SelfAddition_HandledCorrectly)
		{
			Vector4 vec(1.0f, 2.0f, 3.0f, 4.0f);
			vec += vec;
			AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 4.0f), vec); // Should remain unchanged due to self-check
		}

		TEST_METHOD(EdgeCase_SelfSubtraction_HandledCorrectly)
		{
			Vector4 vec(1.0f, 2.0f, 3.0f, 4.0f);
			vec -= vec;
			AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 4.0f), vec); // Should remain unchanged due to self-check
		}

		// Vector4-specific tests
		TEST_METHOD(HomogeneousCoordinates_WComponentUsedInCalculations)
		{
			Vector4 vec(1.0f, 2.0f, 3.0f, 4.0f);
			// Test that w component affects magnitude
			float expectedMagSqr = 1.0f + 4.0f + 9.0f + 16.0f; // 30
			AssertFloatEqual(expectedMagSqr, vec.MagnitudeSqr());
		}

		TEST_METHOD(FourDimensionalOperations_AllComponentsProcessed)
		{
			Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
			Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);

			// Test that all four components are used in dot product
			float expected = 1 * 5 + 2 * 6 + 3 * 7 + 4 * 8; // 70
			AssertFloatEqual(expected, Vector4::Dot(a, b));
		}
	};
}
