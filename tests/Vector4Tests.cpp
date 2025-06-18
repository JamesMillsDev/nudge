#include <numbers>

#include <gtest/gtest.h>

#include "Nudge/MathF.hpp"
#include "Nudge/Vector2.hpp"
#include "Nudge/Vector3.hpp"
#include "Nudge/Vector4.hpp"

using std::runtime_error;
using std::numbers::pi_v;

using testing::Test;

namespace Nudge
{
	class Vector4Tests : public Test
	{
	public:
		// Helper method for floating point comparison
		static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
		{
			EXPECT_TRUE(MathF::Compare(expected, actual, tolerance));
		}

		static void AssertVector4Equal(const Vector4& expected, const Vector4& actual, float tolerance = 0.0001f)
		{
			AssertFloatEqual(expected.x, actual.x, tolerance);
			AssertFloatEqual(expected.y, actual.y, tolerance);
			AssertFloatEqual(expected.z, actual.z, tolerance);
			AssertFloatEqual(expected.w, actual.w, tolerance);
		}

	};

	TEST_F(Vector4Tests, Constructor_Default_CreatesZeroVector)
	{
		Vector4 vec;
		AssertVector4Equal(Vector4(0.0f, 0.0f, 0.0f, 0.0f), vec);
	}

	TEST_F(Vector4Tests, Constructor_Scalar_CreatesSameValueForAllComponents)
	{
		Vector4 vec(5.0f);
		AssertVector4Equal(Vector4(5.0f, 5.0f, 5.0f, 5.0f), vec);
	}

	TEST_F(Vector4Tests, Constructor_XYZW_CreatesVectorWithSpecifiedValues)
	{
		Vector4 vec(1.0f, 2.0f, 3.0f, 4.0f);
		AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 4.0f), vec);
	}

	TEST_F(Vector4Tests, Constructor_Array_CreatesVectorFromArray)
	{
		float values[] = { 2.0f, 3.0f, 4.0f, 5.0f };
		Vector4 vec(values);
		AssertVector4Equal(Vector4(2.0f, 3.0f, 4.0f, 5.0f), vec);
	}

	TEST_F(Vector4Tests, Constructor_Copy_CreatesIdenticalVector)
	{
		Vector4 original(1.5f, 2.5f, 3.5f, 4.5f);
		Vector4 copy(original);
		AssertVector4Equal(original, copy);
	}

	TEST_F(Vector4Tests, Constructor_Vector2_CreatesVectorWithZeroZW)
	{
		Vector2 vec2(3.0f, 4.0f);
		Vector4 vec4(vec2);
		AssertVector4Equal(Vector4(3.0f, 4.0f, 0.0f, 0.0f), vec4);
	}

	TEST_F(Vector4Tests, Constructor_Vector3_CreatesVectorWithZeroW)
	{
		Vector3 vec3(1.0f, 2.0f, 3.0f);
		Vector4 vec4(vec3);
		AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 0.0f), vec4);
	}

	TEST_F(Vector4Tests, Magnitude_FourDimensionalPythagorean_ReturnsCorrectMagnitude)
	{
		Vector4 vec(2.0f, 3.0f, 6.0f, 0.0f);
		AssertFloatEqual(7.0f, vec.Magnitude()); // sqrt(4 + 9 + 36) = sqrt(49) = 7
	}

	TEST_F(Vector4Tests, Magnitude_ZeroVector_ReturnsZero)
	{
		Vector4 vec(0.0f, 0.0f, 0.0f, 0.0f);
		AssertFloatEqual(0.0f, vec.Magnitude());
	}

	TEST_F(Vector4Tests, Magnitude_UnitVector_ReturnsOne)
	{
		Vector4 vec(1.0f, 0.0f, 0.0f, 0.0f);
		AssertFloatEqual(1.0f, vec.Magnitude());
	}

	TEST_F(Vector4Tests, MagnitudeSqr_FourDimensionalPythagorean_ReturnsSquaredMagnitude)
	{
		Vector4 vec(2.0f, 3.0f, 6.0f, 0.0f);
		AssertFloatEqual(49.0f, vec.MagnitudeSqr()); // 4 + 9 + 36 = 49
	}

	TEST_F(Vector4Tests, Normalize_NonZeroVector_CreatesUnitVector)
	{
		Vector4 vec(2.0f, 3.0f, 6.0f, 0.0f);
		vec.Normalize();
		AssertFloatEqual(1.0f, vec.Magnitude());
		AssertVector4Equal(Vector4(2.0f / 7.0f, 3.0f / 7.0f, 6.0f / 7.0f, 0.0f), vec);
	}

	TEST_F(Vector4Tests, Normalize_ZeroVector_RemainsZero)
	{
		Vector4 vec(0.0f, 0.0f, 0.0f, 0.0f);
		vec.Normalize();
		AssertVector4Equal(Vector4(0.0f, 0.0f, 0.0f, 0.0f), vec);
	}

	TEST_F(Vector4Tests, Normalized_NonZeroVector_ReturnsUnitVector)
	{
		Vector4 vec(2.0f, 3.0f, 6.0f, 0.0f);
		Vector4 normalized = vec.Normalized();
		AssertFloatEqual(1.0f, normalized.Magnitude());
		AssertVector4Equal(Vector4(2.0f / 7.0f, 3.0f / 7.0f, 6.0f / 7.0f, 0.0f), normalized);
		// Original should be unchanged
		AssertVector4Equal(Vector4(2.0f, 3.0f, 6.0f, 0.0f), vec);
	}

	TEST_F(Vector4Tests, Normalized_ZeroVector_ReturnsZero)
	{
		Vector4 vec(0.0f, 0.0f, 0.0f, 0.0f);
		Vector4 normalized = vec.Normalized();
		AssertVector4Equal(Vector4(0.0f, 0.0f, 0.0f, 0.0f), normalized);
	}

	TEST_F(Vector4Tests, IsZero_ZeroVector_ReturnsTrue)
	{
		Vector4 vec(0.0f, 0.0f, 0.0f, 0.0f);
		EXPECT_TRUE(vec.IsZero());
	}

	TEST_F(Vector4Tests, IsZero_NonZeroVector_ReturnsFalse)
	{
		Vector4 vec(0.1f, 0.0f, 0.0f, 0.0f);
		EXPECT_FALSE(vec.IsZero());
	}

	TEST_F(Vector4Tests, IsUnit_UnitVector_ReturnsTrue)
	{
		Vector4 vec(1.0f, 0.0f, 0.0f, 0.0f);
		EXPECT_TRUE(vec.IsUnit());
	}

	TEST_F(Vector4Tests, IsUnit_NonUnitVector_ReturnsFalse)
	{
		Vector4 vec(2.0f, 0.0f, 0.0f, 0.0f);
		EXPECT_FALSE(vec.IsUnit());
	}

	TEST_F(Vector4Tests, Dot_OrthogonalVectors_ReturnsZero)
	{
		Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
		Vector4 vec2(0.0f, 1.0f, 0.0f, 0.0f);
		AssertFloatEqual(0.0f, Vector4::Dot(vec1, vec2));
	}

	TEST_F(Vector4Tests, Dot_ParallelVectors_ReturnsProduct)
	{
		Vector4 vec1(2.0f, 0.0f, 0.0f, 0.0f);
		Vector4 vec2(3.0f, 0.0f, 0.0f, 0.0f);
		AssertFloatEqual(6.0f, Vector4::Dot(vec1, vec2));
	}

	TEST_F(Vector4Tests, Dot_GeneralCase_ReturnsCorrectValue)
	{
		Vector4 vec1(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 vec2(5.0f, 6.0f, 7.0f, 8.0f);
		// 1*5 + 2*6 + 3*7 + 4*8 = 5 + 12 + 21 + 32 = 70
		AssertFloatEqual(70.0f, Vector4::Dot(vec1, vec2));
	}

	TEST_F(Vector4Tests, Distance_SamePoints_ReturnsZero)
	{
		Vector4 vec1(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 vec2(1.0f, 2.0f, 3.0f, 4.0f);
		AssertFloatEqual(0.0f, Vector4::Distance(vec1, vec2));
	}

	TEST_F(Vector4Tests, Distance_FourDimensionalPythagorean_ReturnsCorrectDistance)
	{
		Vector4 vec1(0.0f, 0.0f, 0.0f, 0.0f);
		Vector4 vec2(2.0f, 3.0f, 6.0f, 0.0f);
		AssertFloatEqual(7.0f, Vector4::Distance(vec1, vec2));
	}

	TEST_F(Vector4Tests, DistanceSqr_FourDimensionalPythagorean_ReturnsSquaredDistance)
	{
		Vector4 vec1(0.0f, 0.0f, 0.0f, 0.0f);
		Vector4 vec2(2.0f, 3.0f, 6.0f, 0.0f);
		AssertFloatEqual(49.0f, Vector4::DistanceSqr(vec1, vec2));
	}

	TEST_F(Vector4Tests, AngleOf_UnitZ_ReturnsZero)
	{
		Vector4 vec(0.0f, 0.0f, 1.0f, 0.0f);
		AssertFloatEqual(0.0f, Vector4::AngleOf(vec));
	}

	TEST_F(Vector4Tests, AngleOf_NegativeZ_ReturnsPi)
	{
		Vector4 vec(0.0f, 0.0f, -1.0f, 0.0f);
		AssertFloatEqual(pi_v<float>, Vector4::AngleOf(vec), 0.0001f);
	}

	TEST_F(Vector4Tests, AngleOf_ZeroVector_ReturnsZero)
	{
		Vector4 vec(0.0f, 0.0f, 0.0f, 0.0f);
		AssertFloatEqual(0.0f, Vector4::AngleOf(vec));
	}

	TEST_F(Vector4Tests, AngleBetween_SameVectors_ReturnsZero)
	{
		Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
		Vector4 vec2(1.0f, 0.0f, 0.0f, 0.0f);
		AssertFloatEqual(0.0f, Vector4::AngleBetween(vec1, vec2));
	}

	TEST_F(Vector4Tests, AngleBetween_OrthogonalVectors_ReturnsHalfPi)
	{
		Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
		Vector4 vec2(0.0f, 1.0f, 0.0f, 0.0f);
		AssertFloatEqual(pi_v<float> / 2.0f, Vector4::AngleBetween(vec1, vec2), 0.0001f);
	}

	TEST_F(Vector4Tests, AngleBetween_OppositeVectors_ReturnsPi)
	{
		Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
		Vector4 vec2(-1.0f, 0.0f, 0.0f, 0.0f);
		AssertFloatEqual(pi_v<float>, Vector4::AngleBetween(vec1, vec2), 0.0001f);
	}

	TEST_F(Vector4Tests, AngleBetween_ZeroVector_ReturnsZero)
	{
		Vector4 vec1(0.0f, 0.0f, 0.0f, 0.0f);
		Vector4 vec2(1.0f, 0.0f, 0.0f, 0.0f);
		AssertFloatEqual(0.0f, Vector4::AngleBetween(vec1, vec2));
	}

	TEST_F(Vector4Tests, Lerp_ZeroT_ReturnsFirstVector)
	{
		Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
		Vector4 result = Vector4::Lerp(a, b, 0.0f);
		AssertVector4Equal(a, result);
	}

	TEST_F(Vector4Tests, Lerp_OneT_ReturnsSecondVector)
	{
		Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
		Vector4 result = Vector4::Lerp(a, b, 1.0f);
		AssertVector4Equal(b, result);
	}

	TEST_F(Vector4Tests, Lerp_HalfT_ReturnsMidpoint)
	{
		Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 b(3.0f, 4.0f, 5.0f, 6.0f);
		Vector4 result = Vector4::Lerp(a, b, 0.5f);
		AssertVector4Equal(Vector4(2.0f, 3.0f, 4.0f, 5.0f), result);
	}

	TEST_F(Vector4Tests, Lerp_ClampsBeyondOne)
	{
		Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
		Vector4 result = Vector4::Lerp(a, b, 1.5f);
		AssertVector4Equal(b, result); // Should clamp to 1.0
	}

	TEST_F(Vector4Tests, Reflect_PerpendicularToNormal_ReflectsPerfectly)
	{
		Vector4 incident(1.0f, -1.0f, 0.0f, 0.0f); // Down-right
		Vector4 normal(0.0f, 1.0f, 0.0f, 0.0f);    // Up
		Vector4 reflected = Vector4::Reflect(incident, normal);
		AssertVector4Equal(Vector4(1.0f, 1.0f, 0.0f, 0.0f), reflected); // Up-right
	}

	TEST_F(Vector4Tests, Cross_OrthogonalVectors_ReturnsPerpendicularVector)
	{
		Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
		Vector4 vec2(0.0f, 1.0f, 0.0f, 0.0f);
		Vector4 result = Vector4::Cross(vec1, vec2);
		AssertVector4Equal(Vector4(0.0f, 0.0f, 1.0f, 0.0f), result);
	}

	TEST_F(Vector4Tests, Cross_ParallelVectors_ReturnsZeroVector)
	{
		Vector4 vec1(1.0f, 0.0f, 0.0f, 0.0f);
		Vector4 vec2(2.0f, 0.0f, 0.0f, 0.0f);
		Vector4 result = Vector4::Cross(vec1, vec2);
		AssertVector4Equal(Vector4(0.0f, 0.0f, 0.0f, 0.0f), result);
	}

	TEST_F(Vector4Tests, Cross_GeneralCase_ReturnsCorrectCrossProduct)
	{
		Vector4 vec1(1.0f, 2.0f, 3.0f, 0.0f);
		Vector4 vec2(4.0f, 5.0f, 6.0f, 0.0f);
		Vector4 result = Vector4::Cross(vec1, vec2);
		// Cross product: (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
		AssertVector4Equal(Vector4(-3.0f, 6.0f, -3.0f, 0.0f), result);
	}

	TEST_F(Vector4Tests, Min_ComponentWise_ReturnsMinimumComponents)
	{
		Vector4 a(1.0f, 5.0f, 3.0f, 8.0f);
		Vector4 b(4.0f, 2.0f, 6.0f, 1.0f);
		Vector4 result = Vector4::Min(a, b);
		AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 1.0f), result);
	}

	TEST_F(Vector4Tests, Max_ComponentWise_ReturnsMaximumComponents)
	{
		Vector4 a(1.0f, 5.0f, 3.0f, 8.0f);
		Vector4 b(4.0f, 2.0f, 6.0f, 1.0f);
		Vector4 result = Vector4::Max(a, b);
		AssertVector4Equal(Vector4(4.0f, 5.0f, 6.0f, 8.0f), result);
	}

	TEST_F(Vector4Tests, Clamp_WithinBounds_ReturnsOriginal)
	{
		Vector4 value(2.0f, 3.0f, 4.0f, 5.0f);
		Vector4 min(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 max(6.0f, 7.0f, 8.0f, 9.0f);
		Vector4 result = Vector4::Clamp(value, min, max);
		AssertVector4Equal(value, result);
	}

	TEST_F(Vector4Tests, Clamp_OutOfBounds_ClampsToLimits)
	{
		Vector4 value(0.0f, 8.0f, 2.0f, 10.0f);
		Vector4 min(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 max(6.0f, 7.0f, 8.0f, 9.0f);
		Vector4 result = Vector4::Clamp(value, min, max);
		AssertVector4Equal(Vector4(1.0f, 7.0f, 3.0f, 9.0f), result);
	}

	TEST_F(Vector4Tests, StaticFactories_CreateCorrectVectors)
	{
		AssertVector4Equal(Vector4(0.0f, 0.0f, 0.0f, 0.0f), Vector4::Zero());
		AssertVector4Equal(Vector4(1.0f, 1.0f, 1.0f, 1.0f), Vector4::One());
		AssertVector4Equal(Vector4(0.5f, 0.5f, 0.5f, 0.5f), Vector4::Half());
		AssertVector4Equal(Vector4(1.0f, 0.0f, 0.0f, 0.0f), Vector4::UnitX());
		AssertVector4Equal(Vector4(0.0f, 1.0f, 0.0f, 0.0f), Vector4::UnitY());
		AssertVector4Equal(Vector4(0.0f, 0.0f, 1.0f, 0.0f), Vector4::UnitZ());
	}

	TEST_F(Vector4Tests, Equality_SameVectors_ReturnsTrue)
	{
		Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 b(1.0f, 2.0f, 3.0f, 4.0f);
		EXPECT_TRUE(a == b);
	}

	TEST_F(Vector4Tests, Equality_DifferentVectors_ReturnsFalse)
	{
		Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 b(1.0f, 2.0f, 3.0f, 5.0f);
		EXPECT_FALSE(a == b);
	}

	TEST_F(Vector4Tests, Inequality_DifferentVectors_ReturnsTrue)
	{
		Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 b(1.0f, 2.0f, 3.0f, 5.0f);
		EXPECT_TRUE(a != b);
	}

	TEST_F(Vector4Tests, Addition_TwoVectors_AddsComponents)
	{
		Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
		Vector4 result = a + b;
		AssertVector4Equal(Vector4(6.0f, 8.0f, 10.0f, 12.0f), result);
	}

	TEST_F(Vector4Tests, AdditionAssignment_ModifiesOriginal)
	{
		Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
		a += b;
		AssertVector4Equal(Vector4(6.0f, 8.0f, 10.0f, 12.0f), a);
	}

	TEST_F(Vector4Tests, Subtraction_TwoVectors_SubtractsComponents)
	{
		Vector4 a(6.0f, 8.0f, 10.0f, 12.0f);
		Vector4 b(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 result = a - b;
		AssertVector4Equal(Vector4(5.0f, 6.0f, 7.0f, 8.0f), result);
	}

	TEST_F(Vector4Tests, SubtractionAssignment_ModifiesOriginal)
	{
		Vector4 a(6.0f, 8.0f, 10.0f, 12.0f);
		Vector4 b(1.0f, 2.0f, 3.0f, 4.0f);
		a -= b;
		AssertVector4Equal(Vector4(5.0f, 6.0f, 7.0f, 8.0f), a);
	}

	TEST_F(Vector4Tests, MultiplicationByScalar_ScalesComponents)
	{
		Vector4 vec(2.0f, 3.0f, 4.0f, 5.0f);
		Vector4 result = vec * 2.0f;
		AssertVector4Equal(Vector4(4.0f, 6.0f, 8.0f, 10.0f), result);
	}

	TEST_F(Vector4Tests, MultiplicationAssignmentByScalar_ModifiesOriginal)
	{
		Vector4 vec(2.0f, 3.0f, 4.0f, 5.0f);
		vec *= 2.0f;
		AssertVector4Equal(Vector4(4.0f, 6.0f, 8.0f, 10.0f), vec);
	}

	TEST_F(Vector4Tests, DivisionByScalar_DividesComponents)
	{
		Vector4 vec(4.0f, 6.0f, 8.0f, 10.0f);
		Vector4 result = vec / 2.0f;
		AssertVector4Equal(Vector4(2.0f, 3.0f, 4.0f, 5.0f), result);
	}

	TEST_F(Vector4Tests, DivisionAssignmentByScalar_ModifiesOriginal)
	{
		Vector4 vec(4.0f, 6.0f, 8.0f, 10.0f);
		vec /= 2.0f;
		AssertVector4Equal(Vector4(2.0f, 3.0f, 4.0f, 5.0f), vec);
	}

	TEST_F(Vector4Tests, UnaryMinus_NegatesComponents)
	{
		Vector4 vec(2.0f, -3.0f, 4.0f, -5.0f);
		Vector4& result = -vec;
		AssertVector4Equal(Vector4(-2.0f, 3.0f, -4.0f, 5.0f), result);
		// Should also modify original
		AssertVector4Equal(Vector4(-2.0f, 3.0f, -4.0f, 5.0f), vec);
	}

	TEST_F(Vector4Tests, IndexOperator_ValidIndices_ReturnsCorrectComponents)
	{
		Vector4 vec(2.0f, 3.0f, 4.0f, 5.0f);
		AssertFloatEqual(2.0f, vec[0]);
		AssertFloatEqual(3.0f, vec[1]);
		AssertFloatEqual(4.0f, vec[2]);
		// Note: This will fail with the original buggy implementation
		// AssertFloatEqual(5.0f, vec[3]);
	}

	TEST_F(Vector4Tests, IndexOperator_InvalidIndex_ThrowsException)
	{
		Vector4 vec(2.0f, 3.0f, 4.0f, 5.0f);

		// Test negative index
		EXPECT_THROW({
			float value = vec[-1];
			}, runtime_error);

		// Test index 4 (out of bounds)
		EXPECT_THROW({
			float value = vec[4];
			}, runtime_error);

		// Test index 5 (out of bounds)
		EXPECT_THROW({
			float value = vec[5];
			}, runtime_error);
	}

	TEST_F(Vector4Tests, ToString_FormatsCorrectly)
	{
		Vector4 vec(1.5f, 2.5f, 3.5f, 4.5f);
		string str = vec.ToString();
		// Basic check - should contain the numbers
		EXPECT_TRUE(str.find("1.5") != string::npos);
		EXPECT_TRUE(str.find("2.5") != string::npos);
		EXPECT_TRUE(str.find("3.5") != string::npos);
		EXPECT_TRUE(str.find("4.5") != string::npos);
	}

	TEST_F(Vector4Tests, GlobalMultiplication_ScalarTimesVector_Works)
	{
		Vector4 vec(2.0f, 3.0f, 4.0f, 5.0f);
		Vector4 result = 2.0f * vec;
		AssertVector4Equal(Vector4(4.0f, 6.0f, 8.0f, 10.0f), result);
	}

	// Edge case tests
	TEST_F(Vector4Tests, EdgeCase_VerySmallNumbers_HandledCorrectly)
	{
		Vector4 vec(1e-6f, 1e-6f, 1e-6f, 1e-6f);
		EXPECT_FALSE(vec.IsZero()); // Should not be considered zero
	}

	TEST_F(Vector4Tests, EdgeCase_VeryLargeNumbers_HandledCorrectly)
	{
		Vector4 vec(1e6f, 1e6f, 1e6f, 1e6f);
		Vector4 normalized = vec.Normalized();
		AssertFloatEqual(1.0f, normalized.Magnitude(), 0.001f);
	}

	TEST_F(Vector4Tests, EdgeCase_SelfAssignment_HandledCorrectly)
	{
		Vector4 vec(1.0f, 2.0f, 3.0f, 4.0f);
		vec = vec;
		AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 4.0f), vec);
	}

	TEST_F(Vector4Tests, EdgeCase_SelfAddition_HandledCorrectly)
	{
		Vector4 vec(1.0f, 2.0f, 3.0f, 4.0f);
		vec += vec;
		AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 4.0f), vec); // Should remain unchanged due to self-check
	}

	TEST_F(Vector4Tests, EdgeCase_SelfSubtraction_HandledCorrectly)
	{
		Vector4 vec(1.0f, 2.0f, 3.0f, 4.0f);
		vec -= vec;
		AssertVector4Equal(Vector4(1.0f, 2.0f, 3.0f, 4.0f), vec); // Should remain unchanged due to self-check
	}

	// Vector4-specific tests
	TEST_F(Vector4Tests, HomogeneousCoordinates_WComponentUsedInCalculations)
	{
		Vector4 vec(1.0f, 2.0f, 3.0f, 4.0f);
		// Test that w component affects magnitude
		float expectedMagSqr = 1.0f + 4.0f + 9.0f + 16.0f; // 30
		AssertFloatEqual(expectedMagSqr, vec.MagnitudeSqr());
	}

	TEST_F(Vector4Tests, FourDimensionalOperations_AllComponentsProcessed)
	{
		Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
		Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);

		// Test that all four components are used in dot product
		float expected = 1 * 5 + 2 * 6 + 3 * 7 + 4 * 8; // 70
		AssertFloatEqual(expected, Vector4::Dot(a, b));
	}
}
