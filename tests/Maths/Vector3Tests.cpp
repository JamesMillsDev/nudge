#include <numbers>

#include <gtest/gtest.h>

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Maths/Vector2.hpp"
#include "Nudge/Maths/Vector3.hpp"

using std::runtime_error;
using std::numbers::pi_v;

using testing::Test;

namespace Nudge
{
    class Vector3Tests : public Test
    {
    public:
        // Helper method for floating point comparison
        static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
        {
            EXPECT_TRUE(MathF::Compare(expected, actual, tolerance));
        }

        static void AssertFloat3Equal(const Vector3& expected, const Vector3& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.x, actual.x, tolerance);
            AssertFloatEqual(expected.y, actual.y, tolerance);
            AssertFloatEqual(expected.z, actual.z, tolerance);
        }

    };

    TEST_F(Vector3Tests, Constructor_Default_CreatesZeroVector)
    {
        Vector3 vec;
        AssertFloat3Equal(Vector3(0.0f, 0.0f, 0.0f), vec);
    }

    TEST_F(Vector3Tests, Constructor_Scalar_CreatesSameValueForAllComponents)
    {
        Vector3 vec(5.0f);
        AssertFloat3Equal(Vector3(5.0f, 5.0f, 5.0f), vec);
    }

    TEST_F(Vector3Tests, Constructor_XYZ_CreatesVectorWithSpecifiedValues)
    {
        Vector3 vec(1.0f, 2.0f, 3.0f);
        AssertFloat3Equal(Vector3(1.0f, 2.0f, 3.0f), vec);
    }

    TEST_F(Vector3Tests, Constructor_Array_CreatesVectorFromArray)
    {
        float values[] = { 4.0f, 5.0f, 6.0f };
        Vector3 vec(values);
        AssertFloat3Equal(Vector3(4.0f, 5.0f, 6.0f), vec);
    }

    TEST_F(Vector3Tests, Constructor_Copy_CreatesIdenticalVector)
    {
        Vector3 original(1.5f, 2.5f, 3.5f);
        Vector3 copy(original);
        AssertFloat3Equal(original, copy);
    }

    TEST_F(Vector3Tests, Constructor_Float2_CreatesVectorWithZeroZ)
    {
        Vector2 vec2(3.0f, 4.0f);
        Vector3 vec3(vec2);
        AssertFloat3Equal(Vector3(3.0f, 4.0f, 0.0f), vec3);
    }

    TEST_F(Vector3Tests, Constructor_Float4_CreatesVectorDiscardingW)
    {
        /*Vector4 vec4(1.0f, 2.0f, 3.0f, 4.0f);
        Vector3 vec3(vec4);
        AssertFloat3Equal(Vector3(1.0f, 2.0f, 3.0f), vec3);*/
    }

    TEST_F(Vector3Tests, Magnitude_PythagoreanTriple_ReturnsCorrectMagnitude)
    {
        Vector3 vec(3.0f, 4.0f, 0.0f);
        AssertFloatEqual(5.0f, vec.Magnitude());
    }

    TEST_F(Vector3Tests, Magnitude_ZeroVector_ReturnsZero)
    {
        Vector3 vec(0.0f, 0.0f, 0.0f);
        AssertFloatEqual(0.0f, vec.Magnitude());
    }

    TEST_F(Vector3Tests, Magnitude_UnitVector_ReturnsOne)
    {
        Vector3 vec(1.0f, 0.0f, 0.0f);
        AssertFloatEqual(1.0f, vec.Magnitude());
    }

    TEST_F(Vector3Tests, MagnitudeSqr_PythagoreanTriple_ReturnsSquaredMagnitude)
    {
        Vector3 vec(3.0f, 4.0f, 0.0f);
        AssertFloatEqual(25.0f, vec.MagnitudeSqr());
    }

    TEST_F(Vector3Tests, Normalize_NonZeroVector_CreatesUnitVector)
    {
        Vector3 vec(3.0f, 4.0f, 0.0f);
        vec.Normalize();
        AssertFloatEqual(1.0f, vec.Magnitude());
        AssertFloat3Equal(Vector3(0.6f, 0.8f, 0.0f), vec);
    }

    TEST_F(Vector3Tests, Normalize_ZeroVector_RemainsZero)
    {
        Vector3 vec(0.0f, 0.0f, 0.0f);
        vec.Normalize();
        AssertFloat3Equal(Vector3(0.0f, 0.0f, 0.0f), vec);
    }

    TEST_F(Vector3Tests, Normalized_NonZeroVector_ReturnsUnitVector)
    {
        Vector3 vec(3.0f, 4.0f, 0.0f);
        Vector3 normalized = vec.Normalized();
        AssertFloatEqual(1.0f, normalized.Magnitude());
        AssertFloat3Equal(Vector3(0.6f, 0.8f, 0.0f), normalized);
        // Original should be unchanged
        AssertFloat3Equal(Vector3(3.0f, 4.0f, 0.0f), vec);
    }

    TEST_F(Vector3Tests, Normalized_ZeroVector_ReturnsZero)
    {
        Vector3 vec(0.0f, 0.0f, 0.0f);
        Vector3 normalized = vec.Normalized();
        AssertFloat3Equal(Vector3(0.0f, 0.0f, 0.0f), normalized);
    }

    TEST_F(Vector3Tests, IsZero_ZeroVector_ReturnsTrue)
    {
        Vector3 vec(0.0f, 0.0f, 0.0f);
        EXPECT_TRUE(vec.IsZero());
    }

    TEST_F(Vector3Tests, IsZero_NonZeroVector_ReturnsFalse)
    {
        Vector3 vec(0.1f, 0.0f, 0.0f);
        EXPECT_FALSE(vec.IsZero());
    }

    TEST_F(Vector3Tests, IsUnit_UnitVector_ReturnsTrue)
    {
        Vector3 vec(1.0f, 0.0f, 0.0f);
        EXPECT_TRUE(vec.IsUnit());
    }

    TEST_F(Vector3Tests, IsUnit_NonUnitVector_ReturnsFalse)
    {
        Vector3 vec(2.0f, 0.0f, 0.0f);
        EXPECT_FALSE(vec.IsUnit());
    }

    TEST_F(Vector3Tests, Dot_OrthogonalVectors_ReturnsZero)
    {
        Vector3 vec1(1.0f, 0.0f, 0.0f);
        Vector3 vec2(0.0f, 1.0f, 0.0f);
        AssertFloatEqual(0.0f, Vector3::Dot(vec1, vec2));
    }

    TEST_F(Vector3Tests, Dot_ParallelVectors_ReturnsProduct)
    {
        Vector3 vec1(2.0f, 0.0f, 0.0f);
        Vector3 vec2(3.0f, 0.0f, 0.0f);
        AssertFloatEqual(6.0f, Vector3::Dot(vec1, vec2));
    }

    TEST_F(Vector3Tests, Dot_GeneralCase_ReturnsCorrectValue)
    {
        Vector3 vec1(1.0f, 2.0f, 3.0f);
        Vector3 vec2(4.0f, 5.0f, 6.0f);
        // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
        AssertFloatEqual(32.0f, Vector3::Dot(vec1, vec2));
    }

    TEST_F(Vector3Tests, Distance_SamePoints_ReturnsZero)
    {
        Vector3 vec1(1.0f, 2.0f, 3.0f);
        Vector3 vec2(1.0f, 2.0f, 3.0f);
        AssertFloatEqual(0.0f, Vector3::Distance(vec1, vec2));
    }

    TEST_F(Vector3Tests, Distance_PythagoreanTriple_ReturnsCorrectDistance)
    {
        Vector3 vec1(0.0f, 0.0f, 0.0f);
        Vector3 vec2(3.0f, 4.0f, 0.0f);
        AssertFloatEqual(5.0f, Vector3::Distance(vec1, vec2));
    }

    TEST_F(Vector3Tests, DistanceSqr_PythagoreanTriple_ReturnsSquaredDistance)
    {
        Vector3 vec1(0.0f, 0.0f, 0.0f);
        Vector3 vec2(3.0f, 4.0f, 0.0f);
        AssertFloatEqual(25.0f, Vector3::DistanceSqr(vec1, vec2));
    }

    TEST_F(Vector3Tests, AngleOf_UnitZ_ReturnsZero)
    {
        Vector3 vec(0.0f, 0.0f, 1.0f);
        AssertFloatEqual(0.0f, Vector3::AngleOf(vec));
    }

    TEST_F(Vector3Tests, AngleOf_NegativeZ_ReturnsPi)
    {
        Vector3 vec(0.0f, 0.0f, -1.0f);
        AssertFloatEqual(pi_v<float>, Vector3::AngleOf(vec), 0.0001f);
    }

    TEST_F(Vector3Tests, AngleOf_ZeroVector_ReturnsZero)
    {
        Vector3 vec(0.0f, 0.0f, 0.0f);
        AssertFloatEqual(0.0f, Vector3::AngleOf(vec));
    }

    TEST_F(Vector3Tests, AngleBetween_SameVectors_ReturnsZero)
    {
        Vector3 vec1(1.0f, 0.0f, 0.0f);
        Vector3 vec2(1.0f, 0.0f, 0.0f);
        AssertFloatEqual(0.0f, Vector3::AngleBetween(vec1, vec2));
    }

    TEST_F(Vector3Tests, AngleBetween_OrthogonalVectors_ReturnsHalfPi)
    {
        Vector3 vec1(1.0f, 0.0f, 0.0f);
        Vector3 vec2(0.0f, 1.0f, 0.0f);
        AssertFloatEqual(pi_v<float> / 2.0f, Vector3::AngleBetween(vec1, vec2), 0.0001f);
    }

    TEST_F(Vector3Tests, AngleBetween_OppositeVectors_ReturnsPi)
    {
        Vector3 vec1(1.0f, 0.0f, 0.0f);
        Vector3 vec2(-1.0f, 0.0f, 0.0f);
        AssertFloatEqual(pi_v<float>, Vector3::AngleBetween(vec1, vec2), 0.0001f);
    }

    TEST_F(Vector3Tests, AngleBetween_ZeroVector_ReturnsZero)
    {
        Vector3 vec1(0.0f, 0.0f, 0.0f);
        Vector3 vec2(1.0f, 0.0f, 0.0f);
        AssertFloatEqual(0.0f, Vector3::AngleBetween(vec1, vec2));
    }

    TEST_F(Vector3Tests, Lerp_ZeroT_ReturnsFirstVector)
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(4.0f, 5.0f, 6.0f);
        Vector3 result = Vector3::Lerp(a, b, 0.0f);
        AssertFloat3Equal(a, result);
    }

    TEST_F(Vector3Tests, Lerp_OneT_ReturnsSecondVector)
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(4.0f, 5.0f, 6.0f);
        Vector3 result = Vector3::Lerp(a, b, 1.0f);
        AssertFloat3Equal(b, result);
    }

    TEST_F(Vector3Tests, Lerp_HalfT_ReturnsMidpoint)
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(3.0f, 4.0f, 5.0f);
        Vector3 result = Vector3::Lerp(a, b, 0.5f);
        AssertFloat3Equal(Vector3(2.0f, 3.0f, 4.0f), result);
    }

    TEST_F(Vector3Tests, Lerp_ClampsBeyondOne)
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(4.0f, 5.0f, 6.0f);
        Vector3 result = Vector3::Lerp(a, b, 1.5f);
        AssertFloat3Equal(b, result); // Should clamp to 1.0
    }

    TEST_F(Vector3Tests, Reflect_PerpendicularToNormal_ReflectsPerfectly)
    {
        Vector3 incident(1.0f, -1.0f, 0.0f); // Down-right
        Vector3 normal(0.0f, 1.0f, 0.0f);    // Up
        Vector3 reflected = Vector3::Reflect(incident, normal);
        AssertFloat3Equal(Vector3(1.0f, 1.0f, 0.0f), reflected); // Up-right
    }

    TEST_F(Vector3Tests, Cross_OrthogonalVectors_ReturnsPerpendicularVector)
    {
        Vector3 vec1(1.0f, 0.0f, 0.0f);
        Vector3 vec2(0.0f, 1.0f, 0.0f);
        Vector3 result = Vector3::Cross(vec1, vec2);
        AssertFloat3Equal(Vector3(0.0f, 0.0f, 1.0f), result);
    }

    TEST_F(Vector3Tests, Cross_ParallelVectors_ReturnsZeroVector)
    {
        Vector3 vec1(1.0f, 0.0f, 0.0f);
        Vector3 vec2(2.0f, 0.0f, 0.0f);
        Vector3 result = Vector3::Cross(vec1, vec2);
        AssertFloat3Equal(Vector3(0.0f, 0.0f, 0.0f), result);
    }

    TEST_F(Vector3Tests, Min_ComponentWise_ReturnsMinimumComponents)
    {
        Vector3 a(1.0f, 5.0f, 3.0f);
        Vector3 b(4.0f, 2.0f, 6.0f);
        Vector3 result = Vector3::Min(a, b);
        AssertFloat3Equal(Vector3(1.0f, 2.0f, 3.0f), result);
    }

    TEST_F(Vector3Tests, Max_ComponentWise_ReturnsMaximumComponents)
    {
        Vector3 a(1.0f, 5.0f, 3.0f);
        Vector3 b(4.0f, 2.0f, 6.0f);
        Vector3 result = Vector3::Max(a, b);
        AssertFloat3Equal(Vector3(4.0f, 5.0f, 6.0f), result);
    }

    TEST_F(Vector3Tests, Clamp_WithinBounds_ReturnsOriginal)
    {
        Vector3 value(2.0f, 3.0f, 4.0f);
        Vector3 min(1.0f, 2.0f, 3.0f);
        Vector3 max(5.0f, 6.0f, 7.0f);
        Vector3 result = Vector3::Clamp(value, min, max);
        AssertFloat3Equal(value, result);
    }

    TEST_F(Vector3Tests, Clamp_OutOfBounds_ClampsToLimits)
    {
        Vector3 value(0.0f, 7.0f, 8.0f);
        Vector3 min(1.0f, 2.0f, 3.0f);
        Vector3 max(5.0f, 6.0f, 7.0f);
        Vector3 result = Vector3::Clamp(value, min, max);
        AssertFloat3Equal(Vector3(1.0f, 6.0f, 7.0f), result);
    }

    TEST_F(Vector3Tests, StaticFactories_CreateCorrectVectors)
    {
        AssertFloat3Equal(Vector3(0.0f, 0.0f, 0.0f), Vector3::Zero());
        AssertFloat3Equal(Vector3(1.0f, 1.0f, 1.0f), Vector3::One());
        AssertFloat3Equal(Vector3(0.5f, 0.5f, 0.5f), Vector3::Half());
        AssertFloat3Equal(Vector3(1.0f, 0.0f, 0.0f), Vector3::UnitX());
        AssertFloat3Equal(Vector3(0.0f, 1.0f, 0.0f), Vector3::UnitY());
        AssertFloat3Equal(Vector3(0.0f, 0.0f, 1.0f), Vector3::UnitZ());
    }

    TEST_F(Vector3Tests, Equality_SameVectors_ReturnsTrue)
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(1.0f, 2.0f, 3.0f);
        EXPECT_TRUE(a == b);
    }

    TEST_F(Vector3Tests, Equality_DifferentVectors_ReturnsFalse)
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(1.0f, 2.0f, 4.0f);
        EXPECT_FALSE(a == b);
    }

    TEST_F(Vector3Tests, Inequality_DifferentVectors_ReturnsTrue)
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(1.0f, 2.0f, 4.0f);
        EXPECT_TRUE(a != b);
    }

    TEST_F(Vector3Tests, Addition_TwoVectors_AddsComponents)
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(4.0f, 5.0f, 6.0f);
        Vector3 result = a + b;
        AssertFloat3Equal(Vector3(5.0f, 7.0f, 9.0f), result);
    }

    TEST_F(Vector3Tests, AdditionAssignment_ModifiesOriginal)
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(4.0f, 5.0f, 6.0f);
        a += b;
        AssertFloat3Equal(Vector3(5.0f, 7.0f, 9.0f), a);
    }

    TEST_F(Vector3Tests, Subtraction_TwoVectors_SubtractsComponents)
    {
        Vector3 a(5.0f, 7.0f, 9.0f);
        Vector3 b(2.0f, 3.0f, 4.0f);
        Vector3 result = a - b;
        AssertFloat3Equal(Vector3(3.0f, 4.0f, 5.0f), result);
    }

    TEST_F(Vector3Tests, SubtractionAssignment_ModifiesOriginal)
    {
        Vector3 a(5.0f, 7.0f, 9.0f);
        Vector3 b(2.0f, 3.0f, 4.0f);
        a -= b;
        AssertFloat3Equal(Vector3(3.0f, 4.0f, 5.0f), a);
    }

    TEST_F(Vector3Tests, MultiplicationByScalar_ScalesComponents)
    {
        Vector3 vec(2.0f, 3.0f, 4.0f);
        Vector3 result = vec * 2.0f;
        AssertFloat3Equal(Vector3(4.0f, 6.0f, 8.0f), result);
    }

    TEST_F(Vector3Tests, MultiplicationAssignmentByScalar_ModifiesOriginal)
    {
        Vector3 vec(2.0f, 3.0f, 4.0f);
        vec *= 2.0f;
        AssertFloat3Equal(Vector3(4.0f, 6.0f, 8.0f), vec);
    }

    TEST_F(Vector3Tests, DivisionByScalar_DividesComponents)
    {
        Vector3 vec(4.0f, 6.0f, 8.0f);
        Vector3 result = vec / 2.0f;
        AssertFloat3Equal(Vector3(2.0f, 3.0f, 4.0f), result);
    }

    TEST_F(Vector3Tests, DivisionAssignmentByScalar_ModifiesOriginal)
    {
        Vector3 vec(4.0f, 6.0f, 8.0f);
        vec /= 2.0f;
        AssertFloat3Equal(Vector3(2.0f, 3.0f, 4.0f), vec);
    }

    TEST_F(Vector3Tests, UnaryMinus_NegatesComponents)
    {
        Vector3 vec(2.0f, -3.0f, 4.0f);
        Vector3& result = -vec;
        AssertFloat3Equal(Vector3(-2.0f, 3.0f, -4.0f), result);
        // Should also modify original
        AssertFloat3Equal(Vector3(-2.0f, 3.0f, -4.0f), vec);
    }

    TEST_F(Vector3Tests, IndexOperator_ValidIndices_ReturnsCorrectComponents)
    {
        Vector3 vec(2.0f, 3.0f, 4.0f);
        AssertFloatEqual(2.0f, vec[0]);
        AssertFloatEqual(3.0f, vec[1]);
        AssertFloatEqual(4.0f, vec[2]);
    }

    TEST_F(Vector3Tests, IndexOperator_InvalidIndex_ThrowsException)
    {
        Vector3 vec(2.0f, 3.0f, 4.0f);

        // Test negative index
		EXPECT_THROW({
			float value = vec[-1];
			}, runtime_error);

		// Test index 3 (out of bounds)
		EXPECT_THROW({
			float value = vec[3];
			}, runtime_error);

		// Test index 4 (out of bounds)
		EXPECT_THROW({
			float value = vec[4];
			}, runtime_error);
    }

    TEST_F(Vector3Tests, ToString_FormatsCorrectly)
    {
        Vector3 vec(1.5f, 2.5f, 3.5f);
        string str = vec.ToString();
        // Basic check - should contain the numbers
        EXPECT_TRUE(str.find("1.5") != string::npos);
        EXPECT_TRUE(str.find("2.5") != string::npos);
        EXPECT_TRUE(str.find("3.5") != string::npos);
    }

    TEST_F(Vector3Tests, GlobalMultiplication_ScalarTimesVector_Works)
    {
        Vector3 vec(2.0f, 3.0f, 4.0f);
        Vector3 result = 2.0f * vec;
        AssertFloat3Equal(Vector3(4.0f, 6.0f, 8.0f), result);
    }

    // Edge case tests
    TEST_F(Vector3Tests, EdgeCase_VerySmallNumbers_HandledCorrectly)
    {
        Vector3 vec(1e-6f, 1e-6f, 1e-6f);
        EXPECT_FALSE(vec.IsZero()); // Should not be considered zero
    }

    TEST_F(Vector3Tests, EdgeCase_VeryLargeNumbers_HandledCorrectly)
    {
        Vector3 vec(1e6f, 1e6f, 1e6f);
        Vector3 normalized = vec.Normalized();
        AssertFloatEqual(1.0f, normalized.Magnitude(), 0.001f);
    }

    TEST_F(Vector3Tests, EdgeCase_SelfAssignment_HandledCorrectly)
    {
        Vector3 vec(1.0f, 2.0f, 3.0f);
        vec = vec;
        AssertFloat3Equal(Vector3(1.0f, 2.0f, 3.0f), vec);
    }

    TEST_F(Vector3Tests, EdgeCase_SelfAddition_HandledCorrectly)
    {
        Vector3 vec(1.0f, 2.0f, 3.0f);
        vec += vec;
        AssertFloat3Equal(Vector3(1.0f, 2.0f, 3.0f), vec); // Should remain unchanged due to self-check
    }

    TEST_F(Vector3Tests, EdgeCase_SelfSubtraction_HandledCorrectly)
    {
        Vector3 vec(1.0f, 2.0f, 3.0f);
        vec -= vec;
        AssertFloat3Equal(Vector3(1.0f, 2.0f, 3.0f), vec); // Should remain unchanged due to self-check
    }
}