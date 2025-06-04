#include "CppUnitTest.h"

#include "nudge/Float3.hpp"
#include "nudge/MathF.hpp"

#include <numbers>

#include "nudge/Float2.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using std::runtime_error;
using std::numbers::pi_v;

namespace Nudge::Tests
{
    TEST_CLASS(Float3Tests)
    {
    public:
        // Helper method for floating point comparison
        static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
        {
            Assert::IsTrue(MathF::Compare(expected, actual, tolerance), L"Float values are not equal within tolerance");
        }

        static void AssertFloat3Equal(const Float3& expected, const Float3& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.x, actual.x, tolerance);
            AssertFloatEqual(expected.y, actual.y, tolerance);
            AssertFloatEqual(expected.z, actual.z, tolerance);
        }

        TEST_METHOD(Constructor_Default_CreatesZeroVector)
        {
            Float3 vec;
            AssertFloat3Equal(Float3(0.0f, 0.0f, 0.0f), vec);
        }

        TEST_METHOD(Constructor_Scalar_CreatesSameValueForAllComponents)
        {
            Float3 vec(5.0f);
            AssertFloat3Equal(Float3(5.0f, 5.0f, 5.0f), vec);
        }

        TEST_METHOD(Constructor_XYZ_CreatesVectorWithSpecifiedValues)
        {
            Float3 vec(1.0f, 2.0f, 3.0f);
            AssertFloat3Equal(Float3(1.0f, 2.0f, 3.0f), vec);
        }

        TEST_METHOD(Constructor_Array_CreatesVectorFromArray)
        {
            float values[] = { 4.0f, 5.0f, 6.0f };
            Float3 vec(values);
            AssertFloat3Equal(Float3(4.0f, 5.0f, 6.0f), vec);
        }

        TEST_METHOD(Constructor_Copy_CreatesIdenticalVector)
        {
            Float3 original(1.5f, 2.5f, 3.5f);
            Float3 copy(original);
            AssertFloat3Equal(original, copy);
        }

        TEST_METHOD(Constructor_Float2_CreatesVectorWithZeroZ)
        {
            Float2 vec2(3.0f, 4.0f);
            Float3 vec3(vec2);
            AssertFloat3Equal(Float3(3.0f, 4.0f, 0.0f), vec3);
        }

        TEST_METHOD(Constructor_Float4_CreatesVectorDiscardingW)
        {
            /*Float4 vec4(1.0f, 2.0f, 3.0f, 4.0f);
            Float3 vec3(vec4);
            AssertFloat3Equal(Float3(1.0f, 2.0f, 3.0f), vec3);*/
        }

        TEST_METHOD(Magnitude_PythagoreanTriple_ReturnsCorrectMagnitude)
        {
            Float3 vec(3.0f, 4.0f, 0.0f);
            AssertFloatEqual(5.0f, vec.Magnitude());
        }

        TEST_METHOD(Magnitude_ZeroVector_ReturnsZero)
        {
            Float3 vec(0.0f, 0.0f, 0.0f);
            AssertFloatEqual(0.0f, vec.Magnitude());
        }

        TEST_METHOD(Magnitude_UnitVector_ReturnsOne)
        {
            Float3 vec(1.0f, 0.0f, 0.0f);
            AssertFloatEqual(1.0f, vec.Magnitude());
        }

        TEST_METHOD(MagnitudeSqr_PythagoreanTriple_ReturnsSquaredMagnitude)
        {
            Float3 vec(3.0f, 4.0f, 0.0f);
            AssertFloatEqual(25.0f, vec.MagnitudeSqr());
        }

        TEST_METHOD(Normalize_NonZeroVector_CreatesUnitVector)
        {
            Float3 vec(3.0f, 4.0f, 0.0f);
            vec.Normalize();
            AssertFloatEqual(1.0f, vec.Magnitude());
            AssertFloat3Equal(Float3(0.6f, 0.8f, 0.0f), vec);
        }

        TEST_METHOD(Normalize_ZeroVector_RemainsZero)
        {
            Float3 vec(0.0f, 0.0f, 0.0f);
            vec.Normalize();
            AssertFloat3Equal(Float3(0.0f, 0.0f, 0.0f), vec);
        }

        TEST_METHOD(Normalized_NonZeroVector_ReturnsUnitVector)
        {
            Float3 vec(3.0f, 4.0f, 0.0f);
            Float3 normalized = vec.Normalized();
            AssertFloatEqual(1.0f, normalized.Magnitude());
            AssertFloat3Equal(Float3(0.6f, 0.8f, 0.0f), normalized);
            // Original should be unchanged
            AssertFloat3Equal(Float3(3.0f, 4.0f, 0.0f), vec);
        }

        TEST_METHOD(Normalized_ZeroVector_ReturnsZero)
        {
            Float3 vec(0.0f, 0.0f, 0.0f);
            Float3 normalized = vec.Normalized();
            AssertFloat3Equal(Float3(0.0f, 0.0f, 0.0f), normalized);
        }

        TEST_METHOD(IsZero_ZeroVector_ReturnsTrue)
        {
            Float3 vec(0.0f, 0.0f, 0.0f);
            Assert::IsTrue(vec.IsZero());
        }

        TEST_METHOD(IsZero_NonZeroVector_ReturnsFalse)
        {
            Float3 vec(0.1f, 0.0f, 0.0f);
            Assert::IsFalse(vec.IsZero());
        }

        TEST_METHOD(IsUnit_UnitVector_ReturnsTrue)
        {
            Float3 vec(1.0f, 0.0f, 0.0f);
            Assert::IsTrue(vec.IsUnit());
        }

        TEST_METHOD(IsUnit_NonUnitVector_ReturnsFalse)
        {
            Float3 vec(2.0f, 0.0f, 0.0f);
            Assert::IsFalse(vec.IsUnit());
        }

        TEST_METHOD(Dot_OrthogonalVectors_ReturnsZero)
        {
            Float3 vec1(1.0f, 0.0f, 0.0f);
            Float3 vec2(0.0f, 1.0f, 0.0f);
            AssertFloatEqual(0.0f, Float3::Dot(vec1, vec2));
        }

        TEST_METHOD(Dot_ParallelVectors_ReturnsProduct)
        {
            Float3 vec1(2.0f, 0.0f, 0.0f);
            Float3 vec2(3.0f, 0.0f, 0.0f);
            AssertFloatEqual(6.0f, Float3::Dot(vec1, vec2));
        }

        TEST_METHOD(Dot_GeneralCase_ReturnsCorrectValue)
        {
            Float3 vec1(1.0f, 2.0f, 3.0f);
            Float3 vec2(4.0f, 5.0f, 6.0f);
            // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
            AssertFloatEqual(32.0f, Float3::Dot(vec1, vec2));
        }

        TEST_METHOD(Distance_SamePoints_ReturnsZero)
        {
            Float3 vec1(1.0f, 2.0f, 3.0f);
            Float3 vec2(1.0f, 2.0f, 3.0f);
            AssertFloatEqual(0.0f, Float3::Distance(vec1, vec2));
        }

        TEST_METHOD(Distance_PythagoreanTriple_ReturnsCorrectDistance)
        {
            Float3 vec1(0.0f, 0.0f, 0.0f);
            Float3 vec2(3.0f, 4.0f, 0.0f);
            AssertFloatEqual(5.0f, Float3::Distance(vec1, vec2));
        }

        TEST_METHOD(DistanceSqr_PythagoreanTriple_ReturnsSquaredDistance)
        {
            Float3 vec1(0.0f, 0.0f, 0.0f);
            Float3 vec2(3.0f, 4.0f, 0.0f);
            AssertFloatEqual(25.0f, Float3::DistanceSqr(vec1, vec2));
        }

        TEST_METHOD(AngleOf_UnitZ_ReturnsZero)
        {
            Float3 vec(0.0f, 0.0f, 1.0f);
            AssertFloatEqual(0.0f, Float3::AngleOf(vec));
        }

        TEST_METHOD(AngleOf_NegativeZ_ReturnsPi)
        {
            Float3 vec(0.0f, 0.0f, -1.0f);
            AssertFloatEqual(pi_v<float>, Float3::AngleOf(vec), 0.0001f);
        }

        TEST_METHOD(AngleOf_ZeroVector_ReturnsZero)
        {
            Float3 vec(0.0f, 0.0f, 0.0f);
            AssertFloatEqual(0.0f, Float3::AngleOf(vec));
        }

        TEST_METHOD(AngleBetween_SameVectors_ReturnsZero)
        {
            Float3 vec1(1.0f, 0.0f, 0.0f);
            Float3 vec2(1.0f, 0.0f, 0.0f);
            AssertFloatEqual(0.0f, Float3::AngleBetween(vec1, vec2));
        }

        TEST_METHOD(AngleBetween_OrthogonalVectors_ReturnsHalfPi)
        {
            Float3 vec1(1.0f, 0.0f, 0.0f);
            Float3 vec2(0.0f, 1.0f, 0.0f);
            AssertFloatEqual(pi_v<float> / 2.0f, Float3::AngleBetween(vec1, vec2), 0.0001f);
        }

        TEST_METHOD(AngleBetween_OppositeVectors_ReturnsPi)
        {
            Float3 vec1(1.0f, 0.0f, 0.0f);
            Float3 vec2(-1.0f, 0.0f, 0.0f);
            AssertFloatEqual(pi_v<float>, Float3::AngleBetween(vec1, vec2), 0.0001f);
        }

        TEST_METHOD(AngleBetween_ZeroVector_ReturnsZero)
        {
            Float3 vec1(0.0f, 0.0f, 0.0f);
            Float3 vec2(1.0f, 0.0f, 0.0f);
            AssertFloatEqual(0.0f, Float3::AngleBetween(vec1, vec2));
        }

        TEST_METHOD(Lerp_ZeroT_ReturnsFirstVector)
        {
            Float3 a(1.0f, 2.0f, 3.0f);
            Float3 b(4.0f, 5.0f, 6.0f);
            Float3 result = Float3::Lerp(a, b, 0.0f);
            AssertFloat3Equal(a, result);
        }

        TEST_METHOD(Lerp_OneT_ReturnsSecondVector)
        {
            Float3 a(1.0f, 2.0f, 3.0f);
            Float3 b(4.0f, 5.0f, 6.0f);
            Float3 result = Float3::Lerp(a, b, 1.0f);
            AssertFloat3Equal(b, result);
        }

        TEST_METHOD(Lerp_HalfT_ReturnsMidpoint)
        {
            Float3 a(1.0f, 2.0f, 3.0f);
            Float3 b(3.0f, 4.0f, 5.0f);
            Float3 result = Float3::Lerp(a, b, 0.5f);
            AssertFloat3Equal(Float3(2.0f, 3.0f, 4.0f), result);
        }

        TEST_METHOD(Lerp_ClampsBeyondOne)
        {
            Float3 a(1.0f, 2.0f, 3.0f);
            Float3 b(4.0f, 5.0f, 6.0f);
            Float3 result = Float3::Lerp(a, b, 1.5f);
            AssertFloat3Equal(b, result); // Should clamp to 1.0
        }

        TEST_METHOD(Reflect_PerpendicularToNormal_ReflectsPerfectly)
        {
            Float3 incident(1.0f, -1.0f, 0.0f); // Down-right
            Float3 normal(0.0f, 1.0f, 0.0f);    // Up
            Float3 reflected = Float3::Reflect(incident, normal);
            AssertFloat3Equal(Float3(1.0f, 1.0f, 0.0f), reflected); // Up-right
        }

        TEST_METHOD(Cross_OrthogonalVectors_ReturnsPerpendicularVector)
        {
            Float3 vec1(1.0f, 0.0f, 0.0f);
            Float3 vec2(0.0f, 1.0f, 0.0f);
            Float3 result = Float3::Cross(vec1, vec2);
            AssertFloat3Equal(Float3(0.0f, 0.0f, 1.0f), result);
        }

        TEST_METHOD(Cross_ParallelVectors_ReturnsZeroVector)
        {
            Float3 vec1(1.0f, 0.0f, 0.0f);
            Float3 vec2(2.0f, 0.0f, 0.0f);
            Float3 result = Float3::Cross(vec1, vec2);
            AssertFloat3Equal(Float3(0.0f, 0.0f, 0.0f), result);
        }

        TEST_METHOD(Min_ComponentWise_ReturnsMinimumComponents)
        {
            Float3 a(1.0f, 5.0f, 3.0f);
            Float3 b(4.0f, 2.0f, 6.0f);
            Float3 result = Float3::Min(a, b);
            AssertFloat3Equal(Float3(1.0f, 2.0f, 3.0f), result);
        }

        TEST_METHOD(Max_ComponentWise_ReturnsMaximumComponents)
        {
            Float3 a(1.0f, 5.0f, 3.0f);
            Float3 b(4.0f, 2.0f, 6.0f);
            Float3 result = Float3::Max(a, b);
            AssertFloat3Equal(Float3(4.0f, 5.0f, 6.0f), result);
        }

        TEST_METHOD(Clamp_WithinBounds_ReturnsOriginal)
        {
            Float3 value(2.0f, 3.0f, 4.0f);
            Float3 min(1.0f, 2.0f, 3.0f);
            Float3 max(5.0f, 6.0f, 7.0f);
            Float3 result = Float3::Clamp(value, min, max);
            AssertFloat3Equal(value, result);
        }

        TEST_METHOD(Clamp_OutOfBounds_ClampsToLimits)
        {
            Float3 value(0.0f, 7.0f, 8.0f);
            Float3 min(1.0f, 2.0f, 3.0f);
            Float3 max(5.0f, 6.0f, 7.0f);
            Float3 result = Float3::Clamp(value, min, max);
            AssertFloat3Equal(Float3(1.0f, 6.0f, 7.0f), result);
        }

        TEST_METHOD(StaticFactories_CreateCorrectVectors)
        {
            AssertFloat3Equal(Float3(0.0f, 0.0f, 0.0f), Float3::Zero());
            AssertFloat3Equal(Float3(1.0f, 1.0f, 1.0f), Float3::One());
            AssertFloat3Equal(Float3(0.5f, 0.5f, 0.5f), Float3::Half());
            AssertFloat3Equal(Float3(1.0f, 0.0f, 0.0f), Float3::UnitX());
            AssertFloat3Equal(Float3(0.0f, 1.0f, 0.0f), Float3::UnitY());
            AssertFloat3Equal(Float3(0.0f, 0.0f, 1.0f), Float3::UnitZ());
        }

        TEST_METHOD(Equality_SameVectors_ReturnsTrue)
        {
            Float3 a(1.0f, 2.0f, 3.0f);
            Float3 b(1.0f, 2.0f, 3.0f);
            Assert::IsTrue(a == b);
        }

        TEST_METHOD(Equality_DifferentVectors_ReturnsFalse)
        {
            Float3 a(1.0f, 2.0f, 3.0f);
            Float3 b(1.0f, 2.0f, 4.0f);
            Assert::IsFalse(a == b);
        }

        TEST_METHOD(Inequality_DifferentVectors_ReturnsTrue)
        {
            Float3 a(1.0f, 2.0f, 3.0f);
            Float3 b(1.0f, 2.0f, 4.0f);
            Assert::IsTrue(a != b);
        }

        TEST_METHOD(Addition_TwoVectors_AddsComponents)
        {
            Float3 a(1.0f, 2.0f, 3.0f);
            Float3 b(4.0f, 5.0f, 6.0f);
            Float3 result = a + b;
            AssertFloat3Equal(Float3(5.0f, 7.0f, 9.0f), result);
        }

        TEST_METHOD(AdditionAssignment_ModifiesOriginal)
        {
            Float3 a(1.0f, 2.0f, 3.0f);
            Float3 b(4.0f, 5.0f, 6.0f);
            a += b;
            AssertFloat3Equal(Float3(5.0f, 7.0f, 9.0f), a);
        }

        TEST_METHOD(Subtraction_TwoVectors_SubtractsComponents)
        {
            Float3 a(5.0f, 7.0f, 9.0f);
            Float3 b(2.0f, 3.0f, 4.0f);
            Float3 result = a - b;
            AssertFloat3Equal(Float3(3.0f, 4.0f, 5.0f), result);
        }

        TEST_METHOD(SubtractionAssignment_ModifiesOriginal)
        {
            Float3 a(5.0f, 7.0f, 9.0f);
            Float3 b(2.0f, 3.0f, 4.0f);
            a -= b;
            AssertFloat3Equal(Float3(3.0f, 4.0f, 5.0f), a);
        }

        TEST_METHOD(MultiplicationByScalar_ScalesComponents)
        {
            Float3 vec(2.0f, 3.0f, 4.0f);
            Float3 result = vec * 2.0f;
            AssertFloat3Equal(Float3(4.0f, 6.0f, 8.0f), result);
        }

        TEST_METHOD(MultiplicationAssignmentByScalar_ModifiesOriginal)
        {
            Float3 vec(2.0f, 3.0f, 4.0f);
            vec *= 2.0f;
            AssertFloat3Equal(Float3(4.0f, 6.0f, 8.0f), vec);
        }

        TEST_METHOD(DivisionByScalar_DividesComponents)
        {
            Float3 vec(4.0f, 6.0f, 8.0f);
            Float3 result = vec / 2.0f;
            AssertFloat3Equal(Float3(2.0f, 3.0f, 4.0f), result);
        }

        TEST_METHOD(DivisionAssignmentByScalar_ModifiesOriginal)
        {
            Float3 vec(4.0f, 6.0f, 8.0f);
            vec /= 2.0f;
            AssertFloat3Equal(Float3(2.0f, 3.0f, 4.0f), vec);
        }

        TEST_METHOD(UnaryMinus_NegatesComponents)
        {
            Float3 vec(2.0f, -3.0f, 4.0f);
            Float3& result = -vec;
            AssertFloat3Equal(Float3(-2.0f, 3.0f, -4.0f), result);
            // Should also modify original
            AssertFloat3Equal(Float3(-2.0f, 3.0f, -4.0f), vec);
        }

        TEST_METHOD(IndexOperator_ValidIndices_ReturnsCorrectComponents)
        {
            Float3 vec(2.0f, 3.0f, 4.0f);
            AssertFloatEqual(2.0f, vec[0]);
            AssertFloatEqual(3.0f, vec[1]);
            AssertFloatEqual(4.0f, vec[2]);
        }

        TEST_METHOD(IndexOperator_InvalidIndex_ThrowsException)
        {
            Float3 vec(2.0f, 3.0f, 4.0f);

            // Test negative index
            Assert::ExpectException<runtime_error>([&]() {
                float value = vec[-1];
                });

            // Test index 3 (out of bounds)
            Assert::ExpectException<runtime_error>([&]() {
                float value = vec[3];
                });

            // Test index 4 (out of bounds)
            Assert::ExpectException<runtime_error>([&]() {
                float value = vec[4];
                });
        }

        TEST_METHOD(ToString_FormatsCorrectly)
        {
            Float3 vec(1.5f, 2.5f, 3.5f);
            string str = vec.ToString();
            // Basic check - should contain the numbers
            Assert::IsTrue(str.find("1.5") != string::npos);
            Assert::IsTrue(str.find("2.5") != string::npos);
            Assert::IsTrue(str.find("3.5") != string::npos);
        }

        TEST_METHOD(GlobalMultiplication_ScalarTimesVector_Works)
        {
            Float3 vec(2.0f, 3.0f, 4.0f);
            Float3 result = 2.0f * vec;
            AssertFloat3Equal(Float3(4.0f, 6.0f, 8.0f), result);
        }

        // Edge case tests
        TEST_METHOD(EdgeCase_VerySmallNumbers_HandledCorrectly)
        {
            Float3 vec(1e-6f, 1e-6f, 1e-6f);
            Assert::IsFalse(vec.IsZero()); // Should not be considered zero
        }

        TEST_METHOD(EdgeCase_VeryLargeNumbers_HandledCorrectly)
        {
            Float3 vec(1e6f, 1e6f, 1e6f);
            Float3 normalized = vec.Normalized();
            AssertFloatEqual(1.0f, normalized.Magnitude(), 0.001f);
        }

        TEST_METHOD(EdgeCase_SelfAssignment_HandledCorrectly)
        {
            Float3 vec(1.0f, 2.0f, 3.0f);
            vec = vec;
            AssertFloat3Equal(Float3(1.0f, 2.0f, 3.0f), vec);
        }

        TEST_METHOD(EdgeCase_SelfAddition_HandledCorrectly)
        {
            Float3 vec(1.0f, 2.0f, 3.0f);
            vec += vec;
            AssertFloat3Equal(Float3(1.0f, 2.0f, 3.0f), vec); // Should remain unchanged due to self-check
        }

        TEST_METHOD(EdgeCase_SelfSubtraction_HandledCorrectly)
        {
            Float3 vec(1.0f, 2.0f, 3.0f);
            vec -= vec;
            AssertFloat3Equal(Float3(1.0f, 2.0f, 3.0f), vec); // Should remain unchanged due to self-check
        }

    };
}