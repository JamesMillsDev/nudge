#include <numbers>

#include "CppUnitTest.h"

#include "nudge/Float2.hpp"
#include "nudge/MathF.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using std::runtime_error;
using std::numbers::pi_v;

namespace Nudge::Tests
{
    TEST_CLASS(Float2Tests)
    {
    public:
        // Helper method for floating point comparison
        static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
        {
            Assert::IsTrue(MathF::Compare(expected, actual, tolerance), L"Float values are not equal within tolerance");
        }

        static void AssertFloat2Equal(const Float2& expected, const Float2& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.x, actual.x, tolerance);
            AssertFloatEqual(expected.y, actual.y, tolerance);
        }

        TEST_METHOD(Constructor_Default_CreatesZeroVector)
        {
            Float2 vec;
            AssertFloat2Equal(Float2(0.0f, 0.0f), vec);
        }

        TEST_METHOD(Constructor_Scalar_CreatesSameValueForBothComponents)
        {
            Float2 vec(5.0f);
            AssertFloat2Equal(Float2(5.0f, 5.0f), vec);
        }

        TEST_METHOD(Constructor_XY_CreatesVectorWithSpecifiedValues)
        {
            Float2 vec(3.0f, 4.0f);
            AssertFloat2Equal(Float2(3.0f, 4.0f), vec);
        }

        TEST_METHOD(Constructor_Array_CreatesVectorFromArray)
        {
            float values[] = { 2.0f, 7.0f };
            Float2 vec(values);
            AssertFloat2Equal(Float2(2.0f, 7.0f), vec);
        }

        TEST_METHOD(Constructor_Copy_CreatesIdenticalVector)
        {
            Float2 original(1.5f, 2.5f);
            Float2 copy(original);
            AssertFloat2Equal(original, copy);
        }

        TEST_METHOD(Magnitude_UnitVector_ReturnsOne)
        {
            Float2 vec(1.0f, 0.0f);
            AssertFloatEqual(1.0f, vec.Magnitude());
        }

        TEST_METHOD(Magnitude_PythagoreanTriple_ReturnsCorrectMagnitude)
        {
            Float2 vec(3.0f, 4.0f);
            AssertFloatEqual(5.0f, vec.Magnitude());
        }

        TEST_METHOD(Magnitude_ZeroVector_ReturnsZero)
        {
            Float2 vec(0.0f, 0.0f);
            AssertFloatEqual(0.0f, vec.Magnitude());
        }

        TEST_METHOD(MagnitudeSqr_PythagoreanTriple_ReturnsSquaredMagnitude)
        {
            Float2 vec(3.0f, 4.0f);
            AssertFloatEqual(25.0f, vec.MagnitudeSqr());
        }

        TEST_METHOD(Normalize_NonZeroVector_CreatesUnitVector)
        {
            Float2 vec(3.0f, 4.0f);
            vec.Normalize();
            AssertFloatEqual(1.0f, vec.Magnitude());
            AssertFloat2Equal(Float2(0.6f, 0.8f), vec);
        }

        TEST_METHOD(Normalize_ZeroVector_RemainsZero)
        {
            Float2 vec(0.0f, 0.0f);
            vec.Normalize();
            AssertFloat2Equal(Float2(0.0f, 0.0f), vec);
        }

        TEST_METHOD(Normalized_NonZeroVector_ReturnsUnitVector)
        {
            Float2 vec(3.0f, 4.0f);
            Float2 normalized = vec.Normalized();
            AssertFloatEqual(1.0f, normalized.Magnitude());
            AssertFloat2Equal(Float2(0.6f, 0.8f), normalized);
            // Original should be unchanged
            AssertFloat2Equal(Float2(3.0f, 4.0f), vec);
        }

        TEST_METHOD(Normalized_ZeroVector_ReturnsZero)
        {
            Float2 vec(0.0f, 0.0f);
            Float2 normalized = vec.Normalized();
            AssertFloat2Equal(Float2(0.0f, 0.0f), normalized);
        }

        TEST_METHOD(IsZero_ZeroVector_ReturnsTrue)
        {
            Float2 vec(0.0f, 0.0f);
            Assert::IsTrue(vec.IsZero());
        }

        TEST_METHOD(IsZero_NonZeroVector_ReturnsFalse)
        {
            Float2 vec(0.1f, 0.0f);
            Assert::IsFalse(vec.IsZero());
        }

        TEST_METHOD(IsUnit_UnitVector_ReturnsTrue)
        {
            Float2 vec(1.0f, 0.0f);
            Assert::IsTrue(vec.IsUnit());
        }

        TEST_METHOD(IsUnit_NonUnitVector_ReturnsFalse)
        {
            Float2 vec(2.0f, 0.0f);
            Assert::IsFalse(vec.IsUnit());
        }

        TEST_METHOD(Dot_OrthogonalVectors_ReturnsZero)
        {
            Float2 vec1(1.0f, 0.0f);
            Float2 vec2(0.0f, 1.0f);
            AssertFloatEqual(0.0f, Float2::Dot(vec1, vec2));
        }

        TEST_METHOD(Dot_ParallelVectors_ReturnsProduct)
        {
            Float2 vec1(2.0f, 0.0f);
            Float2 vec2(3.0f, 0.0f);
            AssertFloatEqual(6.0f, Float2::Dot(vec1, vec2));
        }

        TEST_METHOD(Dot_GeneralCase_ReturnsCorrectValue)
        {
            Float2 vec1(2.0f, 3.0f);
            Float2 vec2(4.0f, 5.0f);
            // 2*4 + 3*5 = 8 + 15 = 23
            AssertFloatEqual(23.0f, Float2::Dot(vec1, vec2));
        }

        TEST_METHOD(Distance_SamePoints_ReturnsZero)
        {
            Float2 vec1(1.0f, 2.0f);
            Float2 vec2(1.0f, 2.0f);
            AssertFloatEqual(0.0f, Float2::Distance(vec1, vec2));
        }

        TEST_METHOD(Distance_PythagoreanTriple_ReturnsCorrectDistance)
        {
            Float2 vec1(0.0f, 0.0f);
            Float2 vec2(3.0f, 4.0f);
            AssertFloatEqual(5.0f, Float2::Distance(vec1, vec2));
        }

        TEST_METHOD(DistanceSqr_PythagoreanTriple_ReturnsSquaredDistance)
        {
            Float2 vec1(0.0f, 0.0f);
            Float2 vec2(3.0f, 4.0f);
            AssertFloatEqual(25.0f, Float2::DistanceSqr(vec1, vec2));
        }

        TEST_METHOD(AngleOf_UnitX_ReturnsZero)
        {
            Float2 vec(1.0f, 0.0f);
            AssertFloatEqual(0.0f, Float2::AngleOf(vec));
        }

        TEST_METHOD(AngleOf_UnitY_ReturnsHalfPi)
        {
            Float2 vec(0.0f, 1.0f);
            AssertFloatEqual(pi_v<float> / 2.0f, Float2::AngleOf(vec), 0.0001f);
        }

        TEST_METHOD(AngleOf_NegativeX_ReturnsPi)
        {
            Float2 vec(-1.0f, 0.0f);
            AssertFloatEqual(pi_v<float>, Float2::AngleOf(vec), 0.0001f);
        }

        TEST_METHOD(AngleBetween_SameVectors_ReturnsZero)
        {
            Float2 vec1(1.0f, 0.0f);
            Float2 vec2(1.0f, 0.0f);
            AssertFloatEqual(0.0f, Float2::AngleBetween(vec1, vec2));
        }

        TEST_METHOD(AngleBetween_OrthogonalVectors_ReturnsHalfPi)
        {
            Float2 vec1(1.0f, 0.0f);
            Float2 vec2(0.0f, 1.0f);
            AssertFloatEqual(pi_v<float> / 2.0f, Float2::AngleBetween(vec1, vec2), 0.0001f);
        }

        TEST_METHOD(AngleBetween_OppositeVectors_ReturnsPi)
        {
            Float2 vec1(1.0f, 0.0f);
            Float2 vec2(-1.0f, 0.0f);
            AssertFloatEqual(pi_v<float>, Float2::AngleBetween(vec1, vec2), 0.0001f);
        }

        TEST_METHOD(AngleBetween_ZeroVector_ReturnsZero)
        {
            Float2 vec1(0.0f, 0.0f);
            Float2 vec2(1.0f, 0.0f);
            AssertFloatEqual(0.0f, Float2::AngleBetween(vec1, vec2));
        }

        TEST_METHOD(Lerp_ZeroT_ReturnsFirstVector)
        {
            Float2 a(1.0f, 2.0f);
            Float2 b(3.0f, 4.0f);
            Float2 result = Float2::Lerp(a, b, 0.0f);
            AssertFloat2Equal(a, result);
        }

        TEST_METHOD(Lerp_OneT_ReturnsSecondVector)
        {
            Float2 a(1.0f, 2.0f);
            Float2 b(3.0f, 4.0f);
            Float2 result = Float2::Lerp(a, b, 1.0f);
            AssertFloat2Equal(b, result);
        }

        TEST_METHOD(Lerp_HalfT_ReturnsMidpoint)
        {
            Float2 a(1.0f, 2.0f);
            Float2 b(3.0f, 4.0f);
            Float2 result = Float2::Lerp(a, b, 0.5f);
            AssertFloat2Equal(Float2(2.0f, 3.0f), result);
        }

        TEST_METHOD(Lerp_ClampsBeyondOne)
        {
            Float2 a(1.0f, 2.0f);
            Float2 b(3.0f, 4.0f);
            Float2 result = Float2::Lerp(a, b, 1.5f);
            AssertFloat2Equal(b, result); // Should clamp to 1.0
        }

        TEST_METHOD(Reflect_PerpendicularToNormal_ReflectsPerfectly)
        {
            Float2 incident(1.0f, -1.0f); // 45 degrees down-right
            Float2 normal(0.0f, 1.0f);    // Up
            Float2 reflected = Float2::Reflect(incident, normal);
            AssertFloat2Equal(Float2(1.0f, 1.0f), reflected); // 45 degrees up-right
        }

        TEST_METHOD(Perpendicular_UnitX_ReturnsUnitY)
        {
            Float2 vec(1.0f, 0.0f);
            Float2 perp = Float2::Perpendicular(vec);
            AssertFloat2Equal(Float2(0.0f, -1.0f), perp);
        }

        TEST_METHOD(Perpendicular_UnitY_ReturnsNegativeUnitX)
        {
            Float2 vec(0.0f, 1.0f);
            Float2 perp = Float2::Perpendicular(vec);
            AssertFloat2Equal(Float2(1.0f, 0.0f), perp);
        }

        TEST_METHOD(Min_ComponentWise_ReturnsMinimumComponents)
        {
            Float2 a(1.0f, 4.0f);
            Float2 b(3.0f, 2.0f);
            Float2 result = Float2::Min(a, b);
            AssertFloat2Equal(Float2(1.0f, 2.0f), result);
        }

        TEST_METHOD(Max_ComponentWise_ReturnsMaximumComponents)
        {
            Float2 a(1.0f, 4.0f);
            Float2 b(3.0f, 2.0f);
            Float2 result = Float2::Max(a, b);
            AssertFloat2Equal(Float2(3.0f, 4.0f), result);
        }

        TEST_METHOD(Clamp_WithinBounds_ReturnsOriginal)
        {
            Float2 value(2.0f, 3.0f);
            Float2 min(1.0f, 2.0f);
            Float2 max(4.0f, 5.0f);
            Float2 result = Float2::Clamp(value, min, max);
            AssertFloat2Equal(value, result);
        }

        TEST_METHOD(Clamp_OutOfBounds_ClampsToLimits)
        {
            Float2 value(0.0f, 6.0f);
            Float2 min(1.0f, 2.0f);
            Float2 max(4.0f, 5.0f);
            Float2 result = Float2::Clamp(value, min, max);
            AssertFloat2Equal(Float2(1.0f, 5.0f), result);
        }

        TEST_METHOD(StaticFactories_CreateCorrectVectors)
        {
            AssertFloat2Equal(Float2(0.0f, 0.0f), Float2::Zero());
            AssertFloat2Equal(Float2(1.0f, 1.0f), Float2::One());
            AssertFloat2Equal(Float2(0.5f, 0.5f), Float2::Half());
            AssertFloat2Equal(Float2(1.0f, 0.0f), Float2::UnitX());
            AssertFloat2Equal(Float2(0.0f, 1.0f), Float2::UnitY());
        }

        TEST_METHOD(Equality_SameVectors_ReturnsTrue)
        {
            Float2 a(1.0f, 2.0f);
            Float2 b(1.0f, 2.0f);
            Assert::IsTrue(a == b);
        }

        TEST_METHOD(Equality_DifferentVectors_ReturnsFalse)
        {
            Float2 a(1.0f, 2.0f);
            Float2 b(1.0f, 3.0f);
            Assert::IsFalse(a == b);
        }

        TEST_METHOD(Inequality_DifferentVectors_ReturnsTrue)
        {
            Float2 a(1.0f, 2.0f);
            Float2 b(1.0f, 3.0f);
            Assert::IsTrue(a != b);
        }

        TEST_METHOD(Addition_TwoVectors_AddsComponents)
        {
            Float2 a(1.0f, 2.0f);
            Float2 b(3.0f, 4.0f);
            Float2 result = a + b;
            AssertFloat2Equal(Float2(4.0f, 6.0f), result);
        }

        TEST_METHOD(AdditionAssignment_ModifiesOriginal)
        {
            Float2 a(1.0f, 2.0f);
            Float2 b(3.0f, 4.0f);
            a += b;
            AssertFloat2Equal(Float2(4.0f, 6.0f), a);
        }

        TEST_METHOD(Subtraction_TwoVectors_SubtractsComponents)
        {
            Float2 a(5.0f, 7.0f);
            Float2 b(2.0f, 3.0f);
            Float2 result = a - b;
            AssertFloat2Equal(Float2(3.0f, 4.0f), result);
        }

        TEST_METHOD(SubtractionAssignment_ModifiesOriginal)
        {
            Float2 a(5.0f, 7.0f);
            Float2 b(2.0f, 3.0f);
            a -= b;
            AssertFloat2Equal(Float2(3.0f, 4.0f), a);
        }

        TEST_METHOD(MultiplicationByScalar_ScalesComponents)
        {
            Float2 vec(2.0f, 3.0f);
            Float2 result = vec * 2.0f;
            AssertFloat2Equal(Float2(4.0f, 6.0f), result);
        }

        TEST_METHOD(MultiplicationAssignmentByScalar_ModifiesOriginal)
        {
            Float2 vec(2.0f, 3.0f);
            vec *= 2.0f;
            AssertFloat2Equal(Float2(4.0f, 6.0f), vec);
        }

        TEST_METHOD(DivisionByScalar_DividesComponents)
        {
            Float2 vec(4.0f, 6.0f);
            Float2 result = vec / 2.0f;
            AssertFloat2Equal(Float2(2.0f, 3.0f), result);
        }

        TEST_METHOD(DivisionAssignmentByScalar_ModifiesOriginal)
        {
            Float2 vec(4.0f, 6.0f);
            vec /= 2.0f;
            AssertFloat2Equal(Float2(2.0f, 3.0f), vec);
        }

        TEST_METHOD(UnaryMinus_NegatesComponents)
        {
            Float2 vec(2.0f, -3.0f);
            Float2& result = -vec;
            AssertFloat2Equal(Float2(-2.0f, 3.0f), result);
            // Should also modify original
            AssertFloat2Equal(Float2(-2.0f, 3.0f), vec);
        }

        TEST_METHOD(IndexOperator_ValidIndices_ReturnsCorrectComponents)
        {
            Float2 vec(2.0f, 3.0f);
            AssertFloatEqual(2.0f, vec[0]);
            AssertFloatEqual(3.0f, vec[1]);
        }

        TEST_METHOD(IndexOperator_InvalidIndex_ThrowsException)
        {
            Float2 vec(2.0f, 3.0f);

            // Test negative index
            Assert::ExpectException<runtime_error>([&]() {
                float value = vec[-1];
                });

            // Test index 2 (out of bounds)
            Assert::ExpectException<runtime_error>([&]() {
                float value = vec[2];
                });

            // Test index 3 (out of bounds)
            Assert::ExpectException<runtime_error>([&]() {
                float value = vec[3];
                });
        }

        TEST_METHOD(ToString_FormatsCorrectly)
        {
            Float2 vec(1.5f, 2.5f);
            string str = vec.ToString();
            // Basic check - should contain the numbers
            Assert::IsTrue(str.find("1.5") != string::npos);
            Assert::IsTrue(str.find("2.5") != string::npos);
        }

        TEST_METHOD(GlobalMultiplication_ScalarTimesVector_Works)
        {
            Float2 vec(2.0f, 3.0f);
            Float2 result = 2.0f * vec;
            AssertFloat2Equal(Float2(4.0f, 6.0f), result);
        }

        // Edge case tests
        TEST_METHOD(EdgeCase_VerySmallNumbers_HandledCorrectly)
        {
            Float2 vec(1e-6f, 1e-6f);
            Assert::IsFalse(vec.IsZero()); // Should not be considered zero
        }

        TEST_METHOD(EdgeCase_VeryLargeNumbers_HandledCorrectly)
        {
            Float2 vec(1e6f, 1e6f);
            Float2 normalized = vec.Normalized();
            AssertFloatEqual(1.0f, normalized.Magnitude(), 0.001f);
        }
        
    };
}