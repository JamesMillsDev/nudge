#include <numbers>

#include "CppUnitTest.h"

#include "Nudge/MathF.hpp"
#include "Nudge/Vector2.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using std::runtime_error;
using std::numbers::pi_v;

namespace Nudge
{
    TEST_CLASS(Vector2Tests)
    {
    public:
        // Helper method for floating point comparison
        static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
        {
            Assert::IsTrue(MathF::Compare(expected, actual, tolerance), L"Float values are not equal within tolerance");
        }

        static void AssertFloat2Equal(const Vector2& expected, const Vector2& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.x, actual.x, tolerance);
            AssertFloatEqual(expected.y, actual.y, tolerance);
        }

        TEST_METHOD(Constructor_Default_CreatesZeroVector)
        {
            Vector2 vec;
            AssertFloat2Equal(Vector2(0.0f, 0.0f), vec);
        }

        TEST_METHOD(Constructor_Scalar_CreatesSameValueForBothComponents)
        {
            Vector2 vec(5.0f);
            AssertFloat2Equal(Vector2(5.0f, 5.0f), vec);
        }

        TEST_METHOD(Constructor_XY_CreatesVectorWithSpecifiedValues)
        {
            Vector2 vec(3.0f, 4.0f);
            AssertFloat2Equal(Vector2(3.0f, 4.0f), vec);
        }

        TEST_METHOD(Constructor_Array_CreatesVectorFromArray)
        {
            float values[] = { 2.0f, 7.0f };
            Vector2 vec(values);
            AssertFloat2Equal(Vector2(2.0f, 7.0f), vec);
        }

        TEST_METHOD(Constructor_Copy_CreatesIdenticalVector)
        {
            Vector2 original(1.5f, 2.5f);
            Vector2 copy(original);
            AssertFloat2Equal(original, copy);
        }

        TEST_METHOD(Magnitude_UnitVector_ReturnsOne)
        {
            Vector2 vec(1.0f, 0.0f);
            AssertFloatEqual(1.0f, vec.Magnitude());
        }

        TEST_METHOD(Magnitude_PythagoreanTriple_ReturnsCorrectMagnitude)
        {
            Vector2 vec(3.0f, 4.0f);
            AssertFloatEqual(5.0f, vec.Magnitude());
        }

        TEST_METHOD(Magnitude_ZeroVector_ReturnsZero)
        {
            Vector2 vec(0.0f, 0.0f);
            AssertFloatEqual(0.0f, vec.Magnitude());
        }

        TEST_METHOD(MagnitudeSqr_PythagoreanTriple_ReturnsSquaredMagnitude)
        {
            Vector2 vec(3.0f, 4.0f);
            AssertFloatEqual(25.0f, vec.MagnitudeSqr());
        }

        TEST_METHOD(Normalize_NonZeroVector_CreatesUnitVector)
        {
            Vector2 vec(3.0f, 4.0f);
            vec.Normalize();
            AssertFloatEqual(1.0f, vec.Magnitude());
            AssertFloat2Equal(Vector2(0.6f, 0.8f), vec);
        }

        TEST_METHOD(Normalize_ZeroVector_RemainsZero)
        {
            Vector2 vec(0.0f, 0.0f);
            vec.Normalize();
            AssertFloat2Equal(Vector2(0.0f, 0.0f), vec);
        }

        TEST_METHOD(Normalized_NonZeroVector_ReturnsUnitVector)
        {
            Vector2 vec(3.0f, 4.0f);
            Vector2 normalized = vec.Normalized();
            AssertFloatEqual(1.0f, normalized.Magnitude());
            AssertFloat2Equal(Vector2(0.6f, 0.8f), normalized);
            // Original should be unchanged
            AssertFloat2Equal(Vector2(3.0f, 4.0f), vec);
        }

        TEST_METHOD(Normalized_ZeroVector_ReturnsZero)
        {
            Vector2 vec(0.0f, 0.0f);
            Vector2 normalized = vec.Normalized();
            AssertFloat2Equal(Vector2(0.0f, 0.0f), normalized);
        }

        TEST_METHOD(IsZero_ZeroVector_ReturnsTrue)
        {
            Vector2 vec(0.0f, 0.0f);
            Assert::IsTrue(vec.IsZero());
        }

        TEST_METHOD(IsZero_NonZeroVector_ReturnsFalse)
        {
            Vector2 vec(0.1f, 0.0f);
            Assert::IsFalse(vec.IsZero());
        }

        TEST_METHOD(IsUnit_UnitVector_ReturnsTrue)
        {
            Vector2 vec(1.0f, 0.0f);
            Assert::IsTrue(vec.IsUnit());
        }

        TEST_METHOD(IsUnit_NonUnitVector_ReturnsFalse)
        {
            Vector2 vec(2.0f, 0.0f);
            Assert::IsFalse(vec.IsUnit());
        }

        TEST_METHOD(Dot_OrthogonalVectors_ReturnsZero)
        {
            Vector2 vec1(1.0f, 0.0f);
            Vector2 vec2(0.0f, 1.0f);
            AssertFloatEqual(0.0f, Vector2::Dot(vec1, vec2));
        }

        TEST_METHOD(Dot_ParallelVectors_ReturnsProduct)
        {
            Vector2 vec1(2.0f, 0.0f);
            Vector2 vec2(3.0f, 0.0f);
            AssertFloatEqual(6.0f, Vector2::Dot(vec1, vec2));
        }

        TEST_METHOD(Dot_GeneralCase_ReturnsCorrectValue)
        {
            Vector2 vec1(2.0f, 3.0f);
            Vector2 vec2(4.0f, 5.0f);
            // 2*4 + 3*5 = 8 + 15 = 23
            AssertFloatEqual(23.0f, Vector2::Dot(vec1, vec2));
        }

        TEST_METHOD(Distance_SamePoints_ReturnsZero)
        {
            Vector2 vec1(1.0f, 2.0f);
            Vector2 vec2(1.0f, 2.0f);
            AssertFloatEqual(0.0f, Vector2::Distance(vec1, vec2));
        }

        TEST_METHOD(Distance_PythagoreanTriple_ReturnsCorrectDistance)
        {
            Vector2 vec1(0.0f, 0.0f);
            Vector2 vec2(3.0f, 4.0f);
            AssertFloatEqual(5.0f, Vector2::Distance(vec1, vec2));
        }

        TEST_METHOD(DistanceSqr_PythagoreanTriple_ReturnsSquaredDistance)
        {
            Vector2 vec1(0.0f, 0.0f);
            Vector2 vec2(3.0f, 4.0f);
            AssertFloatEqual(25.0f, Vector2::DistanceSqr(vec1, vec2));
        }

        TEST_METHOD(AngleOf_UnitX_ReturnsZero)
        {
            Vector2 vec(1.0f, 0.0f);
            AssertFloatEqual(0.0f, Vector2::AngleOf(vec));
        }

        TEST_METHOD(AngleOf_UnitY_ReturnsHalfPi)
        {
            Vector2 vec(0.0f, 1.0f);
            AssertFloatEqual(pi_v<float> / 2.0f, Vector2::AngleOf(vec), 0.0001f);
        }

        TEST_METHOD(AngleOf_NegativeX_ReturnsPi)
        {
            Vector2 vec(-1.0f, 0.0f);
            AssertFloatEqual(pi_v<float>, Vector2::AngleOf(vec), 0.0001f);
        }

        TEST_METHOD(AngleBetween_SameVectors_ReturnsZero)
        {
            Vector2 vec1(1.0f, 0.0f);
            Vector2 vec2(1.0f, 0.0f);
            AssertFloatEqual(0.0f, Vector2::AngleBetween(vec1, vec2));
        }

        TEST_METHOD(AngleBetween_OrthogonalVectors_ReturnsHalfPi)
        {
            Vector2 vec1(1.0f, 0.0f);
            Vector2 vec2(0.0f, 1.0f);
            AssertFloatEqual(pi_v<float> / 2.0f, Vector2::AngleBetween(vec1, vec2), 0.0001f);
        }

        TEST_METHOD(AngleBetween_OppositeVectors_ReturnsPi)
        {
            Vector2 vec1(1.0f, 0.0f);
            Vector2 vec2(-1.0f, 0.0f);
            AssertFloatEqual(pi_v<float>, Vector2::AngleBetween(vec1, vec2), 0.0001f);
        }

        TEST_METHOD(AngleBetween_ZeroVector_ReturnsZero)
        {
            Vector2 vec1(0.0f, 0.0f);
            Vector2 vec2(1.0f, 0.0f);
            AssertFloatEqual(0.0f, Vector2::AngleBetween(vec1, vec2));
        }

        TEST_METHOD(Lerp_ZeroT_ReturnsFirstVector)
        {
            Vector2 a(1.0f, 2.0f);
            Vector2 b(3.0f, 4.0f);
            Vector2 result = Vector2::Lerp(a, b, 0.0f);
            AssertFloat2Equal(a, result);
        }

        TEST_METHOD(Lerp_OneT_ReturnsSecondVector)
        {
            Vector2 a(1.0f, 2.0f);
            Vector2 b(3.0f, 4.0f);
            Vector2 result = Vector2::Lerp(a, b, 1.0f);
            AssertFloat2Equal(b, result);
        }

        TEST_METHOD(Lerp_HalfT_ReturnsMidpoint)
        {
            Vector2 a(1.0f, 2.0f);
            Vector2 b(3.0f, 4.0f);
            Vector2 result = Vector2::Lerp(a, b, 0.5f);
            AssertFloat2Equal(Vector2(2.0f, 3.0f), result);
        }

        TEST_METHOD(Lerp_ClampsBeyondOne)
        {
            Vector2 a(1.0f, 2.0f);
            Vector2 b(3.0f, 4.0f);
            Vector2 result = Vector2::Lerp(a, b, 1.5f);
            AssertFloat2Equal(b, result); // Should clamp to 1.0
        }

        TEST_METHOD(Reflect_PerpendicularToNormal_ReflectsPerfectly)
        {
            Vector2 incident(1.0f, -1.0f); // 45 degrees down-right
            Vector2 normal(0.0f, 1.0f);    // Up
            Vector2 reflected = Vector2::Reflect(incident, normal);
            AssertFloat2Equal(Vector2(1.0f, 1.0f), reflected); // 45 degrees up-right
        }

        TEST_METHOD(Perpendicular_UnitX_ReturnsUnitY)
        {
            Vector2 vec(1.0f, 0.0f);
            Vector2 perp = Vector2::Perpendicular(vec);
            AssertFloat2Equal(Vector2(0.0f, -1.0f), perp);
        }

        TEST_METHOD(Perpendicular_UnitY_ReturnsNegativeUnitX)
        {
            Vector2 vec(0.0f, 1.0f);
            Vector2 perp = Vector2::Perpendicular(vec);
            AssertFloat2Equal(Vector2(1.0f, 0.0f), perp);
        }

        TEST_METHOD(Min_ComponentWise_ReturnsMinimumComponents)
        {
            Vector2 a(1.0f, 4.0f);
            Vector2 b(3.0f, 2.0f);
            Vector2 result = Vector2::Min(a, b);
            AssertFloat2Equal(Vector2(1.0f, 2.0f), result);
        }

        TEST_METHOD(Max_ComponentWise_ReturnsMaximumComponents)
        {
            Vector2 a(1.0f, 4.0f);
            Vector2 b(3.0f, 2.0f);
            Vector2 result = Vector2::Max(a, b);
            AssertFloat2Equal(Vector2(3.0f, 4.0f), result);
        }

        TEST_METHOD(Clamp_WithinBounds_ReturnsOriginal)
        {
            Vector2 value(2.0f, 3.0f);
            Vector2 min(1.0f, 2.0f);
            Vector2 max(4.0f, 5.0f);
            Vector2 result = Vector2::Clamp(value, min, max);
            AssertFloat2Equal(value, result);
        }

        TEST_METHOD(Clamp_OutOfBounds_ClampsToLimits)
        {
            Vector2 value(0.0f, 6.0f);
            Vector2 min(1.0f, 2.0f);
            Vector2 max(4.0f, 5.0f);
            Vector2 result = Vector2::Clamp(value, min, max);
            AssertFloat2Equal(Vector2(1.0f, 5.0f), result);
        }

        TEST_METHOD(StaticFactories_CreateCorrectVectors)
        {
            AssertFloat2Equal(Vector2(0.0f, 0.0f), Vector2::Zero());
            AssertFloat2Equal(Vector2(1.0f, 1.0f), Vector2::One());
            AssertFloat2Equal(Vector2(0.5f, 0.5f), Vector2::Half());
            AssertFloat2Equal(Vector2(1.0f, 0.0f), Vector2::UnitX());
            AssertFloat2Equal(Vector2(0.0f, 1.0f), Vector2::UnitY());
        }

        TEST_METHOD(Equality_SameVectors_ReturnsTrue)
        {
            Vector2 a(1.0f, 2.0f);
            Vector2 b(1.0f, 2.0f);
            Assert::IsTrue(a == b);
        }

        TEST_METHOD(Equality_DifferentVectors_ReturnsFalse)
        {
            Vector2 a(1.0f, 2.0f);
            Vector2 b(1.0f, 3.0f);
            Assert::IsFalse(a == b);
        }

        TEST_METHOD(Inequality_DifferentVectors_ReturnsTrue)
        {
            Vector2 a(1.0f, 2.0f);
            Vector2 b(1.0f, 3.0f);
            Assert::IsTrue(a != b);
        }

        TEST_METHOD(Addition_TwoVectors_AddsComponents)
        {
            Vector2 a(1.0f, 2.0f);
            Vector2 b(3.0f, 4.0f);
            Vector2 result = a + b;
            AssertFloat2Equal(Vector2(4.0f, 6.0f), result);
        }

        TEST_METHOD(AdditionAssignment_ModifiesOriginal)
        {
            Vector2 a(1.0f, 2.0f);
            Vector2 b(3.0f, 4.0f);
            a += b;
            AssertFloat2Equal(Vector2(4.0f, 6.0f), a);
        }

        TEST_METHOD(Subtraction_TwoVectors_SubtractsComponents)
        {
            Vector2 a(5.0f, 7.0f);
            Vector2 b(2.0f, 3.0f);
            Vector2 result = a - b;
            AssertFloat2Equal(Vector2(3.0f, 4.0f), result);
        }

        TEST_METHOD(SubtractionAssignment_ModifiesOriginal)
        {
            Vector2 a(5.0f, 7.0f);
            Vector2 b(2.0f, 3.0f);
            a -= b;
            AssertFloat2Equal(Vector2(3.0f, 4.0f), a);
        }

        TEST_METHOD(MultiplicationByScalar_ScalesComponents)
        {
            Vector2 vec(2.0f, 3.0f);
            Vector2 result = vec * 2.0f;
            AssertFloat2Equal(Vector2(4.0f, 6.0f), result);
        }

        TEST_METHOD(MultiplicationAssignmentByScalar_ModifiesOriginal)
        {
            Vector2 vec(2.0f, 3.0f);
            vec *= 2.0f;
            AssertFloat2Equal(Vector2(4.0f, 6.0f), vec);
        }

        TEST_METHOD(DivisionByScalar_DividesComponents)
        {
            Vector2 vec(4.0f, 6.0f);
            Vector2 result = vec / 2.0f;
            AssertFloat2Equal(Vector2(2.0f, 3.0f), result);
        }

        TEST_METHOD(DivisionAssignmentByScalar_ModifiesOriginal)
        {
            Vector2 vec(4.0f, 6.0f);
            vec /= 2.0f;
            AssertFloat2Equal(Vector2(2.0f, 3.0f), vec);
        }

        TEST_METHOD(UnaryMinus_NegatesComponents)
        {
            Vector2 vec(2.0f, -3.0f);
            Vector2& result = -vec;
            AssertFloat2Equal(Vector2(-2.0f, 3.0f), result);
            // Should also modify original
            AssertFloat2Equal(Vector2(-2.0f, 3.0f), vec);
        }

        TEST_METHOD(IndexOperator_ValidIndices_ReturnsCorrectComponents)
        {
            Vector2 vec(2.0f, 3.0f);
            AssertFloatEqual(2.0f, vec[0]);
            AssertFloatEqual(3.0f, vec[1]);
        }

        TEST_METHOD(IndexOperator_InvalidIndex_ThrowsException)
        {
            Vector2 vec(2.0f, 3.0f);

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
            Vector2 vec(1.5f, 2.5f);
            string str = vec.ToString();
            // Basic check - should contain the numbers
            Assert::IsTrue(str.find("1.5") != string::npos);
            Assert::IsTrue(str.find("2.5") != string::npos);
        }

        TEST_METHOD(GlobalMultiplication_ScalarTimesVector_Works)
        {
            Vector2 vec(2.0f, 3.0f);
            Vector2 result = 2.0f * vec;
            AssertFloat2Equal(Vector2(4.0f, 6.0f), result);
        }

        // Edge case tests
        TEST_METHOD(EdgeCase_VerySmallNumbers_HandledCorrectly)
        {
            Vector2 vec(1e-6f, 1e-6f);
            Assert::IsFalse(vec.IsZero()); // Should not be considered zero
        }

        TEST_METHOD(EdgeCase_VeryLargeNumbers_HandledCorrectly)
        {
            Vector2 vec(1e6f, 1e6f);
            Vector2 normalized = vec.Normalized();
            AssertFloatEqual(1.0f, normalized.Magnitude(), 0.001f);
        }
        
    };
}