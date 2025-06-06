/**
 * @file MathFTests.cpp
 * @brief Unit tests for MathF mathematical utility functions
 */

#include <numbers>

#include "CppUnitTest.h"

#include "Nudge/MathF.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using std::numbers::pi_v;

namespace Nudge
{
    TEST_CLASS(MathFTests)
    {
    public:
        // Helper method for floating point comparison
        static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
        {
            Assert::IsTrue(MathF::Compare(expected, actual, tolerance), L"Float values are not equal within tolerance");
        }

        static void AssertBoolEqual(const bool expected, const bool actual)
        {
            Assert::AreEqual(expected, actual);
        }

        static void AssertIntEqual(const int expected, const int actual)
        {
            Assert::AreEqual(expected, actual);
        }

        // Constants Tests
        TEST_METHOD(Constants_Pi_IsCorrect)
        {
            AssertFloatEqual(3.14159265f, MathF::pi, 0.00001f);
        }

        TEST_METHOD(Constants_E_IsCorrect)
        {
            AssertFloatEqual(2.71828183f, MathF::e, 0.00001f);
        }

        TEST_METHOD(Constants_Infinity_IsPositiveInfinity)
        {
            Assert::IsTrue(MathF::infinity > 0.0f);
            Assert::IsTrue(std::isinf(MathF::infinity));
        }

        TEST_METHOD(Constants_NegativeInfinity_IsNegativeInfinity)
        {
            Assert::IsTrue(MathF::negativeInfinity < 0.0f);
            Assert::IsTrue(std::isinf(MathF::negativeInfinity));
        }

        TEST_METHOD(Constants_Epsilon_IsPositive)
        {
            Assert::IsTrue(MathF::epsilon > 0.0f);
        }

        // Comparison Tests
        TEST_METHOD(IsNearZero_Zero_ReturnsTrue)
        {
            AssertBoolEqual(true, MathF::IsNearZero(0.0f));
        }

        TEST_METHOD(IsNearZero_SmallPositive_WithinThreshold_ReturnsTrue)
        {
            AssertBoolEqual(true, MathF::IsNearZero(0.00001f, 0.0001f));
        }

        TEST_METHOD(IsNearZero_SmallNegative_WithinThreshold_ReturnsTrue)
        {
            AssertBoolEqual(true, MathF::IsNearZero(-0.00001f, 0.0001f));
        }

        TEST_METHOD(IsNearZero_LargeValue_ReturnsFalse)
        {
            AssertBoolEqual(false, MathF::IsNearZero(0.1f));
        }

        TEST_METHOD(Compare_SameValues_ReturnsTrue)
        {
            AssertBoolEqual(true, MathF::Compare(1.0f, 1.0f));
        }

        TEST_METHOD(Compare_CloseValues_WithinThreshold_ReturnsTrue)
        {
            AssertBoolEqual(true, MathF::Compare(1.0001f, 1.0002f, 0.001f));
        }

        TEST_METHOD(Compare_DifferentValues_ReturnsFalse)
        {
            AssertBoolEqual(false, MathF::Compare(1.0f, 2.0f));
        }

        TEST_METHOD(Compare_Zero_ReturnsTrue)
        {
            AssertBoolEqual(true, MathF::Compare(0.0f, 0.0f));
        }

        // Clamping Tests
        TEST_METHOD(Clamp_ValueWithinRange_ReturnsValue)
        {
            AssertFloatEqual(5.0f, MathF::Clamp(5.0f, 0.0f, 10.0f));
        }

        TEST_METHOD(Clamp_ValueBelowMin_ReturnsMin)
        {
            AssertFloatEqual(0.0f, MathF::Clamp(-5.0f, 0.0f, 10.0f));
        }

        TEST_METHOD(Clamp_ValueAboveMax_ReturnsMax)
        {
            AssertFloatEqual(10.0f, MathF::Clamp(15.0f, 0.0f, 10.0f));
        }

        TEST_METHOD(Clamp_ValueAtBoundaries_ReturnsBoundary)
        {
            AssertFloatEqual(0.0f, MathF::Clamp(0.0f, 0.0f, 10.0f));
            AssertFloatEqual(10.0f, MathF::Clamp(10.0f, 0.0f, 10.0f));
        }

        TEST_METHOD(Clamp01_ValueWithinRange_ReturnsValue)
        {
            AssertFloatEqual(0.5f, MathF::Clamp01(0.5f));
        }

        TEST_METHOD(Clamp01_ValueBelowZero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Clamp01(-0.5f));
        }

        TEST_METHOD(Clamp01_ValueAboveOne_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Clamp01(1.5f));
        }

        TEST_METHOD(Clamp01_ValueAtBoundaries_ReturnsBoundary)
        {
            AssertFloatEqual(0.0f, MathF::Clamp01(0.0f));
            AssertFloatEqual(1.0f, MathF::Clamp01(1.0f));
        }

        // Angle Conversion Tests
        TEST_METHOD(Degrees_Pi_Returns180)
        {
            AssertFloatEqual(180.0f, MathF::Degrees(MathF::pi), 0.001f);
        }

        TEST_METHOD(Degrees_HalfPi_Returns90)
        {
            AssertFloatEqual(90.0f, MathF::Degrees(MathF::pi / 2.0f), 0.001f);
        }

        TEST_METHOD(Degrees_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Degrees(0.0f));
        }

        TEST_METHOD(Radians_180_ReturnsPi)
        {
            AssertFloatEqual(MathF::pi, MathF::Radians(180.0f), 0.001f);
        }

        TEST_METHOD(Radians_90_ReturnsHalfPi)
        {
            AssertFloatEqual(MathF::pi / 2.0f, MathF::Radians(90.0f), 0.001f);
        }

        TEST_METHOD(Radians_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Radians(0.0f));
        }

        // Power Functions Tests
        TEST_METHOD(Squared_PositiveValue_ReturnsSquare)
        {
            AssertFloatEqual(9.0f, MathF::Squared(3.0f));
        }

        TEST_METHOD(Squared_NegativeValue_ReturnsPositiveSquare)
        {
            AssertFloatEqual(9.0f, MathF::Squared(-3.0f));
        }

        TEST_METHOD(Squared_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Squared(0.0f));
        }

        TEST_METHOD(Cubed_PositiveValue_ReturnsCube)
        {
            AssertFloatEqual(27.0f, MathF::Cubed(3.0f));
        }

        TEST_METHOD(Cubed_NegativeValue_ReturnsNegativeCube)
        {
            AssertFloatEqual(-27.0f, MathF::Cubed(-3.0f));
        }

        TEST_METHOD(Cubed_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Cubed(0.0f));
        }

        // Trigonometric Function Tests
        TEST_METHOD(Sin_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Sin(0.0f), 0.001f);
        }

        TEST_METHOD(Sin_HalfPi_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Sin(MathF::pi / 2.0f), 0.001f);
        }

        TEST_METHOD(Sin_Pi_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Sin(MathF::pi), 0.001f);
        }

        TEST_METHOD(Cos_Zero_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Cos(0.0f), 0.001f);
        }

        TEST_METHOD(Cos_HalfPi_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Cos(MathF::pi / 2.0f), 0.001f);
        }

        TEST_METHOD(Cos_Pi_ReturnsNegativeOne)
        {
            AssertFloatEqual(-1.0f, MathF::Cos(MathF::pi), 0.001f);
        }

        TEST_METHOD(Tan_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Tan(0.0f), 0.001f);
        }

        TEST_METHOD(Tan_QuarterPi_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Tan(MathF::pi / 4.0f), 0.001f);
        }

        TEST_METHOD(Asin_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Asin(0.0f), 0.001f);
        }

        TEST_METHOD(Asin_One_ReturnsHalfPi)
        {
            AssertFloatEqual(MathF::pi / 2.0f, MathF::Asin(1.0f), 0.001f);
        }

        TEST_METHOD(Acos_One_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Acos(1.0f), 0.001f);
        }

        TEST_METHOD(Acos_Zero_ReturnsHalfPi)
        {
            AssertFloatEqual(MathF::pi / 2.0f, MathF::Acos(0.0f), 0.001f);
        }

        TEST_METHOD(Atan_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Atan(0.0f), 0.001f);
        }

        TEST_METHOD(Atan_One_ReturnsQuarterPi)
        {
            AssertFloatEqual(MathF::pi / 4.0f, MathF::Atan(1.0f), 0.001f);
        }

        TEST_METHOD(Atan2_PositiveXY_ReturnsCorrectAngle)
        {
            AssertFloatEqual(MathF::pi / 4.0f, MathF::Atan2(1.0f, 1.0f), 0.001f);
        }

        TEST_METHOD(Atan2_NegativeY_PositiveX_ReturnsNegativeAngle)
        {
            AssertFloatEqual(-MathF::pi / 4.0f, MathF::Atan2(-1.0f, 1.0f), 0.001f);
        }

        TEST_METHOD(Atan2_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Atan2(0.0f, 1.0f), 0.001f);
        }

        // Hyperbolic Function Tests
        TEST_METHOD(Sinh_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Sinh(0.0f), 0.001f);
        }

        TEST_METHOD(Cosh_Zero_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Cosh(0.0f), 0.001f);
        }

        TEST_METHOD(Tanh_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Tanh(0.0f), 0.001f);
        }

        // Power and Exponential Tests
        TEST_METHOD(Pow_TwoToThree_ReturnsEight)
        {
            AssertFloatEqual(8.0f, MathF::Pow(2.0f, 3.0f));
        }

        TEST_METHOD(Pow_BaseOne_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Pow(1.0f, 100.0f));
        }

        TEST_METHOD(Pow_ExponentZero_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Pow(5.0f, 0.0f));
        }

        TEST_METHOD(Sqrt_Four_ReturnsTwo)
        {
            AssertFloatEqual(2.0f, MathF::Sqrt(4.0f));
        }

        TEST_METHOD(Sqrt_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Sqrt(0.0f));
        }

        TEST_METHOD(Sqrt_One_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Sqrt(1.0f));
        }

        TEST_METHOD(Cbrt_Eight_ReturnsTwo)
        {
            AssertFloatEqual(2.0f, MathF::Cbrt(8.0f), 0.001f);
        }

        TEST_METHOD(Cbrt_NegativeEight_ReturnsNegativeTwo)
        {
            AssertFloatEqual(-2.0f, MathF::Cbrt(-8.0f), 0.001f);
        }

        TEST_METHOD(Exp_Zero_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Exp(0.0f), 0.001f);
        }

        TEST_METHOD(Exp_One_ReturnsE)
        {
            AssertFloatEqual(MathF::e, MathF::Exp(1.0f), 0.001f);
        }

        TEST_METHOD(Log_E_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Log(MathF::e), 0.001f);
        }

        TEST_METHOD(Log_One_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Log(1.0f), 0.001f);
        }

        TEST_METHOD(Log10_Ten_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Log10(10.0f), 0.001f);
        }

        TEST_METHOD(Log10_Hundred_ReturnsTwo)
        {
            AssertFloatEqual(2.0f, MathF::Log10(100.0f), 0.001f);
        }

        TEST_METHOD(Log2_Two_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Log2(2.0f), 0.001f);
        }

        TEST_METHOD(Log2_Eight_ReturnsThree)
        {
            AssertFloatEqual(3.0f, MathF::Log2(8.0f), 0.001f);
        }

        // Rounding Function Tests
        TEST_METHOD(Floor_PositiveDecimal_ReturnsFloor)
        {
            AssertFloatEqual(3.0f, MathF::Floor(3.7f));
        }

        TEST_METHOD(Floor_NegativeDecimal_ReturnsFloor)
        {
            AssertFloatEqual(-4.0f, MathF::Floor(-3.2f));
        }

        TEST_METHOD(Floor_Integer_ReturnsInteger)
        {
            AssertFloatEqual(5.0f, MathF::Floor(5.0f));
        }

        TEST_METHOD(Ceil_PositiveDecimal_ReturnsCeiling)
        {
            AssertFloatEqual(4.0f, MathF::Ceil(3.2f));
        }

        TEST_METHOD(Ceil_NegativeDecimal_ReturnsCeiling)
        {
            AssertFloatEqual(-3.0f, MathF::Ceil(-3.7f));
        }

        TEST_METHOD(Ceil_Integer_ReturnsInteger)
        {
            AssertFloatEqual(5.0f, MathF::Ceil(5.0f));
        }

        TEST_METHOD(Round_PositiveHalf_RoundsUp)
        {
            AssertFloatEqual(4.0f, MathF::Round(3.5f));
        }

        TEST_METHOD(Round_NegativeHalf_RoundsAwayFromZero)
        {
            AssertFloatEqual(-4.0f, MathF::Round(-3.5f));
        }

        TEST_METHOD(Round_LessThanHalf_RoundsDown)
        {
            AssertFloatEqual(3.0f, MathF::Round(3.4f));
        }

        TEST_METHOD(Trunc_PositiveDecimal_TruncatesTowardZero)
        {
            AssertFloatEqual(3.0f, MathF::Trunc(3.7f));
        }

        TEST_METHOD(Trunc_NegativeDecimal_TruncatesTowardZero)
        {
            AssertFloatEqual(-3.0f, MathF::Trunc(-3.7f));
        }

        TEST_METHOD(Frac_PositiveValue_ReturnsFractionalPart)
        {
            AssertFloatEqual(0.7f, MathF::Frac(3.7f), 0.001f);
        }

        TEST_METHOD(Frac_NegativeValue_ReturnsNegativeFractionalPart)
        {
            AssertFloatEqual(-0.3f, MathF::Frac(-2.3f), 0.001f);
        }

        TEST_METHOD(Frac_Integer_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Frac(5.0f), 0.001f);
        }

        // Basic Math Function Tests
        TEST_METHOD(Abs_PositiveValue_ReturnsValue)
        {
            AssertFloatEqual(5.0f, MathF::Abs(5.0f));
        }

        TEST_METHOD(Abs_NegativeValue_ReturnsPositiveValue)
        {
            AssertFloatEqual(5.0f, MathF::Abs(-5.0f));
        }

        TEST_METHOD(Abs_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Abs(0.0f));
        }

        TEST_METHOD(Sign_PositiveValue_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::Sign(5.0f));
        }

        TEST_METHOD(Sign_NegativeValue_ReturnsNegativeOne)
        {
            AssertFloatEqual(-1.0f, MathF::Sign(-5.0f));
        }

        TEST_METHOD(Sign_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::Sign(0.0f));
        }

        TEST_METHOD(Min_TwoValues_ReturnsSmaller)
        {
            AssertFloatEqual(3.0f, MathF::Min(3.0f, 5.0f));
            AssertFloatEqual(3.0f, MathF::Min(5.0f, 3.0f));
        }

        TEST_METHOD(Min_EqualValues_ReturnsValue)
        {
            AssertFloatEqual(4.0f, MathF::Min(4.0f, 4.0f));
        }

        TEST_METHOD(Max_TwoValues_ReturnsLarger)
        {
            AssertFloatEqual(5.0f, MathF::Max(3.0f, 5.0f));
            AssertFloatEqual(5.0f, MathF::Max(5.0f, 3.0f));
        }

        TEST_METHOD(Max_EqualValues_ReturnsValue)
        {
            AssertFloatEqual(4.0f, MathF::Max(4.0f, 4.0f));
        }

        // Interpolation Function Tests
        TEST_METHOD(Lerp_AtStart_ReturnsStartValue)
        {
            AssertFloatEqual(1.0f, MathF::Lerp(1.0f, 5.0f, 0.0f));
        }

        TEST_METHOD(Lerp_AtEnd_ReturnsEndValue)
        {
            AssertFloatEqual(5.0f, MathF::Lerp(1.0f, 5.0f, 1.0f));
        }

        TEST_METHOD(Lerp_AtMiddle_ReturnsMiddleValue)
        {
            AssertFloatEqual(3.0f, MathF::Lerp(1.0f, 5.0f, 0.5f));
        }

        TEST_METHOD(Lerp_BeyondRange_ClampsToRange)
        {
            AssertFloatEqual(1.0f, MathF::Lerp(1.0f, 5.0f, -0.5f));
            AssertFloatEqual(5.0f, MathF::Lerp(1.0f, 5.0f, 1.5f));
        }

        TEST_METHOD(LerpUnclamped_BeyondRange_DoesNotClamp)
        {
            AssertFloatEqual(-1.0f, MathF::LerpUnclamped(1.0f, 5.0f, -0.5f));
            AssertFloatEqual(7.0f, MathF::LerpUnclamped(1.0f, 5.0f, 1.5f));
        }

        TEST_METHOD(InverseLerp_StartValue_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::InverseLerp(1.0f, 5.0f, 1.0f));
        }

        TEST_METHOD(InverseLerp_EndValue_ReturnsOne)
        {
            AssertFloatEqual(1.0f, MathF::InverseLerp(1.0f, 5.0f, 5.0f));
        }

        TEST_METHOD(InverseLerp_MiddleValue_ReturnsHalf)
        {
            AssertFloatEqual(0.5f, MathF::InverseLerp(1.0f, 5.0f, 3.0f));
        }

        TEST_METHOD(SmoothStep_AtEdges_ReturnsExpectedValues)
        {
            AssertFloatEqual(0.0f, MathF::SmoothStep(0.0f, 1.0f, 0.0f));
            AssertFloatEqual(1.0f, MathF::SmoothStep(0.0f, 1.0f, 1.0f));
        }

        TEST_METHOD(SmoothStep_AtMiddle_ReturnsSmoothedValue)
        {
            AssertFloatEqual(0.5f, MathF::SmoothStep(0.0f, 1.0f, 0.5f));
        }

        TEST_METHOD(SmoothStep_BeyondRange_ClampsToRange)
        {
            AssertFloatEqual(0.0f, MathF::SmoothStep(0.0f, 1.0f, -0.5f));
            AssertFloatEqual(1.0f, MathF::SmoothStep(0.0f, 1.0f, 1.5f));
        }

        TEST_METHOD(SmootherStep_AtEdges_ReturnsExpectedValues)
        {
            AssertFloatEqual(0.0f, MathF::SmootherStep(0.0f, 1.0f, 0.0f));
            AssertFloatEqual(1.0f, MathF::SmootherStep(0.0f, 1.0f, 1.0f));
        }

        TEST_METHOD(SmootherStep_AtMiddle_ReturnsSmoothedValue)
        {
            AssertFloatEqual(0.5f, MathF::SmootherStep(0.0f, 1.0f, 0.5f));
        }

        // Modulo and Wrapping Tests
        TEST_METHOD(Mod_PositiveValues_ReturnsRemainder)
        {
            AssertFloatEqual(1.0f, MathF::Mod(7.0f, 3.0f), 0.001f);
        }

        TEST_METHOD(Mod_NegativeValue_ReturnsCorrectRemainder)
        {
            AssertFloatEqual(-1.0f, MathF::Mod(-7.0f, 3.0f), 0.001f);
        }

        TEST_METHOD(Repeat_WithinRange_ReturnsValue)
        {
            AssertFloatEqual(2.0f, MathF::Repeat(2.0f, 5.0f));
        }

        TEST_METHOD(Repeat_BeyondRange_WrapsValue)
        {
            AssertFloatEqual(2.0f, MathF::Repeat(7.0f, 5.0f), 0.001f);
        }

        TEST_METHOD(Repeat_NegativeValue_WrapsPositively)
        {
            AssertFloatEqual(3.0f, MathF::Repeat(-2.0f, 5.0f), 0.001f);
        }

        TEST_METHOD(PingPong_WithinRange_ReturnsValue)
        {
            AssertFloatEqual(2.0f, MathF::PingPong(2.0f, 5.0f));
        }

        TEST_METHOD(PingPong_BeyondRange_PingPongsBack)
        {
            AssertFloatEqual(3.0f, MathF::PingPong(7.0f, 5.0f), 0.001f);
        }

        TEST_METHOD(PingPong_FarBeyondRange_ContinuesPingPong)
        {
            AssertFloatEqual(1.0f, MathF::PingPong(11.0f, 5.0f), 0.001f);
        }

        // Random Function Tests (Basic validation)
        TEST_METHOD(Random01_ReturnsValueInRange)
        {
            float value = MathF::Random01();
            Assert::IsTrue(value >= 0.0f && value <= 1.0f);
        }

        TEST_METHOD(RandomRange_Float_ReturnsValueInRange)
        {
            float value = MathF::RandomRange(5.0f, 10.0f);
            Assert::IsTrue(value >= 5.0f && value <= 10.0f);
        }

        TEST_METHOD(RandomRange_Int_ReturnsValueInRange)
        {
            int value = MathF::RandomRange(5, 10);
            Assert::IsTrue(value >= 5 && value <= 10);
        }

        // Utility Function Tests
        TEST_METHOD(IsPowerOfTwo_PowersOfTwo_ReturnsTrue)
        {
            AssertBoolEqual(true, MathF::IsPowerOfTwo(1));
            AssertBoolEqual(true, MathF::IsPowerOfTwo(2));
            AssertBoolEqual(true, MathF::IsPowerOfTwo(4));
            AssertBoolEqual(true, MathF::IsPowerOfTwo(8));
            AssertBoolEqual(true, MathF::IsPowerOfTwo(16));
            AssertBoolEqual(true, MathF::IsPowerOfTwo(1024));
        }

        TEST_METHOD(IsPowerOfTwo_NonPowersOfTwo_ReturnsFalse)
        {
            AssertBoolEqual(false, MathF::IsPowerOfTwo(0));
            AssertBoolEqual(false, MathF::IsPowerOfTwo(3));
            AssertBoolEqual(false, MathF::IsPowerOfTwo(5));
            AssertBoolEqual(false, MathF::IsPowerOfTwo(6));
            AssertBoolEqual(false, MathF::IsPowerOfTwo(7));
            AssertBoolEqual(false, MathF::IsPowerOfTwo(15));
        }

        TEST_METHOD(NextPowerOfTwo_PowerOfTwo_ReturnsSameValue)
        {
            AssertIntEqual(8, MathF::NextPowerOfTwo(8));
            AssertIntEqual(16, MathF::NextPowerOfTwo(16));
        }

        TEST_METHOD(NextPowerOfTwo_NonPowerOfTwo_ReturnsNextPower)
        {
            AssertIntEqual(8, MathF::NextPowerOfTwo(5));
            AssertIntEqual(16, MathF::NextPowerOfTwo(9));
            AssertIntEqual(32, MathF::NextPowerOfTwo(17));
        }

        TEST_METHOD(NextPowerOfTwo_Zero_ReturnsOne)
        {
            AssertIntEqual(1, MathF::NextPowerOfTwo(0));
        }

        TEST_METHOD(NextPowerOfTwo_One_ReturnsOne)
        {
            AssertIntEqual(1, MathF::NextPowerOfTwo(1));
        }

        TEST_METHOD(Wrap_WithinRange_ReturnsValue)
        {
            AssertFloatEqual(5.0f, MathF::Wrap(5.0f, 0.0f, 10.0f));
        }

        TEST_METHOD(Wrap_AboveMax_ReturnsMin)
        {
            AssertFloatEqual(0.0f, MathF::Wrap(15.0f, 0.0f, 10.0f));
        }

        TEST_METHOD(Wrap_BelowMin_ReturnsMax)
        {
            AssertFloatEqual(10.0f, MathF::Wrap(-5.0f, 0.0f, 10.0f));
        }

        TEST_METHOD(MoveTowards_CloseToTarget_ReturnsTarget)
        {
            AssertFloatEqual(10.0f, MathF::MoveTowards(9.5f, 10.0f, 1.0f));
        }

        TEST_METHOD(MoveTowards_FarFromTarget_MovesMaxDelta)
        {
            AssertFloatEqual(6.0f, MathF::MoveTowards(5.0f, 10.0f, 1.0f));
        }

        TEST_METHOD(MoveTowards_NegativeDirection_MovesCorrectly)
        {
            AssertFloatEqual(9.0f, MathF::MoveTowards(10.0f, 5.0f, 1.0f));
        }

        // SmoothDamp Tests (Basic functionality)
        TEST_METHOD(SmoothDamp_AtTarget_StaysAtTarget)
        {
            float velocity = 0.0f;
            float result = MathF::SmoothDamp(10.0f, 10.0f, velocity, 1.0f);
            AssertFloatEqual(10.0f, result, 0.001f);
        }

        TEST_METHOD(SmoothDamp_MovesTowardsTarget)
        {
            float velocity = 0.0f;
            float result = MathF::SmoothDamp(0.0f, 10.0f, velocity, 1.0f);
            Assert::IsTrue(result > 0.0f && result < 10.0f);
        }

        TEST_METHOD(SmoothDamp_RespectMaxSpeed)
        {
            float velocity = 0.0f;
            float result = MathF::SmoothDamp(0.0f, 100.0f, velocity, 1.0f, 1.0f, 1.0f);
            Assert::IsTrue(result <= 1.0f); // Should not exceed max speed * deltaTime
        }

        // Gamma Correction Tests
        TEST_METHOD(LinearToGamma_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::LinearToGamma(0.0f), 0.001f);
        }

        TEST_METHOD(LinearToGamma_SmallValue_UsesLinearSegment)
        {
            float value = 0.001f;
            float expected = value * 12.92f;
            AssertFloatEqual(expected, MathF::LinearToGamma(value), 0.001f);
        }

        TEST_METHOD(LinearToGamma_LargeValue_UsesPowerCurve)
        {
            float value = 0.5f;
            float expected = 1.055f * MathF::Pow(value, 1.0f / 2.4f) - 0.055f;
            AssertFloatEqual(expected, MathF::LinearToGamma(value), 0.001f);
        }

        TEST_METHOD(GammaToLinear_Zero_ReturnsZero)
        {
            AssertFloatEqual(0.0f, MathF::GammaToLinear(0.0f), 0.001f);
        }

        TEST_METHOD(GammaToLinear_SmallValue_UsesLinearSegment)
        {
            float value = 0.01f;
            float expected = value / 12.92f;
            AssertFloatEqual(expected, MathF::GammaToLinear(value), 0.001f);
        }

        TEST_METHOD(GammaToLinear_LargeValue_UsesPowerCurve)
        {
            float value = 0.5f;
            float expected = MathF::Pow((value + 0.055f) / 1.055f, 2.4f);
            AssertFloatEqual(expected, MathF::GammaToLinear(value), 0.001f);
        }

        // Round-trip gamma correction tests
        TEST_METHOD(GammaCorrection_RoundTrip_LinearToGammaToLinear)
        {
            float original = 0.5f;
            float gamma = MathF::LinearToGamma(original);
            float backToLinear = MathF::GammaToLinear(gamma);
            AssertFloatEqual(original, backToLinear, 0.001f);
        }

        TEST_METHOD(GammaCorrection_RoundTrip_GammaToLinearToGamma)
        {
            float original = 0.5f;
            float linear = MathF::GammaToLinear(original);
            float backToGamma = MathF::LinearToGamma(linear);
            AssertFloatEqual(original, backToGamma, 0.001f);
        }

        // Edge Case Tests
        TEST_METHOD(EdgeCase_VerySmallNumbers_HandledCorrectly)
        {
            AssertFloatEqual(1e-6f, MathF::Abs(1e-6f));
            AssertFloatEqual(1e-6f, MathF::Max(1e-6f, 1e-7f));
        }

        TEST_METHOD(EdgeCase_VeryLargeNumbers_HandledCorrectly)
        {
            AssertFloatEqual(1e6f, MathF::Abs(1e6f));
            AssertFloatEqual(1e6f, MathF::Max(1e6f, 1e5f));
        }

        TEST_METHOD(EdgeCase_NegativeValues_HandledCorrectly)
        {
            AssertFloatEqual(5.0f, MathF::Abs(-5.0f));
            AssertFloatEqual(-1.0f, MathF::Sign(-5.0f));
            AssertFloatEqual(-2.0f, MathF::Min(-2.0f, -1.0f));
        }

        // Mathematical Property Tests
        TEST_METHOD(Property_AbsoluteValue_AlwaysNonNegative)
        {
            Assert::IsTrue(MathF::Abs(-100.0f) >= 0.0f);
            Assert::IsTrue(MathF::Abs(100.0f) >= 0.0f);
            Assert::IsTrue(MathF::Abs(0.0f) >= 0.0f);
        }

        TEST_METHOD(Property_MinMax_Relationship)
        {
            float a = 3.0f, b = 7.0f;
            Assert::IsTrue(MathF::Min(a, b) <= MathF::Max(a, b));
            AssertFloatEqual(a + b, MathF::Min(a, b) + MathF::Max(a, b));
        }

        TEST_METHOD(Property_TrigonometricIdentity_SinCos)
        {
            float angle = MathF::pi / 6.0f; // 30 degrees
            float sinVal = MathF::Sin(angle);
            float cosVal = MathF::Cos(angle);
            AssertFloatEqual(1.0f, sinVal * sinVal + cosVal * cosVal, 0.001f);
        }

        TEST_METHOD(Property_Logarithm_Exponential_Inverse)
        {
            float value = 2.5f;
            float expLog = MathF::Exp(MathF::Log(value));
            AssertFloatEqual(value, expLog, 0.001f);
        }

        TEST_METHOD(Property_SquareRoot_Square_Inverse)
        {
            float value = 7.0f;
            float sqrtSquare = MathF::Sqrt(MathF::Squared(value));
            AssertFloatEqual(value, sqrtSquare, 0.001f);
        }

        TEST_METHOD(Property_AngleConversion_RoundTrip)
        {
            float degrees = 45.0f;
            float radians = MathF::Radians(degrees);
            float backToDegrees = MathF::Degrees(radians);
            AssertFloatEqual(degrees, backToDegrees, 0.001f);
        }

        TEST_METHOD(Property_Clamp_AlwaysWithinBounds)
        {
            float min = 5.0f, max = 10.0f;
            Assert::IsTrue(MathF::Clamp(-100.0f, min, max) >= min);
            Assert::IsTrue(MathF::Clamp(100.0f, min, max) <= max);
            Assert::IsTrue(MathF::Clamp(7.5f, min, max) >= min && MathF::Clamp(7.5f, min, max) <= max);
        }

        TEST_METHOD(Property_Lerp_BoundedByInputs)
        {
            float a = 2.0f, b = 8.0f;
            float lerped = MathF::Lerp(a, b, 0.3f);
            Assert::IsTrue(lerped >= MathF::Min(a, b) && lerped <= MathF::Max(a, b));
        }
    };
}