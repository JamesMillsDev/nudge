/**
 * @file MathF.hpp
 * @brief Mathematical utility class providing common game development math functions
 */

#pragma once

namespace Nudge
{
    /**
     * @class MathF
     * @brief Static utility class for mathematical operations commonly used in game development
     *
     * Provides a comprehensive set of mathematical functions including:
     * - Basic arithmetic and comparison operations
     * - Trigonometric and hyperbolic functions
     * - Interpolation and easing functions
     * - Random number generation
     * - Gamma correction for color spaces
     * - Utility functions for game programming
     */
    class MathF
    {
    public:
        // Mathematical constants
        static float pi;                    ///< Pi constant (3.14159...)
        static float epsilon;               ///< Machine epsilon for floating-point comparisons
        static float e;                     ///< Euler's number (2.71828...)
        static float infinity;              ///< Positive infinity
        static float negativeInfinity;      ///< Negative infinity

    public:
        // Basic comparison and clamping functions

        /**
         * @brief Checks if a value is approximately zero within a threshold
         * @param value The value to test against zero
         * @param threshold The tolerance for considering the value zero (default: machine epsilon)
         * @return true if the absolute value is within the threshold
         */
        static bool IsNearZero(float value, float threshold = epsilon);

        /**
         * @brief Compares two floating-point values with relative tolerance
         * @param a First value to compare
         * @param b Second value to compare
         * @param threshold Additional threshold beyond machine epsilon (default: 0.00001f)
         * @return true if values are approximately equal
         */
        static bool Compare(float a, float b, float threshold = .00001f);

        /**
         * @brief Constrains a value between minimum and maximum bounds
         * @param value The value to clamp
         * @param min The minimum allowed value
         * @param max The maximum allowed value
         * @return The clamped value
         */
        static float Clamp(float value, float min, float max);

        /**
         * @brief Constrains a value between 0 and 1 (unit interval)
         * @param value The value to clamp
         * @return The clamped value in range [0, 1]
         */
        static float Clamp01(float value);

        /**
         * @brief Converts radians to degrees
         * @param radians Angle in radians
         * @return Angle in degrees
         */
        static float Degrees(float radians);

        /**
         * @brief Converts degrees to radians
         * @param degrees Angle in degrees
         * @return Angle in radians
         */
        static float Radians(float degrees);

        /**
         * @brief Returns the square of a value (value^2)
         * @param val Input value
         * @return val squared
         */
        static float Squared(float val);

        /**
         * @brief Returns the cube of a value (value^3)
         * @param val Input value
         * @return val cubed
         */
        static float Cubed(float val);

        // Trigonometric functions

        /**
         * @brief Calculates the sine of an angle in radians
         * @param radians Angle in radians
         * @return Sine value
         */
        static float Sin(float radians);

        /**
         * @brief Calculates the cosine of an angle in radians
         * @param radians Angle in radians
         * @return Cosine value
         */
        static float Cos(float radians);

        /**
         * @brief Calculates the tangent of an angle in radians
         * @param radians Angle in radians
         * @return Tangent value
         */
        static float Tan(float radians);

        /**
         * @brief Calculates the arcsine (inverse sine) of a value
         * @param value Input value, must be in range [-1, 1]
         * @return Angle in radians
         */
        static float Asin(float value);

        /**
         * @brief Calculates the arccosine (inverse cosine) of a value
         * @param value Input value, must be in range [-1, 1]
         * @return Angle in radians
         */
        static float Acos(float value);

        /**
         * @brief Calculates the arctangent (inverse tangent) of a value
         * @param value Input value
         * @return Angle in radians
         */
        static float Atan(float value);

        /**
         * @brief Calculates the two-argument arctangent of y/x
         * @param y Y coordinate
         * @param x X coordinate
         * @return Angle in radians from -pi to pi, handles quadrants correctly
         */
        static float Atan2(float y, float x);

        // Hyperbolic functions

        /**
         * @brief Calculates the hyperbolic sine of a value
         * @param value Input value
         * @return Hyperbolic sine
         */
        static float Sinh(float value);

        /**
         * @brief Calculates the hyperbolic cosine of a value
         * @param value Input value
         * @return Hyperbolic cosine
         */
        static float Cosh(float value);

        /**
         * @brief Calculates the hyperbolic tangent of a value
         * @param value Input value
         * @return Hyperbolic tangent
         */
        static float Tanh(float value);

        // Power and exponential functions

        /**
         * @brief Raises base to the power of exponent
         * @param base Base value
         * @param exponent Exponent value
         * @return base raised to the power of exponent
         */
        static float Pow(float base, float exponent);

        /**
         * @brief Calculates the square root of a value
         * @param value Input value, must be non-negative
         * @return Square root of value
         */
        static float Sqrt(float value);

        /**
         * @brief Calculates the cube root of a value
         * @param value Input value
         * @return Cube root of value
         */
        static float Cbrt(float value);

        /**
         * @brief Calculates e raised to the power of value
         * @param value Exponent value
         * @return e^value
         */
        static float Exp(float value);

        /**
         * @brief Calculates the natural logarithm (base e) of a value
         * @param value Input value, must be positive
         * @return Natural logarithm of value
         */
        static float Log(float value);

        /**
         * @brief Calculates the base-10 logarithm of a value
         * @param value Input value, must be positive
         * @return Base-10 logarithm of value
         */
        static float Log10(float value);

        /**
         * @brief Calculates the base-2 logarithm of a value
         * @param value Input value, must be positive
         * @return Base-2 logarithm of value
         */
        static float Log2(float value);

        // Rounding and comparison functions

        /**
         * @brief Returns the largest integer less than or equal to value
         * @param value Input value
         * @return Floor of value
         */
        static float Floor(float value);

        /**
         * @brief Returns the smallest integer greater than or equal to value
         * @param value Input value
         * @return Ceiling of value
         */
        static float Ceil(float value);

        /**
         * @brief Rounds value to the nearest integer
         * @param value Input value
         * @return Rounded value
         */
        static float Round(float value);

        /**
         * @brief Truncates the decimal part of a value (rounds toward zero)
         * @param value Input value
         * @return Truncated value
         */
        static float Trunc(float value);

        /**
         * @brief Returns the fractional part of a value
         * @param value Input value
         * @return Fractional part (preserves sign)
         */
        static float Frac(float value);

        /**
         * @brief Returns the absolute value of a number
         * @param value Input value
         * @return Absolute value (always non-negative)
         */
        static float Abs(float value);

        /**
         * @brief Returns the sign of a value
         * @param value Input value
         * @return 1.0f if positive, -1.0f if negative, 0.0f if zero
         */
        static float Sign(float value);

        /**
         * @brief Returns the smaller of two values
         * @param a First value
         * @param b Second value
         * @return Minimum value
         */
        static float Min(float a, float b);

        /**
         * @brief Returns the larger of two values
         * @param a First value
         * @param b Second value
         * @return Maximum value
         */
        static float Max(float a, float b);

        // Interpolation and easing functions

        /**
         * @brief Linear interpolation between two values (clamped)
         * @param a Start value
         * @param b End value
         * @param t Interpolation factor (clamped to [0, 1])
         * @return Interpolated value
         */
        static float Lerp(float a, float b, float t);

        /**
         * @brief Linear interpolation between two values (unclamped)
         * @param a Start value
         * @param b End value
         * @param t Interpolation factor (allows extrapolation)
         * @return Interpolated value
         */
        static float LerpUnclamped(float a, float b, float t);

        /**
         * @brief Inverse linear interpolation - finds t for a given interpolated value
         * @param a Start value
         * @param b End value
         * @param value The interpolated value to find t for
         * @return The t parameter that would produce this value
         */
        static float InverseLerp(float a, float b, float value);

        /**
         * @brief Hermite interpolation with smooth acceleration/deceleration
         * @param edge0 Lower edge of interpolation range
         * @param edge1 Upper edge of interpolation range
         * @param x Input value
         * @return Smoothly interpolated value using cubic polynomial
         */
        static float SmoothStep(float edge0, float edge1, float x);

        /**
         * @brief Enhanced Hermite interpolation with even smoother curves
         * @param edge0 Lower edge of interpolation range
         * @param edge1 Upper edge of interpolation range
         * @param x Input value
         * @return Very smoothly interpolated value using quintic polynomial
         */
        static float SmootherStep(float edge0, float edge1, float x);

        // Modulo and wrapping functions

        /**
         * @brief Floating-point remainder of division
         * @param a Dividend
         * @param b Divisor
         * @return Remainder of a/b
         */
        static float Mod(float a, float b);

        /**
         * @brief Wraps a value to repeat within a given length (sawtooth wave)
         * @param t Input value
         * @param length Length of the repeating interval
         * @return Value wrapped to [0, length) range
         */
        static float Repeat(float t, float length);

        /**
         * @brief Creates a ping-pong (triangle wave) pattern
         * @param t Input value
         * @param length Maximum value before ping-ponging back
         * @return Value that oscillates between 0 and length
         */
        static float PingPong(float t, float length);

        // Random number generation utilities

        /**
         * @brief Generates a random float between 0 and 1
         * @return Random value in range [0, 1]
         */
        static float Random01();

        /**
         * @brief Generates a random float within a specified range
         * @param min Minimum value (inclusive)
         * @param max Maximum value (inclusive)
         * @return Random float value
         */
        static float RandomRange(float min, float max);

        /**
         * @brief Generates a random integer within a specified range
         * @param min Minimum value (inclusive)
         * @param max Maximum value (inclusive)
         * @return Random integer value
         */
        static int RandomRange(int min, int max);

        // Utility functions for game development

        /**
         * @brief Checks if an integer is a power of 2
         * @param value Integer to test
         * @return true if value is a power of 2
         */
        static bool IsPowerOfTwo(int value);

        /**
         * @brief Finds the next power of 2 greater than or equal to the input
         * @param value Input integer
         * @return Smallest power of 2 >= value
         */
        static int NextPowerOfTwo(int value);

        /**
         * @brief Wraps a value between min and max bounds (simple boundary wrapping)
         * @param value Value to wrap
         * @param min Minimum bound
         * @param max Maximum bound
         * @return Wrapped value (jumps from max to min)
         */
        static float Wrap(float value, float min, float max);

        /**
         * @brief Moves current value toward target at constant speed
         * @param current Starting value
         * @param target Destination value
         * @param maxDelta Maximum distance to move per call
         * @return New position after moving (stops exactly at target)
         */
        static float MoveTowards(float current, float target, float maxDelta);

        /**
         * @brief Smoothly damps a value toward a target using spring physics
         * @param current Current value
         * @param target Target value
         * @param velocity Current velocity (modified by reference)
         * @param smoothTime Approximate time to reach target
         * @param maxSpeed Maximum speed limit (default: infinity)
         * @param deltaTime Time step for this frame (default: 0.016f for 60fps)
         * @return New smoothly damped value
         */
        static float SmoothDamp(float current, float target, float& velocity, float smoothTime, float maxSpeed = infinity, float deltaTime = 0.016f);

        // Gamma correction for color spaces

        /**
         * @brief Converts linear color value to gamma-corrected (sRGB) space
         * @param value Linear color value [0, 1]
         * @return Gamma-corrected sRGB value [0, 1]
         */
        static float LinearToGamma(float value);

        /**
         * @brief Converts gamma-corrected (sRGB) color value to linear space
         * @param value sRGB gamma-corrected value [0, 1]
         * @return Linear color value [0, 1]
         */
        static float GammaToLinear(float value);
    };
}