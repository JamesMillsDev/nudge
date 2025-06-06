/**
 * @file MathF.cpp
 * @brief Implementation of mathematical utility functions for game development
 */

#include "Nudge/MathF.hpp"

#include <bitset>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>

using std::numbers::pi_v;
using std::numbers::e_v;

using std::bitset;
using std::mt19937;
using std::numeric_limits;
using std::random_device;
using std::uniform_int_distribution;
using std::uniform_real_distribution;

namespace Nudge
{
	// Mathematical constants initialization
	float MathF::pi = pi_v<float>;                                    // Pi (3.14159...)
	float MathF::epsilon = numeric_limits<float>::epsilon();          // Machine epsilon for float precision
	float MathF::e = e_v<float>;                                      // Euler's number (2.71828...)
	float MathF::infinity = numeric_limits<float>::infinity();        // Positive infinity
	float MathF::negativeInfinity = -numeric_limits<float>::infinity(); // Negative infinity

	/**
	 * @brief Checks if a value is approximately zero within a threshold
	 * @param value The value to test
	 * @param threshold The tolerance for considering the value zero
	 * @return true if the absolute value is within the threshold
	 */
	bool MathF::IsNearZero(const float value, const float threshold)
	{
		return Abs(value) <= threshold; // Absolute comparison for zero
	}

	/**
	 * @brief Compares two floating-point values with relative tolerance
	 * Uses adaptive epsilon scaling based on the magnitude of the values
	 * @param a First value to compare
	 * @param b Second value to compare
	 * @param threshold Additional threshold beyond machine epsilon
	 * @return true if values are approximately equal
	 */
	bool MathF::Compare(const float a, const float b, const float threshold)
	{
		return Abs(a - b) <= (epsilon + threshold) * Max(1.f, Max(Abs(a), Abs(b)));
	}

	/**
	 * @brief Constrains a value between minimum and maximum bounds
	 * @param value The value to clamp
	 * @param min The minimum allowed value
	 * @param max The maximum allowed value
	 * @return The clamped value
	 */
	float MathF::Clamp(const float value, const float min, const float max)
	{
		if (value < min)
		{
			return min;
		}

		if (value > max)
		{
			return max;
		}

		return value;
	}

	/**
	 * @brief Constrains a value between 0 and 1 (unit interval)
	 * @param value The value to clamp
	 * @return The clamped value in range [0, 1]
	 */
	float MathF::Clamp01(const float value)
	{
		if (value < 0.f)
		{
			return 0.f;
		}

		if (value > 1.f)
		{
			return 1.f;
		}

		return value;
	}

	/**
	 * @brief Converts radians to degrees
	 * @param radians Angle in radians
	 * @return Angle in degrees
	 */
	float MathF::Degrees(float radians)
	{
		return 180.f / pi * radians;
	}

	/**
	 * @brief Converts degrees to radians
	 * @param degrees Angle in degrees
	 * @return Angle in radians
	 */
	float MathF::Radians(float degrees)
	{
		return pi / 180.f * degrees;
	}

	/**
	 * @brief Returns the square of a value (value^2)
	 * @param val Input value
	 * @return val squared
	 */
	float MathF::Squared(float val)
	{
		return val * val;
	}

	/**
	 * @brief Returns the cube of a value (value^3)
	 * @param val Input value
	 * @return val cubed
	 */
	float MathF::Cubed(float val)
	{
		return val * val * val;
	}

	/**
	 * @brief Calculates the sine of an angle in radians
	 * @param radians Angle in radians
	 * @return Sine value
	 */
	float MathF::Sin(float radians)
	{
		return sinf(radians);
	}

	/**
	 * @brief Calculates the cosine of an angle in radians
	 * @param radians Angle in radians
	 * @return Cosine value
	 */
	float MathF::Cos(float radians)
	{
		return cosf(radians);
	}

	/**
	 * @brief Calculates the tangent of an angle in radians
	 * @param radians Angle in radians
	 * @return Tangent value
	 */
	float MathF::Tan(float radians)
	{
		return tanf(radians);
	}

	/**
	 * @brief Calculates the arcsine (inverse sine) of a value
	 * @param value Input value, must be in range [-1, 1]
	 * @return Angle in radians
	 */
	float MathF::Asin(float value)
	{
		return asinf(value);
	}

	/**
	 * @brief Calculates the arccosine (inverse cosine) of a value
	 * @param value Input value, must be in range [-1, 1]
	 * @return Angle in radians
	 */
	float MathF::Acos(float value)
	{
		return acosf(value);
	}

	/**
	 * @brief Calculates the arctangent (inverse tangent) of a value
	 * @param value Input value
	 * @return Angle in radians
	 */
	float MathF::Atan(float value)
	{
		return atanf(value);
	}

	/**
	 * @brief Calculates the two-argument arctangent of y/x
	 * Handles quadrant correctly and division by zero
	 * @param y Y coordinate
	 * @param x X coordinate
	 * @return Angle in radians from -pi to pi
	 */
	float MathF::Atan2(float y, float x)
	{
		return atan2f(y, x);
	}

	/**
	 * @brief Calculates the hyperbolic sine of a value
	 * @param value Input value
	 * @return Hyperbolic sine
	 */
	float MathF::Sinh(float value)
	{
		return sinhf(value);
	}

	/**
	 * @brief Calculates the hyperbolic cosine of a value
	 * @param value Input value
	 * @return Hyperbolic cosine
	 */
	float MathF::Cosh(float value)
	{
		return coshf(value);
	}

	/**
	 * @brief Calculates the hyperbolic tangent of a value
	 * @param value Input value
	 * @return Hyperbolic tangent
	 */
	float MathF::Tanh(float value)
	{
		return tanhf(value);
	}

	/**
	 * @brief Raises base to the power of exponent
	 * @param base Base value
	 * @param exponent Exponent value
	 * @return base raised to the power of exponent
	 */
	float MathF::Pow(float base, float exponent)
	{
		return powf(base, exponent);
	}

	/**
	 * @brief Calculates the square root of a value
	 * @param value Input value, must be non-negative
	 * @return Square root of value
	 */
	float MathF::Sqrt(float value)
	{
		return sqrtf(value);
	}

	/**
	 * @brief Calculates the cube root of a value
	 * @param value Input value
	 * @return Cube root of value
	 */
	float MathF::Cbrt(float value)
	{
		return cbrtf(value);
	}

	/**
	 * @brief Calculates e raised to the power of value
	 * @param value Exponent value
	 * @return e^value
	 */
	float MathF::Exp(float value)
	{
		return expf(value);
	}

	/**
	 * @brief Calculates the natural logarithm (base e) of a value
	 * @param value Input value, must be positive
	 * @return Natural logarithm of value
	 */
	float MathF::Log(float value)
	{
		return logf(value);
	}

	/**
	 * @brief Calculates the base-10 logarithm of a value
	 * @param value Input value, must be positive
	 * @return Base-10 logarithm of value
	 */
	float MathF::Log10(float value)
	{
		return log10f(value);
	}

	/**
	 * @brief Calculates the base-2 logarithm of a value
	 * @param value Input value, must be positive
	 * @return Base-2 logarithm of value
	 */
	float MathF::Log2(float value)
	{
		return log2f(value);
	}

	/**
	 * @brief Returns the largest integer less than or equal to value
	 * @param value Input value
	 * @return Floor of value
	 */
	float MathF::Floor(float value)
	{
		return floorf(value);
	}

	/**
	 * @brief Returns the smallest integer greater than or equal to value
	 * @param value Input value
	 * @return Ceiling of value
	 */
	float MathF::Ceil(float value)
	{
		return ceilf(value);
	}

	/**
	 * @brief Rounds value to the nearest integer
	 * @param value Input value
	 * @return Rounded value
	 */
	float MathF::Round(float value)
	{
		return roundf(value);
	}

	/**
	 * @brief Truncates the decimal part of a value (rounds toward zero)
	 * @param value Input value
	 * @return Truncated value
	 */
	float MathF::Trunc(float value)
	{
		return truncf(value);
	}

	/**
	 * @brief Returns the fractional part of a value
	 * Uses modf to efficiently extract both integer and fractional parts
	 * @param value Input value
	 * @return Fractional part (preserves sign)
	 */
	float MathF::Frac(float value)
	{
		float integerPart;

		return modf(value, &integerPart);
	}

	/**
	 * @brief Returns the absolute value of a number
	 * @param value Input value
	 * @return Absolute value (always non-negative)
	 */
	float MathF::Abs(float value)
	{
		return fabsf(value);
	}

	/**
	 * @brief Returns the sign of a value
	 * @param value Input value
	 * @return 1.0f if positive, -1.0f if negative, 0.0f if zero
	 */
	float MathF::Sign(float value)
	{
		if (value > 0.f)
		{
			return 1.f;
		}

		if (value < 0.f)
		{
			return -1.f;
		}

		return 0.f;
	}

	/**
	 * @brief Returns the smaller of two values
	 * @param a First value
	 * @param b Second value
	 * @return Minimum value
	 */
	float MathF::Min(float a, float b)
	{
		return fminf(a, b);
	}

	/**
	 * @brief Returns the larger of two values
	 * @param a First value
	 * @param b Second value
	 * @return Maximum value
	 */
	float MathF::Max(float a, float b)
	{
		return fmaxf(a, b);
	}

	/**
	 * @brief Linear interpolation between two values
	 * Clamps t to [0, 1] range for safety
	 * @param a Start value
	 * @param b End value
	 * @param t Interpolation factor (clamped to [0, 1])
	 * @return Interpolated value
	 */
	float MathF::Lerp(float a, float b, float t)
	{
		t = Clamp01(t);

		return a * (1.f - t) + b * t;
	}

	/**
	 * @brief Linear interpolation between two values without clamping
	 * Allows extrapolation beyond [0, 1] range
	 * @param a Start value
	 * @param b End value
	 * @param t Interpolation factor (unclamped)
	 * @return Interpolated value
	 */
	float MathF::LerpUnclamped(float a, float b, float t)
	{
		return a * (1.f - t) + b * t;
	}

	/**
	 * @brief Inverse linear interpolation - finds t for a given interpolated value
	 * @param a Start value
	 * @param b End value
	 * @param value The interpolated value to find t for
	 * @return The t parameter that would produce this value
	 */
	float MathF::InverseLerp(float a, float b, float value)
	{
		return (value - a) / (b - a);
	}

	/**
	 * @brief Hermite interpolation with smooth acceleration/deceleration
	 * Uses cubic polynomial: 3t^2 - 2t^3
	 * @param edge0 Lower edge of interpolation range
	 * @param edge1 Upper edge of interpolation range
	 * @param x Input value
	 * @return Smoothly interpolated value
	 */
	float MathF::SmoothStep(float edge0, float edge1, float x)
	{
		x = Clamp01((x - edge0) / (edge1 - edge0));

		return x * x * (3.0f - 2.0f * x);
	}

	/**
	 * @brief Enhanced Hermite interpolation with even smoother curves
	 * Uses quintic polynomial: 6t^5 - 15t^4 + 10t^3
	 * @param edge0 Lower edge of interpolation range
	 * @param edge1 Upper edge of interpolation range
	 * @param x Input value
	 * @return Very smoothly interpolated value
	 */
	float MathF::SmootherStep(float edge0, float edge1, float x)
	{
		x = Clamp01((x - edge0) / (edge1 - edge0));

		return x * x * x * (x * (6.f * x - 15.f) + 10.f);
	}

	/**
	 * @brief Floating-point remainder of division
	 * @param a Dividend
	 * @param b Divisor
	 * @return Remainder of a/b
	 */
	float MathF::Mod(float a, float b)
	{
		return fmodf(a, b);
	}

	/**
	 * @brief Wraps a value to repeat within a given length
	 * Creates a sawtooth wave pattern
	 * @param t Input value
	 * @param length Length of the repeating interval
	 * @return Value wrapped to [0, length) range
	 */
	float MathF::Repeat(float t, float length)
	{
		return t - Floor(t / length) * length;
	}

	/**
	 * @brief Creates a ping-pong (triangle wave) pattern
	 * Value oscillates back and forth between 0 and length
	 * @param t Input value
	 * @param length Maximum value before ping-ponging back
	 * @return Value that bounces between 0 and length
	 */
	float MathF::PingPong(float t, float length)
	{
		return length - Abs(Repeat(t, 2.f * length) - length);
	}

	/**
	 * @brief Generates a random float between 0 and 1
	 * @return Random value in range [0, 1]
	 */
	float MathF::Random01()
	{
		return RandomRange(0.f, 1.f);
	}

	/**
	 * @brief Generates a random float within a specified range
	 * @param min Minimum value (inclusive)
	 * @param max Maximum value (inclusive)
	 * @return Random float value
	 */
	float MathF::RandomRange(float min, float max)
	{
		random_device rd;
		mt19937 gen(rd());

		uniform_real_distribution range = uniform_real_distribution(min, max);

		return range(gen);
	}

	/**
	 * @brief Generates a random integer within a specified range
	 * @param min Minimum value (inclusive)
	 * @param max Maximum value (inclusive)
	 * @return Random integer value
	 */
	int MathF::RandomRange(int min, int max)
	{
		random_device rd;
		mt19937 gen(rd());

		uniform_int_distribution range = uniform_int_distribution(min, max);

		return range(gen);
	}

	/**
	 * @brief Checks if an integer is a power of 2
	 * Uses bit counting method to verify exactly one bit is set
	 * @param value Integer to test
	 * @return true if value is a power of 2
	 */
	bool MathF::IsPowerOfTwo(int value)
	{
		constexpr int bitCount = 32;

		bitset<bitCount> bits = bitset<bitCount>(value);

		int bitSetCount = 0;
		for (int i = 0; i < bitCount; ++i)
		{
			if (bits[i] == 1)
			{
				bitSetCount++;
			}
		}

		return bitSetCount == 1;
	}

	/**
	 * @brief Finds the next power of 2 greater than or equal to the input
	 * Uses bit manipulation for efficient calculation
	 * @param value Input integer
	 * @return Smallest power of 2 >= value
	 */
	int MathF::NextPowerOfTwo(int value)
	{
		// Handle edge cases
		if (value <= 1)
		{
			return 1;
		}

		if (value > 1 << 30)
		{
			return 1 << 30; // Prevent overflow for 32-bit int
		}

		// If already a power of 2, return it
		if ((value & value - 1) == 0)
		{
			return value;
		}

		// Bit manipulation approach
		value--; // Decrement to handle case where input is already power of 2

		// Set all bits to the right of the most significant bit
		value |= value >> 1;
		value |= value >> 2;
		value |= value >> 4;
		value |= value >> 8;
		value |= value >> 16;

		// Add 1 to get the next power of 2
		return value + 1;
	}

	/**
	 * @brief Wraps a value between min and max bounds
	 * Simple boundary wrapping (jumps from max to min)
	 * @param value Value to wrap
	 * @param min Minimum bound
	 * @param max Maximum bound
	 * @return Wrapped value
	 */
	float MathF::Wrap(float value, float min, float max)
	{
		if (value > max)
		{
			return min;
		}

		if (value < min)
		{
			return max;
		}

		return value;
	}

	/**
	 * @brief Moves current value toward target at constant speed
	 * Stops exactly at target when reached
	 * @param current Starting value
	 * @param target Destination value
	 * @param maxDelta Maximum distance to move per call
	 * @return New position after moving
	 */
	float MathF::MoveTowards(float current, float target, float maxDelta)
	{
		if (Abs(target - current) <= maxDelta)
		{
			return target;
		}

		return current + Sign(target - current) * maxDelta;
	}

	/**
	 * @brief Smoothly damps a value toward a target using spring physics
	 * Implements critically damped spring-mass system for smooth motion
	 * @param current Current value
	 * @param target Target value
	 * @param velocity Current velocity (modified by reference)
	 * @param smoothTime Approximate time to reach target
	 * @param maxSpeed Maximum speed limit
	 * @param deltaTime Time step for this frame
	 * @return New smoothly damped value
	 */
	float MathF::SmoothDamp(float current, float target, float& velocity, float smoothTime, float maxSpeed,
		float deltaTime)
	{
		const float omega = 2.f / smoothTime;                          // Angular frequency
		const float x = omega * deltaTime;                             // Damping factor
		const float maxChange = maxSpeed * deltaTime;                  // Maximum change this frame

		// Approximation of exponential decay for stability
		const float exp = 1.f / (1.f + x + 0.48f * Squared(x) + 0.235f * Cubed(x));

		const float originalTo = target;                               // Store original target
		float change = current - target;                               // Distance to target

		// Clamp the change to maxChange (speed limiting)
		if (Abs(change) > maxChange)
		{
			change = Sign(change) * maxChange;
			target = current - change;  // Modify target due to speed limiting
		}

		// Apply spring-damper equations
		const float temp = (velocity + omega * change) * deltaTime;
		velocity = (velocity - omega * temp) * exp;

		float result = target + (change + temp) * exp;

		// Prevent overshooting the original target
		if ((current - originalTo > 0) == (result - originalTo > 0))
		{
			return result;
		}

		// If we would overshoot, clamp to target and adjust velocity
		velocity = (result - originalTo) / deltaTime;
		return originalTo;
	}

	/**
	 * @brief Converts linear color value to gamma-corrected (sRGB) space
	 * Implements official sRGB specification with piecewise function
	 * @param value Linear color value [0, 1]
	 * @return Gamma-corrected sRGB value [0, 1]
	 */
	float MathF::LinearToGamma(float value)
	{
		if (value <= 0.0031308f)
		{
			return value * 12.92f;                                     // Linear segment for dark values
		}

		return 1.055f * Pow(value, 1.0f / 2.4f) - 0.055f;            // Power curve for bright values
	}

	/**
	 * @brief Converts gamma-corrected (sRGB) color value to linear space
	 * Implements inverse of sRGB gamma correction
	 * @param value sRGB gamma-corrected value [0, 1]
	 * @return Linear color value [0, 1]
	 */
	float MathF::GammaToLinear(float value)
	{
		if (value <= 0.04045f)
		{
			return value / 12.92f;                                     // Inverse linear segment
		}

		return Pow((value + 0.055f) / 1.055f, 2.4f);                 // Inverse power curve
	}
}