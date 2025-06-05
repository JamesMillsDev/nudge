#include "nudge/MathF.hpp"

#include <cfloat>
#include <cmath>
#include <numbers>

using std::numbers::pi_v;

namespace Nudge
{
	float MathF::pi = pi_v<float>;

	bool MathF::IsNearZero(const float value, const float threshold)
	{
		return fabsf(value) <= threshold;  // Absolute comparison for zero
	}

	bool MathF::Compare(const float a, const float b, const float threshold)
	{
		return fabsf(a - b) <= (FLT_EPSILON + threshold) * fmaxf(1.f, fmaxf(fabsf(a), fabsf(b)));
	}

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

	float MathF::Degrees(float radians)
	{
		return 180.f / pi * radians;
	}

	float MathF::Radians(float degrees)
	{
		return pi / 180.f * degrees;
	}
}
