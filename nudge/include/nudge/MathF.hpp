#pragma once
#include <float.h>

namespace Nudge
{
	class MathF
	{
	public:
		static float pi;

	public:
		static bool IsNearZero(float value, float threshold = FLT_EPSILON);
		static bool Compare(float a, float b, float threshold = .00001f);

		static float Clamp(float value, float min, float max);
		static float Clamp01(float value);

		static float Degrees(float radians);
		static float Radians(float degrees);

	};
}
