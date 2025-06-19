#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Triangle
	{
	public:
		union
		{
			struct
			{
				Vector3 a;
				Vector3 b;
				Vector3 c;
			};

			Vector3 points[3];
			float values[9];
		};

	public:
		Triangle();
		Triangle(const Vector3& a, const Vector3& b, const Vector3& c);

	};
}