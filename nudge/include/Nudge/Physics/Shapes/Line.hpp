#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Line
	{
	public:
		Vector3 start;
		Vector3 end;

	public:
		Line();
		Line(const Vector3& start, const Vector3& end);

	public:
		float Length() const;
		float LengthSqr() const;

	public:
		bool Contains(const Vector3& point) const;
		Vector3 ClosestPoint(const Vector3& point) const;

	};
}