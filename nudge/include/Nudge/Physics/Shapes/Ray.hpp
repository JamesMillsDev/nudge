#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Ray
	{
	public:
		static Ray FromPoints(const Vector3& from, const Vector3& to);

	public:
		Vector3 origin;
		Vector3 direction;

	public:
		Ray();
		Ray(const Vector3& origin, const Vector3& direction);

	public:
		void Normalize();
		bool Contains(const Vector3& point) const;
		Vector3 ClosestPoint(const Vector3& point) const;

	};
}