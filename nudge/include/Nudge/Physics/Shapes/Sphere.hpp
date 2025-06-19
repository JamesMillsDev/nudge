#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Obb;
	class Plane;

	class Sphere
	{
	public:
		Vector3 origin;
		float radius;

	public:
		Sphere();
		Sphere(const Vector3& origin, float radius);

	public:
		bool Contains(const Vector3& point) const;
		Vector3 ClosestPoint(const Vector3& point) const;

		bool Overlaps(const Sphere& other) const;
		bool Overlaps(const Aabb& other) const;
		bool Overlaps(const Obb& other) const;
		bool Overlaps(const Plane& other) const;

	};
}