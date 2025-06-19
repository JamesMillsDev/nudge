#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Obb;
	class Plane;
	class Triangle;

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

		bool Intersects(const Sphere& other) const;
		bool Intersects(const Aabb& other) const;
		bool Intersects(const Obb& other) const;
		bool Intersects(const Plane& other) const;
		bool Intersects(const Triangle& other) const;

	};
}