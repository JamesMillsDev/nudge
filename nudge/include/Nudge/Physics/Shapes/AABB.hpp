#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Obb;
	class Plane;
	class Sphere;
	class Triangle;

	class Aabb
	{
	public:
		static Aabb FromMinMax(const Vector3& min, const Vector3& max);

	public:
		Vector3 origin;
		Vector3 extents;

	public:
		Aabb();
		Aabb(const Vector3& origin, const Vector3& extents);

	public:
		Vector3 Min() const;
		Vector3 Max() const;

		bool Contains(const Vector3& point) const;
		Vector3 ClosestPoint(const Vector3& point) const;

		bool Intersects(const Aabb& other) const;
		bool Intersects(const Obb& other) const;
		bool Intersects(const Plane& other) const;
		bool Intersects(const Sphere& other) const;
		bool Intersects(const Triangle& other) const;

	};
}