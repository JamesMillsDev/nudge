#pragma once

#include "Nudge/Maths/Matrix3.hpp"
#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Plane;
	class Sphere;
	class Triangle;

	class Obb
	{
	public:
		Vector3 origin;
		Vector3 extents;
		Matrix3 orientation;

	public:
		Obb();
		Obb(const Vector3& origin, const Vector3& extents);
		Obb(const Vector3& origin, const Vector3& extents, const Matrix3& orientation);

	public:
		bool Contains(const Vector3& point) const;
		Vector3 ClosestPoint(const Vector3& point) const;

		bool Intersects(const Aabb& other) const;
		bool Intersects(const Obb& other) const;
		bool Intersects(const Plane& other) const;
		bool Intersects(const Sphere& other) const;
		bool Intersects(const Triangle& other) const;

	};
}
