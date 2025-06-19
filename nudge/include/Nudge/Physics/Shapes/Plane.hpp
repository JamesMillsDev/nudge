#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Obb;
	class Sphere;

	class Plane
	{
	public:
		static float PlaneEquation(const Vector3& point, const Plane& plane);

	public:
		Vector3 normal;
		float distance;

	public:
		Plane();
		Plane(const Vector3& normal, float distance);

	public:
		bool Contains(const Vector3& point) const;
		Vector3 ClosestPoint(const Vector3& point) const;

		bool Overlaps(const Aabb& other) const;
		bool Overlaps(const Obb& other) const;
		bool Overlaps(const Plane& other) const;
		bool Overlaps(const Sphere& other) const;

	};
}