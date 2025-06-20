#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Obb;
	class Sphere;
	class Triangle;

	class Plane
	{
	public:
		static Plane From(const Triangle& tri);

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

		bool Intersects(const Aabb& other) const;
		bool Intersects(const Obb& other) const;
		bool Intersects(const Plane& other) const;
		bool Intersects(const Sphere& other) const;
		bool Intersects(const Triangle& other) const;

	};
}