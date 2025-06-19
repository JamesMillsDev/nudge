#include "Nudge/Physics/Shapes/Sphere.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Physics/Shapes/AABB.hpp"
#include "Nudge/Physics/Shapes/OBB.hpp"
#include "Nudge/Physics/Shapes/Plane.hpp"
#include "Nudge/Physics/Shapes/Triangle.hpp"

namespace Nudge
{
	Sphere::Sphere()
		: Sphere(Vector3{ 0.f }, 1.f)
	{
	}

	Sphere::Sphere(const Vector3& origin, const float radius)
		: origin{ origin }, radius{ radius }
	{
	}

	bool Sphere::Contains(const Vector3& point) const
	{
		const float magSqr = (point - origin).MagnitudeSqr();
		const float radii = MathF::Squared(radius);

		return magSqr < radii;
	}

	Vector3 Sphere::ClosestPoint(const Vector3& point) const
	{
		return (point - origin).Normalized() + origin;
	}

	bool Sphere::Intersects(const Sphere& other) const
	{
		const float radiiSum = MathF::Squared(radius + other.radius);
		const float sqrDist = (origin - other.origin).MagnitudeSqr();
		
		return sqrDist < radiiSum;
	}

	bool Sphere::Intersects(const Aabb& other) const
	{
		const Vector3 closest = other.ClosestPoint(origin);
		const float distSqr = (origin - closest).MagnitudeSqr();

		return distSqr < MathF::Squared(radius);
	}

	bool Sphere::Intersects(const Obb& other) const
	{
		const Vector3 closest = other.ClosestPoint(origin);
		const float distSqr = (origin - closest).MagnitudeSqr();

		return distSqr < MathF::Squared(radius);
	}

	bool Sphere::Intersects(const Plane& other) const
	{
		const Vector3 closest = other.ClosestPoint(origin);
		const float distSqr = (origin - closest).MagnitudeSqr();

		return distSqr < MathF::Squared(radius);
	}

	bool Sphere::Intersects(const Triangle& other) const
	{
		return other.Intersects(*this);
	}
}
