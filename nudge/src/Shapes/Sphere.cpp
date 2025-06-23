#include "Nudge/Shapes/Sphere.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/OBB.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Triangle.hpp"

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
		// Calculate squared distance from sphere center to the point
		const float magSqr = (point - origin).MagnitudeSqr();
		// Calculate squared radius for comparison (avoids expensive square root)
		const float radii = MathF::Squared(radius);

		// Point is inside if distance squared is less than radius squared
		return magSqr < radii;
	}

	Vector3 Sphere::ClosestPoint(const Vector3& point) const
	{
		// Get direction from sphere center to the point and normalize it
		// Then scale by radius and add to sphere center to get closest surface point
		return (point - origin).Normalized() * radius + origin;
	}

	bool Sphere::Intersects(const Sphere& other) const
	{
		// Calculate squared sum of radii for comparison
		const float radiiSum = MathF::Squared(radius + other.radius);
		// Calculate squared distance between sphere centers
		const float sqrDist = (origin - other.origin).MagnitudeSqr();
		
		// Spheres intersect if center distance is less than sum of radii
		return sqrDist < radiiSum;
	}

	bool Sphere::Intersects(const Aabb& other) const
	{
		// Find closest point on AABB surface to sphere center
		const Vector3 closest = other.ClosestPoint(origin);
		// Calculate squared distance from sphere center to closest point
		const float distSqr = (origin - closest).MagnitudeSqr();

		// Sphere intersects AABB if closest point is within sphere radius
		return distSqr < MathF::Squared(radius);
	}

	bool Sphere::Intersects(const Obb& other) const
	{
		// Find closest point on OBB surface to sphere center
		const Vector3 closest = other.ClosestPoint(origin);
		// Calculate squared distance from sphere center to closest point
		const float distSqr = (origin - closest).MagnitudeSqr();

		// Sphere intersects OBB if closest point is within sphere radius
		return distSqr < MathF::Squared(radius);
	}

	bool Sphere::Intersects(const Plane& other) const
	{
		// Find closest point on plane to sphere center
		const Vector3 closest = other.ClosestPoint(origin);
		// Calculate squared distance from sphere center to closest point on plane
		const float distSqr = (origin - closest).MagnitudeSqr();

		// Sphere intersects plane if closest point is within sphere radius
		return distSqr < MathF::Squared(radius);
	}

	bool Sphere::Intersects(const Triangle& other) const
	{
		// Delegate to triangle's intersection method (avoids code duplication)
		return other.Intersects(*this);
	}
}