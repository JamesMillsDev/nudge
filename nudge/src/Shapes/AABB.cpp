#include "Nudge/Shapes/AABB.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/Interval.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"
#include "Nudge/Shapes/Triangle.hpp"

namespace Nudge
{
	Aabb Aabb::FromMinMax(const Vector3& min, const Vector3& max)
	{
		// Calculate center as midpoint between min and max
		// Calculate extents as half the distance from min to max
		return { (min + max) * .5f, (max - min) * .5f };
	}

	Aabb::Aabb()
		: origin{ 0.f }, extents{ 1.f }
	{
	}

	Aabb::Aabb(const Vector3& origin, const Vector3& extents)
		: origin{ origin }, extents{ extents }
	{
	}

	Vector3 Aabb::Min() const
	{
		// Calculate both possible corner points
		const Vector3 p1 = origin + extents;
		const Vector3 p2 = origin - extents;

		// Return the component-wise minimum
		return Vector3::Min(p1, p2);
	}

	Vector3 Aabb::Max() const
	{
		// Calculate both possible corner points
		const Vector3 p1 = origin + extents;
		const Vector3 p2 = origin - extents;

		// Return the component-wise maximum
		return Vector3::Max(p1, p2);
	}

	bool Aabb::Contains(const Vector3& point) const
	{
		// Get the AABB bounds
		const Vector3 min = Min();
		const Vector3 max = Max();

		// Check if point is within bounds on all three axes
		return point.x >= min.x && point.y >= min.y && point.z >= min.z &&
		       point.x <= max.x && point.y <= max.y && point.z <= max.z;
	}

	Vector3 Aabb::ClosestPoint(const Vector3& point) const
	{
		// Start with the input point
		Vector3 result = point;
		const Vector3 min = Min();
		const Vector3 max = Max();

		// Clamp each component to the minimum bounds
		result.x = result.x < min.x ? min.x : result.x;
		result.y = result.y < min.y ? min.y : result.y;
		result.z = result.z < min.z ? min.z : result.z;

		// Clamp each component to the maximum bounds
		result.x = result.x > max.x ? max.x : result.x;
		result.y = result.y > max.y ? max.y : result.y;
		result.z = result.z > max.z ? max.z : result.z;

		return result;
	}

	bool Aabb::Intersects(const Aabb& other) const
	{
		// Get bounds for both AABBs
		const Vector3 aMin = Min();
		const Vector3 aMax = Max();

		const Vector3 bMin = other.Min();
		const Vector3 bMax = other.Max();

		// Check for overlap on all three axes using separating axis theorem
		return aMin.x <= bMax.x && aMax.x >= bMin.x &&
		       aMin.y <= bMax.y && aMax.y >= bMin.y &&
		       aMin.z <= bMax.z && aMax.z >= bMin.z;
	}

	bool Aabb::Intersects(const Obb& other) const
	{
		// Delegate to specialized AABB-OBB intersection algorithm
		return Interval::AabbObb(*this, other);
	}

	bool Aabb::Intersects(const Plane& other) const
	{
		// Calculate the projection length of the AABB onto the plane normal
		const float pLen = extents.x * MathF::Abs(other.normal.x) +
		             extents.y * MathF::Abs(other.normal.y) +
		             extents.z * MathF::Abs(other.normal.z);

		// Calculate the distance from AABB center to the plane
		const float dot = Vector3::Dot(other.normal, origin);

		// Check if the distance is within the projection length
		return MathF::Abs(dot - other.distance) <= pLen;
	}

	bool Aabb::Intersects(const Sphere& other) const
	{
		// Delegate to the sphere's intersection method (avoids code duplication)
		return other.Intersects(*this);
	}

	bool Aabb::Intersects(const Triangle& other) const
	{
		// Delegate to the triangle's intersection method (avoids code duplication)
		return other.Intersects(*this);
	}
}