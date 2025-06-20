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
		const Vector3 p1 = origin + extents;
		const Vector3 p2 = origin - extents;

		return Vector3::Min(p1, p2);
	}

	Vector3 Aabb::Max() const
	{
		const Vector3 p1 = origin + extents;
		const Vector3 p2 = origin - extents;

		return Vector3::Max(p1, p2);
	}

	bool Aabb::Contains(const Vector3& point) const
	{
		const Vector3 min = Min();
		const Vector3 max = Max();

		return point.x > min.x && point.y > min.y && point.z > min.z &&
		       point.x < max.x && point.y < max.y && point.z < max.z;
	}

	Vector3 Aabb::ClosestPoint(const Vector3& point) const
	{
		Vector3 result = point;
		const Vector3 min = Min();
		const Vector3 max = Max();

		result.x = result.x < min.x ? min.x : result.x;
		result.y = result.y < min.y ? min.y : result.y;
		result.z = result.z < min.z ? min.z : result.z;

		result.x = result.x > max.x ? max.x : result.x;
		result.y = result.y > max.y ? max.y : result.y;
		result.z = result.z > max.z ? max.z : result.z;

		return result;
	}

	bool Aabb::Intersects(const Aabb& other) const
	{
		const Vector3 aMin = Min();
		const Vector3 aMax = Max();

		const Vector3 bMin = other.Min();
		const Vector3 bMax = other.Max();

		return aMin.x <= bMax.x && aMax.x >= bMin.x &&
		       aMin.y <= bMax.y && aMax.y >= bMin.y &&
		       aMin.z <= bMax.z && aMax.z >= bMin.z;
	}

	bool Aabb::Intersects(const Obb& other) const
	{
		return Interval::AabbObb(*this, other);
	}

	bool Aabb::Intersects(const Plane& other) const
	{
		const float pLen = extents.x * MathF::Abs(other.normal.x) +
		             extents.y * MathF::Abs(other.normal.y) +
		             extents.z * MathF::Abs(other.normal.z);

		const float dot = Vector3::Dot(other.normal, origin);

		return MathF::Abs(dot - other.distance) <= pLen;
	}

	bool Aabb::Intersects(const Sphere& other) const
	{
		return other.Intersects(*this);
	}

	bool Aabb::Intersects(const Triangle& other) const
	{
		return other.Intersects(*this);
	}
}
