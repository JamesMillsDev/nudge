#include "Nudge/Physics/Shapes/Plane.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Physics/Shapes/AABB.hpp"
#include "Nudge/Physics/Shapes/OBB.hpp"
#include "Nudge/Physics/Shapes/Sphere.hpp"

namespace Nudge
{
	float Plane::PlaneEquation(const Vector3& point, const Plane& plane)
	{
		return Vector3::Dot(point, plane.normal) - plane.distance;
	}

	Plane::Plane()
		: Plane{ Vector3{ 1.f, 0.f, 0.f }, 0.f }
	{
	}

	Plane::Plane(const Vector3& normal, float distance)
		: normal{ normal }, distance{ distance }
	{
	}

	bool Plane::Contains(const Vector3& point) const
	{
		return MathF::IsNearZero(Vector3::Dot(point, normal) - distance);
	}

	Vector3 Plane::ClosestPoint(const Vector3& point) const
	{
		const float dist = Vector3::Dot(normal, point) - distance;

		return point - normal * dist;
	}

	bool Plane::Overlaps(const Aabb& other) const
	{
		return other.Overlaps(*this);
	}

	bool Plane::Overlaps(const Obb& other) const
	{
		return other.Overlaps(*this);
	}

	bool Plane::Overlaps(const Plane& other) const
	{
		const Vector3 d = Vector3::Cross(normal, other.normal);

		return !MathF::IsNearZero(Vector3::Dot(d, d));
	}

	bool Plane::Overlaps(const Sphere& other) const
	{
		return other.Overlaps(*this);
	}
}
