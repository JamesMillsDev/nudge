#include "Nudge/Shapes/Plane.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/OBB.hpp"
#include "Nudge/Shapes/Sphere.hpp"
#include "Nudge/Shapes/Triangle.hpp"

namespace Nudge
{
	Plane Plane::From(const Triangle& tri)
	{
		Plane result;

		result.normal = Vector3::Cross(tri.b - tri.a, tri.c - tri.a).Normalized();
		result.distance = Vector3::Dot(result.normal, tri.a);

		return result;
	}

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

	bool Plane::Intersects(const Aabb& other) const
	{
		return other.Intersects(*this);
	}

	bool Plane::Intersects(const Obb& other) const
	{
		return other.Intersects(*this);
	}

	bool Plane::Intersects(const Plane& other) const
	{
		const Vector3 d = Vector3::Cross(normal, other.normal);

		return !MathF::IsNearZero(Vector3::Dot(d, d));
	}

	bool Plane::Intersects(const Sphere& other) const
	{
		return other.Intersects(*this);
	}

	bool Plane::Intersects(const Triangle& other) const
	{
		return other.Intersects(*this);
	}
}
