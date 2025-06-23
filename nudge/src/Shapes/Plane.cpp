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

		// Calculate plane normal using cross product of two triangle edges
		// Cross product gives a vector perpendicular to both edges (the plane normal)
		result.normal = Vector3::Cross(tri.b - tri.a, tri.c - tri.a).Normalized();
		// Calculate signed distance from origin to plane using any point on the plane
		result.distance = Vector3::Dot(result.normal, tri.a);

		return result;
	}

	float Plane::PlaneEquation(const Vector3& point, const Plane& plane)
	{
		// Evaluate the plane equation: ax + by + cz - d = 0
		// Returns signed distance from point to plane
		// Positive = in front of plane, negative = behind plane, zero = on plane
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
		// Check if point lies on the plane by evaluating plane equation
		// Point is on plane if the equation evaluates to near zero
		return MathF::IsNearZero(Vector3::Dot(point, normal) - distance);
	}

	Vector3 Plane::ClosestPoint(const Vector3& point) const
	{
		// Calculate signed distance from point to plane
		const float dist = Vector3::Dot(normal, point) - distance;

		// Project point onto plane by moving along normal by the distance
		return point - normal * dist;
	}

	bool Plane::Intersects(const Aabb& other) const
	{
		// Delegate to AABB's intersection method (avoids code duplication)
		return other.Intersects(*this);
	}

	bool Plane::Intersects(const Obb& other) const
	{
		// Delegate to OBB's intersection method (avoids code duplication)
		return other.Intersects(*this);
	}

	bool Plane::Intersects(const Plane& other) const
	{
		// Two planes intersect if their normals are not parallel
		// Cross product of parallel vectors is zero, non-parallel gives non-zero
		const Vector3 d = Vector3::Cross(normal, other.normal);

		// Check if cross product magnitude is non-zero (planes not parallel)
		return !MathF::IsNearZero(Vector3::Dot(d, d));
	}

	bool Plane::Intersects(const Sphere& other) const
	{
		// Delegate to sphere's intersection method (avoids code duplication)
		return other.Intersects(*this);
	}

	bool Plane::Intersects(const Triangle& other) const
	{
		// Delegate to triangle's intersection method (avoids code duplication)
		return other.Intersects(*this);
	}
}