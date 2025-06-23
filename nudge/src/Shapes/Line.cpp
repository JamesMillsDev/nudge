#include "Nudge/Shapes/Line.hpp"

#include "Nudge/Ray.hpp"
#include "Nudge/RaycastHit.hpp"
#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"

namespace Nudge
{
	Line::Line()
		: Line(Vector3{ 0.f }, Vector3{ 0.f, 1.f, 0.f })
	{
	}

	Line::Line(const Vector3& start, const Vector3& end)
		: start{ start }, end{ end }
	{
	}

	float Line::Length() const
	{
		// Use square root of squared length for efficiency
		return MathF::Sqrt(LengthSqr());
	}

	float Line::LengthSqr() const
	{
		// Calculate vector from start to end and return its squared magnitude
		return (start - end).MagnitudeSqr();
	}

	bool Line::Contains(const Vector3& point) const
	{
		// Find the closest point on the line segment to the test point
		const Vector3 closest = ClosestPoint(point);

		// Check if the closest point on the line is essentially the same as the input point
		return MathF::IsNearZero((closest - point).MagnitudeSqr());
	}

	Vector3 Line::ClosestPoint(const Vector3& point) const
	{
		// Calculate the direction vector of the line segment
		const Vector3 lVec = end - start;
		const float lengthSqr = Vector3::Dot(lVec, lVec);

		// Handle degenerate case (zero-length line)
		if (MathF::IsNearZero(lengthSqr))
		{
			return start;  // Return either endpoint for degenerate line
		}

		// Calculate parameter t using projection, then clamp to [0,1] for line segment
		const float t = MathF::Clamp01(Vector3::Dot(point - start, lVec) / lengthSqr);
		// Interpolate along the line segment using the clamped parameter
		return start + lVec * t;
	}

	bool Line::Test(const Aabb& other) const
	{
		// Create a ray from start point in direction of line segment
		const Ray ray = { start, (end - start) };
		RaycastHit hit;

		// Perform raycast against the AABB
		if (!ray.CastAgainst(other, &hit))
		{
			return false;  // No intersection with infinite ray
		}

		// Check if intersection occurs within the line segment bounds
		const float t = hit.distance;
		return t >= 0.f && MathF::Squared(t) <= LengthSqr();
	}

	bool Line::Test(const Obb& other) const
	{
		// Create a ray from start point in direction of line segment
		const Ray ray = { start, (end - start) };
		RaycastHit hit;

		// Perform raycast against the OBB
		if (!ray.CastAgainst(other, &hit))
		{
			return false;  // No intersection with infinite ray
		}

		// Check if intersection occurs within the line segment bounds
		const float t = hit.distance;
		return t >= 0.f && MathF::Squared(t) <= LengthSqr();
	}

	bool Line::Test(const Plane& other) const
	{
		// Calculate signed distances from both endpoints to the plane
		const float distStart = Vector3::Dot(other.normal, start) - other.distance;
	    const float distEnd = Vector3::Dot(other.normal, end) - other.distance;
	    
	    // Line intersects plane if endpoints are on opposite sides (different signs)
	    // or if either endpoint lies exactly on the plane
	    return (distStart * distEnd <= 0.f);
	}

	bool Line::Test(const Sphere& other) const
	{
		// Find the closest point on the line segment to the sphere center
		Vector3 closest = ClosestPoint(other.origin);
		// Check if this closest point is within the sphere's radius
		return closest.MagnitudeSqr() <= MathF::Squared(other.radius);
	}

	bool Line::Test(const Triangle& other) const
	{
		// Create a ray from start point in direction of line segment
		const Ray ray = { start, (end - start) };
		RaycastHit hit;

		// Perform raycast against the triangle
		if (!ray.CastAgainst(other, &hit))
		{
			return false;  // No intersection with infinite ray
		}

		// Check if intersection occurs within the line segment bounds
		const float t = hit.distance;
		// Use double precision for large value comparisons
	    const double tSqr = static_cast<double>(t) * static_cast<double>(t);
	    const double lengthSqr = static_cast<double>(LengthSqr());
	    
	    return t >= 0.f && tSqr <= lengthSqr;
	}
}